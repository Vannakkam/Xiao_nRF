/*
  XIAO nRF52840 Sense: 200 Hz IMU recorder over Web Bluetooth

  Arduino IDE requirements:
    1. Boards Manager URL:
       https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
    2. Install "Seeed nRF52 Boards" and select
       "Seeed XIAO nRF52840 Sense" from that package.
    3. Install "Seeed Arduino LSM6DS3" (Seeed_Arduino_LSM6DS3).
       BLE comes from the Bluefruit52Lib bundled with the board package;
       do not install or include ArduinoBLE for this version.

  Protocol (all multibyte fields are little-endian):
    Control characteristic, browser -> board:
      START: [0x01][epoch_ms:uint64]
      STOP:  [0x02]
      STATUS:[0x03]

    Data notification:
      [0xA1][count:u8][packet_seq:u16][first_sample_seq:u32]
      [first_unix_epoch_us:uint64]
      followed by count * [gx,gy,gz,ax,ay,az:int16].
      Later samples in the packet are spaced exactly 5000 us apart.

    Status notification:
      ['S']['T'][version:u8][state:u8][buffered:u16]
      [dropped:u32][captured:u32][reserved:u16]
*/

#include <bluefruit.h>
#include <LSM6DS3.h>
#include <Wire.h>

// Custom UUIDs shared with index.html.
static const char SERVICE_UUID[] = "91bad492-b950-4226-aa2b-4ede9fa42f59";
static const char DATA_UUID[]    = "cba1d466-344c-4be3-ab3f-189f80dd7518";
static const char CONTROL_UUID[] = "cba1d466-344c-4be3-ab3f-189f80dd7519";
static const char STATUS_UUID[]  = "cba1d466-344c-4be3-ab3f-189f80dd7520";

constexpr uint32_t SAMPLE_PERIOD_US = 5000;  // exactly 200 exported samples/s
constexpr uint8_t MAX_SAMPLES_PER_PACKET = 12;
constexpr size_t DATA_HEADER_BYTES = 16;
constexpr size_t BYTES_PER_SAMPLE = 12;
constexpr size_t MAX_DATA_BYTES =
    DATA_HEADER_BYTES + MAX_SAMPLES_PER_PACKET * BYTES_PER_SAMPLE;
constexpr uint16_t RING_CAPACITY = 512;       // 2.56 seconds at 200 Hz

BLEService imuService = BLEService(SERVICE_UUID);
BLECharacteristic dataCharacteristic = BLECharacteristic(DATA_UUID);
BLECharacteristic controlCharacteristic = BLECharacteristic(CONTROL_UUID);
BLECharacteristic statusCharacteristic = BLECharacteristic(STATUS_UUID);

// The built-in LSM6DS3TR-C responds at I2C address 0x6A.
LSM6DS3 imu(I2C_MODE, 0x6A);

struct Sample {
  uint32_t sequence;
  int16_t gx;
  int16_t gy;
  int16_t gz;
  int16_t ax;
  int16_t ay;
  int16_t az;
};

Sample ringBuffer[RING_CAPACITY];
uint16_t ringHead = 0;
uint16_t ringTail = 0;
uint16_t ringCount = 0;

enum RecorderState : uint8_t { IDLE = 0, RECORDING = 1, FLUSHING = 2 };
RecorderState recorderState = IDLE;
uint32_t nextSampleUs = 0;
uint32_t sampleSequence = 0;
uint16_t packetSequence = 0;
uint32_t droppedSamples = 0;
uint64_t startEpochMs = 0;
uint32_t lastStatusMs = 0;
volatile uint8_t pendingCommand = 0;
volatile uint64_t pendingEpochMs = 0;

static void putU16(uint8_t* out, uint16_t value) {
  out[0] = (uint8_t)value;
  out[1] = (uint8_t)(value >> 8);
}

static void putU32(uint8_t* out, uint32_t value) {
  out[0] = (uint8_t)value;
  out[1] = (uint8_t)(value >> 8);
  out[2] = (uint8_t)(value >> 16);
  out[3] = (uint8_t)(value >> 24);
}

static void putU64(uint8_t* out, uint64_t value) {
  for (uint8_t i = 0; i < 8; ++i) {
    out[i] = (uint8_t)value;
    value >>= 8;
  }
}

static void putI16(uint8_t* out, int16_t value) {
  putU16(out, (uint16_t)value);
}

static uint64_t getU64(const uint8_t* in) {
  uint64_t value = 0;
  for (int i = 7; i >= 0; --i) value = (value << 8) | in[i];
  return value;
}

static void clearRing() {
  ringHead = ringTail = ringCount = 0;
}

static void setImuActive(bool active) {
  // CTRL1_XL / CTRL2_G: 208 Hz, +/-4 g and +/-500 dps when active;
  // ODR=power-down when idle. This avoids running the IMU between recordings.
  imu.writeRegister(0x10, active ? 0x58 : 0x00);
  imu.writeRegister(0x11, active ? 0x54 : 0x00);
}

static void sendStatus() {
  uint8_t status[16] = { 'S', 'T', 1, (uint8_t)recorderState };
  putU16(status + 4, ringCount);
  putU32(status + 6, droppedSamples);
  putU32(status + 10, sampleSequence);
  statusCharacteristic.write(status, sizeof(status));
  if (statusCharacteristic.notifyEnabled()) {
    statusCharacteristic.notify(status, sizeof(status));
  }
  lastStatusMs = millis();
}

static void beginRecording(uint64_t epochMs) {
  clearRing();
  setImuActive(true);
  startEpochMs = epochMs;
  sampleSequence = 0;
  packetSequence = 0;
  droppedSamples = 0;
  nextSampleUs = micros() + SAMPLE_PERIOD_US;
  recorderState = RECORDING;
  sendStatus();
}

static void controlWriteCallback(uint16_t connHandle,
                                 BLECharacteristic* characteristic,
                                 uint8_t* command,
                                 uint16_t length) {
  (void)connHandle;
  (void)characteristic;
  if (length < 1) return;

  // The callback runs in the BLE task. Defer state/ring-buffer changes to the
  // Arduino loop so sampling and control never mutate the buffer concurrently.
  if (command[0] == 0x01 && length >= 9) {
    pendingEpochMs = getU64(command + 1);
    pendingCommand = 0x01;
  } else if (command[0] == 0x02 || command[0] == 0x03) {
    pendingCommand = command[0];
  }
}

static void processPendingControl() {
  const uint8_t command = pendingCommand;
  if (command == 0) return;
  pendingCommand = 0;

  if (command == 0x01) {
    beginRecording(pendingEpochMs);
  } else if (command == 0x02) {
    if (recorderState == RECORDING) {
      setImuActive(false);
      recorderState = FLUSHING;
    }
    sendStatus();
  } else if (command == 0x03) {
    sendStatus();
  }
}

static int16_t readI16LE(const uint8_t* data) {
  return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static void acquireSample() {
  // One contiguous read: gyro XYZ at 0x22, then accelerometer XYZ at 0x28.
  uint8_t raw[12];
  if (imu.readRegisterRegion(raw, 0x22, sizeof(raw)) != 0) {
    ++droppedSamples;
    ++sampleSequence;
    return;
  }

  Sample sample;
  sample.sequence = sampleSequence++;
  sample.gx = readI16LE(raw + 0);
  sample.gy = readI16LE(raw + 2);
  sample.gz = readI16LE(raw + 4);
  sample.ax = readI16LE(raw + 6);
  sample.ay = readI16LE(raw + 8);
  sample.az = readI16LE(raw + 10);

  if (ringCount == RING_CAPACITY) {
    // Keep the newest measurements and make the loss visible to the browser.
    ringTail = (ringTail + 1) % RING_CAPACITY;
    --ringCount;
    ++droppedSamples;
  }
  ringBuffer[ringHead] = sample;
  ringHead = (ringHead + 1) % RING_CAPACITY;
  ++ringCount;
}

static bool sendOnePacket() {
  if (ringCount == 0 || !dataCharacteristic.notifyEnabled()) return false;

  uint8_t count = min((uint16_t)MAX_SAMPLES_PER_PACKET, ringCount);
  // End a packet at a sequence gap (for example, a failed IMU read). This lets
  // the browser detect the exact missing slot from the next packet header.
  uint16_t scan = ringTail;
  const uint32_t firstSequence = ringBuffer[ringTail].sequence;
  for (uint8_t i = 1; i < count; ++i) {
    scan = (scan + 1) % RING_CAPACITY;
    if (ringBuffer[scan].sequence != firstSequence + i) {
      count = i;
      break;
    }
  }
  uint8_t packet[MAX_DATA_BYTES];
  packet[0] = 0xA1;
  packet[1] = count;
  putU16(packet + 2, packetSequence);
  putU32(packet + 4, firstSequence);
  putU64(packet + 8,
         startEpochMs * 1000ULL + (uint64_t)firstSequence * SAMPLE_PERIOD_US);

  uint16_t index = ringTail;
  size_t offset = DATA_HEADER_BYTES;
  for (uint8_t i = 0; i < count; ++i) {
    const Sample& s = ringBuffer[index];
    putI16(packet + offset + 0, s.gx);
    putI16(packet + offset + 2, s.gy);
    putI16(packet + offset + 4, s.gz);
    putI16(packet + offset + 6, s.ax);
    putI16(packet + offset + 8, s.ay);
    putI16(packet + offset + 10, s.az);
    offset += BYTES_PER_SAMPLE;
    index = (index + 1) % RING_CAPACITY;
  }

  // Do not remove samples unless the SoftDevice accepted the notification.
  if (!dataCharacteristic.notify(packet, offset)) return false;

  ringTail = index;
  ringCount -= count;
  ++packetSequence;
  return true;
}

void setup() {
  Wire.setClock(400000);

  // The IMU hardware runs at its supported 208 Hz rate. The scheduler below
  // reads it every 5 ms, producing the requested 200 Hz timestamp grid.
  imu.settings.accelEnabled = 1;
  imu.settings.accelRange = 4;          // +/-4 g, 0.000122 g/LSB
  imu.settings.accelSampleRate = 208;
  imu.settings.gyroEnabled = 1;
  imu.settings.gyroRange = 500;         // +/-500 dps, 0.0175 dps/LSB
  imu.settings.gyroSampleRate = 208;

  if (imu.begin() != 0) {
    while (true) delay(1000);           // IMU not detected
  }
  setImuActive(false);
  // Allocate a 247-byte ATT MTU and a larger notification queue. Must be
  // configured before Bluefruit.begin().
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin(1, 0);
  Bluefruit.setTxPower(0);
  Bluefruit.setName("XIAO IMU 200Hz");
  Bluefruit.Periph.setConnInterval(6, 12);  // 7.5-15 ms

  // A service must begin before its characteristics in Bluefruit52Lib.
  imuService.begin();

  dataCharacteristic.setProperties(CHR_PROPS_NOTIFY);
  dataCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  dataCharacteristic.setMaxLen(MAX_DATA_BYTES);
  dataCharacteristic.begin();

  controlCharacteristic.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  controlCharacteristic.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  controlCharacteristic.setMaxLen(16);
  controlCharacteristic.setWriteCallback(controlWriteCallback);
  controlCharacteristic.begin();

  statusCharacteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  statusCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  statusCharacteristic.setFixedLen(16);
  statusCharacteristic.begin();

  sendStatus();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(imuService);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}

void loop() {
  processPendingControl();

  if (!Bluefruit.connected() && recorderState != IDLE) {
    setImuActive(false);
    recorderState = IDLE;
    clearRing();
  }

  if (recorderState == RECORDING) {
    uint32_t now = micros();
    // Catch up after a short BLE delay, but cap work so BLE remains responsive.
    uint8_t catchUp = 0;
    while ((int32_t)(now - nextSampleUs) >= 0 && catchUp < 4) {
      acquireSample();
      nextSampleUs += SAMPLE_PERIOD_US;
      ++catchUp;
    }
    if ((int32_t)(now - nextSampleUs) >= 0) {
      // A long stall exceeded the catch-up allowance. Account for skipped slots.
      uint32_t skipped = (now - nextSampleUs) / SAMPLE_PERIOD_US + 1;
      sampleSequence += skipped;
      droppedSamples += skipped;
      nextSampleUs += skipped * SAMPLE_PERIOD_US;
    }
  }

  if (ringCount >= MAX_SAMPLES_PER_PACKET || recorderState == FLUSHING) {
    sendOnePacket();
  }

  if (recorderState == FLUSHING && ringCount == 0) {
    recorderState = IDLE;
    sendStatus();
  } else if (Bluefruit.connected() && millis() - lastStatusMs >= 1000) {
    sendStatus();
  }

  // Yield to the SoftDevice/FreeRTOS task when no immediate work is pending.
  delay(1);
}
