/*
 * Seeed Studio XIAO nRF52840 Sense — 208 Hz 6-Axis IMU BLE Streamer
 * Board Package: "Seeed nRF52 mbed-enabled Boards" -> Seeed XIAO BLE Sense - nRF52840
 * Libraries: ArduinoBLE (built-in / Library Manager), Wire (built-in)
 *
 * Features:
 *  - Direct register access to LSM6DS3TR-C over Wire1 @ 400 kHz (208 Hz, ±4g, ±500 dps)
 *  - Hardware FIFO watermark (12 samples) + INT1 hardware ISR timestamping
 *  - Compact 158-byte binary packets with microsecond Epoch timestamps (0.1 us step)
 *  - 200-packet (~11.5 s) RAM ring buffer for brief BLE disconnects
 *  - PDM Microphone & QSPI Flash powered down; IMU powered down when idle
 */

#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Wire.h>

// ---------------- Pin Fallbacks for XIAO nRF52840 Sense (mbed) ----------------
#ifndef PIN_LSM6DS3TR_C_INT1
#define PIN_LSM6DS3TR_C_INT1 18
#endif
#ifndef PIN_PDM_PWR
#define PIN_PDM_PWR 19
#endif
#ifndef PIN_QSPI_SCK
#define PIN_QSPI_SCK 24
#endif
#ifndef PIN_QSPI_CS
#define PIN_QSPI_CS 25
#endif
#ifndef PIN_QSPI_IO0
#define PIN_QSPI_IO0 26
#endif

// ---------------- LSM6DS3TR-C Registers & Settings ----------------
static const uint8_t IMU_ADDR        = 0x6A;
static const uint8_t REG_FIFO_CTRL1  = 0x06;
static const uint8_t REG_FIFO_CTRL2  = 0x07;
static const uint8_t REG_FIFO_CTRL3  = 0x08;
static const uint8_t REG_FIFO_CTRL4  = 0x09;
static const uint8_t REG_FIFO_CTRL5  = 0x0A;
static const uint8_t REG_INT1_CTRL   = 0x0D;
static const uint8_t REG_WHO_AM_I    = 0x0F;
static const uint8_t REG_CTRL1_XL    = 0x10;
static const uint8_t REG_CTRL2_G     = 0x11;
static const uint8_t REG_CTRL3_C     = 0x12;
static const uint8_t REG_FIFO_STATUS1 = 0x3A;
static const uint8_t REG_FIFO_STATUS2 = 0x3B;
static const uint8_t REG_FIFO_STATUS3 = 0x3C;
static const uint8_t REG_FIFO_DATA_L  = 0x3E;

// 208 Hz nominal = 4807.6923 us -> 48077 in 0.1 us units
static const uint16_t NOMINAL_DT_TENTHS_US = 48077;
static const uint8_t  SAMPLES_PER_PACKET   = 12;
static const uint16_t WORDS_PER_SAMPLE     = 6; // Gx, Gy, Gz, Ax, Ay, Az
static const uint16_t FIFO_WATERMARK_WORDS = SAMPLES_PER_PACKET * WORDS_PER_SAMPLE; // 72

// Disconnect buffering config
static const uint16_t RING_BUFFER_CAPACITY        = 200;    // 200 * 12 = 2400 samples (~11.5 s)
static const uint32_t DISCONNECT_BUFFER_TIMEOUT_MS = 60000; // Auto-stop if disconnected > 60 s

// ---------------- Binary BLE Packet Layout (158 bytes max) ----------------
struct __attribute__((packed)) ImuSampleRaw {
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t gx;
  int16_t gy;
  int16_t gz;
};

struct __attribute__((packed)) ImuPacket {
  uint16_t     seq;          // Rolling packet sequence number
  uint8_t      count;        // Number of valid samples in this packet (1..12)
  uint8_t      flags;        // bit0: timeSynced, bit1: buffered, bit2: imuFifoOvr, bit3: ramOvr
  uint64_t     startEpochUs; // Epoch timestamp (us) of samples[0]
  uint16_t     dtTenthsUs;   // Sample period in 0.1 us units (e.g. 48077 = 4807.7 us)
  ImuSampleRaw samples[SAMPLES_PER_PACKET];
};

struct __attribute__((packed)) StatusPayload {
  uint8_t  state;            // 0=idle, 1=recording, 2=error
  uint8_t  flags;            // bit0=timeSynced, bit1=imuReady
  uint16_t bufferedPackets;  // Packets queued in RAM
  uint32_t totalSamples;     // Total samples captured since last reset
};

// ---------------- BLE GATT Service & Characteristics ----------------
BLEService        imuService("19b10000-e8f2-537e-4f6c-d104768a1214");
BLECharacteristic imuDataChar("19b10001-e8f2-537e-4f6c-d104768a1214", BLENotify, sizeof(ImuPacket));
BLECharacteristic timeSyncChar("19b10002-e8f2-537e-4f6c-d104768a1214", BLERead | BLEWrite, 8, true);
BLECharacteristic controlChar("19b10003-e8f2-537e-4f6c-d104768a1214", BLEWrite, 1, true);
BLECharacteristic statusChar("19b10004-e8f2-537e-4f6c-d104768a1214", BLERead | BLENotify, sizeof(StatusPayload), true);

// Commands from Dashboard
enum ControlCmd : uint8_t {
  CMD_START = 0x01,
  CMD_STOP  = 0x02,
  CMD_RESET = 0x03
};

// ---------------- State & Timing ----------------
volatile bool     fifoWatermarkFlag = false;
volatile uint32_t isrMicros32        = 0;

bool     imuReady            = false;
bool     isRecording         = false;
bool     wasConnected        = false;
uint32_t disconnectedSinceMs = 0;

// 64-bit monotonic microsecond clock (rollover-safe)
uint32_t lastMicros32   = 0;
uint32_t microsOverflow = 0;

// Epoch time sync state
bool     isTimeSynced   = false;
uint64_t syncEpochBaseUs = 0;
uint64_t syncMonoBaseUs  = 0;

// Sample timing & counters
uint16_t packetSeq          = 0;
uint32_t totalSamples       = 0;
bool     hasPrevBatch       = false;
uint64_t prevBatchEndEpochUs = 0;
uint16_t smoothedDtTenthsUs = NOMINAL_DT_TENTHS_US;
bool     pendingFifoOverrun = false;
bool     pendingRamOverrun  = false;

// RAM Ring Buffer
ImuPacket ringBuf[RING_BUFFER_CAPACITY];
uint16_t  rbHead  = 0;
uint16_t  rbTail  = 0;
uint16_t  rbCount = 0;

// ---------------- Utility: LEDs & Power ----------------
void setRgbLed(bool r, bool g, bool b) {
#ifdef LED_RED
  digitalWrite(LED_RED,   r ? LOW : HIGH);
#endif
#ifdef LED_GREEN
  digitalWrite(LED_GREEN, g ? LOW : HIGH);
#endif
#ifdef LED_BLUE
  digitalWrite(LED_BLUE,  b ? LOW : HIGH);
#endif
}

void pulseLed(bool r, bool g, bool b, uint16_t ms = 12) {
  setRgbLed(r, g, b);
  delay(ms);
  setRgbLed(false, false, false);
}

void powerOffUnusedPeripherals() {
  // Ensure PDM microphone stays completely powered off
  pinMode(PIN_PDM_PWR, OUTPUT);
  digitalWrite(PIN_PDM_PWR, LOW);
#ifdef PIN_PDM_CLK
  pinMode(PIN_PDM_CLK, INPUT);
#endif
#ifdef PIN_PDM_DIN
  pinMode(PIN_PDM_DIN, INPUT);
#endif

  // Put onboard QSPI Flash (P25Q16H) into Deep Power-Down (0xB9)
  pinMode(PIN_QSPI_CS, OUTPUT);
  pinMode(PIN_QSPI_SCK, OUTPUT);
  pinMode(PIN_QSPI_IO0, OUTPUT);
  digitalWrite(PIN_QSPI_CS, HIGH);
  digitalWrite(PIN_QSPI_SCK, LOW);
  delayMicroseconds(5);

  digitalWrite(PIN_QSPI_CS, LOW);
  uint8_t cmd = 0xB9;
  for (int i = 7; i >= 0; --i) {
    digitalWrite(PIN_QSPI_IO0, (cmd >> i) & 0x01);
    digitalWrite(PIN_QSPI_SCK, HIGH);
    delayMicroseconds(1);
    digitalWrite(PIN_QSPI_SCK, LOW);
  }
  digitalWrite(PIN_QSPI_CS, HIGH);

  // Release QSPI lines cleanly
  pinMode(PIN_QSPI_CS, INPUT_PULLUP);
  pinMode(PIN_QSPI_SCK, INPUT);
  pinMode(PIN_QSPI_IO0, INPUT);
}

// ---------------- 64-bit Microsecond & Epoch Clock ----------------
uint64_t getMonoMicros64() {
  uint32_t now = micros();
  if (now < lastMicros32) {
    microsOverflow++;
  }
  lastMicros32 = now;
  return ((uint64_t)microsOverflow << 32) | (uint64_t)now;
}

uint64_t mono32ToMono64(uint32_t captured32) {
  uint32_t now = micros();
  if (now < lastMicros32) {
    microsOverflow++;
  }
  lastMicros32 = now;
  uint32_t ovf = microsOverflow;
  // Handle edge case where ISR fired just before rollover
  if (captured32 > now && (captured32 - now) > 0x80000000UL && ovf > 0) {
    ovf--;
  }
  return ((uint64_t)ovf << 32) | (uint64_t)captured32;
}

uint64_t monoToEpochUs(uint64_t monoUs) {
  if (!isTimeSynced) {
    return monoUs; // Fallback to uptime us if unsynced
  }
  if (monoUs >= syncMonoBaseUs) {
    return syncEpochBaseUs + (monoUs - syncMonoBaseUs);
  } else {
    return syncEpochBaseUs - (syncMonoBaseUs - monoUs);
  }
}

// ---------------- I2C Register Helpers (Wire1) ----------------
bool imuWriteReg(uint8_t reg, uint8_t val) {
  Wire1.beginTransmission(IMU_ADDR);
  Wire1.write(reg);
  Wire1.write(val);
  return (Wire1.endTransmission() == 0);
}

bool imuReadReg(uint8_t reg, uint8_t &val) {
  Wire1.beginTransmission(IMU_ADDR);
  Wire1.write(reg);
  if (Wire1.endTransmission(false) != 0) return false;
  if (Wire1.requestFrom((int)IMU_ADDR, 1) != 1) return false;
  val = Wire1.read();
  return true;
}

bool imuReadMulti(uint8_t startReg, uint8_t *buf, size_t len) {
  // Read in <= 24-byte bursts for mbed Wire buffer safety
  size_t offset = 0;
  while (offset < len) {
    size_t chunk = min((size_t)24, len - offset);
    Wire1.beginTransmission(IMU_ADDR);
    Wire1.write(startReg);
    if (Wire1.endTransmission(false) != 0) return false;
    if (Wire1.requestFrom((int)IMU_ADDR, (int)chunk) != (int)chunk) return false;
    for (size_t i = 0; i < chunk; ++i) {
      buf[offset + i] = Wire1.read();
    }
    offset += chunk;
  }
  return true;
}

void onFifoWatermarkISR() {
  isrMicros32 = micros();
  fifoWatermarkFlag = true;
}

// ---------------- IMU Power & FIFO Control ----------------
bool initImu() {
#ifdef PIN_LSM6DS3TR_C_POWER
  pinMode(PIN_LSM6DS3TR_C_POWER, OUTPUT);
  digitalWrite(PIN_LSM6DS3TR_C_POWER, HIGH);
  delay(20);
#endif
  Wire1.begin();
  Wire1.setClock(400000); // 400 kHz Fast I2C

  uint8_t who = 0;
  if (!imuReadReg(REG_WHO_AM_I, who)) return false;
  if (who != 0x6A && who != 0x69) return false;

  // Software reset & wait
  imuWriteReg(REG_CTRL3_C, 0x01);
  delay(20);

  // BDU = 1, IF_INC = 1
  imuWriteReg(REG_CTRL3_C, 0x44);

  // Keep sensors powered down until CMD_START
  imuWriteReg(REG_CTRL1_XL, 0x00);
  imuWriteReg(REG_CTRL2_G,  0x00);
  imuWriteReg(REG_FIFO_CTRL5, 0x00);
  imuWriteReg(REG_INT1_CTRL,  0x00);

  pinMode(PIN_LSM6DS3TR_C_INT1, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_LSM6DS3TR_C_INT1), onFifoWatermarkISR, RISING);
  return true;
}

void imuStart208HzFifo() {
  noInterrupts();
  fifoWatermarkFlag = false;
  interrupts();

  // Reset FIFO via Bypass mode
  imuWriteReg(REG_INT1_CTRL,  0x00);
  imuWriteReg(REG_CTRL1_XL,   0x00);
  imuWriteReg(REG_CTRL2_G,    0x00);
  imuWriteReg(REG_FIFO_CTRL5, 0x00);
  delay(5);

  // Set Watermark = 72 words (12 samples * 6 axes)
  imuWriteReg(REG_FIFO_CTRL1, (uint8_t)(FIFO_WATERMARK_WORDS & 0xFF));
  imuWriteReg(REG_FIFO_CTRL2, (uint8_t)((FIFO_WATERMARK_WORDS >> 8) & 0x07));

  // Gyro & Accel in FIFO with no decimation (Gyro first, then Accel)
  imuWriteReg(REG_FIFO_CTRL3, 0x09);
  imuWriteReg(REG_FIFO_CTRL4, 0x00);

  // Continuous FIFO mode @ 208 Hz (0101_110 = 0x2E)
  imuWriteReg(REG_FIFO_CTRL5, 0x2E);

  // Route FIFO threshold watermark to INT1
  imuWriteReg(REG_INT1_CTRL, 0x08);

  // Enable Accel: 208 Hz (0101), ±4g (10) -> 0x58
  // Enable Gyro:  208 Hz (0101), ±500 dps (01) -> 0x54
  imuWriteReg(REG_CTRL1_XL, 0x58);
  imuWriteReg(REG_CTRL2_G,  0x54);
}

void imuStopPowerDown() {
  imuWriteReg(REG_INT1_CTRL,  0x00);
  imuWriteReg(REG_CTRL1_XL,   0x00);
  imuWriteReg(REG_CTRL2_G,    0x00);
  imuWriteReg(REG_FIFO_CTRL5, 0x00);
  noInterrupts();
  fifoWatermarkFlag = false;
  interrupts();
}

// ---------------- Ring Buffer Helpers ----------------
void ringClear() {
  rbHead = rbTail = rbCount = 0;
  pendingRamOverrun = false;
}

void ringPush(const ImuPacket &pkt) {
  if (rbCount == RING_BUFFER_CAPACITY) {
    // Overwrite oldest packet
    rbTail = (rbTail + 1) % RING_BUFFER_CAPACITY;
    rbCount--;
    pendingRamOverrun = true;
  }
  ringBuf[rbHead] = pkt;
  rbHead = (rbHead + 1) % RING_BUFFER_CAPACITY;
  rbCount++;
}

bool ringPop(ImuPacket &pkt) {
  if (rbCount == 0) return false;
  pkt = ringBuf[rbTail];
  rbTail = (rbTail + 1) % RING_BUFFER_CAPACITY;
  rbCount--;
  return true;
}

// ---------------- BLE Status Helper ----------------
void updateStatusCharacteristic() {
  StatusPayload st;
  st.state = imuReady ? (isRecording ? 1 : 0) : 2;
  st.flags = (isTimeSynced ? 0x01 : 0x00) | (imuReady ? 0x02 : 0x00);
  st.bufferedPackets = rbCount;
  st.totalSamples    = totalSamples;
  statusChar.writeValue((const uint8_t *)&st, sizeof(st));
}

// ---------------- Drain IMU Hardware FIFO into Packets ----------------
void drainImuFifo(bool flushRemainder, uint64_t captureEpochUs) {
  uint8_t s1 = 0, s2 = 0, s3 = 0;
  if (!imuReadReg(REG_FIFO_STATUS1, s1) || !imuReadReg(REG_FIFO_STATUS2, s2)) {
    return;
  }

  if (s2 & 0x40) {
    pendingFifoOverrun = true;
  }

  uint16_t unreadWords = ((uint16_t)(s2 & 0x07) << 8) | (uint16_t)s1;

  // Ensure word alignment starts at Gx (FIFO_PATTERN % 6 == 0)
  if (unreadWords > 0 && imuReadReg(REG_FIFO_STATUS3, s3)) {
    uint8_t mod = s3 % WORDS_PER_SAMPLE;
    if (mod != 0) {
      uint16_t skipWords = WORDS_PER_SAMPLE - mod;
      if (skipWords > unreadWords) skipWords = unreadWords;
      uint8_t discardBuf[12];
      imuReadMulti(REG_FIFO_DATA_L, discardBuf, skipWords * 2);
      unreadWords -= skipWords;
    }
  }

  uint16_t availSamples = unreadWords / WORDS_PER_SAMPLE;
  if (availSamples == 0) return;

  // If not flushing on stop, only read full 12-sample packets
  while (availSamples >= SAMPLES_PER_PACKET || (flushRemainder && availSamples > 0)) {
    uint8_t batchCount = (availSamples >= SAMPLES_PER_PACKET)
                           ? SAMPLES_PER_PACKET
                           : (uint8_t)availSamples;

    uint8_t rawBytes[SAMPLES_PER_PACKET * 12];
    if (!imuReadMulti(REG_FIFO_DATA_L, rawBytes, batchCount * 12)) {
      break;
    }

    ImuPacket pkt;
    pkt.seq   = packetSeq++;
    pkt.count = batchCount;
    pkt.flags = (isTimeSynced ? 0x01 : 0x00) |
                (!BLE.connected() ? 0x02 : 0x00) |
                (pendingFifoOverrun ? 0x04 : 0x00) |
                (pendingRamOverrun ? 0x08 : 0x00);
    pendingFifoOverrun = false;
    pendingRamOverrun  = false;

    // Decode FIFO words: Gyro (Gx, Gy, Gz) first, then Accel (Ax, Ay, Az)
    for (uint8_t i = 0; i < batchCount; ++i) {
      const uint8_t *p = &rawBytes[i * 12];
      int16_t gx = (int16_t)((uint16_t)p[0]  | ((uint16_t)p[1]  << 8));
      int16_t gy = (int16_t)((uint16_t)p[2]  | ((uint16_t)p[3]  << 8));
      int16_t gz = (int16_t)((uint16_t)p[4]  | ((uint16_t)p[5]  << 8));
      int16_t ax = (int16_t)((uint16_t)p[6]  | ((uint16_t)p[7]  << 8));
      int16_t ay = (int16_t)((uint16_t)p[8]  | ((uint16_t)p[9]  << 8));
      int16_t az = (int16_t)((uint16_t)p[10] | ((uint16_t)p[11] << 8));

      pkt.samples[i].ax = ax;
      pkt.samples[i].ay = ay;
      pkt.samples[i].az = az;
      pkt.samples[i].gx = gx;
      pkt.samples[i].gy = gy;
      pkt.samples[i].gz = gz;
    }

    // Compute drift-compensated microsecond Epoch timestamps
    uint64_t batchEndEpochUs = captureEpochUs;
    if (availSamples > batchCount) {
      // Earlier batch inside a multi-batch backlog: step backward by nominal period
      uint64_t extraSamples = availSamples - batchCount;
      batchEndEpochUs -= (extraSamples * (uint64_t)smoothedDtTenthsUs) / 10ULL;
    }

    if (!hasPrevBatch) {
      smoothedDtTenthsUs = NOMINAL_DT_TENTHS_US;
      uint64_t spanUs = ((uint64_t)(batchCount - 1) * smoothedDtTenthsUs + 5ULL) / 10ULL;
      pkt.startEpochUs = (batchEndEpochUs > spanUs) ? (batchEndEpochUs - spanUs) : 0;
      pkt.dtTenthsUs   = smoothedDtTenthsUs;
      prevBatchEndEpochUs = batchEndEpochUs;
      hasPrevBatch = true;
    } else {
      int64_t deltaUs = (int64_t)(batchEndEpochUs - prevBatchEndEpochUs);
      if (deltaUs > 0 && batchCount > 0) {
        uint32_t instDtTenths = (uint32_t)((deltaUs * 10LL) / batchCount);
        // Smooth ODR clock estimate within ±6% of 208 Hz to absorb polling jitter
        if (instDtTenths >= 45000 && instDtTenths <= 51000) {
          smoothedDtTenthsUs = (uint16_t)((smoothedDtTenthsUs * 7UL + instDtTenths) / 8UL);
        }
      }
      pkt.dtTenthsUs   = smoothedDtTenthsUs;
      pkt.startEpochUs = prevBatchEndEpochUs + (smoothedDtTenthsUs + 5ULL) / 10ULL;
      prevBatchEndEpochUs = pkt.startEpochUs + ((uint64_t)(batchCount - 1) * smoothedDtTenthsUs + 5ULL) / 10ULL;
    }

    totalSamples += batchCount;
    availSamples -= batchCount;
    ringPush(pkt);
  }
}

// ---------------- Control & Sync Handlers ----------------
void startRecording() {
  if (!imuReady) return;
  hasPrevBatch = false;
  smoothedDtTenthsUs = NOMINAL_DT_TENTHS_US;
  isRecording = true;
  imuStart208HzFifo();
  updateStatusCharacteristic();
  pulseLed(false, true, false, 15); // Brief green flash
}

void stopRecording() {
  if (isRecording) {
    uint64_t stopEpochUs = monoToEpochUs(getMonoMicros64());
    drainImuFifo(true, stopEpochUs); // Flush partial samples < 12
    imuStopPowerDown();
    isRecording = false;
    hasPrevBatch = false;
    updateStatusCharacteristic();
    pulseLed(true, false, false, 15); // Brief red flash
  }
}

void resetAllData() {
  imuStopPowerDown();
  isRecording = false;
  hasPrevBatch = false;
  ringClear();
  packetSeq = 0;
  totalSamples = 0;
  pendingFifoOverrun = false;
  pendingRamOverrun = false;
  smoothedDtTenthsUs = NOMINAL_DT_TENTHS_US;
  updateStatusCharacteristic();
  pulseLed(false, false, true, 20); // Brief blue flash
}

void handleBleWrites() {
  if (timeSyncChar.written()) {
    uint8_t buf[8] = {0};
    int len = timeSyncChar.readValue(buf, 8);
    if (len == 8) {
      uint64_t newEpochUs = 0;
      memcpy(&newEpochUs, buf, 8);
      uint64_t nowMono = getMonoMicros64();

      // Re-anchor active stream seamlessly if already recording
      if (isTimeSynced && hasPrevBatch) {
        int64_t offsetDelta = (int64_t)newEpochUs - (int64_t)monoToEpochUs(nowMono);
        prevBatchEndEpochUs = (uint64_t)((int64_t)prevBatchEndEpochUs + offsetDelta);
      }

      syncEpochBaseUs = newEpochUs;
      syncMonoBaseUs  = nowMono;
      isTimeSynced    = true;
      updateStatusCharacteristic();
    }
  }

  if (controlChar.written()) {
    uint8_t cmd = 0;
    controlChar.readValue(&cmd, 1);
    switch (cmd) {
      case CMD_START: startRecording(); break;
      case CMD_STOP:  stopRecording();  break;
      case CMD_RESET: resetAllData();   break;
      default: break;
    }
  }
}

// ---------------- Setup & Main Loop ----------------
void setup() {
  Serial.begin(115200);

#ifdef LED_RED
  pinMode(LED_RED, OUTPUT);
#endif
#ifdef LED_GREEN
  pinMode(LED_GREEN, OUTPUT);
#endif
#ifdef LED_BLUE
  pinMode(LED_BLUE, OUTPUT);
#endif
  setRgbLed(false, false, false);

  powerOffUnusedPeripherals();
  imuReady = initImu();

  if (!BLE.begin()) {
    while (1) {
      pulseLed(true, false, false, 100);
      delay(400);
    }
  }

  BLE.setLocalName("XIAO-IMU-208Hz");
  BLE.setDeviceName("XIAO-IMU-208Hz");
  BLE.setAdvertisedService(imuService);

  imuService.addCharacteristic(imuDataChar);
  imuService.addCharacteristic(timeSyncChar);
  imuService.addCharacteristic(controlChar);
  imuService.addCharacteristic(statusChar);
  BLE.addService(imuService);

  // 15 ms – 30 ms connection interval: fast backlog flush + good battery efficiency
  BLE.setConnectionInterval(12, 24);
  BLE.setAdvertisingInterval(320); // 200 ms advertising

  updateStatusCharacteristic();
  BLE.advertise();

  pulseLed(false, imuReady, !imuReady, 40);
}

void loop() {
  BLE.poll();
  getMonoMicros64(); // Keep 64-bit microsecond rollover tracker up to date

  bool connected = BLE.connected();
  if (connected && !wasConnected) {
    wasConnected = true;
    disconnectedSinceMs = 0;
    updateStatusCharacteristic();
    pulseLed(false, false, true, 15);
  } else if (!connected && wasConnected) {
    wasConnected = false;
    disconnectedSinceMs = millis();
    BLE.advertise();
  }

  // Stop recording if disconnected too long to protect battery
  if (!connected && isRecording && disconnectedSinceMs != 0 &&
      (millis() - disconnectedSinceMs > DISCONNECT_BUFFER_TIMEOUT_MS)) {
    stopRecording();
  }

  if (connected) {
    handleBleWrites();
  }

  // Drain IMU hardware FIFO when watermark fires (or every 65 ms failsafe)
  static uint32_t lastDrainMs = 0;
  if (isRecording && (fifoWatermarkFlag || (millis() - lastDrainMs >= 65))) {
    uint32_t captured32;
    noInterrupts();
    bool triggered = fifoWatermarkFlag;
    captured32 = isrMicros32;
    fifoWatermarkFlag = false;
    interrupts();

    uint64_t monoCapUs = triggered ? mono32ToMono64(captured32) : getMonoMicros64();
    drainImuFifo(false, monoToEpochUs(monoCapUs));
    lastDrainMs = millis();
  }

  // Stream queued packets over BLE when connected & subscribed
  if (connected && imuDataChar.subscribed() && rbCount > 0) {
    // Send up to 2 packets per loop iteration so backlog flushes smoothly
    uint8_t burst = (rbCount > 2) ? 2 : (uint8_t)rbCount;
    for (uint8_t i = 0; i < burst; ++i) {
      ImuPacket pkt;
      if (ringPop(pkt)) {
        size_t pktSize = 14 + (size_t)pkt.count * sizeof(ImuSampleRaw);
        imuDataChar.writeValue((const uint8_t *)&pkt, pktSize);
        BLE.poll();
      }
    }
  }

  // Periodic status update (~1 Hz) when idle or connected
  static uint32_t lastStatusMs = 0;
  if (connected && (millis() - lastStatusMs >= 1000)) {
    updateStatusCharacteristic();
    lastStatusMs = millis();
  }

  // Yield to Mbed RTOS tickless sleep when no backlog is waiting
  if (rbCount == 0) {
    delay(isRecording ? 4 : 25);
  } else {
    delay(1);
  }
}