/*
 * IMU_BLE_Recorder_Optimized.ino
 * 
 * Xiao nRF52840 Sense — BLE IMU Streamer (mbed core)
 * Optimizations: burst I2C read, threshold-based send, no timer drift
 * 
 * For multi-sensor setup, change DEVICE_NAME to:
 *   "IMU-Right", "IMU-Left", or "IMU-Torso"
 * 
 * Board: Seeed XIAO BLE Sense - nRF52840 (mbed-enabled)
 * Libraries: ArduinoBLE, Seeed Arduino LSM6DS3
 */

#include <ArduinoBLE.h>
#include <LSM6DS3.h>
#include <Wire.h>

// ==================== CONFIG ====================
// Change this for each sensor: Right, Left, Torso
const char* DEVICE_NAME = "IMU-Recorder";

const char* SERVICE_UUID      = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
const char* CHAR_TIME_UUID    = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
const char* CHAR_CONTROL_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a9";
const char* CHAR_DATA_UUID    = "beb5483e-36e1-4688-b7f5-ea07361b26aa";

// Samples per BLE packet (4 = 66 bytes, safe for all MTUs)
const int SAMPLES_PER_PACKET = 4;

// ==================== LED ====================
#define LED_PIN LED_BUILTIN
#define LED_ON  LOW    // Active LOW
#define LED_OFF HIGH

// ==================== IMU ====================
LSM6DS3 imu(I2C_MODE, 0x6A);

// ==================== TIMING ====================
const unsigned long SAMPLE_INTERVAL_US = 5000UL;  // 200 Hz
unsigned long lastSampleUs = 0;

// ==================== SAMPLE BUFFER ====================
struct Sample {
  uint32_t relMs;
  int16_t  ax, ay, az;
  int16_t  gx, gy, gz;
};

#define BUFFER_SIZE 2048
Sample sampleBuffer[BUFFER_SIZE];
volatile uint16_t bufferHead = 0;
volatile uint16_t bufferTail = 0;

inline int bufferCount() {
  return (bufferHead - bufferTail + BUFFER_SIZE) % BUFFER_SIZE;
}
inline bool bufferEmpty() {
  return bufferHead == bufferTail;
}
inline bool bufferFull() {
  return ((bufferHead + 1) % BUFFER_SIZE) == bufferTail;
}

// ==================== STATE ====================
volatile bool     recording    = false;
volatile uint32_t baseEpochMs  = 0;
volatile uint32_t baseMillis   = 0;
uint8_t seqNum = 0;

// ==================== BLE ====================
BLEService imuService(SERVICE_UUID);
BLECharacteristic timeChar(CHAR_TIME_UUID, BLEWrite, 4);
BLECharacteristic controlChar(CHAR_CONTROL_UUID, BLEWrite, 1);
BLECharacteristic dataChar(CHAR_DATA_UUID, BLENotify, 200);

// ==================== FORWARD DECLARATIONS ====================
void onTimeWritten(BLEDevice central, BLECharacteristic characteristic);
void onControlWritten(BLEDevice central, BLECharacteristic characteristic);
void sampleIMU();
void sendSamples();
void readIMU(int16_t* ax, int16_t* ay, int16_t* az,
             int16_t* gx, int16_t* gy, int16_t* gz);

// ==================== SETUP ====================
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_OFF);

  Serial.begin(115200);
  for (int i = 0; i < 10 && !Serial; i++) delay(100);

  Serial.println("\n=== IMU BLE Recorder (Optimized) ===");
  Serial.print("Device name: ");
  Serial.println(DEVICE_NAME);

  Wire.begin();
  Wire.setClock(400000);

  imu.settings.accelSampleRate = 416;
  imu.settings.gyroSampleRate  = 416;
  imu.settings.accelRange      = 2;
  imu.settings.gyroRange       = 245;

  int imuTries = 0;
  while (imu.begin() != 0 && imuTries < 5) {
    Serial.println("IMU init failed, retrying...");
    digitalWrite(LED_PIN, LED_ON); delay(100);
    digitalWrite(LED_PIN, LED_OFF); delay(500);
    imuTries++;
  }

  if (imuTries >= 5) {
    Serial.println("IMU FATAL!");
    while (1) {
      digitalWrite(LED_PIN, LED_ON); delay(100);
      digitalWrite(LED_PIN, LED_OFF); delay(100);
    }
  }

  // 416 Hz, +/-2g, +/-250 dps
  imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x60);
  imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, 0x60);

  Serial.println("IMU ready (burst I2C mode)");

  if (!BLE.begin()) {
    Serial.println("BLE FATAL!");
    while (1) {
      digitalWrite(LED_PIN, LED_ON); delay(50);
      digitalWrite(LED_PIN, LED_OFF); delay(50);
    }
  }

  BLE.setLocalName(DEVICE_NAME);
  BLE.setDeviceName(DEVICE_NAME);
  BLE.setAdvertisedService(imuService);

  imuService.addCharacteristic(timeChar);
  imuService.addCharacteristic(controlChar);
  imuService.addCharacteristic(dataChar);
  BLE.addService(imuService);

  uint8_t zero[200] = {0};
  dataChar.writeValue(zero, 200);
  timeChar.writeValue((uint32_t)0);
  controlChar.writeValue((uint8_t)0);

  timeChar.setEventHandler(BLEWritten, onTimeWritten);
  controlChar.setEventHandler(BLEWritten, onControlWritten);

  BLE.advertise();

  Serial.println("Advertising...");
  digitalWrite(LED_PIN, LED_ON);   // Solid = ready
}

// ==================== MAIN LOOP ====================
void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected: ");
    Serial.println(central.address());

    recording = false;
    seqNum = 0;
    bufferHead = 0;
    bufferTail = 0;

    while (central.connected()) {
      unsigned long nowUs = micros();

      // ---- Sample at 200 Hz (reset timer to prevent drift) ----
      if (nowUs - lastSampleUs >= SAMPLE_INTERVAL_US) {
        lastSampleUs = nowUs;
        if (recording) sampleIMU();
      }

      // ---- Send when a full batch is ready ----
      // Natural pacing: 4 samples @ 200 Hz => every 20 ms (50 Hz)
      if (recording && bufferCount() >= SAMPLES_PER_PACKET) {
        sendSamples();
      }

      // ---- LED feedback ----
      if (recording) {
        digitalWrite(LED_PIN, ((millis() / 250) % 2) ? LED_ON : LED_OFF);
      }

      BLE.poll();
    }

    Serial.println("Disconnected");
    recording = false;
    digitalWrite(LED_PIN, LED_ON);
  }
}

// ==================== BURST I2C READ ====================
// Reads all 12 bytes (gyro + accel) in ONE transaction.
// LSM6DS3 registers 0x22..0x2D are contiguous.
void readIMU(int16_t* ax, int16_t* ay, int16_t* az,
             int16_t* gx, int16_t* gy, int16_t* gz) {
  Wire.beginTransmission(0x6A);
  Wire.write(0x22);               // OUTX_L_G
  Wire.endTransmission(false);
  Wire.requestFrom(0x6A, 12);

  uint8_t buf[12];
  for (int i = 0; i < 12; i++) {
    buf[i] = Wire.available() ? Wire.read() : 0;
  }

  *gx = (int16_t)(buf[1] << 8 | buf[0]);
  *gy = (int16_t)(buf[3] << 8 | buf[2]);
  *gz = (int16_t)(buf[5] << 8 | buf[4]);
  *ax = (int16_t)(buf[7] << 8 | buf[6]);
  *ay = (int16_t)(buf[9] << 8 | buf[8]);
  *az = (int16_t)(buf[11] << 8 | buf[10]);
}

// ==================== SAMPLING ====================
void sampleIMU() {
  if (bufferFull()) {
    bufferTail = (bufferTail + 1) % BUFFER_SIZE;  // drop oldest
  }

  Sample s;
  s.relMs = (uint32_t)(millis() - baseMillis);

  readIMU(&s.ax, &s.ay, &s.az, &s.gx, &s.gy, &s.gz);

  uint16_t idx = bufferHead;
  sampleBuffer[idx] = s;
  bufferHead = (idx + 1) % BUFFER_SIZE;
}

// ==================== BLE TRANSMISSION ====================
void sendSamples() {
  int available = bufferCount();
  if (available < SAMPLES_PER_PACKET) return;

  int count = (available < SAMPLES_PER_PACKET) ? available : SAMPLES_PER_PACKET;
  uint8_t packet[2 + count * 16];

  packet[0] = seqNum;
  packet[1] = count;

  int offset = 2;
  for (int i = 0; i < count; i++) {
    uint16_t idx = (bufferTail + i) % BUFFER_SIZE;
    Sample s = sampleBuffer[idx];

    packet[offset++] = (uint8_t)(s.relMs);
    packet[offset++] = (uint8_t)(s.relMs >> 8);
    packet[offset++] = (uint8_t)(s.relMs >> 16);
    packet[offset++] = (uint8_t)(s.relMs >> 24);

    packet[offset++] = (uint8_t)(s.ax);
    packet[offset++] = (uint8_t)(s.ax >> 8);
    packet[offset++] = (uint8_t)(s.ay);
    packet[offset++] = (uint8_t)(s.ay >> 8);
    packet[offset++] = (uint8_t)(s.az);
    packet[offset++] = (uint8_t)(s.az >> 8);

    packet[offset++] = (uint8_t)(s.gx);
    packet[offset++] = (uint8_t)(s.gx >> 8);
    packet[offset++] = (uint8_t)(s.gy);
    packet[offset++] = (uint8_t)(s.gy >> 8);
    packet[offset++] = (uint8_t)(s.gz);
    packet[offset++] = (uint8_t)(s.gz >> 8);
  }

  int packetLen = 2 + count * 16;
  if (dataChar.writeValue(packet, packetLen)) {
    seqNum = (seqNum + 1) & 0xFF;
    bufferTail = (bufferTail + count) % BUFFER_SIZE;
  }
  // If TX queue full, retry on next loop iteration
}

// ==================== BLE EVENT HANDLERS ====================
void onTimeWritten(BLEDevice central, BLECharacteristic characteristic) {
  if (characteristic.valueLength() == 4) {
    const uint8_t* v = characteristic.value();
    uint32_t epochSec = ((uint32_t)v[0]) |
                        ((uint32_t)v[1] << 8) |
                        ((uint32_t)v[2] << 16) |
                        ((uint32_t)v[3] << 24);

    baseEpochMs = (uint32_t)epochSec * 1000UL;
    baseMillis  = millis();

    Serial.print("Time synced: ");
    Serial.println(epochSec);
  }
}

void onControlWritten(BLEDevice central, BLECharacteristic characteristic) {
  if (characteristic.valueLength() == 1) {
    uint8_t cmd = characteristic.value()[0];

    if (cmd == 0x01) {
      recording = true;
      seqNum = 0;
      bufferHead = 0;
      bufferTail = 0;
      lastSampleUs = micros();
      Serial.println(">>> START");

    } else if (cmd == 0x00) {
      recording = false;
      Serial.println(">>> STOP");
      Serial.print("Buffered: ");
      Serial.println(bufferCount());
    }
  }
}
