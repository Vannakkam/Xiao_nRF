/*
 * IMU_BLE_Recorder_Optimized.ino
 * 
 * Xiao nRF52840 Sense — BLE IMU Streamer (mbed core)
 * Batches 4 samples/packet to eliminate packet loss
 * 
 * Board: Seeed XIAO BLE Sense - nRF52840 (mbed-enabled)
 * Libraries: ArduinoBLE, Seeed Arduino LSM6DS3
 */

#include <ArduinoBLE.h>
#include <LSM6DS3.h>
#include <Wire.h>

// ==================== BLE UUIDs ====================
const char* SERVICE_UUID      = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
const char* CHAR_TIME_UUID    = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
const char* CHAR_CONTROL_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a9";
const char* CHAR_DATA_UUID    = "beb5483e-36e1-4688-b7f5-ea07361b26aa";

const char* DEVICE_NAME = "IMU-Recorder";

// ==================== LED ====================
#define LED_PIN LED_BUILTIN
#define LED_ON  LOW    // Active LOW
#define LED_OFF HIGH

// ==================== IMU ====================
LSM6DS3 imu(I2C_MODE, 0x6A);

// ==================== Timing ====================
const unsigned long SAMPLE_INTERVAL_US = 5000UL;   // 200 Hz sample
const unsigned long SEND_INTERVAL_US   = 20000UL;  // 50 Hz send (every 20 ms)
unsigned long lastSampleUs = 0;
unsigned long lastSendUs   = 0;

// ==================== Sample Buffer ====================
struct Sample {
  uint32_t relMs;
  int16_t  ax, ay, az;
  int16_t  gx, gy, gz;
};

#define BUFFER_SIZE 1024   // 5 seconds @ 200 Hz
Sample sampleBuffer[BUFFER_SIZE];
volatile uint16_t bufferHead = 0;
volatile uint16_t bufferTail = 0;

inline int bufferCount() { return (bufferHead - bufferTail + BUFFER_SIZE) % BUFFER_SIZE; }
inline bool bufferEmpty() { return bufferHead == bufferTail; }
inline bool bufferFull()  { return ((bufferHead + 1) % BUFFER_SIZE) == bufferTail; }

// ==================== State ====================
volatile bool     recording    = false;
volatile uint32_t baseEpochMs  = 0;
volatile uint32_t baseMillis   = 0;
uint8_t seqNum = 0;

// ==================== BLE Objects ====================
BLEService imuService(SERVICE_UUID);
BLECharacteristic timeChar(CHAR_TIME_UUID, BLEWrite, 4);
BLECharacteristic controlChar(CHAR_CONTROL_UUID, BLEWrite, 1);
// Size 200 encourages the browser to negotiate a larger MTU
BLECharacteristic dataChar(CHAR_DATA_UUID, BLENotify, 200);

// ==================== Forward Declarations ====================
void onTimeWritten(BLEDevice central, BLECharacteristic characteristic);
void onControlWritten(BLEDevice central, BLECharacteristic characteristic);
void sampleIMU();
void sendSamples();

// ==================== Setup ====================
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_OFF);

  Serial.begin(115200);
  for (int i = 0; i < 10 && !Serial; i++) delay(100);

  Serial.println("=== IMU BLE Recorder (Optimized) ===");

  Wire.begin();
  Wire.setClock(400000);

  imu.settings.accelSampleRate = 416;
  imu.settings.gyroSampleRate  = 416;
  imu.settings.accelRange      = 2;
  imu.settings.gyroRange       = 245;

  int imuTries = 0;
  while (imu.begin() != 0 && imuTries < 5) {
    Serial.println("IMU init failed, retrying...");
    digitalWrite(LED_PIN, LED_ON); delay(100); digitalWrite(LED_PIN, LED_OFF);
    delay(500);
    imuTries++;
  }

  if (imuTries >= 5) {
    Serial.println("IMU init failed permanently!");
    while (1) { digitalWrite(LED_PIN, LED_ON); delay(100); digitalWrite(LED_PIN, LED_OFF); delay(100); }
  }

  imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x60);
  imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, 0x60);

  Serial.println("IMU ready");

  if (!BLE.begin()) {
    Serial.println("BLE init failed!");
    while (1) { digitalWrite(LED_PIN, LED_ON); delay(50); digitalWrite(LED_PIN, LED_OFF); delay(50); }
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

  Serial.println("Advertising as 'IMU-Recorder'");
  digitalWrite(LED_PIN, LED_ON);
}

// ==================== Main Loop ====================
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

      // Sample IMU at 200 Hz
      if ((long)(nowUs - lastSampleUs) >= (long)SAMPLE_INTERVAL_US) {
        lastSampleUs += SAMPLE_INTERVAL_US;
        if (recording) sampleIMU();
      }

      // Send batched packet at 50 Hz (every 20 ms)
      if (recording && ((long)(nowUs - lastSendUs) >= (long)SEND_INTERVAL_US)) {
        lastSendUs += SEND_INTERVAL_US;
        sendSamples();
      }

      // LED feedback
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

// ==================== IMU Sampling ====================
void sampleIMU() {
  if (bufferFull()) {
    bufferTail = (bufferTail + 1) % BUFFER_SIZE; // drop oldest
  }

  Sample s;
  s.relMs = (uint32_t)(millis() - baseMillis);

  s.ax = imu.readRawAccelX();
  s.ay = imu.readRawAccelY();
  s.az = imu.readRawAccelZ();
  s.gx = imu.readRawGyroX();
  s.gy = imu.readRawGyroY();
  s.gz = imu.readRawGyroZ();

  uint16_t idx = bufferHead;
  sampleBuffer[idx] = s;
  bufferHead = (idx + 1) % BUFFER_SIZE;
}

// ==================== BLE Transmission ====================
void sendSamples() {
  const int MAX_PER_PACKET = 4;  // 4 samples = 66 bytes payload
  int available = bufferCount();
  if (available == 0) return;

  int count = available < MAX_PER_PACKET ? available : MAX_PER_PACKET;
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
    // Only consume data on successful transmit
    seqNum = (seqNum + 1) & 0xFF;
    bufferTail = (bufferTail + count) % BUFFER_SIZE;
  } else {
    // BLE TX queue full — retry next 20 ms window
    Serial.println("TX fail");
  }
}

// ==================== BLE Event Handlers ====================
void onTimeWritten(BLEDevice central, BLECharacteristic characteristic) {
  if (characteristic.valueLength() == 4) {
    const uint8_t* v = characteristic.value();
    uint32_t epochSec = ((uint32_t)v[0]) |
                        ((uint32_t)v[1] << 8) |
                        ((uint32_t)v[2] << 16) |
                        ((uint32_t)v[3] << 24);

    baseEpochMs = (uint32_t)epochSec * 1000UL;
    baseMillis  = millis();

    Serial.print("Time synced. Epoch: ");
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
      lastSendUs   = micros();
      Serial.println(">>> Recording STARTED");

    } else if (cmd == 0x00) {
      recording = false;
      Serial.println(">>> Recording STOPPED");
      Serial.print("    Buffered samples: ");
      Serial.println(bufferCount());
    }
  }
}
