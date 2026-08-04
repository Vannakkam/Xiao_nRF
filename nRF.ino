/*
 * IMU_BLE_Recorder_Final.ino
 * 
 * Xiao nRF52840 Sense — BLE IMU Streamer
 * Safe I2C, threshold send, 4-sample batching
 */

#include <ArduinoBLE.h>
#include <LSM6DS3.h>
#include <Wire.h>

const char* DEVICE_NAME = "IMU-Recorder";  // Change to "IMU-Right", "IMU-Left", "IMU-Torso" for multi-sensor

const char* SERVICE_UUID      = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
const char* CHAR_TIME_UUID    = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
const char* CHAR_CONTROL_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a9";
const char* CHAR_DATA_UUID    = "beb5483e-36e1-4688-b7f5-ea07361b26aa";

#define LED_PIN LED_BUILTIN
#define LED_ON  LOW
#define LED_OFF HIGH

LSM6DS3 imu(I2C_MODE, 0x6A);

const unsigned long SAMPLE_INTERVAL_US = 5000UL;
unsigned long lastSampleUs = 0;

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

volatile bool     recording    = false;
volatile uint32_t baseEpochMs  = 0;
volatile uint32_t baseMillis   = 0;
uint8_t seqNum = 0;

BLEService imuService(SERVICE_UUID);
BLECharacteristic timeChar(CHAR_TIME_UUID, BLEWrite, 4);
BLECharacteristic controlChar(CHAR_CONTROL_UUID, BLEWrite, 1);
BLECharacteristic dataChar(CHAR_DATA_UUID, BLENotify, 200);

void onTimeWritten(BLEDevice central, BLECharacteristic characteristic);
void onControlWritten(BLEDevice central, BLECharacteristic characteristic);

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_OFF);

  Serial.begin(115200);
  for (int i = 0; i < 10 && !Serial; i++) delay(100);

  Serial.println("\n=== IMU BLE Recorder ===");
  Serial.println(DEVICE_NAME);

  Wire.begin();
  Wire.setClock(400000);

  imu.settings.accelSampleRate = 416;
  imu.settings.gyroSampleRate  = 416;
  imu.settings.accelRange      = 2;
  imu.settings.gyroRange       = 245;

  int imuTries = 0;
  while (imu.begin() != 0 && imuTries < 5) {
    Serial.println("IMU retry...");
    digitalWrite(LED_PIN, LED_ON); delay(100);
    digitalWrite(LED_PIN, LED_OFF); delay(400);
    imuTries++;
  }
  if (imuTries >= 5) {
    Serial.println("IMU FATAL");
    while (1) {
      digitalWrite(LED_PIN, LED_ON); delay(100);
      digitalWrite(LED_PIN, LED_OFF); delay(100);
    }
  }

  imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x60);
  imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, 0x60);
  Serial.println("IMU OK");

  if (!BLE.begin()) {
    Serial.println("BLE FATAL");
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
  Serial.println("Advertising");
  digitalWrite(LED_PIN, LED_ON);  // Solid ON
}

void loop() {
  BLE.poll();  // CRITICAL: keep stack alive even when not connected

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

      if (nowUs - lastSampleUs >= SAMPLE_INTERVAL_US) {
        lastSampleUs = nowUs;
        if (recording) sampleIMU();
      }

      if (recording && bufferCount() >= 4) {
        sendSamples();
      }

      if (recording) {
        digitalWrite(LED_PIN, ((millis() / 250) % 2) ? LED_ON : LED_OFF);
      }

      BLE.poll();
    }

    Serial.println("Disconnected");
    recording = false;
    BLE.advertise();  // Restart advertising after disconnect
    digitalWrite(LED_PIN, LED_ON);
  }
}

void sampleIMU() {
  if (bufferFull()) {
    bufferTail = (bufferTail + 1) % BUFFER_SIZE;
  }
  Sample s;
  s.relMs = (uint32_t)(millis() - baseMillis);

  // Library raw reads (safe, well-tested)
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

void sendSamples() {
  int available = bufferCount();
  if (available < 4) return;

  int count = available < 4 ? available : 4;
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
}

void onTimeWritten(BLEDevice central, BLECharacteristic characteristic) {
  if (characteristic.valueLength() == 4) {
    const uint8_t* v = characteristic.value();
    uint32_t epochSec = ((uint32_t)v[0]) | ((uint32_t)v[1] << 8) | ((uint32_t)v[2] << 16) | ((uint32_t)v[3] << 24);
    baseEpochMs = (uint32_t)epochSec * 1000UL;
    baseMillis  = millis();
    Serial.print("Time: "); Serial.println(epochSec);
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
      Serial.println("START");
    } else if (cmd == 0x00) {
      recording = false;
      Serial.println("STOP");
    }
  }
}
