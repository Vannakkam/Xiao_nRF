/*
 * To be compiled and checked 30?7/26
 * XIAO nRF52840 Sense — Web Bluetooth IMU Streamer
 * - LSM6DS3TR-C IMU sampled at ~200 Hz
 * - Batched BLE notifications (5 samples / packet) to save battery
 * - Browser syncs Unix epoch time; device stamps samples with offsets
 *
 * BLE Service UUID : 19B10000-E8F2-537E-4F6C-D104768A1214
 *   IMU Data (notify) : 19B10001-...
 *   Control   (write) : 19B10002-...   (1=start, 0=stop)
 *   Time Sync (write) : 19B10003-...   (uint32 epoch seconds)
 */

#include <ArduinoBLE.h>
#include <LSM6DS3.h>
#include <Wire.h>

#define BATCH_SIZE      5          // samples per BLE notification
#define SAMPLE_PERIOD_MS 5         // 200 Hz
#define BLE_DEVICE_NAME "XIAO-IMU"

// --- BLE Service & Characteristics ---
BLEService imuService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLECharacteristic imuDataChar("19B10001-E8F2-537E-4F6C-D104768A1214",
                              BLERead | BLENotify, 74);
BLEByteCharacteristic controlChar("19B10002-E8F2-537E-4F6C-D104768A1214",
                                  BLEWrite);
BLEUnsignedLongCharacteristic timeChar("19B10003-E8F2-537E-4F6C-D104768A1214",
                                       BLEWrite);

// --- IMU ---
LSM6DS3 imu(I2C_MODE, 0x6A);   // XIAO Sense default address

// --- State ---
uint32_t baseEpochSec = 0;       // synced from browser
uint32_t recStartMs   = 0;       // millis() when recording began
bool     recording    = false;
uint32_t lastSampleMs = 0;

struct Sample {
  uint16_t offsetMs;   // ms since recStartMs
  int16_t  ax, ay, az; // accel in milli-g
  int16_t  gx, gy, gz; // gyro in 10 mdps (scaled x100)
};
Sample batchBuf[BATCH_SIZE];
uint8_t  batchCount = 0;

// --- Forward decls ---
void onControlWrite(BLEDevice central, BLECharacteristic ch);
void onTimeWrite   (BLEDevice central, BLECharacteristic ch);
void onConnect     (BLEDevice central);
void onDisconnect  (BLEDevice central);
void sendBatch();

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) { /* wait up to 2s */ }

  // IMU init
  if (!imu.begin()) {
    Serial.println("LSM6DS3 init failed");
    while (1) { digitalWrite(LED_BUILTIN, HIGH); delay(200); digitalWrite(LED_BUILTIN, LOW); delay(200); }
  }
  // 208 Hz ODR is the closest standard above 200 Hz
  imu.writeRegister(LSM6DS3_CTRL1_XL, 0x60); // accel 208Hz, ±4g
  imu.writeRegister(LSM6DS3_CTRL2_G,  0x60); // gyro  208Hz, ±2000dps

  // BLE init
  if (!BLE.begin()) {
    Serial.println("BLE init failed");
    while (1);
  }
  BLE.setLocalName(BLE_DEVICE_NAME);
  BLE.setDeviceName(BLE_DEVICE_NAME);
  BLE.setAdvertisedService(imuService);
  imuService.addCharacteristic(imuDataChar);
  imuService.addCharacteristic(controlChar);
  imuService.addCharacteristic(timeChar);
  BLE.addService(imuService);

  controlChar.setEventHandler(BLEWritten, onControlWrite);
  timeChar.setEventHandler   (BLEWritten, onTimeWrite);
  BLE.setEventHandler        (BLEConnected,    onConnect);
  BLE.setEventHandler        (BLEDisconnected, onDisconnect);

  BLE.advertise();
  Serial.println("Advertising as " BLE_DEVICE_NAME);
}

void loop() {
  BLE.poll();

  if (!recording) return;

  uint32_t now = millis();
  if (now - lastSampleMs >= SAMPLE_PERIOD_MS) {
    lastSampleMs = now;

    float ax, ay, az, gx, gy, gz;
    imu.readAllAxes(ax, ay, az, gx, gy, gz);

    Sample &s = batchBuf[batchCount];
    s.offsetMs = (uint16_t)(now - recStartMs);
    s.ax = (int16_t)(ax * 1000.0f);   // g -> mg
    s.ay = (int16_t)(ay * 1000.0f);
    s.az = (int16_t)(az * 1000.0f);
    s.gx = (int16_t)(gx * 100.0f);    // dps -> 10*mdps
    s.gy = (int16_t)(gy * 100.0f);
    s.gz = (int16_t)(gz * 100.0f);
    batchCount++;

    if (batchCount >= BATCH_SIZE) sendBatch();
  }
}

void sendBatch() {
  // Packet layout:
  //   [0..3]  uint32  baseEpochMs  (big-endian)
  //   [4]     uint8   sampleCount
  //   [5..]   per sample: uint16 offsetMs, int16 ax,ay,az,gx,gy,gz (14 bytes)
  uint8_t pkt[5 + BATCH_SIZE * 14];
  uint32_t baseMs = baseEpochSec * 1000UL;
  pkt[0] = (baseMs >> 24) & 0xFF;
  pkt[1] = (baseMs >> 16) & 0xFF;
  pkt[2] = (baseMs >>  8) & 0xFF;
  pkt[3] =  baseMs        & 0xFF;
  pkt[4] = batchCount;

  size_t idx = 5;
  for (uint8_t i = 0; i < batchCount; i++) {
    const Sample &s = batchBuf[i];
    pkt[idx++] = (s.offsetMs >> 8) & 0xFF;
    pkt[idx++] =  s.offsetMs       & 0xFF;
    pkt[idx++] = (s.ax >> 8) & 0xFF; pkt[idx++] = s.ax & 0xFF;
    pkt[idx++] = (s.ay >> 8) & 0xFF; pkt[idx++] = s.ay & 0xFF;
    pkt[idx++] = (s.az >> 8) & 0xFF; pkt[idx++] = s.az & 0xFF;
    pkt[idx++] = (s.gx >> 8) & 0xFF; pkt[idx++] = s.gx & 0xFF;
    pkt[idx++] = (s.gy >> 8) & 0xFF; pkt[idx++] = s.gy & 0xFF;
    pkt[idx++] = (s.gz >> 8) & 0xFF; pkt[idx++] = s.gz & 0xFF;
  }
  imuDataChar.writeValue(pkt, idx);
  batchCount = 0;
}

// --- BLE event handlers ---
void onControlWrite(BLEDevice central, BLECharacteristic ch) {
  uint8_t cmd = controlChar.value();
  if (cmd == 1) {
    recording    = true;
    recStartMs   = millis();
    batchCount   = 0;
    lastSampleMs = millis();
    Serial.println("Recording STARTED");
  } else if (cmd == 0) {
    recording = false;
    Serial.println("Recording STOPPED");
  }
}

void onTimeWrite(BLEDevice central, BLECharacteristic ch) {
  baseEpochSec = timeChar.value();
  Serial.print("Time synced, epoch="); Serial.println(baseEpochSec);
}

void onConnect(BLEDevice central) {
  Serial.print("Connected: "); Serial.println(central.address());
}
void onDisconnect(BLEDevice central) {
  recording = false;
  Serial.println("Disconnected");
  BLE.advertise();
}
