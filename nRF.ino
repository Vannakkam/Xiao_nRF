#include <ArduinoBLE.h>
#include <Arduino_LSM6DS3.h>

// BLE Service and Characteristic UUIDs
BLEService imuService("19B10000-E8F2-537E-4F6C-D104768a1214");
BLECharacteristic imuDataChar("19B10001-E8F2-537E-4F6C-D104768a1214", BLERead | BLENotify, 20);
BLEUnsignedIntCharacteristic timeSyncChar("19B10002-E8F2-537E-4F6C-D104768a1214", BLEWrite);

typedef struct __attribute__((packed)) {
  uint32_t timestamp_ms;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
} IMUSample;

#define SAMPLES_PER_PACKET 2
typedef struct __attribute__((packed)) {
  uint32_t base_epoch_sec;
  IMUSample samples[SAMPLES_PER_PACKET];
} IMUPacket;

IMUPacket packetBuffer;
uint8_t sampleCount = 0;

unsigned long baseEpochSec = 0;
unsigned long syncLocalMillis = 0;
unsigned long lastSampleMicros = 0;
const unsigned long sampleIntervalMicros = 5000; // 5 ms interval (200 Hz)

bool streamingActive = false;

void setup() {
  Serial.begin(115200);
  
  // REMOVED: while (!Serial); 
  // This allows the board to boot independently on battery or without open serial monitor.
  // Optional delay to let serial port catch if needed during debugging:
  delay(1500); 

  // Initialize LED pin for visual status feedback (Built-in LED on XIAO nRF52840 is active LOW)
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // Turn off initially

  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1) {
      digitalWrite(LED_BUILTIN, LOW); // Flash LED rapidly on error
      delay(100);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
    }
  }

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1) {
      digitalWrite(LED_BUILTIN, LOW); // Flash LED slowly on IMU error
      delay(500);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(500);
    }
  }

  // BLE configuration
  BLE.setLocalName("XIAO-IMU");
  BLE.setAdvertisedService(imuService);

  imuService.addCharacteristic(imuDataChar);
  imuService.addCharacteristic(timeSyncChar);
  BLE.addService(imuService);

  BLE.advertise();
  Serial.println("BLE IMU Peripheral active, advertising...");
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    
    // Solid LED indicates active BLE connection
    digitalWrite(LED_BUILTIN, LOW); 

    while (central.connected()) {
      // Check if PC/Browser wrote new epoch time sync packet
      if (timeSyncChar.written()) {
        baseEpochSec = timeSyncChar.value();
        syncLocalMillis = millis();
        streamingActive = true;
        lastSampleMicros = micros();
        Serial.print("Time Synchronized via BLE Epoch: ");
        Serial.println(baseEpochSec);
      }

      // Sample at 5ms intervals when streaming is active
      if (streamingActive && baseEpochSec > 0) {
        unsigned long currentMicros = micros();
        if (currentMicros - lastSampleMicros >= sampleIntervalMicros) {
          lastSampleMicros += sampleIntervalMicros;

          float ax, ay, az, gx, gy, gz;
          if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
            IMU.readAcceleration(ax, ay, az);
            IMU.readGyroscope(gx, gy, gz);

            uint32_t relMs = millis() - syncLocalMillis;
            packetBuffer.base_epoch_sec = baseEpochSec;
            packetBuffer.samples[sampleCount].timestamp_ms = relMs;
            packetBuffer.samples[sampleCount].ax = (int16_t)(ax * 1000);
            packetBuffer.samples[sampleCount].ay = (int16_t)(ay * 1000);
            packetBuffer.samples[sampleCount].az = (int16_t)(az * 1000);
            packetBuffer.samples[sampleCount].gx = (int16_t)(gx * 10);
            packetBuffer.samples[sampleCount].gy = (int16_t)(gy * 10);
            packetBuffer.samples[sampleCount].gz = (int16_t)(gz * 10);

            sampleCount++;

            if (sampleCount >= SAMPLES_PER_PACKET) {
              imuDataChar.writeValue((uint8_t*)&packetBuffer, sizeof(IMUPacket));
              sampleCount = 0;
            }
          }
        }
      }
    }

    streamingActive = false;
    digitalWrite(LED_BUILTIN, HIGH); // Turn off LED when disconnected, return to advertising
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}
