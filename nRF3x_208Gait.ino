/* ============================================================================
 * GaitIMU v13 — 208 Hz FIFO BLE IMU streamer + sample-index loss detection
 * Board: Seeed XIAO nRF52840 Sense · Libraries: ArduinoBLE + LSM6DS3 (Seeed)
 *
 * Preserved from v11b (known working baseline):
 *   - IMU on Wire1 via Seeed library bring-up + requestFrom(int,int) pattern
 *   - FIFO acquisition at 208 Hz; 4-sample binary batched notifications (~52/s)
 *   - 1- and 2-byte CMD writes, WHO-verified boot diagnostics
 *   - D2 high (IMU power enable), 208 Hz ODR, ±8 g / ±1000 dps
 *
 * Features retained from v11b:
 *   - Standard BLE Battery Service (0x180F / 0x2A19), notify every 30 s
 *   - VBAT pin auto-detected at boot (stable mid-range ADC candidate)
 *   - LiPo voltage → percentage lookup with calibration constant
 *   - Boot pin scan prints all ADC candidates for manual verification
 *
 * BLE protocol v2 (matches index.html):
 *   Service 9f000001-...-1b2c ; ID(read) ; CMD(write: 01 start, 02 stop,
 *   03 [n] samples/packet) ; TIME(write+notify) ; DATA(notify)
 * DATA v2: [N][first sample index u24][first relative time u32][N×12B samples]
 *   single-sample packet is 20 bytes (fits default BLE ATT notification payload)
 *   + Battery Service 180F (read+notify)
 *
 * Scaling: ±8 g → g = raw × 0.000244 ; ±1000 dps → dps = raw × 0.035
 *
 * Battery notes:
 *   - The XIAO has no fuel gauge — level is estimated from voltage.
 *   - LiPo discharge is flat (3.7–3.9 V for most of the cycle), so the
 *     percentage is approximate (±10–15 %). Treat < 20 % as "recharge".
 *   - When USB is attached, the charger holds the cell at 4.2 V → reads 100 %.
 *   - CALIBRATION: after boot, compare the printed volts with a multimeter
 *     across the battery pads; adjust VBAT_SCALE until they match.
 * ==========================================================================*/

#include <ArduinoBLE.h>
#include <Wire.h>
#include <LSM6DS3.h>
#include "mbed.h"
#include <string.h>

/* ───────── USER CONFIG — set per module before flashing ───────── */
#define MODULE_ID "RightLeg"        /* "RightLeg" | "LeftLeg" | "Torso" */
/* ──────────────────────────────────────────────────────────────── */

#define DEVICE_NAME "GaitIMU-" MODULE_ID

/* Battery monitoring */
#define VBAT_PIN      31            /* P0.31 — used if auto-detect fails  */
#define VBAT_SCALE    7.2           /* calibrate: volts = normalized × SCALE */

/* Sampling / packetisation */
#define SAMPLE_RATE_HZ         208
#define SAMPLE_PERIOD_US        (1000000UL / SAMPLE_RATE_HZ)
#define MAX_SAMPLES_PER_PACKET 10
#define SAMPLES_PER_PACKET     4
#define PACKET_HEADER_LEN      8   /* N, first sample index u24, first relative time u32 */
#define MAX_PACKET_LEN         (PACKET_HEADER_LEN + 12 * MAX_SAMPLES_PER_PACKET)

/* BLE UUIDs — must match index.html */
#define GAIT_SERVICE_UUID "9f000001-8c5a-4e1f-9a7b-3d6e5f0a1b2c"
#define CMD_CHAR_UUID     "9f000002-8c5a-4e1f-9a7b-3d6e5f0a1b2c"
#define TIME_CHAR_UUID    "9f000003-8c5a-4e1f-9a7b-3d6e5f0a1b2c"
#define DATA_CHAR_UUID    "9f000004-8c5a-4e1f-9a7b-3d6e5f0a1b2c"
#define ID_CHAR_UUID      "9f000005-8c5a-4e1f-9a7b-3d6e5f0a1b2c"

#define CMD_START   0x01
#define CMD_STOP    0x02
#define CMD_SET_SPP 0x03

/* LSM6DS3 registers */
#define IMU_ADDR      0x6A
#define REG_WHO_AM_I  0x0F
#define REG_CTRL1_XL  0x10
#define REG_CTRL2_G   0x11
#define REG_CTRL3_C   0x12
#define REG_STATUS    0x1E
#define REG_FIFO_CTRL1 0x06
#define REG_FIFO_CTRL2 0x07
#define REG_FIFO_CTRL3 0x08
#define REG_FIFO_CTRL4 0x09
#define REG_FIFO_CTRL5 0x0A
#define REG_FIFO_STATUS1 0x3A
#define REG_FIFO_STATUS2 0x3B
#define REG_FIFO_STATUS3 0x3C
#define REG_FIFO_STATUS4 0x3D
#define REG_FIFO_DATA_OUT_L 0x3E
#define REG_OUTX_L_G  0x22
#define VAL_CTRL1_XL  0x5C          /* 208 Hz ODR, ±8 g      */
#define VAL_CTRL2_G   0x58          /* 208 Hz ODR, ±1000 dps */

/* ── BLE objects ── */
BLEService        gaitService(GAIT_SERVICE_UUID);
BLECharacteristic cmdChar (CMD_CHAR_UUID,  BLEWrite, 8);
BLECharacteristic timeChar(TIME_CHAR_UUID, BLEWrite | BLENotify, 8);
BLECharacteristic dataChar(DATA_CHAR_UUID, BLENotify | BLERead, MAX_PACKET_LEN);
BLECharacteristic idChar  (ID_CHAR_UUID,   BLERead, 20);

/* Standard Battery Service */
BLEService            batteryService("180F");
BLEByteCharacteristic battChar("2A19", BLERead | BLENotify);

/* ── the IMU, exactly as in Seeed's wiki ── */
LSM6DS3 imuLib(I2C_MODE, IMU_ADDR);

/* ── I2C bus selection (from boot diagnostic) ── */
static TwoWire* bus       = &Wire;
static bool     useIntArg  = true;

/* ── battery state ── */
static mbed::AnalogIn* vbat = nullptr;

/* ── streaming state ── */
bool     streaming        = false;
uint32_t syncMicros       = 0;
uint32_t timingGaps        = 0;
uint32_t txErrors          = 0;
uint8_t  samplesPerPacket = SAMPLES_PER_PACKET;
uint8_t  packetFill       = 0;
uint32_t packetSample0    = 0;
uint32_t nextSampleIndex  = 0;
uint32_t packetT0         = 0;
uint8_t  packetBuf[MAX_PACKET_LEN];
uint32_t statLast = 0, pktCount = 0, smpCount = 0, i2cErrors = 0;
uint32_t fifoOverruns = 0, estimatedLostSamples = 0;
uint16_t fifoWordsBuf[6];
uint8_t fifoWordFill = 0, fifoPhase = 0;
bool fifoPhaseKnown = false;
uint32_t lastFifoDrainUs = 0;
uint32_t batLast = 0;
bool     imuOk = false;

/* ══ battery helpers ══ */
static float readVBat() {
  if (!vbat) return 0;
  float s = 0;
  for (int i = 0; i < 4; i++) s += vbat->read();
  return (s / 4.0f) * VBAT_SCALE;
}

static uint8_t lipoPct(float v) {
  /* Piecewise-linear LiPo voltage → percentage (approximate) */
  static const float P[][2] = {
    {3.30,  0}, {3.50, 10}, {3.60, 20}, {3.70, 40},
    {3.80, 60}, {3.90, 75}, {4.00, 85}, {4.10, 93}, {4.20, 100}
  };
  if (v <= P[0][0]) return 0;
  for (int i = 1; i < 9; i++) {
    if (v <= P[i][0]) {
      float f = (v - P[i-1][0]) / (P[i][0] - P[i-1][0]);
      return (uint8_t)(P[i-1][1] + f * (P[i][1] - P[i-1][1]));
    }
  }
  return 100;
}

/* Boot-time ADC pin scan: prints all SAADC-capable pins and auto-detects
   the VBAT pin (stable mid-range reading with battery attached).        */
static PinName vbatScan() {
  const int pins[8] = {2, 3, 4, 5, 28, 29, 30, 31};
  PinName detected = NC;

  Serial.println("[BAT] ADC pin scan (4 reads each, normalized 0..1):");
  Serial.println("     (VBAT = stable mid-range reading; floats wander; GND = 0)");
  for (int i = 0; i < 8; i++) {
    mbed::AnalogIn a((PinName)pins[i]);
    float vals[4];
    float sum = 0;
    for (int k = 0; k < 4; k++) {
      vals[k] = a.read();
      sum += vals[k];
    }
    float mean = sum / 4.0f;
    float maxDev = 0;
    for (int k = 0; k < 4; k++) {
      float d = vals[k] - mean;
      if (d < 0) d = -d;
      if (d > maxDev) maxDev = d;
    }

    Serial.print("  P0.");
    if (pins[i] < 10) Serial.print('0');
    Serial.print(pins[i]);
    Serial.print(":  ");
    for (int k = 0; k < 4; k++) { Serial.print(vals[k], 3); Serial.print(' '); }
    Serial.print(" (mean="); Serial.print(mean, 3);
    Serial.print(", dev=");  Serial.print(maxDev, 3); Serial.print(')');

    /* VBAT signature: stable (low deviation) AND mid-range value */
    if (maxDev < 0.02 && mean > 0.25 && mean < 0.75) {
      Serial.println("  ← VBAT candidate");
      if (detected == NC) detected = (PinName)pins[i];
    } else {
      Serial.println();
    }
  }
  return detected;
}

/* ══ library-style I2C (transactions fired, return codes ignored,
      data counted via available() — the pattern proven in v10/v11) ══ */
static int reqSel(int qty) {
  if (useIntArg) return bus->requestFrom((int)IMU_ADDR, (int)qty);
  return bus->requestFrom((uint8_t)IMU_ADDR, (uint8_t)qty);
}
static void writeReg(uint8_t reg, uint8_t val) {
  bus->beginTransmission(IMU_ADDR);
  bus->write(reg);
  bus->write(val);
  bus->endTransmission();                 /* return ignored (library style) */
}
static uint8_t readReg(uint8_t reg) {
  bus->beginTransmission(IMU_ADDR);
  bus->write(reg);
  bus->endTransmission();                 /* return ignored */
  reqSel(1);                              /* return ignored */
  uint8_t v = 0xFF;
  if (bus->available() >= 1) v = (uint8_t)bus->read();
  while (bus->available()) bus->read();
  return v;
}
static uint16_t fifoWordCount(uint8_t* status2Out) {
  const uint8_t lo = readReg(REG_FIFO_STATUS1);
  const uint8_t hi = readReg(REG_FIFO_STATUS2);
  *status2Out = hi;
  if (hi == 0xFF) return 0; /* reserved bit(s) set: treat as I2C read failure */
  return (uint16_t)lo | ((uint16_t)(hi & 0x07) << 8);
}
static bool fifoPattern(uint16_t* patternOut) {
  const uint8_t lo = readReg(REG_FIFO_STATUS3);
  const uint8_t hi = readReg(REG_FIFO_STATUS4);
  if (lo == 0xFF || hi == 0xFF) return false;
  *patternOut = (uint16_t)lo | ((uint16_t)(hi & 0x03) << 8);
  return true;
}
/* Read sequential 16-bit FIFO words. LSM6DS3 FIFO_DATA_OUT supports a
   multi-byte read; IF_INC is enabled. Returns actual words obtained. */
static uint8_t readFifoWords(uint16_t* dst, uint8_t maxWords) {
  const uint8_t bytesWanted = (uint8_t)(maxWords * 2);
  bus->beginTransmission(IMU_ADDR);
  bus->write(REG_FIFO_DATA_OUT_L);
  if (bus->endTransmission() != 0) return 0;
  reqSel(bytesWanted);
  int avail = bus->available();
  if (avail > bytesWanted) avail = bytesWanted;
  uint8_t words = (uint8_t)(avail / 2);
  for (uint8_t i = 0; i < words; i++) {
    const uint8_t lo = (uint8_t)bus->read();
    const uint8_t hi = (uint8_t)bus->read();
    dst[i] = (uint16_t)lo | ((uint16_t)hi << 8);
  }
  while (bus->available()) bus->read();
  return words;
}
static void fifoConfigure(bool enable) {
  /* Bypass before changing FIFO settings; paired G and XL, no decimation.
     FIFO ODR=208 Hz (0101), continuous mode (110) => FIFO_CTRL5=0x2E. */
  writeReg(REG_FIFO_CTRL5, 0x00);
  writeReg(REG_FIFO_CTRL1, 12);  /* watermark: two paired samples (12 words) */
  writeReg(REG_FIFO_CTRL2, 0x00);
  writeReg(REG_FIFO_CTRL3, 0x09); /* gyro no-decimation + XL no-decimation */
  writeReg(REG_FIFO_CTRL4, 0x00);
  if (enable) writeReg(REG_FIFO_CTRL5, 0x2E);
}

/* ══ boot diagnostic (from v10/v11) ══ */
static uint8_t diagWho(TwoWire& b, bool intArg, const char* tag) {
  b.beginTransmission(IMU_ADDR);
  b.write(REG_WHO_AM_I);
  int et = b.endTransmission();
  int ret;
  if (intArg) ret = b.requestFrom((int)IMU_ADDR, (int)1);
  else        ret = b.requestFrom((uint8_t)IMU_ADDR, (uint8_t)1);
  int av = b.available();
  uint8_t who = (av >= 1) ? (uint8_t)b.read() : 0xFF;
  while (b.available()) b.read();
  Serial.print("[IMU] "); Serial.print(tag);
  Serial.print(" endTransmission()="); Serial.print(et);
  Serial.print("  requestFrom ret="); Serial.print(ret);
  Serial.print("  avail=");           Serial.print(av);
  Serial.print("  WHO=0x");           Serial.println(who, HEX);
  return who;
}

static bool librarySelfTest() {
  Serial.println("[IMU] library self-test (tilt the board):");
  float magMax = 0;
  for (int i = 0; i < 4; i++) {
    float ax = imuLib.readFloatAccelX(), ay = imuLib.readFloatAccelY(), az = imuLib.readFloatAccelZ();
    float gx = imuLib.readFloatGyroX(),  gy = imuLib.readFloatGyroY(),  gz = imuLib.readFloatGyroZ();
    Serial.print("      a[g] ");
    Serial.print(ax, 3); Serial.print(' '); Serial.print(ay, 3); Serial.print(' '); Serial.print(az, 3);
    Serial.print("   w[dps] ");
    Serial.print(gx, 1); Serial.print(' '); Serial.print(gy, 1); Serial.print(' '); Serial.println(gz, 1);
    float m = sqrtf(ax * ax + ay * ay + az * az);
    if (m > magMax) magMax = m;
    delay(150);
  }
  bool alive = (magMax > 0.10f && magMax < 10.0f);
  Serial.println(alive ? "      -> ALIVE (gravity visible)" : "      -> DEAD");
  return alive;
}

/* ══ IMU init (unchanged from v11) ══ */
static bool imuInit() {
  pinMode(2, OUTPUT); digitalWrite(2, HIGH);      /* D2 high, as documented */
  delay(100);
  Wire.begin();
  int st = imuLib.begin();
  Serial.print("[IMU] library begin() -> "); Serial.println(st);

  /* which bus / which requestFrom form actually talks? */
  uint8_t w = diagWho(Wire, true, "Wire  (int,int)  :");
  bool ok = (w == 0x6A);
  if (!ok) {
    uint8_t w2 = diagWho(Wire, false, "Wire  (byte,byte):");
    if (w2 == 0x6A) { ok = true; useIntArg = false; }
  }
#if defined(WIRE_HOWMANY) && (WIRE_HOWMANY > 1)
  if (!ok) {
    Wire1.begin();
    uint8_t w3 = diagWho(Wire1, true, "Wire1 (int,int)  :");
    if (w3 == 0x6A) { ok = true; bus = &Wire1; useIntArg = true; }
    else {
      uint8_t w4 = diagWho(Wire1, false, "Wire1 (byte,byte):");
      if (w4 == 0x6A) { ok = true; bus = &Wire1; useIntArg = false; }
    }
  }
#endif
  if (!ok) return false;

  Serial.print("[IMU] using ");
  Serial.print(bus == &Wire ? "Wire" : "Wire1");
  Serial.println(useIntArg ? " + requestFrom(int,int)" : " + requestFrom(uint8_t,uint8_t)");

  /* tune: 208 Hz ODR, ±8 g, ±1000 dps */
  writeReg(REG_CTRL1_XL, VAL_CTRL1_XL);
  writeReg(REG_CTRL2_G,  VAL_CTRL2_G);
  uint8_t c3 = readReg(REG_CTRL3_C);
  if (c3 == 0xFF) c3 = 0x04;
  writeReg(REG_CTRL3_C, (uint8_t)(c3 | 0x44));    /* BDU + IF_INC (BDU bit 6, IF_INC bit 2) */
  fifoConfigure(false);
  delay(50);

  uint8_t cb = readReg(REG_CTRL1_XL);
  uint8_t cg = readReg(REG_CTRL2_G);
  Serial.print("[IMU] CTRL1_XL read-back: 0x"); Serial.println(cb, HEX);
  Serial.print("[IMU] CTRL2_G  read-back: 0x"); Serial.println(cg, HEX);
  if (cb != VAL_CTRL1_XL || cg != VAL_CTRL2_G)
    Serial.println("[IMU] WARNING: ODR/range register read-back differs from requested settings.");
  return true;
}

/* ══ packet build / send ══ */
static void flushPacket() {
  if (packetFill == 0) return;
  packetBuf[0] = packetFill;
  packetBuf[1] = (uint8_t)(packetSample0 & 0xFF);
  packetBuf[2] = (uint8_t)((packetSample0 >> 8) & 0xFF);
  packetBuf[3] = (uint8_t)((packetSample0 >> 16) & 0xFF);
  memcpy(&packetBuf[4], &packetT0, 4);
  const int len = PACKET_HEADER_LEN + 12 * packetFill;
  if (!dataChar.writeValue(packetBuf, len)) txErrors++;
  packetFill = 0;
  pktCount++;
}
static void addSample(uint32_t sampleRelUs, int16_t acc[3], int16_t gyr[3]) {
  if (packetFill == 0) {
    packetSample0 = nextSampleIndex;
    packetT0 = sampleRelUs;
  }
  uint8_t* p = &packetBuf[PACKET_HEADER_LEN + 12 * packetFill];
  memcpy(p, acc, 6);
  memcpy(p + 6, gyr, 6);
  packetFill++;
  smpCount++;
  nextSampleIndex = (nextSampleIndex + 1) & 0xFFFFFFUL; /* 24-bit modular sample index */
  if (packetFill >= samplesPerPacket) flushPacket();
}
static void consumeFifoWord(uint16_t word, uint32_t sampleRelUs) {
  /* FIFO pattern for equal ODR/no decimation: Gx,Gy,Gz,Ax,Ay,Az.
     Align at Gx; never combine words from an uncertain/overrun frame. */
  if (!fifoPhaseKnown) { fifoPhase = 0; fifoPhaseKnown = true; }
  if (fifoPhase == 0) { fifoWordsBuf[0] = word; fifoWordFill = 1; }
  else if (fifoWordFill == fifoPhase) { fifoWordsBuf[fifoPhase] = word; fifoWordFill++; }
  else { fifoWordFill = 0; }
  fifoPhase = (uint8_t)((fifoPhase + 1) % 6);
  if (fifoPhase == 0 && fifoWordFill == 6) {
    int16_t gyr[3], acc[3];
    for (int i = 0; i < 3; i++) gyr[i] = (int16_t)fifoWordsBuf[i];
    for (int i = 0; i < 3; i++) acc[i] = (int16_t)fifoWordsBuf[3 + i];
    addSample(sampleRelUs, acc, gyr);
    fifoWordFill = 0;
  }
}
static void drainFifo() {
  uint8_t st2 = 0xFF;
  uint16_t words = fifoWordCount(&st2);
  const uint32_t nowUs = micros();
  if (st2 == 0xFF) { i2cErrors++; return; }
  const bool overrun = (st2 & 0x40) != 0;
  uint16_t patternRaw = 0;
  if (!fifoPattern(&patternRaw)) { i2cErrors++; return; }
  const uint8_t pattern = (uint8_t)(patternRaw % 6);

  if (!fifoPhaseKnown) {
    fifoPhase = pattern;
    fifoPhaseKnown = true;
    if (fifoPhase != 0) fifoWordFill = 0; /* discard to next complete Gx */
  }
  if (overrun) {
    fifoOverruns++;
    timingGaps++;
    flushPacket();                 /* keep sample-index gaps packet-visible */
    const uint8_t partialBeforeOverrun = fifoWordFill;
    fifoWordFill = 0;
    fifoPhase = pattern;
    fifoPhaseKnown = true; /* known phase; skip forward until next Gx if needed */
    const uint32_t elapsed = lastFifoDrainUs ? (uint32_t)(nowUs - lastFifoDrainUs) : 0;
    const uint32_t expected = (elapsed * SAMPLE_RATE_HZ + 500000UL) / 1000000UL;
    const uint32_t availablePairs = (words + partialBeforeOverrun) / 6;
    uint32_t missing = (expected > availablePairs) ? expected - availablePairs : 1;
    estimatedLostSamples += missing;
    nextSampleIndex = (nextSampleIndex + missing) & 0xFFFFFFUL;
    Serial.print("[FIFO] OVERRUN; estimated lost samples="); Serial.println(missing);
  }

  bool drainedAny = false;
  uint32_t sampleOrdinal = 0;
  const uint32_t pairsInSnapshot = (uint32_t)((words + fifoWordFill) / 6);
  const uint32_t approximateBatchStart = nowUs - ((pairsInSnapshot > 0 ? pairsInSnapshot - 1 : 0) * SAMPLE_PERIOD_US);
  uint16_t temp[48];
  while (words > 0) {
    uint8_t ask = (words > 48) ? 48 : (uint8_t)words;
    uint8_t got = readFifoWords(temp, ask);
    if (got == 0) {
      i2cErrors++;
      flushPacket();
      fifoWordFill = 0; fifoPhaseKnown = false;
      break;
    }
    drainedAny = true;
    if (got < ask) i2cErrors++;
    for (uint8_t i = 0; i < got; i++) {
      const uint32_t sampleUs = approximateBatchStart + sampleOrdinal * SAMPLE_PERIOD_US;
      const bool completesPair = (fifoPhase == 5 && fifoWordFill == 5);
      consumeFifoWord(temp[i], sampleUs - syncMicros);
      if (completesPair) sampleOrdinal++;
    }
    words = (words > got) ? (uint16_t)(words - got) : 0;
    if (got < ask) break;
  }
  if (drainedAny) lastFifoDrainUs = nowUs;
}

/* ══ setup / loop ══ */
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2500) {}
  Serial.println();
  Serial.println("GaitIMU v13 FIFO node: " DEVICE_NAME);

#ifdef LED_BUILTIN
  pinMode(LED_BUILTIN, OUTPUT);
#endif

  /* ── IMU bring-up (proven path) ── */
  imuOk = imuInit();
  if (!imuOk) Serial.println("[IMU] *** init failed — press reset and report ***");

  /* ── Battery: scan pins, detect VBAT, create AnalogIn ── */
  PinName vbatPin = vbatScan();
  if (vbatPin == NC) {
    vbatPin = (PinName)VBAT_PIN;
    Serial.print("[BAT] auto-detect failed — using default P0.");
    Serial.println(VBAT_PIN);
    Serial.println("     (attach battery and reset, or set VBAT_PIN manually)");
  }
  vbat = new mbed::AnalogIn(vbatPin);
  float bv = readVBat();
  Serial.print("[BAT] Battery: ");
  Serial.print(bv, 2);
  Serial.print(" V  (");
  Serial.print(lipoPct(bv));
  Serial.println(" %)");
  Serial.println("     Calibrate: adjust VBAT_SCALE until V matches a multimeter.");
  Serial.println("     Note: USB attached = charger holds 4.2 V → reads 100 %.");

  /* ── BLE setup ── */
  if (!BLE.begin()) { Serial.println("BLE.begin() failed — halting"); while (1) delay(1000); }
  BLE.setDeviceName(DEVICE_NAME);
  BLE.setLocalName(DEVICE_NAME);
  BLE.setConnectionInterval(8, 30);

  BLE.setAdvertisedService(gaitService);
  gaitService.addCharacteristic(idChar);
  gaitService.addCharacteristic(cmdChar);
  gaitService.addCharacteristic(timeChar);
  gaitService.addCharacteristic(dataChar);
  BLE.addService(gaitService);

  batteryService.addCharacteristic(battChar);
  BLE.addService(batteryService);
  battChar.writeValue((uint8_t)lipoPct(bv));      /* initial level */

  idChar.writeValue((const uint8_t*)MODULE_ID, strlen(MODULE_ID));
  BLE.advertise();
  Serial.println("Advertising as " DEVICE_NAME " — waiting for a connection…");
}

void loop() {
  BLE.poll();

  /* central tracking */
  BLEDevice central = BLE.central();
  static bool wasConnected = false;
  bool isConnected = (central && central.connected());
  if (wasConnected && !isConnected) {
    streaming  = false;
    packetFill = 0;
    Serial.println("[BLE] central disconnected — advertising again");
    BLE.advertise();
  }
  wasConnected = isConnected;

  /* time sync: browser writes epoch, we latch and ACK */
  if (timeChar.written()) {
    uint8_t buf[6] = {0,0,0,0,0,0};
    timeChar.readValue(buf, sizeof(buf));
    uint32_t sec; uint16_t ms10;
    memcpy(&sec, &buf[0], 4); memcpy(&ms10, &buf[4], 2);
    syncMicros = micros();
    uint32_t rel = micros() - syncMicros;
    uint8_t  ack[4];
    memcpy(ack, &rel, 4);
    timeChar.writeValue(ack, 4);
    Serial.print("[TIME] epoch "); Serial.print(sec);
    Serial.print('.'); Serial.println(ms10);
  }

  /* commands: 1- or 2-byte writes both accepted */
  if (cmdChar.written()) {
    uint8_t b[8] = {0,0,0,0,0,0,0,0};
    cmdChar.readValue(b, sizeof(b));
    switch (b[0]) {
      case CMD_START:
        if (!streaming) {
          packetFill = 0;
          nextSampleIndex = 0; fifoWordFill = 0; fifoPhaseKnown = false;
          lastFifoDrainUs = micros();
          timingGaps = txErrors = i2cErrors = pktCount = smpCount = 0;
          fifoOverruns = estimatedLostSamples = 0;
          statLast = millis();
          fifoConfigure(true);
          streaming = true;
          Serial.println("[CMD] streaming started");
        }
        break;
      case CMD_STOP:
        if (streaming) {
          streaming = false;
          drainFifo();
          flushPacket();
          fifoConfigure(false);
          Serial.println("[CMD] streaming stopped");
        }
        break;
      case CMD_SET_SPP:
        flushPacket();
        if (b[1] >= 1 && b[1] <= MAX_SAMPLES_PER_PACKET) samplesPerPacket = b[1];
        else                                             samplesPerPacket = 1;
        Serial.print("[CMD] samples/packet = "); Serial.println(samplesPerPacket);
        break;
    }
  }

  /* battery level update every 30 s (notify to subscribed centrals) */
  if (vbat && battChar && millis() - batLast > 30000) {
    batLast = millis();
    battChar.writeValue((uint8_t)lipoPct(readVBat()));
  }

  if (!streaming) { delay(2); return; }

  /* Drain sensor FIFO; sensor samples continue at 208 Hz while BLE is serviced. */
  drainFifo();

#ifdef LED_BUILTIN
  static uint32_t lastBlink = 0;
  if (millis() - lastBlink > 500) {
    lastBlink = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
#endif

  if (millis() - statLast >= 1000) {
    statLast = millis();
    if (Serial) {
      Serial.print("[STAT] samples="); Serial.print(smpCount);
      Serial.print(" packets=");       Serial.print(pktCount);
      Serial.print(" i2cErrors=");     Serial.print(i2cErrors);
      Serial.print(" timingGaps=");    Serial.print(timingGaps);
      Serial.print(" txErrors=");      Serial.print(txErrors);
      Serial.print(" fifoOverruns="); Serial.print(fifoOverruns);
      Serial.print(" estimatedLostSamples="); Serial.println(estimatedLostSamples);
    }
  }
  /* No fixed 1 ms delay: keep polling status between BLE service calls. */
}
