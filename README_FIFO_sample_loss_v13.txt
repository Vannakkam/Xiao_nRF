Gait IMU recorder v13 — FIFO acquisition + sample-loss detection

Files:
- nRF_Gait.ino: firmware for Seeed XIAO nRF52840 Sense / LSM6DS3
- index.html: matching Web Bluetooth dashboard

Flash the same sketch to each of the three modules, changing MODULE_ID to
RightLeg, LeftLeg, or Torso before compiling each one. Use this v13 dashboard
with the v13 firmware; the DATA notification packet format changed and is not
compatible with the older v12 dashboard/firmware pair.

Acquisition:
- Accelerometer and gyroscope remain configured for 208 Hz.
- LSM6DS3 FIFO is configured for continuous 208 Hz acquisition, gyro + accel,
  no decimation. FIFO pattern is Gx,Gy,Gz,Ax,Ay,Az.
- Firmware polls FIFO status and drains FIFO words in multi-word I2C reads.
- FIFO overrun is logged to Serial and estimated missing sample count is
  advanced in the sample index so the browser can show the gap.

Sample loss:
- DATA v2 packet: N (u8), first sample index (u24 LE), first relative time
  in microseconds (u32 LE), then N × 12 bytes (six int16 values per sample).
- Sample index is 24-bit and wraps modulo 2^24. A single-sample packet is
  exactly 20 bytes and fits a default 20-byte ATT notification payload.
- Dashboard counts sample-index gaps as lost samples and records sample_index
  in the CSV. The bad-packet counter remains separate.
- FIFO overrun lost-sample count is an estimate based on elapsed time and FIFO
  occupancy; it is not an exact hardware-provided overrun count.

Diagnostics:
Open Serial Monitor at 115200 baud. Periodic [STAT] output includes samples,
packets, i2cErrors, timingGaps (FIFO overruns), txErrors, fifoOverruns, and
estimatedLostSamples. A [FIFO] OVERRUN line reports each estimated gap.

Calibration workflow:
This update does not alter your existing two-stage calibration procedure:
(1) all three sensors together in their slots on the stationary disc;
(2) sensors attached to the subject while standing still.

Validation note:
The dashboard JavaScript syntax and HTML structure were checked. An Arduino
board build and hardware/FIFO validation could not be performed in this
runtime; test the three nodes together before collecting study trials.
