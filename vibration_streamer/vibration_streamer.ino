/*
 * vibration_streamer.ino
 * ──────────────────────────────────────────────────────────────────
 * Real-Time Vibration Data Streamer
 * Hardware : Arduino Uno  +  MPU6050 (GY-521 breakout)
 * Protocol : I2C @ 400 kHz  →  Serial CSV @ 115 200 baud
 * Rate     : 100 Hz (10 000 µs non-blocking loop via micros())
 * Scale    : ±2 g  (sensitivity 16 384 LSB/g)
 * Output   : timestamp_us, ax_raw, ay_raw, az_raw\r\n
 * ──────────────────────────────────────────────────────────────────
 */

#include <Wire.h>

/* ── MPU6050 Register Map (subset) ─────────────────────────────── */
static const uint8_t MPU6050_ADDR         = 0x68;
static const uint8_t REG_PWR_MGMT_1       = 0x6B;
static const uint8_t REG_ACCEL_CONFIG     = 0x1C;
static const uint8_t REG_ACCEL_XOUT_H     = 0x3B;

/* ── Timing ────────────────────────────────────────────────────── */
static const unsigned long SAMPLE_INTERVAL_US = 10000UL;   // 100 Hz
static unsigned long previousMicros = 0;

/* ── Raw accelerometer readings ────────────────────────────────── */
static int16_t ax, ay, az;

/* ────────────────────────────────────────────────────────────────
 * writeMPU6050Register
 * Write a single byte to an MPU6050 register over I2C.
 * ──────────────────────────────────────────────────────────────── */
static void writeMPU6050Register(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}

/* ────────────────────────────────────────────────────────────────
 * initMPU6050
 * Wake the sensor and set full-scale range to ±2 g.
 * ──────────────────────────────────────────────────────────────── */
static void initMPU6050() {
  // Wake from sleep — select internal 8 MHz oscillator
  writeMPU6050Register(REG_PWR_MGMT_1, 0x00);

  // ACCEL_CONFIG register: AFS_SEL = 0  →  ±2 g (16 384 LSB/g)
  writeMPU6050Register(REG_ACCEL_CONFIG, 0x00);
}

/* ────────────────────────────────────────────────────────────────
 * readAccelerometer
 * Burst-read 6 bytes (XH, XL, YH, YL, ZH, ZL) starting at 0x3B.
 * ──────────────────────────────────────────────────────────────── */
static void readAccelerometer() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);           // repeated-start
  Wire.requestFrom(MPU6050_ADDR, (uint8_t)6, (uint8_t)true);

  ax = (int16_t)(Wire.read() << 8 | Wire.read());
  ay = (int16_t)(Wire.read() << 8 | Wire.read());
  az = (int16_t)(Wire.read() << 8 | Wire.read());
}

/* ── setup() ───────────────────────────────────────────────────── */
void setup() {
  Serial.begin(115200);
  while (!Serial) { /* wait for native-USB boards */ }

  Wire.begin();
  Wire.setClock(400000);                  // Fast-mode I2C (400 kHz)

  initMPU6050();

  // Print CSV header so downstream parsers can auto-detect columns
  Serial.println(F("timestamp_us,ax,ay,az"));

  previousMicros = micros();
}

/* ── loop() — non-blocking, jitter-compensated 100 Hz sampler ── */
void loop() {
  unsigned long now = micros();

  // Guard: only proceed when a full interval has elapsed.
  // Comparison handles the ~71-minute micros() rollover correctly.
  if (now - previousMicros < SAMPLE_INTERVAL_US) {
    return;
  }

  // Advance the anchor (preserves cadence even if a read is slow)
  previousMicros += SAMPLE_INTERVAL_US;

  // Burst-read raw accelerometer data
  readAccelerometer();

  // Stream CSV line: timestamp_us, ax, ay, az
  Serial.print(now);
  Serial.print(',');
  Serial.print(ax);
  Serial.print(',');
  Serial.print(ay);
  Serial.print(',');
  Serial.println(az);
}