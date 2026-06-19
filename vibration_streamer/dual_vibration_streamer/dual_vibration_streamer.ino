/*
 * dual_vibration_streamer.ino
 * Hardware : Arduino Uno + MPU6050 (Tip) + MPU6500 (Middle)
 * Protocol : I2C @ 400 kHz  →  Serial CSV @ 500000 baud
 * Rate     : 1000 Hz (1000 µs non-blocking loop via micros())
 */

#include <Wire.h>

// Sensor I2C Addresses
static const uint8_t MPU_TIP = 0x68;   // MPU6050 (AD0 to GND/Open)
static const uint8_t MPU_MID = 0x69;   // MPU6500 (AD0 to 3.3V)

// Register Map
static const uint8_t REG_PWR_MGMT_1   = 0x6B;
static const uint8_t REG_ACCEL_CONFIG = 0x1C;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;

// 1000 Hz Sampling Target
static const unsigned long SAMPLE_INTERVAL_US = 1000UL;   
static unsigned long previousMicros = 0;

static int16_t ax1, ay1, az1; // Tip data
static int16_t ax2, ay2, az2; // Mid data

void writeRegister(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}

void initSensors() {
  // Wake up and configure Tip Sensor (MPU6050)
  writeRegister(MPU_TIP, REG_PWR_MGMT_1, 0x00);
  writeRegister(MPU_TIP, REG_ACCEL_CONFIG, 0x00); // +/- 2g

  // Wake up and configure Middle Sensor (MPU6500)
  writeRegister(MPU_MID, REG_PWR_MGMT_1, 0x00);
  writeRegister(MPU_MID, REG_ACCEL_CONFIG, 0x00); // +/- 2g
}

void readSensor(uint8_t addr, int16_t &ax, int16_t &ay, int16_t &az) {
  Wire.beginTransmission(addr);
  Wire.write(REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);           
  Wire.requestFrom(addr, (uint8_t)6, (uint8_t)true);

  ax = (int16_t)(Wire.read() << 8 | Wire.read());
  ay = (int16_t)(Wire.read() << 8 | Wire.read());
  az = (int16_t)(Wire.read() << 8 | Wire.read());
}

void setup() {
  // High-speed baud rate to prevent serial bottleneck
  Serial.begin(500000);
  while (!Serial) { }

  Wire.begin();
  Wire.setClock(400000); // Fast I2C                  

  initSensors();

  // Print 7-column CSV header
  Serial.println(F("timestamp_us,ax_tip,ay_tip,az_tip,ax_mid,ay_mid,az_mid"));
  previousMicros = micros();
}

void loop() {
  unsigned long now = micros();

  if (now - previousMicros < SAMPLE_INTERVAL_US) {
    return;
  }
  previousMicros += SAMPLE_INTERVAL_US;

  // Read both sensors back-to-back
  readSensor(MPU_TIP, ax1, ay1, az1);
  readSensor(MPU_MID, ax2, ay2, az2);

  // Stream data out over Serial
  Serial.print(now); Serial.print(',');
  Serial.print(ax1); Serial.print(','); Serial.print(ay1); Serial.print(','); Serial.print(az1); Serial.print(',');
  Serial.print(ax2); Serial.print(','); Serial.print(ay2); Serial.print(','); Serial.println(az2);
}