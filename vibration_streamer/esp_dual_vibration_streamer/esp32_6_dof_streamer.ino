/*
 * ESP32 Dual-Sensor 6-DoF Streamer (I2C STABILITY PATCH)
 * Grabs both Accel (XYZ) and Gyro (XYZ) at 1000 Hz, 500,000 Baud
 */
#include <Wire.h>

static const uint8_t MPU_TIP = 0x68;   // Tip Sensor (Free End)
static const uint8_t MPU_MID = 0x69;   // Mid Sensor (AD0 to 3.3V)

static const uint8_t REG_PWR_MGMT_1   = 0x6B;
static const uint8_t REG_GYRO_CONFIG  = 0x1B;
static const uint8_t REG_ACCEL_CONFIG = 0x1C;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;

static const unsigned long SAMPLE_INTERVAL_US = 1000UL;   
static unsigned long previousMicros = 0;

// Variables for 12 data points
static int16_t ax1, ay1, az1, gx1, gy1, gz1; 
static int16_t ax2, ay2, az2, gx2, gy2, gz2; 

void writeRegister(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(); // STABILITY FIX: Standard STOP bit
}

void setup() {
  Serial.begin(500000); 
  while (!Serial) { }

  Wire.begin(21, 22);   
  Wire.setClock(100000); // 100kHz for Stability                  
  delay(50); 

  // Wake up and configure Tip Sensor
  writeRegister(MPU_TIP, REG_PWR_MGMT_1, 0x00);
  writeRegister(MPU_TIP, REG_ACCEL_CONFIG, 0x00); // +/- 2g
  writeRegister(MPU_TIP, REG_GYRO_CONFIG, 0x08);  // +/- 500 deg/s

  // Wake up and configure Mid Sensor
  writeRegister(MPU_MID, REG_PWR_MGMT_1, 0x00);
  writeRegister(MPU_MID, REG_ACCEL_CONFIG, 0x00); // +/- 2g
  writeRegister(MPU_MID, REG_GYRO_CONFIG, 0x08);  // +/- 500 deg/s

  Serial.println(F("timestamp_us,ax_tip,ay_tip,az_tip,gx_tip,gy_tip,gz_tip,ax_mid,ay_mid,az_mid,gx_mid,gy_mid,gz_mid"));
  previousMicros = micros();
}

void readSensor6DoF(uint8_t addr, int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(addr);
  Wire.write(REG_ACCEL_XOUT_H);
  Wire.endTransmission();           
  
  // Request 14 bytes: 6 Accel + 2 Temp + 6 Gyro
  Wire.requestFrom(addr, (uint8_t)14);
  
  if (Wire.available() >= 14) {
    ax = (int16_t)(Wire.read() << 8 | Wire.read());
    ay = (int16_t)(Wire.read() << 8 | Wire.read());
    az = (int16_t)(Wire.read() << 8 | Wire.read());
    
    // Read and discard the 2 Temperature bytes
    Wire.read(); Wire.read();
    
    gx = (int16_t)(Wire.read() << 8 | Wire.read());
    gy = (int16_t)(Wire.read() << 8 | Wire.read());
    gz = (int16_t)(Wire.read() << 8 | Wire.read());
  }
}

void loop() {
  unsigned long now = micros();
  if (now - previousMicros < SAMPLE_INTERVAL_US) return;
  previousMicros += SAMPLE_INTERVAL_US;

  // Read all 14 bytes from both sensors
  readSensor6DoF(MPU_TIP, ax1, ay1, az1, gx1, gy1, gz1);
  readSensor6DoF(MPU_MID, ax2, ay2, az2, gx2, gy2, gz2);

  // Print 13 columns (Timestamp + 12 axes)
  Serial.print(now); Serial.print(',');
  Serial.print(ax1); Serial.print(','); Serial.print(ay1); Serial.print(','); Serial.print(az1); Serial.print(',');
  Serial.print(gx1); Serial.print(','); Serial.print(gy1); Serial.print(','); Serial.print(gz1); Serial.print(',');
  Serial.print(ax2); Serial.print(','); Serial.print(ay2); Serial.print(','); Serial.print(az2); Serial.print(',');
  Serial.print(gx2); Serial.print(','); Serial.print(gy2); Serial.print(','); Serial.println(gz2);
}