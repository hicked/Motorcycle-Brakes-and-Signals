#include "gyro.h"
#include <Arduino.h>
#include <math.h>

// SDA = A4
// SCL = A5

Gyro::Gyro(Button* button) {
  Wire.begin();
  initializeMPU();
  this->button = button;
  
  // This sets the initial gyro value to ensure that it is calibrated. Sample size in this case is 500
  // It will calculate the average gyro magniture over this time, if the value is over x, assume the calibration failed
  // Like if it was done while accelerating, so set a conservative amount EXPECTED_ACC_MAGNITUDE
  float sum = 0.0;
  float sumX = 0.0;
  float sumY = 0.0;
  float sumZ = 0.0;

  int n = 0;
  
  if (FORCE_EXPECTED_MAGNITUDE) {
    this->idleAcc = EXPECTED_ACC_MAGNITUDE;
    return;
  }

  unsigned long lastTime = millis();
  unsigned int failedAttempts = 0;
  while (n < CALIBRATION_SAMPLE_SIZE) {
    unsigned long currentTime = millis();
    if (currentTime - lastTime > 100) {
      Serial.print("Calibrating Gyroscope: ");
      Serial.print(n + 1);
      Serial.print("/");
      Serial.println(CALIBRATION_SAMPLE_SIZE);
      lastTime = currentTime;
    }

    if (readRawAccel()) {
      int magnitude = sqrt(
        this->measuredAccX * this->measuredAccX + 
        this->measuredAccY * this->measuredAccY + 
        this->measuredAccZ * this->measuredAccZ);

      n++;
      sumX += (int)this->measuredAccX;
      sumY += (int)this->measuredAccY;
      sumZ += (int)this->measuredAccZ;

    } else {
      Serial.print("Failed Attempt to read from MPU: ");
      Serial.print(failedAttempts + 1);
      Serial.print("/");
      Serial.println(CALIBRATION_SAMPLE_SIZE);
      failedAttempts++;
      if (failedAttempts > CALIBRATION_SAMPLE_SIZE) {
        button->mode = BRAKE_MODE_STATIC;  // Set to static mode if calibration fails
        Serial.println("Calibration failed, setting to static mode.");
        this->idleAcc = EXPECTED_ACC_MAGNITUDE;
        break;
      }
      continue;
    }
    delay(5);
  }
  sum = sqrt(sumX*sumX + sumY*sumY + sumZ*sumZ);
  this->idleAcc = (sum / n);

  if (abs(this->idleAcc - EXPECTED_ACC_MAGNITUDE) > CALIBRATION_ACC_DELTA) {
    this->idleAcc = EXPECTED_ACC_MAGNITUDE;
    Serial.print("OVERRIDING IDLE ACCELERATION TO: ");
    Serial.println(this->idleAcc);
  }
  this->idleAcc = EXPECTED_ACC_MAGNITUDE;

  this->lastUpdateTime = millis();  // Initialize the last update time

  Serial.println("\nIdle magnitude:");
  Serial.println(this->idleAcc);
  Serial.println("\nMounting Offsets (X,Y,Z): ");
  Serial.print(sumX/n);
  Serial.print(", ");
  Serial.print(sumY/n);
  Serial.print(", ");
  Serial.println(sumZ/n);
  Serial.println("==================================");
}


void Gyro::update() {
  if (!readRawAccel()) {
    Serial.println("MPU read failed");
    return;
  }

  // Print raw values for debugging
  // Serial.print("Raw Y/Z: ");
  // Serial.print(measuredAccX);
  // Serial.print(", ");
  // Serial.print(measuredAccY);
  // Serial.print(", ");
  // Serial.println(measuredAccZ);

  
  this->smoothedAccX = this->prevSmoothedAccX*(1 - X_SMOOTHING) + 
                        this->measuredAccX * (X_SMOOTHING);

  this->smoothedAccY = this->prevSmoothedAccY*(1 - Y_SMOOTHING) + 
                        this->measuredAccY * (Y_SMOOTHING);

  this->smoothedAccZ = this->prevSmoothedAccZ*(1 - Z_SMOOTHING) + 
                        this->measuredAccZ * (Z_SMOOTHING);

  // Calculate magnitude of Y and Z (gravity-compensated)
  long accMagnitude = sqrt(
                            (long) this->smoothedAccX * (long) this->smoothedAccX + 
                            (long) this->smoothedAccY * (long) this->smoothedAccY +
                            (long) this->measuredAccZ * (long) this->measuredAccZ);

  // Compute deviation from expected gravity
  int inputAcceleration = (accMagnitude - EXPECTED_ACC_MAGNITUDE) * (this->smoothedAccY < 0 ? 1 : -1);

  // Store deviation (positive/negative indicates acceleration direction)
  this->sampleAccelerationMagnitudes[numMedianSample] = inputAcceleration;
  this->numMedianSample++;

  if (this->numMedianSample >= MEDIAN_SAMPLE_SIZE) {
    long medAcc = median(sampleAccelerationMagnitudes, MEDIAN_SAMPLE_SIZE);
    
    // Apply smoothing to the deviation (not the raw magnitude)
    this->smoothedAndCorrectedYAcc = this->smoothedAndCorrectedYAcc*(1 - GLOBAL_SMOOTHING) + 
                               medAcc * GLOBAL_SMOOTHING;

    this->prevSmoothedAndCorrectedYAcc = this->smoothedAndCorrectedYAcc;
    this->numMedianSample = 0;

    Serial.println(this->smoothedAndCorrectedYAcc);
  }
  
  this->prevSmoothedAccX = this->smoothedAccX;
  this->prevSmoothedAccY = this->smoothedAccY;
  this->prevSmoothedAccZ = this->smoothedAccZ;
}


void Gyro::initializeMPU() {
  debugI2CPins();
  Wire.begin();
  
  // Add I2C scanner for debugging
  Serial.println("Scanning I2C devices...");
  bool deviceFound = false;
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found I2C device at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      deviceFound = true;
    }
  }
  if (!deviceFound) {
    Serial.println("No I2C devices found!");
  }

  Wire.beginTransmission(MPU6050_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("MPU6050 not detected at expected address!");
    return;
  }

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1
  Wire.write(0);     // Wake up
  Wire.endTransmission(true);

  // Set accelerometer to ±8g (optional)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C);  // ACCEL_CONFIG
  Wire.write(0x10);  // ±8g
  Wire.endTransmission(true);

  delay(100);
  
  // Test read after initialization
  Serial.println("Testing initial read...");
  if (readRawAccel()) {
    Serial.println("Initial MPU6050 read successful!");
  } else {
    Serial.println("Initial MPU6050 read failed!");
  }
}

bool Gyro::readRawAccel() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);  // Start at ACCEL_XOUT_H
  byte error = Wire.endTransmission(false);
  if (error != 0) {
    Serial.print("I2C transmission error: ");
    Serial.println(error);
    return false;
  }

  byte bytesReceived = Wire.requestFrom(MPU6050_ADDR, 6);
  if (bytesReceived != 6) {
    Serial.print("Expected 6 bytes, got: ");
    Serial.println(bytesReceived);
    return false;
  }

  // Read and convert to signed 16-bit integers
  int16_t rawX = Wire.read() << 8 | Wire.read();
  int16_t rawY = Wire.read() << 8 | Wire.read();
  int16_t rawZ = Wire.read() << 8 | Wire.read();

  measuredAccX = rawX;
  measuredAccY = rawY;
  measuredAccZ = rawZ;

  return true;
}

// Median function implementation
int Gyro::median(int samples[], int size) {
  // Copy array for sorting
  int temp[size];
  for (int i = 0; i < size; i++) {
    temp[i] = samples[i];
  }

  for (int i = 0; i < size - 1; i++) {
    for (int j = 0; j < size - i - 1; j++) {
      if (temp[j] > temp[j + 1]) {
        int swap = temp[j];
        temp[j] = temp[j + 1];
        temp[j + 1] = swap;
      }
    }
  }

  return temp[size / 2];
}

void Gyro::debugI2CPins() {
  // Read analog values (0-1023 for 0-5V)
  int sda_analog = analogRead(A4);
  int scl_analog = analogRead(A5);
  
  // Convert to voltage (assuming 5V reference)
  float sda_voltage = (sda_analog / 1023.0) * 5.0;
  float scl_voltage = (scl_analog / 1023.0) * 5.0;
  
  // Read digital state
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  int sda_digital = digitalRead(A4);
  int scl_digital = digitalRead(A5);
  
  Serial.print("SDA (A4) - Analog: ");
  Serial.print(sda_analog);
  Serial.print(" (");
  Serial.print(sda_voltage);
  Serial.print("V), Digital: ");
  Serial.println(sda_digital);
  
  Serial.print("SCL (A5) - Analog: ");
  Serial.print(scl_analog);
  Serial.print(" (");
  Serial.print(scl_voltage);
  Serial.print("V), Digital: ");
  Serial.println(scl_digital);
  
  // Reinitialize I2C after reading pins
  Wire.begin();
  delay(1000);
}