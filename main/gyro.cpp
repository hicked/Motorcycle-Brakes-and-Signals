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


bool Gyro::readRawAccel() {
  static uint8_t errorCount = 0;
  
  initializeMPU();
  if (Wire.endTransmission(false) != 0) {
    errorCount++;
    if (errorCount > 5) {
      // Attempt to recover the I2C bus
      Wire.end();
      delay(100);
      Wire.begin();
      initializeMPU();  // You'll need to implement this
      errorCount = 0;
    }
    return false;
  }
  
  if (Wire.requestFrom(MPU6050_ADDR, 6, true) != 6) {
    errorCount++;
    return false;
  }
  
  // Reset error count on successful read
  errorCount = 0;
  
  measuredAccX = Wire.read() << 8 | Wire.read();
  measuredAccY = Wire.read() << 8 | Wire.read();
  measuredAccZ = Wire.read() << 8 | Wire.read();
  
  if (abs(this->measuredAccX) > 32768 || abs(this->measuredAccY) > 32768 || abs(this->measuredAccZ) > 32768) {
    return false;  // Invalid sensor values
  }
  
  return true;
}

void Gyro::initializeMPU() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Wake up the MPU-6050
  Wire.endTransmission(true);
  delay(100);
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
