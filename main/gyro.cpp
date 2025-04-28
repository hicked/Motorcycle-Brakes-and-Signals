#include "gyro.h"
#include <Arduino.h>
#include <math.h>

// SDA = A4
// SCL = A5
// BUTONNNNNNNNNNNNNNNNNNNNNN
Gyro::Gyro(Button* button) {
  Wire.begin();
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // This sets the initial gyro value to ensure that it is calibrated. Sample size in this case is 500
  // It will calculate the average gyro magniture over this time, if the value is over x, assume the calibration failed
  // Like if it was done while accelerating, so set a conservative amount EXPECTED_ACC_MAGNITUDE
  float sum = 0.0;
  float sumX = 0.0;
  float sumY = 0.0;
  float sumZ = 0.0;
  this->button = button;

  int n = 0;
  delay(1000);
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
      float magnitude = sqrt(
        this->measuredAccX * this->measuredAccX + this->measuredAccY * this->measuredAccY + this->measuredAccZ * this->measuredAccZ);
      n++;
      sumX += this->measuredAccX;
      sumY += this->measuredAccY;
      sumZ += this->measuredAccZ;
      sum += magnitude;

    } else {
      Serial.print("Failed Attempt to read from MPU: ");
      Serial.print(failedAttempts + 1);
      Serial.print("/");
      Serial.println(CALIBRATION_SAMPLE_SIZE);
      failedAttempts++;
      if (failedAttempts > CALIBRATION_SAMPLE_SIZE) {
        break;
      }
      continue;
    }
    delay(5);
  }

  this->idleAcc = (float)(sum / n);
  this->mountingOffsetX = (float)(sumX / n);
  this->mountingOffsetY = (float)(sumY / n);
  this->mountingOffsetZ = (float)(sumZ / n) - EXPECTED_ACC_MAGNITUDE;

  if (abs(this->idleAcc - EXPECTED_ACC_MAGNITUDE) > CALIBRATION_ACC_DELTA) {
    this->idleAcc = EXPECTED_ACC_MAGNITUDE;
    Serial.print("OVERRIDING IDLE ACCELERATION TO: ");
    Serial.println(this->idleAcc);
  }

  this->lastUpdateTime = millis();  // Initialize the last update time
  this->numHillSamples = 0;         // Reset the hill sample count

  Serial.println("\nIdle magnitude:");
  Serial.println(this->idleAcc);
  Serial.println("\nMounting Offsets (X,Y,Z): ");
  Serial.print(this->mountingOffsetX);
  Serial.print(", ");
  Serial.print(this->mountingOffsetY);
  Serial.print(", ");
  Serial.println(this->mountingOffsetZ);
}



void Gyro::update() {
  if (!readRawAccel()) {
    Serial.println("MPU read failed in update(), or value out of bound");
    return;
  }

  this->xSamples[this->numHillSamples] = this->measuredAccX;
  this->ySamples[this->numHillSamples] = this->measuredAccY;
  this->zSamples[this->numHillSamples] = this->measuredAccZ;
  this->numHillSamples++;

  if (this->numHillSamples >= HILL_SAMPLE_SIZE) {
    calculateHillCorrection();
  }

  // Apply correction and smoothing
  this->correctedAcc = (this->measuredAccY-this->mountingOffsetY) - this->correction;
  this->prevSmoothedAndCorrectedYAcc = this->smoothedAndCorrectedYAcc;
  this->smoothedAndCorrectedYAcc = this->smoothedAndCorrectedYAcc * (1 - SMOOTHING_FACTOR)
                      + this->correctedAcc * SMOOTHING_FACTOR;
  
  if (numHillSamples == 0) {
    Serial.println("Smoothed Y Acceleration (braking/accelerating): ");
    Serial.println(this->smoothedAndCorrectedYAcc);
    Serial.println("==================================");
  }
}


void Gyro::calculateHillCorrection() {
  float avgX = median(xSamples, HILL_SAMPLE_SIZE);
  float avgY = median(ySamples, HILL_SAMPLE_SIZE);
  float avgZ = median(zSamples, HILL_SAMPLE_SIZE);

  // Debug output
  Serial.println("Raw Acceleration (X,Y,Z): ");
  Serial.print(avgX);
  Serial.print(", ");
  Serial.print(avgY);
  Serial.print(", ");
  Serial.println(avgZ);

  float mountingOffsetCorrectedX = avgX - this->mountingOffsetX;
  float mountingOffsetCorrectedY = avgY - this->mountingOffsetY;
  float mountingOffsetCorrectedZ = avgZ - this->mountingOffsetZ;
  float magnitude = sqrt(mountingOffsetCorrectedX * mountingOffsetCorrectedX
                         + mountingOffsetCorrectedY * mountingOffsetCorrectedY
                         + mountingOffsetCorrectedZ * mountingOffsetCorrectedZ);

  float normX = mountingOffsetCorrectedX / magnitude;
  float normY = mountingOffsetCorrectedY / magnitude;
  float normZ = mountingOffsetCorrectedZ / magnitude;

  this->pitch = atan2(normY, normZ);
  
  float sqrtArg = normY * normY + normZ * normZ;
  this->roll = atan2(-normX, sqrt(sqrtArg));


  // Calculate the forward gravity component (hill offset)
  // This represents how much gravity is acting in the forward/backward direction
  float gravityForward = sin(pitch) * cos(roll);

  // Calculate the correction needed to compensate for the hill
  // idleAcc is the expected gravity magnitude (typically ~16384 for ±2g range)
  float hillOffset = this->idleAcc * gravityForward;

  // Apply smoothing to the correction factor
  this->correction = this->correction * (1 - HILL_CORRECTION_SMOOTHING_FACTOR)
                     + hillOffset * HILL_CORRECTION_SMOOTHING_FACTOR;

  this->numHillSamples = 0;  // Reset the sample count for the next batch of readings

  Serial.println("\nCorrected Acceleration (Mounting/Hill/Lean/Gravity) (X,Y,Z): ");
  Serial.print(mountingOffsetCorrectedX);
  Serial.print(", ");
  Serial.print(mountingOffsetCorrectedY-this->correction);
  Serial.print(", ");
  Serial.println(mountingOffsetCorrectedZ-EXPECTED_ACC_MAGNITUDE);

  // Angles
  Serial.print("\nPitch: ");
  Serial.print(pitch * 180.0 / PI);
  Serial.println("°");
  Serial.print("Roll: ");
  Serial.print(roll * 180.0 / PI);
  Serial.println("°");
  Serial.println();
}


bool Gyro::readRawAccel() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);  // ACCEL_XOUT_H register
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(MPU6050_ADDR, 6, true) != 6) {
    return false;
  }
  measuredAccX = Wire.read() << 8 | Wire.read();
  measuredAccY = Wire.read() << 8 | Wire.read();
  measuredAccZ = Wire.read() << 8 | Wire.read();
  if (abs(this->measuredAccX) > 32768 || abs(this->measuredAccY) > 32768 || abs(this->measuredAccZ) > 32768) {
    return false;  // Invalid sensor values
  }
  return true;
}

// Median function implementation
float Gyro::median(float samples[], int size) {
  // Copy array for sorting
  float temp[size];
  for (int i = 0; i < size; i++) {
    temp[i] = samples[i];
  }

  for (int i = 0; i < size - 1; i++) {
    for (int j = 0; j < size - i - 1; j++) {
      if (temp[j] > temp[j + 1]) {
        float swap = temp[j];
        temp[j] = temp[j + 1];
        temp[j + 1] = swap;
      }
    }
  }

  return temp[size / 2];
}
