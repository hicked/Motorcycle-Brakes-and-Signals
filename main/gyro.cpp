#include "gyro.h"
#include <Arduino.h>
#include <math.h>

// SDA = A4
// SCL = A5

Gyro::Gyro() {
    Wire.begin();
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);  
    Wire.write(0);
    Wire.endTransmission(true);

    // This sets the initial gyro value to ensure that it is calibrated. Sample size in this case is 500
    // It will calculate the average gyro magniture over this time, if the value is over x, assume the calibration failed
    // Like if it was done while accelerating, so set a conservative amount EXPECTED_ACC_MAGNITUDE
    float sum = 0.0;
    int n = 0;
    delay(3000);
    unsigned long lastTime = millis();
    unsigned int failedAttempts = 0;
    while (n < CALIBRATION_SAMPLE_SIZE) {
        unsigned long currentTime = millis();
        if (currentTime - lastTime > 100) {
            Serial.print("Calibrating Gyroscope: ");
            Serial.print(n+1);
            Serial.print("/");
            Serial.println(CALIBRATION_SAMPLE_SIZE);
            lastTime = currentTime;
        }
        Wire.beginTransmission(MPU);
        Wire.write(0x3B);  
        Wire.endTransmission(false);
        Wire.requestFrom(MPU, 6, true);
        if (Wire.available() == 6) {
            this->measuredAccX = Wire.read() << 8 | Wire.read();    
            this->measuredAccY = Wire.read() << 8 | Wire.read();  
            this->measuredAccZ = Wire.read() << 8 | Wire.read();
            
            this->correctedAcc = sqrt(
                            this->measuredAccX * this->measuredAccX +
                            this->measuredAccY * this->measuredAccY +
                            this->measuredAccZ * this->measuredAccZ);
            n++;
            sum += this->correctedAcc;

        } else {
            Serial.print("Failed Attempt to read from MPU: ");
            Serial.print(failedAttempts+1);
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
    Serial.println("Calibratin Gyroscope: COMPLETED");

    this->idleAcc = sum/n;
    Serial.print("IDLE ACC: ");
    Serial.println(this->idleAcc);
    if (abs(this->idleAcc - EXPECTED_ACC_MAGNITUDE) > CALIBRATION_ACC_DELTA) {
        this->idleAcc = EXPECTED_ACC_MAGNITUDE;
        Serial.print("OVERRIDING ACC TO: ");
        Serial.println(this->idleAcc);
    } 
    
    this->lastUpdateTime = millis(); // Initialize the last update time
}

void Gyro::update() {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);  
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);  // 6 pieces of data cause we don't care about rotational

    if (Wire.available() == 6) {
        this->measuredAccX = Wire.read() << 8 | Wire.read();    
        this->measuredAccY = Wire.read() << 8 | Wire.read();  
        this->measuredAccZ = Wire.read() << 8 | Wire.read();
    } else {
        Serial.println("Failed to read from MPU");
        return;
    }
    this->prevAcc = this->smoothedAcc;

    // Calculate the corrected acceleration
    this->correctedAcc = (sqrt(this->measuredAccX * this->measuredAccX +
                            this->measuredAccY * this->measuredAccY +
                            this->measuredAccZ * this->measuredAccZ) 
                            - this->idleAcc)
                            * (this->measuredAccY >= 0 ? 1 : -1);
    //this->correctedAcc = this->measuredAccY;

    // Check for sudden bumps
    if (abs(this->prevAcc - this->correctedAcc) > BUMP_THRESHOLD && FILTER_DELTA) {
        unsigned long currentTime = millis();
        this->numBumps++;

        // Calculate the corrected acceleration 
        if (this->correctedAcc < this->minBump) { 
            this->minBump = this->correctedAcc; 
        } 
        if (this->correctedAcc > this->maxBump) { 
            this->maxBump = this->correctedAcc; 
        }

        // Overrides if bumps are within a delta (therefore not a bump)
        if (this->numBumps > SAMPLE_SIZE_BUMPS) {
            if (this->maxBump - this->minBump < BUMP_THRESHOLD) {
                Serial.println("Bumps seem consitent: Overriding");
                this->smoothedAcc = this->correctedAcc;
                this->prevAcc = this->correctedAcc;
            }
            this->minBump = 100000;
            this->maxBump = -100000;
            this->numBumps = 0;
        }

        // Forcing update if too much time has passed
        else if (currentTime - this->lastUpdateTime > BUMP_OVERRIDE_TIME && currentTime - this->lastForceUpdate > MIN_TIME_BETWEEN_OVERRIDES) {
            Serial.println("Too much time passed since last update: Overriding."); // forcing changes
            // Force an update with the current corrected value
            this->smoothedAcc = this->correctedAcc;
            this->prevAcc = this->correctedAcc;
            this->lastUpdateTime = currentTime; // Reset the last update time
            this->lastForceUpdate = currentTime;
            this->minBump = 100000;
            this->maxBump = -100000;
            this->numBumps = 0;
        } else {
            // Skip the update since its a bump
            Serial.print("BUMP DETECTED: ");
            Serial.println(this->correctedAcc);
            return;
        }
    }

    if (!FILTER_AVG) {
        this->numSamples = AVG_SAMPLE_SIZE;
        this->sumSamples = this->correctedAcc;
    }
    if (this->numSamples < AVG_SAMPLE_SIZE) {
        this->sumSamples += this->correctedAcc;
        this->numSamples++;
        // Serial.print("Raw: ");
        // Serial.println(this->correctedAcc);
    }
    else {
        // Calculate the average acceleration
        this->avgAcc = this->sumSamples / AVG_SAMPLE_SIZE;
        // Serial.print("Avg: ");
        // Serial.println(this->avgAcc);

        if (FILTER_SMOOTHING) {
            this->smoothedAcc = this->smoothedAcc * (1 - SMOOTHING_FACTOR) + this->correctedAcc * SMOOTHING_FACTOR;
        }
        else {
            this->smoothedAcc = this->avgAcc;
        }

        // Update previous corrected acceleration and timestamp and reset the sample data
        this->numSamples = 0;
        this->avgAcc = 0.0;
        this->sumSamples = 0.0;

        lastUpdateTime = millis();
        Serial.print("Acc: ");
        Serial.println(this->smoothedAcc);
    }
}