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

    // Serial.print("X: ");
    // Serial.print(this->measuredAccX);
    // Serial.print(" Y: ");
    // Serial.print(this->measuredAccY);
    // Serial.print(" Z: ");
    // Serial.println(this->measuredAccZ);
    

    // Calculate the corrected acceleration
    // Compute roll (leaning) angle
    float ratioRoll = this->measuredAccX / this->idleAcc;
    ratioRoll = constrain(ratioRoll, -1.0, 1.0);
    float phi = sqrt(asin(ratioRoll)*asin(ratioRoll));  // Tilt due to leaning

    // Compute pitch (hills) angle
    float ratioPitch = this->measuredAccZ / this->idleAcc;
    ratioPitch = constrain(ratioPitch, -1.0, 1.0);
    float theta = sqrt((acos(ratioPitch) - phi)*(acos(ratioPitch) - phi));  // Tilt due to hills

    // Compute gravity effect correction for hills and lean
    float correctionPitch = this->idleAcc * sin(theta);  // Gravity effect from hills
    float correctionRoll = this->idleAcc * sin(phi);    // Gravity effect from leaning

    // Corrected acceleration
    this->correctedAcc = this->measuredAccY 
                        + (this->measuredAccY < 0 ? correctionPitch : -correctionPitch);

    // Debugging
    // Serial.print("Pitch (theta, deg): ");
    // Serial.println(theta * 180.0 / PI);

    // Serial.print("Roll (phi, deg): ");
    // Serial.println(phi * 180.0 / PI);

    // Serial.print("Corrected AccY: ");
    // Serial.println(this->correctedAcc);


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