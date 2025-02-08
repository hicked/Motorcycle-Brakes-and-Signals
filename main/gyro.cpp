#include "gyro.h"
#include <Arduino.h>
#include <math.h>

// SDA = A4
// SCL = A5

Gyro::Gyro(Button *button) {
    this->button = button;

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
                this->button->mode = BRAKE_MODE_STATIC;
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
    this->prevMeasuredAccZ = this->measuredAccZ;
}


void Gyro::update() {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);  
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 14, true);  // 6 pieces of data cause we don't care about rotational

    if (Wire.available() == 14) {
        this->measuredAccX = Wire.read() << 8 | Wire.read();    
        this->measuredAccY = Wire.read() << 8 | Wire.read();  
        this->measuredAccZ = Wire.read() << 8 | Wire.read();

        this->temp = Wire.read() << 8 | Wire.read();

        this->measuredGyroX = Wire.read() << 8 | Wire.read();
        this->measuredGyroY = Wire.read() << 8 | Wire.read();
        this->measuredGyroZ = Wire.read() << 8 | Wire.read();
    } else {
        Serial.println("Failed to read from MPU");
        return;
    }

    // Serial.print("MEASURED: ");
    // Serial.println(this->measuredAccZ);
    // Serial.print("PREV: ");
    // Serial.println(this->prevMeasuredAccZ);
    // Serial.print("DELTA: ");
    
    
    if (FILTER_DELTA) { // this should always be true, otherwise brake might be too sensitive, but it's here anyways
        float delta = sqrt(this->measuredAccZ*this->measuredAccZ) - sqrt(this->prevMeasuredAccZ*this->prevMeasuredAccZ);
        if (sqrt(delta*delta) > BUMP_THRESHOLD && millis() - lastUpdateTime < BUMP_OVERRIDE) { // override if bump has been detected for too long
            //Serial.println("BUMP DETECTED.");
            this->correctedAcc = this->measuredAccY;
            
        }
        else {
            // Calculate the corrected acceleration (in gs)
            float accX = this->measuredAccX / this->idleAcc;
            float accY = this->measuredAccY / this->idleAcc;
            float accZ = this->measuredAccZ / this->idleAcc;

            float theta = atan2(accY, accZ);  // Angle relative to ground (3d)

            // Compute gravity effect correction for hills and lean
            float correction = this->idleAcc * sin(theta);
            this->correctedAcc = this->measuredAccY - correction;
            
            lastUpdateTime = millis();
            this->prevMeasuredAccZ = this->measuredAccZ;
            this->prevSmoothedAcc = this->smoothedAcc;
        }
    }


    if (FILTER_SMOOTHING) {
        this->smoothedAcc = this->smoothedAcc * (1 - SMOOTHING_FACTOR) + this->correctedAcc * SMOOTHING_FACTOR;
    }
    else {
        this->smoothedAcc = this->correctedAcc;
    }

    Serial.print("Acc: ");
    Serial.println(this->smoothedAcc);

    // Serial.print("Temperature: ");
    // Serial.println(temp/340.0 + 36.53);

    // Serial.print("X: ");
    // Serial.print(this->measuredAccX);
    // Serial.print(" Y: ");
    // Serial.print(this->measuredAccY);
    // Serial.print(" Z: ");
    // Serial.println(this->measuredAccZ);

    // Debugging
    // Serial.print("Angle (theta, deg): ");
    // Serial.println(theta * 180.0 / PI);

    // Serial.print("Corrected AccY: ");
    // Serial.println(this->correctedAcc);
}