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


    // Method 1
    // Serial.print("MEASURED: ");
    // Serial.println(this->measuredAccZ);
    // Serial.print("PREV: ");
    // Serial.println(this->prevMeasuredAccZ);
    // Serial.print("DELTA: ");

    this->sumHillSamplesX += this->measuredAccX;
    this->sumHillSamplesY += this->measuredAccY;
    this->sumHillSamplesZ += this->measuredAccZ;
    this->numHillSamples++;

    if (this->numHillSamples >= HILL_SAMPLE_SIZE) {
        // Calculate the average acceleration
        float averageMeasuredAccX = this->sumHillSamplesX / this->numHillSamples;
        float averageMeasuredAccY = this->sumHillSamplesY / this->numHillSamples;
        float averageMeasuredAccZ = this->sumHillSamplesZ / this->numHillSamples;

        // What we can do now is compair the average magnitude over this time period with the idleAcc
        float avgMagnitude = sqrt(averageMeasuredAccX*averageMeasuredAccX + 
                                  averageMeasuredAccY*averageMeasuredAccY + 
                                  averageMeasuredAccZ*averageMeasuredAccZ);
        // Serial.print("Avg Magnitude: ");
        // Serial.println(avgMagnitude);

        // Whatever the delta is, should be from Y acceleration (braking/accelerating)
        float deltaAccY = avgMagnitude - this->idleAcc;
        // Serial.print("Delta Acc: ");
        // Serial.println(deltaAccY);
        
        // Subtract the delta from the average measured acceleration to get the corrected acceleration for angles
        averageMeasuredAccY -= deltaAccY;

        float accX = averageMeasuredAccX / this->idleAcc;
        float accY = averageMeasuredAccY / this->idleAcc;
        float accZ = averageMeasuredAccZ / this->idleAcc;

        float theta = atan2(accY, accZ);  // Angle relative to ground (3d)

        // Calculate the correction factor
        this->correction = this->idleAcc * sin(theta);

        this->sumHillSamplesX = 0;
        this->sumHillSamplesY = 0;
        this->sumHillSamplesZ = 0;
        this->numHillSamples = 0;

        // Serial.print("Theta: ");
        // Serial.println(theta * 180.0 / PI);
        // Serial.println("Correction: ");
        // Serial.println(this->correction);
    }


    // Method 2
    // under normal circumstances (no incline) this value should be equal to 1g
    combinedAccXZ = abs(this->measuredAccZ) + abs(this->measuredAccX);

    // If its above 1g, lets just ignore it, assuming there is a acceleration from initiating lean,
    // or from bumps. But if its below 1g, lets smoothly change the correction factor (assuming the difference is due to a hill)
    if (combinedAccXZ < this->idleAcc) {
        hillOffset = (this->idleAcc - combinedAccXZ) * (this->measuredAccY > 0 ? 1 : -1);
        
        float theta = acos(combinedAccXZ / this->idleAcc);
        // Serial.print("Theta: ");
        // Serial.println(theta * 180.0 / PI);

        this->correction = this->correction * (1 - HILL_SMOOTHING_FACTOR) + hillOffset * HILL_SMOOTHING_FACTOR;
    }

    
    // apply the correction factor to the acceleration. This works for both methods
    this->correctedAcc = this->measuredAccY - this->correction;

    this->prevSmoothedAcc = this->smoothedAcc;
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