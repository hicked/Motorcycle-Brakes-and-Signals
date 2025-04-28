#pragma once
#include <Wire.h>
#include "button.h"

// green light on gyro towards back
#define MPU6050_ADDR 0x68  // MPU6050 I2C address
#define APPLY_MOUNTING_OFFSET false

// divide all these values by 17000 to get the amount in g force
#define MIN_GYRO_BREAKING 500 // Buffer area where gyro wont do anything
#define MAX_GYRO_BREAKING 5000 // Max value of gyro when braking. anything more will be emergency braking

#define MIN_GYRO_ACCELERATING 500 // buffer area where gyro wont do anything
#define MAX_GYRO_ACCELERATING 5000 // maximum value of gyro when accelerating

#define FORCE_EXPECTED_MAGNITUDE true  // Whether to override bad calibration values
#define EXPECTED_ACC_MAGNITUDE 16800.0 // expected value of acceleration when stationary (gravity only). Defaults to this if calibration fails
#define CALIBRATION_SAMPLE_SIZE 500 // sample size for calibration phase
#define CALIBRATION_ACC_DELTA 2000 // ensures that the acc found during calibration is within this of EXPECTED_ACC_MAGNITUDE

#define HILL_SAMPLE_SIZE 25 // higher value is more accurate, but slower. Used to detect hills
#define HILL_CORRECTION_SMOOTHING_FACTOR 1 // lower value is more smoothing, less reactive/fast, 1 means no smothing
// Not sure why you would want smoothing for detecting hills since it's an average anyways, but it's there if you need it

// Finaly, it smoothes the previous and new values together instead of just setting them
#define SMOOTHING_FACTOR 0.4 // lower value is more smoothing, less vibrations, but less reactive/fast. 1 means no smoothing

class Gyro {
    private:
        // Hardware references
        Button* button;
        
        // Calibration data
        double idleAcc = 0.0f;
        float mountingOffsetX = 0.0f;
        float mountingOffsetY = 0.0f;
        float mountingOffsetZ = 0.0f;

        // Sensor readings
        float measuredAccX = 0.0f;
        float measuredAccY = 0.0f;
        float measuredAccZ = 0.0f;

        // Hill detection
        float xSamples[HILL_SAMPLE_SIZE];
        float ySamples[HILL_SAMPLE_SIZE];
        float zSamples[HILL_SAMPLE_SIZE];
        float sumHillSamplesX = 0.0f;
        float sumHillSamplesY = 0.0f;
        float sumHillSamplesZ = 0.0f;
        int numHillSamples = 0;
        float correction = 0.0f;

        // Filtered outputs
        float correctedAcc = 0.0f;

        // Timing
        unsigned long lastUpdateTime = 0;

        bool readRawAccel();
        float median(float samples[], int size);
        void calculateHillCorrection();

    public:
        float pitch = 0.0f;
        float roll = 0.0f;
        float smoothedAndCorrectedYAcc = 0.0f;
        float prevSmoothedAndCorrectedYAcc = 0.0f;

        Gyro(Button* button);
        void update();
        float getAcceleration();
        float getRawAcceleration();
};