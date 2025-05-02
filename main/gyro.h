#pragma once
#include <Wire.h>
#include "button.h"

// green light on gyro towards back
#define MPU6050_ADDR 0x68  // MPU6050 I2C address
#define APPLY_MOUNTING_OFFSET true

// divide all these values by 17000 to get the amount in g force
#define MIN_GYRO_BREAKING 1000 // Buffer area where gyro wont do anything
#define MAX_GYRO_BREAKING 7000 // Max value of gyro when braking. anything more will be emergency braking

#define MIN_GYRO_ACCELERATING 1000 // buffer area where gyro wont do anything
#define MAX_GYRO_ACCELERATING 5000 // maximum value of gyro when accelerating

#define FORCE_EXPECTED_MAGNITUDE true  // Whether to override bad calibration values
#define EXPECTED_ACC_MAGNITUDE 16700.0 // expected value of acceleration when stationary (gravity only). Defaults to this if calibration fails
#define CALIBRATION_SAMPLE_SIZE 250 // sample size for calibration phase
#define CALIBRATION_ACC_DELTA 2000 // ensures that the acc found during calibration is within this of EXPECTED_ACC_MAGNITUDE

// WARNING!!! HILL_SAMPLE_SIZE IS VERY MEMORY INTENSIVE. STICK TO 25-150 OR MEMORY WILL FRAGMENT
#define HILL_SAMPLE_SIZE 25 // higher value is more accurate, but slower. Used to detect hills
#define HILL_CORRECTION_SMOOTHING_FACTOR 1 // lower value is more smoothing, less reactive/fast, 1 means no smoothing

// Finaly, it smoothes the previous and new values together instead of just setting them
#define SMOOTHING_FACTOR 0.25 // lower value is more smoothing, less vibrations, but less reactive/fast. 1 means no smoothing

class Gyro {
    private:
        // Hardware references
        Button* button;
        
        // Calibration data
        int idleAcc = 0;
        int mountingOffsetX = 0;
        int mountingOffsetY = 0;
        int mountingOffsetZ = 0;

        // Sensor readings
        int measuredAccX = 0;
        int measuredAccY = 0;
        int measuredAccZ = 0;

        // Hill detection
        long xSamplesSum = 0;
        long ySamplesSum = 0;
        long zSamplesSum = 0;
        long sampleAccelerationMagnitudes[HILL_SAMPLE_SIZE];
        int zSamples[HILL_SAMPLE_SIZE];
        long accelerationMagnitudeSum = 0;
        int sumHillSamplesX = 0;
        int sumHillSamplesY = 0;
        int sumHillSamplesZ = 0;
        int numHillSamples = 0;
        int correction = 0;

        // Filtered outputs
        int correctedYAcceleration = 0;

        // Timing
        unsigned long lastUpdateTime = 0;

        bool readRawAccel();
        int median(long samples[], int size);
        void calculateHillCorrection();

    public:
        float pitch = 0.0f;
        float roll = 0.0f;
        int smoothedAndCorrectedYAcc = 0;
        int prevSmoothedAndCorrectedYAcc = 0;

        Gyro(Button* button);
        void update();
        float getAcceleration();
        float getRawAcceleration();
};