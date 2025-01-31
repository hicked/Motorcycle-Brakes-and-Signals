#pragma once
#include <Wire.h>
// green light on gyro towards back

// divide all these values by 17000 to get the amount in g force
#define MIN_GYRO_BREAKING 2000 // Buffer area where gyro wont do anything
#define MAX_GYRO_BREAKING 6000 // Max value of gyro when braking. anything more will be emergency braking

#define MIN_GYRO_ACCELERATING 1000 // buffer area where gyro wont do anything
#define MAX_GYRO_ACCELERATING 5000 // maximum value of gyro when accelerating

#define EXPECTED_ACC_MAGNITUDE 16800.0 // expected value of acceleration when stationary (gravity only). Defaults to this if calibration fails
#define CALIBRATION_SAMPLE_SIZE 500 // sample size for calibration phase
#define CALIBRATION_ACC_DELTA 2000 // ensures that the acc found during calibration is within this of EXPECTED_ACC_MAGNITUDE


// Filtering and smoothing

// First, it will check the raw value of the acceleration. If this value is too dissimilar from the previous value,
// the data will be disregarded. It automatically detect errors, overriding bumps if it detects continuous bumps for x amount of time.
// side note, there is a bug where it gets in an infinite loop if multiple overrides stack. For this reason there is a minimum time between overrides
#define FILTER_DELTA false

#define BUMP_THRESHOLD 1000 // if the acceleration delta (changes) by this much within a tick, its probably a bump
#define SAMPLE_SIZE_BUMPS 20 // if there are this many bumps in a row that are within BUMP_THRESHHOLD of each other, its not a bump, so override

#define BUMP_OVERRIDE_TIME 3000 // force an update if bumps are detected for this many millis
#define MIN_TIME_BETWEEN_OVERRIDES 6000 // wont override consistently. Should be larger than BUMP_OVERRIDE_TIME


// Secondly, the program takes a sample size of gyro values
// It will take the average of AVG_SAMPLE_SIZE data points, and calculate the average
#define FILTER_AVG true
#define AVG_SAMPLE_SIZE 15


// Finaly, it smoothed the previous and new values together instead of just setting them
#define FILTER_SMOOTHING true
#define SMOOTHING_FACTOR 0.3 // lower value is more smoothing, less vibrations, but less reactive/fast

class Gyro {
private:
    const int MPU = 0x68; // MPU6050 I2C address

    unsigned long lastUpdateTime = 0;
    unsigned long lastForceUpdate = 0;
    float idleAcc = 0.0;

    float measuredAccX; // left and right
    float measuredAccY; // forward+ backwards- (ASSUMING BOARD IS FACING UP)
    float measuredAccZ; // up and down

    float measuredGyroX;
    float measuredGyroY;
    float measuredGyroZ;

    float minBump = 100000.0;
    float maxBump = -100000.0;
    int numBumps = 0;

    int numSamples = 0;
    float sumSamples = 0.0;
    float avgAcc = 0.0;

    float correctedAcc = 0.0; // disregarding idle gravity

public:
    float prevAcc = 0.0; // previous corrected acceleration, used to filter out bumps
    float smoothedAcc = 0.0; // smoothed out acceleration for dealing with bumps and irregularities, signed
    Gyro();
    void update();
};