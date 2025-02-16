#pragma once
#include <Wire.h>
#include "button.h"
// green light on gyro towards back

// divide all these values by 17000 to get the amount in g force
#define MIN_GYRO_BREAKING 500 // Buffer area where gyro wont do anything
#define MAX_GYRO_BREAKING 5000 // Max value of gyro when braking. anything more will be emergency braking

#define MIN_GYRO_ACCELERATING 500 // buffer area where gyro wont do anything
#define MAX_GYRO_ACCELERATING 5000 // maximum value of gyro when accelerating

#define EXPECTED_ACC_MAGNITUDE 16800.0 // expected value of acceleration when stationary (gravity only). Defaults to this if calibration fails
#define CALIBRATION_SAMPLE_SIZE 500 // sample size for calibration phase
#define CALIBRATION_ACC_DELTA 2000 // ensures that the acc found during calibration is within this of EXPECTED_ACC_MAGNITUDE

#define HILL_SAMPLE_SIZE 200 // higher value is more accurate, but slower. Used to detect hills


// Filtering and smoothing

// First, it will check the raw value of the acceleration. If this value is too dissimilar from the previous value,
// the data will be disregarded. It automatically detect errors, overriding bumps if it detects continuous bumps for x amount of time.
// side note, there is a bug where it gets in an infinite loop if multiple overrides stack. For this reason there is a minimum time between overrides
#define FILTER_DELTA true
#define BUMP_THRESHOLD 3000 // if the acceleration delta (changes) by this much within a tick, its probably a bump
#define BUMP_OVERRIDE 250 // Will overide the bump if it happens for this long

// Secondly, the program takes a sample size of gyro values
// It will take the average of AVG_SAMPLE_SIZE data points, and calculate the average
#define FILTER_AVG true
#define AVG_SAMPLE_SIZE 15

// Finaly, it smoothed the previous and new values together instead of just setting them
#define FILTER_SMOOTHING true
#define SMOOTHING_FACTOR 0.3 // lower value is more smoothing, less vibrations, but less reactive/fast
#define HILL_SMOOTHING_FACTOR 0.1 // lower value is more smoothing, less vibrations, but less reactive/fast

class Gyro {
private:
    Button *button;
    
    const int MPU = 0x68; // MPU6050 I2C address

    unsigned long lastUpdateTime = 0; // last time the gyro was updated
    float idleAcc = 0.0; // idle acceleration, used to calculate the correction factor by noting what gravity SHOULD be

    float measuredAccX = 0.0; // raw acceleration values
    float measuredAccY = 0.0;
    float measuredAccZ = 0.0;

    float measuredGyroX = 0.0; // raw gyro values
    float measuredGyroY = 0.0;
    float measuredGyroZ = 0.0;

    float sumHillSamplesX = 0.0; // used to calculate the correction factor
    float sumHillSamplesY = 0.0;
    float sumHillSamplesZ = 0.0;
    int numHillSamples = 0;
    float correction = 0.0; // correction factor for the acceleration

    float temp;

    float correctedAcc = 0.0; // disregarding idle gravity
    float prevMeasuredAccZ = 0.0; // previous corrected acceleration, used to filter out bumps

public:
    float prevSmoothedAcc = 0.0;
    float smoothedAcc = 0.0; // smoothed out acceleration for dealing with bumps and irregularities, signed
    Gyro(Button *button);
    void update();
};