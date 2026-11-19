#ifndef IMU_FILTERS_H
#define IMU_FILTERS_H

#include <Arduino.h>

class MedianFilter5
{
public:
    void reset();
    void update(const float input[3], float output[3]);
private:
    float history[3][5] = {};
    uint8_t count = 0;
};

class ButterworthFilter3
{
public:
    void configure(float cutoffHz, float sampleHz);
    void reset();
    void update(const float input[3], float output[3]);
private:
    float b0 = 1.0f, b1 = 0.0f, b2 = 0.0f, a1 = 0.0f, a2 = 0.0f;
    float state[3][2] = {};
    bool initialized = false;
};

class AdaptiveKalman3
{
public:
    void configure(float processNoise, float measurementNoise,
                   float innovationScale);
    void reset();
    void update(const float input[3], float output[3]);
private:
    float processNoise = 0.001f;
    float measurementNoise = 0.05f;
    float innovationScale = 4.0f;
    float estimate[3] = {};
    float covariance[3] = {};
    bool initialized = false;
};

class ImuFilterPipeline
{
public:
    void configure(float cutoffHz, float sampleHz, float processNoise,
                   float measurementNoise);
    void reset();
    void update(const float input[3], float output[3]);
private:
    MedianFilter5 median;
    ButterworthFilter3 butterworth;
    AdaptiveKalman3 adaptiveKalman;
};

#endif
