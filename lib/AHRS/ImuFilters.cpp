#include "ImuFilters.h"
#include <math.h>

void MedianFilter3::reset()
{
    memset(history, 0, sizeof(history));
    count = 0;
}

void MedianFilter3::update(const float input[3], float output[3])
{
    for (uint8_t axis = 0; axis < 3; ++axis)
    {
        history[axis][2] = history[axis][1];
        history[axis][1] = history[axis][0];
        history[axis][0] = input[axis];
        float values[3] = {history[axis][0], history[axis][1], history[axis][2]};
        if (values[0] > values[1]) { float t = values[0]; values[0] = values[1]; values[1] = t; }
        if (values[1] > values[2]) { float t = values[1]; values[1] = values[2]; values[2] = t; }
        if (values[0] > values[1]) { float t = values[0]; values[0] = values[1]; values[1] = t; }
        output[axis] = count < 2 ? input[axis] : values[1];
    }
    if (count < 3) ++count;
}

void ButterworthFilter3::configure(float cutoffHz, float sampleHz)
{
    const float omega = 2.0f * PI * cutoffHz / sampleHz;
    const float sine = sinf(omega);
    const float cosine = cosf(omega);
    const float alpha = sine / (2.0f * 0.70710678f);
    const float a0 = 1.0f + alpha;
    b0 = (1.0f - cosine) * 0.5f / a0;
    b1 = (1.0f - cosine) / a0;
    b2 = b0;
    a1 = -2.0f * cosine / a0;
    a2 = (1.0f - alpha) / a0;
    reset();
}

void ButterworthFilter3::reset()
{
    memset(state, 0, sizeof(state));
    initialized = false;
}

void ButterworthFilter3::update(const float input[3], float output[3])
{
    for (uint8_t axis = 0; axis < 3; ++axis)
    {
        if (!initialized)
        {
            state[axis][0] = input[axis];
            state[axis][1] = input[axis];
        }
        const float result = b0 * input[axis] + state[axis][0];
        state[axis][0] = b1 * input[axis] - a1 * result + state[axis][1];
        state[axis][1] = b2 * input[axis] - a2 * result;
        output[axis] = result;
    }
    initialized = true;
}

void AdaptiveKalman3::configure(float processNoiseValue,
                                float measurementNoiseValue,
                                float innovationScaleValue)
{
    processNoise = processNoiseValue;
    measurementNoise = measurementNoiseValue;
    innovationScale = innovationScaleValue;
    reset();
}

void AdaptiveKalman3::reset()
{
    memset(estimate, 0, sizeof(estimate));
    for (float &value : covariance) value = measurementNoise;
    initialized = false;
}

void AdaptiveKalman3::update(const float input[3], float output[3])
{
    for (uint8_t axis = 0; axis < 3; ++axis)
    {
        if (!initialized)
        {
            estimate[axis] = input[axis];
            covariance[axis] = measurementNoise;
            output[axis] = input[axis];
            continue;
        }
        covariance[axis] += processNoise;
        const float innovation = input[axis] - estimate[axis];
        const float adaptiveMeasurementNoise =
            measurementNoise + innovationScale * innovation * innovation;
        const float gain = covariance[axis] /
                           (covariance[axis] + adaptiveMeasurementNoise);
        estimate[axis] += gain * innovation;
        covariance[axis] = (1.0f - gain) * covariance[axis];
        output[axis] = estimate[axis];
    }
    initialized = true;
}

void ImuFilterPipeline::configure(float cutoffHz, float sampleHz,
                                  float processNoise, float measurementNoise)
{
    median.reset();
    butterworth.configure(cutoffHz, sampleHz);
    adaptiveKalman.configure(processNoise, measurementNoise, 4.0f);
}

void ImuFilterPipeline::reset()
{
    median.reset();
    butterworth.reset();
    adaptiveKalman.reset();
}

void ImuFilterPipeline::update(const float input[3], float output[3])
{
    float medianOutput[3];
    float butterworthOutput[3];
    median.update(input, medianOutput);
    butterworth.update(medianOutput, butterworthOutput);
    adaptiveKalman.update(butterworthOutput, output);
}
