#include "ImuFilters.h"

namespace imu
{

// ===========================================================================
// LowPassFilter2p  (port of PX4 src/lib/mathlib/math/filter/LowPassFilter2p.hpp)
// ===========================================================================
void LowPassFilter2p::setCutoffFrequency(float sampleHz, float cutoffHz)
{
    if (sampleHz <= 0.0f || cutoffHz <= 0.0f || cutoffHz >= sampleHz * 0.5f ||
        !isFinitef(sampleHz) || !isFinitef(cutoffHz))
    {
        _enabled = false;
        _b0 = 1.0f;
        _b1 = _b2 = _a1 = _a2 = 0.0f;
        _delay1 = _delay2 = 0.0f;
        return;
    }

    _delay1 = 0.0f;
    _delay2 = 0.0f;

    _cutoffHz = fmaxf(cutoffHz, sampleHz * 0.001f);
    _sampleHz = sampleHz;

    const float fr = _sampleHz / _cutoffHz;
    const float ohm = tanf(kPi / fr);
    const float c = 1.0f + 2.0f * cosf(kPi / 4.0f) * ohm + ohm * ohm;

    _b0 = ohm * ohm / c;
    _b1 = 2.0f * _b0;
    _b2 = _b0;
    _a1 = 2.0f * (ohm * ohm - 1.0f) / c;
    _a2 = (1.0f - 2.0f * cosf(kPi / 4.0f) * ohm + ohm * ohm) / c;

    _enabled = isFinitef(_b0) && isFinitef(_b1) && isFinitef(_b2) && isFinitef(_a1) &&
               isFinitef(_a2);
    if (!_enabled)
    {
        _b0 = 1.0f;
        _b1 = _b2 = _a1 = _a2 = 0.0f;
    }
}

void LowPassFilter2p::reset()
{
    _delay1 = 0.0f;
    _delay2 = 0.0f;
}

float LowPassFilter2p::apply(float sample)
{
    if (!_enabled)
        return sample;

    const float delay0 = sample - _delay1 * _a1 - _delay2 * _a2;
    const float output = delay0 * _b0 + _delay1 * _b1 + _delay2 * _b2;
    _delay2 = _delay1;
    _delay1 = delay0;
    return output;
}

void LowPassFilter2pVec3::setCutoffFrequency(float sampleHz, float cutoffHz)
{
    _x.setCutoffFrequency(sampleHz, cutoffHz);
    _y.setCutoffFrequency(sampleHz, cutoffHz);
    _z.setCutoffFrequency(sampleHz, cutoffHz);
}

void LowPassFilter2pVec3::reset()
{
    _x.reset();
    _y.reset();
    _z.reset();
}

Vec3 LowPassFilter2pVec3::apply(const Vec3 &sample)
{
    return {_x.apply(sample.x), _y.apply(sample.y), _z.apply(sample.z)};
}

// ===========================================================================
// NotchFilter  (port of PX4 src/lib/mathlib/math/filter/NotchFilter.hpp)
// ===========================================================================
bool NotchFilter::setParameters(float sampleHz, float notchHz, float bandwidthHz)
{
    if (sampleHz <= 0.0f || notchHz <= 0.0f || bandwidthHz <= 0.0f ||
        notchHz >= sampleHz * 0.5f || !isFinitef(sampleHz) || !isFinitef(notchHz) ||
        !isFinitef(bandwidthHz))
    {
        disable();
        return false;
    }

    const float freqMin = sampleHz * 0.001f;
    const float notchNew = fmaxf(notchHz, freqMin);
    const float bandwidthNew = fmaxf(bandwidthHz, freqMin);

    const float sampleDiff = fabsf(sampleHz - _sampleHz);
    const float bandwidthDiff = fabsf(bandwidthNew - _bandwidthHz);
    const bool sampleChanged = sampleDiff > 1e-7f;
    const bool bandwidthChanged = bandwidthDiff > 1e-7f;

    if (!sampleChanged && !bandwidthChanged)
    {
        const float notchDiff = fabsf(notchNew - _notchHz);
        if (notchDiff <= 1e-7f)
            return true;

        // Only the notch frequency moved: keep the filter history.
        _notchHz = notchNew;
        const float beta = -cosf(kTwoPi * _notchHz / _sampleHz);
        _b1 = 2.0f * beta * _b0;
        _a1 = _b1;
        if (notchDiff > _bandwidthHz)
            _enabled = false;
        if (!isFinitef(_b1))
        {
            disable();
            return false;
        }
        return true;
    }

    _sampleHz = sampleHz;
    _notchHz = notchNew;
    _bandwidthHz = bandwidthNew;

    const float alpha = tanf(kPi * _bandwidthHz / _sampleHz);
    const float beta = -cosf(kTwoPi * _notchHz / _sampleHz);
    const float a0Inverse = 1.0f / (alpha + 1.0f);

    _b0 = a0Inverse;
    _b1 = 2.0f * beta * a0Inverse;
    _b2 = a0Inverse;
    _a1 = _b1;
    _a2 = (1.0f - alpha) * a0Inverse;

    if (!isFinitef(_b0) || !isFinitef(_b1) || !isFinitef(_b2) || !isFinitef(_a2))
    {
        disable();
        return false;
    }

    if (bandwidthDiff > 0.01f * _bandwidthHz || sampleDiff > 0.01f * _sampleHz)
        _enabled = false;

    return true;
}

void NotchFilter::reset()
{
    _enabled = false;
}

void NotchFilter::disable()
{
    _notchHz = 0.0f;
    _bandwidthHz = 0.0f;
    _sampleHz = 0.0f;
    _b0 = 1.0f;
    _b1 = _b2 = _a1 = _a2 = 0.0f;
    _enabled = false;
}

float NotchFilter::apply(float sample)
{
    if (!_enabled)
    {
        // PX4 seeds the delay line with the first sample so the filter does not
        // start with a transient; do the same and then run normally.
        const float input = isFinitef(sample) ? sample : 0.0f;
        _delayIn1 = _delayIn2 = input;
        const float denominator = 1.0f + _a1 + _a2;
        const float seed = (fabsf(denominator) > 1e-9f)
                               ? input * (_b0 + _b1 + _b2) / denominator
                               : input;
        _delayOut1 = _delayOut2 = seed;
        _enabled = true;
    }
    return applyInternal(sample);
}

float NotchFilter::applyInternal(float sample)
{
    const float output = _b0 * sample + _b1 * _delayIn1 + _b2 * _delayIn2 -
                         _a1 * _delayOut1 - _a2 * _delayOut2;
    _delayIn2 = _delayIn1;
    _delayIn1 = sample;
    _delayOut2 = _delayOut1;
    _delayOut1 = output;
    return output;
}

bool NotchFilterVec3::setParameters(float sampleHz, float notchHz, float bandwidthHz)
{
    const bool x = _x.setParameters(sampleHz, notchHz, bandwidthHz);
    const bool y = _y.setParameters(sampleHz, notchHz, bandwidthHz);
    const bool z = _z.setParameters(sampleHz, notchHz, bandwidthHz);
    return x && y && z;
}

void NotchFilterVec3::reset()
{
    _x.reset();
    _y.reset();
    _z.reset();
}

Vec3 NotchFilterVec3::apply(const Vec3 &sample)
{
    return {_x.apply(sample.x), _y.apply(sample.y), _z.apply(sample.z)};
}

// ===========================================================================
// MedianFilter3  (port of PX4 src/lib/mathlib/math/filter/MedianFilter.hpp)
// ===========================================================================
void MedianFilter3::reset()
{
    _buffer[0] = _buffer[1] = _buffer[2] = 0.0f;
    _head = 0;
    _count = 0;
}

float MedianFilter3::apply(float sample)
{
    _head = static_cast<uint8_t>((_head + 1) % 3);
    _buffer[_head] = sample;
    if (_count < 3)
        ++_count;
    if (_count < 3)
        return sample;
    return median();
}

float MedianFilter3::median() const
{
    float a = _buffer[0], b = _buffer[1], c = _buffer[2];
    if (a > b)
    {
        const float swap = a;
        a = b;
        b = swap;
    }
    if (b > c)
    {
        const float swap = b;
        b = c;
        c = swap;
    }
    if (a > b)
    {
        const float swap = a;
        a = b;
        b = swap;
    }
    return b;
}

void MedianFilter3Vec3::reset()
{
    _x.reset();
    _y.reset();
    _z.reset();
}

Vec3 MedianFilter3Vec3::apply(const Vec3 &sample)
{
    return {_x.apply(sample.x), _y.apply(sample.y), _z.apply(sample.z)};
}

// ===========================================================================
// AlphaFilter  (port of PX4 src/lib/mathlib/math/filter/AlphaFilter.hpp)
// ===========================================================================
void AlphaFilter::setParameters(float sampleIntervalSeconds, float timeConstantSeconds)
{
    const float denominator = timeConstantSeconds + sampleIntervalSeconds;
    if (denominator > 1e-7f)
        _alpha = sampleIntervalSeconds / denominator;
    _timeConstant = timeConstantSeconds;
}

void AlphaFilter::reset(float value)
{
    _state = value;
}

float AlphaFilter::apply(float sample, float dtSeconds)
{
    setParameters(dtSeconds, _timeConstant);
    _state = _state + _alpha * (sample - _state);
    return _state;
}

// ===========================================================================
// Pipelines
// ===========================================================================
void GyroFilterPipeline::configure(float sampleHz, float cutoffHz, float notchHz,
                                   float notchBandwidthHz)
{
    _median.reset();
    _notch.reset();
    _notch.setParameters(sampleHz, notchHz, notchBandwidthHz);
    _lowPass.setCutoffFrequency(sampleHz, cutoffHz);
}

void GyroFilterPipeline::reset()
{
    _median.reset();
    _notch.reset();
    _lowPass.reset();
}

Vec3 GyroFilterPipeline::apply(const Vec3 &sample)
{
    return _lowPass.apply(_notch.apply(_median.apply(sample)));
}

void AccelFilterPipeline::configure(float sampleHz, float cutoffHz)
{
    _lowPass.setCutoffFrequency(sampleHz, cutoffHz);
}

void AccelFilterPipeline::reset()
{
    _lowPass.reset();
}

Vec3 AccelFilterPipeline::apply(const Vec3 &sample)
{
    return _lowPass.apply(sample);
}

void MagFilterPipeline::configure(float sampleHz, float cutoffHz)
{
    _lowPass.setCutoffFrequency(sampleHz, cutoffHz);
}

void MagFilterPipeline::reset()
{
    _lowPass.reset();
}

Vec3 MagFilterPipeline::apply(const Vec3 &sample)
{
    return _lowPass.apply(sample);
}

} // namespace imu
