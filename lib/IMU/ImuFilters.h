#ifndef IMU_FILTERS_H
#define IMU_FILTERS_H

// Signal filters for the IMU pipeline.
//
// These are direct ports of the filters PX4 uses on its IMU path
// (src/lib/mathlib/math/filter/*.hpp), rewritten without the matrix/px4
// dependencies so they build for both ESP32 and the host test harness.
//
//   LowPassFilter2p  <- PX4 LowPassFilter2p.hpp   (2nd order Butterworth, DF-II)
//   NotchFilter      <- PX4 NotchFilter.hpp       (2nd order notch, DF-I)
//   MedianFilter3    <- PX4 MedianFilter.hpp      (window 3 spike rejection)
//   AlphaFilter      <- PX4 AlphaFilter.hpp       (1st order, time-constant based)
//
// All filters are "disabled" (pass-through) when their cutoff is invalid, which
// is how PX4 behaves and keeps the pipeline safe if a parameter is nonsense.

#include <math.h>
#include <string.h>
#include <cstdint>

#include "ImuMath.h"

namespace imu
{

// ---------------------------------------------------------------------------
// 2nd order Butterworth low-pass, Direct Form II (PX4 LowPassFilter2p)
// ---------------------------------------------------------------------------
class LowPassFilter2p
{
public:
    void setCutoffFrequency(float sampleHz, float cutoffHz);
    void reset();
    float apply(float sample);
    bool enabled() const { return _enabled; }
    float cutoffFrequency() const { return _cutoffHz; }

private:
    float _b0 = 1.0f, _b1 = 0.0f, _b2 = 0.0f;
    float _a1 = 0.0f, _a2 = 0.0f;
    float _delay1 = 0.0f, _delay2 = 0.0f;
    float _cutoffHz = 0.0f;
    float _sampleHz = 0.0f;
    bool _enabled = false;
};

class LowPassFilter2pVec3
{
public:
    void setCutoffFrequency(float sampleHz, float cutoffHz);
    void reset();
    Vec3 apply(const Vec3 &sample);
    bool enabled() const { return _x.enabled(); }

private:
    LowPassFilter2p _x, _y, _z;
};

// ---------------------------------------------------------------------------
// 2nd order notch, Direct Form I (PX4 NotchFilter)
// ---------------------------------------------------------------------------
class NotchFilter
{
public:
    // Returns false (and disables the filter) when the parameters are invalid.
    bool setParameters(float sampleHz, float notchHz, float bandwidthHz);
    void reset();
    void disable();
    float apply(float sample);
    bool enabled() const { return _enabled; }
    float notchFrequency() const { return _notchHz; }

private:
    float applyInternal(float sample);

    float _delayIn1 = 0.0f, _delayIn2 = 0.0f;
    float _delayOut1 = 0.0f, _delayOut2 = 0.0f;
    float _a1 = 0.0f, _a2 = 0.0f;
    float _b0 = 1.0f, _b1 = 0.0f, _b2 = 0.0f;
    float _notchHz = 0.0f;
    float _bandwidthHz = 0.0f;
    float _sampleHz = 0.0f;
    bool _enabled = false;
};

class NotchFilterVec3
{
public:
    bool setParameters(float sampleHz, float notchHz, float bandwidthHz);
    void reset();
    Vec3 apply(const Vec3 &sample);
    bool enabled() const { return _x.enabled(); }

private:
    NotchFilter _x, _y, _z;
};

// ---------------------------------------------------------------------------
// Median spike rejection (PX4 MedianFilter, window 3)
// ---------------------------------------------------------------------------
class MedianFilter3
{
public:
    void reset();
    float apply(float sample);
    float median() const;

private:
    float _buffer[3] = {0.0f, 0.0f, 0.0f};
    uint8_t _head = 0;
    uint8_t _count = 0;
};

class MedianFilter3Vec3
{
public:
    void reset();
    Vec3 apply(const Vec3 &sample);

private:
    MedianFilter3 _x, _y, _z;
};

// ---------------------------------------------------------------------------
// 1st order alpha filter with a physical time constant (PX4 AlphaFilter)
// ---------------------------------------------------------------------------
class AlphaFilter
{
public:
    void setTimeConstant(float timeConstantSeconds) { _timeConstant = timeConstantSeconds; }
    void setParameters(float sampleIntervalSeconds, float timeConstantSeconds);
    void reset(float value);
    float apply(float sample, float dtSeconds);
    float state() const { return _state; }

private:
    float _timeConstant = 0.0f;
    float _alpha = 0.0f;
    float _state = 0.0f;
};

// ---------------------------------------------------------------------------
// Composed pipelines
// ---------------------------------------------------------------------------

// Gyro path: median(3) -> notch -> 2nd order low-pass.
// This mirrors PX4's vehicle_angular_velocity chain (notch bank then LPF).
class GyroFilterPipeline
{
public:
    void configure(float sampleHz, float cutoffHz, float notchHz, float notchBandwidthHz);
    void reset();
    Vec3 apply(const Vec3 &sample);
    bool lowPassEnabled() const { return _lowPass.enabled(); }
    bool notchEnabled() const { return _notch.enabled(); }

private:
    MedianFilter3Vec3 _median;
    NotchFilterVec3 _notch;
    LowPassFilter2pVec3 _lowPass;
};

// Accelerometer path: 2nd order low-pass only, like PX4's vehicle_acceleration.
class AccelFilterPipeline
{
public:
    void configure(float sampleHz, float cutoffHz);
    void reset();
    Vec3 apply(const Vec3 &sample);
    bool enabled() const { return _lowPass.enabled(); }

private:
    LowPassFilter2pVec3 _lowPass;
};

// Magnetometer path: light low-pass to tame AK8963 noise without adding
// meaningful phase lag at the 100 Hz mag output rate.
class MagFilterPipeline
{
public:
    void configure(float sampleHz, float cutoffHz);
    void reset();
    Vec3 apply(const Vec3 &sample);
    bool enabled() const { return _lowPass.enabled(); }

private:
    LowPassFilter2pVec3 _lowPass;
};

} // namespace imu

#endif // IMU_FILTERS_H
