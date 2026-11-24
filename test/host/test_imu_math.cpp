// Host-side tests for the IMU math, filters, estimator and calibration.
//
// These compile the exact same sources the firmware uses (lib/IMU/ImuMath.h,
// ImuFilters.*, ImuAttitudeEstimator.*, ImuCalibration.*, ImuRotation.*) with a
// normal desktop compiler, so a regression in the math shows up without
// flashing the robot.
//
// Build & run:
//   test/host/run.sh
//
// The test binary is deliberately dependency-free (no gtest, no Arduino).

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "../../lib/IMU/ImuMath.h"
#include "../../lib/IMU/ImuFilters.h"
#include "../../lib/IMU/ImuAttitudeEstimator.h"
#include "../../lib/IMU/ImuCalibration.h"
#include "../../lib/IMU/ImuRotation.h"
#include "../../include/config.h"

namespace
{

int gFailures = 0;
int gChecks = 0;

void check(bool condition, const char *what)
{
    ++gChecks;
    if (!condition)
    {
        ++gFailures;
        printf("  FAIL: %s\n", what);
    }
}

void checkNear(float actual, float expected, float tolerance, const char *what)
{
    ++gChecks;
    if (!(fabsf(actual - expected) <= tolerance))
    {
        ++gFailures;
        printf("  FAIL: %s (got %.6f, expected %.6f +- %.6f)\n", what, actual, expected,
               tolerance);
    }
}

void section(const char *name)
{
    printf("\n[%s]\n", name);
}

// ---------------------------------------------------------------------------
// Quaternion / matrix basics
// ---------------------------------------------------------------------------
void testQuaternionBasics()
{
    section("quaternion basics");

    // Identity rotates nothing.
    const imu::Quat identity;
    const imu::Vec3 v{1.0f, 2.0f, 3.0f};
    const imu::Vec3 rotated = identity.rotate(v);
    checkNear(rotated.x, 1.0f, 1e-6f, "identity rotate x");
    checkNear(rotated.y, 2.0f, 1e-6f, "identity rotate y");
    checkNear(rotated.z, 3.0f, 1e-6f, "identity rotate z");

    // 90 deg about Z maps +X to +Y.
    const float half = imu::kPi * 0.25f;
    const imu::Quat yaw90{cosf(half), 0.0f, 0.0f, sinf(half)};
    const imu::Vec3 xAxis{1.0f, 0.0f, 0.0f};
    const imu::Vec3 mapped = yaw90.rotate(xAxis);
    checkNear(mapped.x, 0.0f, 1e-6f, "yaw90 x->y (x)");
    checkNear(mapped.y, 1.0f, 1e-6f, "yaw90 x->y (y)");
    checkNear(mapped.z, 0.0f, 1e-6f, "yaw90 x->y (z)");

    // rotateInverse undoes rotate.
    const imu::Vec3 back = yaw90.rotateInverse(mapped);
    checkNear(back.x, 1.0f, 1e-6f, "rotateInverse round trip x");
    checkNear(back.y, 0.0f, 1e-6f, "rotateInverse round trip y");

    // Matrix <-> quaternion round trip.
    const imu::Mat3 matrix = yaw90.toRotationMatrix();
    const imu::Quat rebuilt = imu::Quat::fromRotationMatrix(matrix);
    const imu::Vec3 rebuiltMapped = rebuilt.rotate(xAxis);
    checkNear(rebuiltMapped.y, 1.0f, 1e-5f, "matrix->quat round trip");

    // Euler extraction matches the project convention.
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
    yaw90.toEulerDegrees(roll, pitch, yaw);
    checkNear(roll, 0.0f, 1e-4f, "yaw90 euler roll");
    checkNear(pitch, 0.0f, 1e-4f, "yaw90 euler pitch");
    checkNear(yaw, 90.0f, 1e-4f, "yaw90 euler yaw");
}

// ---------------------------------------------------------------------------
// The project rotation contract must stay bit-compatible
// ---------------------------------------------------------------------------
void testRotationContract()
{
    section("rotation contract (rotateBodyToWorld)");

    // Level: world == body.
    {
        const float body[3] = {1.0f, 2.0f, 3.0f};
        float world[3];
        rotateBodyToWorld(0.0f, 0.0f, 0.0f, body, world);
        checkNear(world[0], 1.0f, 1e-6f, "level x");
        checkNear(world[1], 2.0f, 1e-6f, "level y");
        checkNear(world[2], 3.0f, 1e-6f, "level z");
    }

    // Roll 90: body +Y goes to world +Z, body +Z goes to world -Y.
    {
        const float bodyY[3] = {0.0f, 1.0f, 0.0f};
        float world[3];
        rotateBodyToWorld(90.0f, 0.0f, 0.0f, bodyY, world);
        checkNear(world[0], 0.0f, 1e-5f, "roll90 +Y -> x");
        checkNear(world[1], 0.0f, 1e-5f, "roll90 +Y -> y");
        checkNear(world[2], 1.0f, 1e-5f, "roll90 +Y -> z");

        const float bodyZ[3] = {0.0f, 0.0f, 1.0f};
        rotateBodyToWorld(90.0f, 0.0f, 0.0f, bodyZ, world);
        checkNear(world[1], -1.0f, 1e-5f, "roll90 +Z -> -Y");
    }

    // Yaw 90: body +X goes to world +Y.
    {
        const float bodyX[3] = {1.0f, 0.0f, 0.0f};
        float world[3];
        rotateBodyToWorld(0.0f, 0.0f, 90.0f, bodyX, world);
        checkNear(world[0], 0.0f, 1e-5f, "yaw90 +X -> x");
        checkNear(world[1], 1.0f, 1e-5f, "yaw90 +X -> y");
    }

    // The matrix form must agree with the sequential form.
    {
        const float body[3] = {0.3f, -1.2f, 2.4f};
        float sequential[3];
        rotateBodyToWorld(17.0f, -23.0f, 41.0f, body, sequential);
        const imu::Mat3 matrix = bodyToWorldMatrix(17.0f, -23.0f, 41.0f);
        const imu::Vec3 viaMatrix = matrix * imu::Vec3{body};
        checkNear(viaMatrix.x, sequential[0], 1e-5f, "matrix vs sequential x");
        checkNear(viaMatrix.y, sequential[1], 1e-5f, "matrix vs sequential y");
        checkNear(viaMatrix.z, sequential[2], 1e-5f, "matrix vs sequential z");
    }

    // Gravity removal: at rest, level, the world Z component must be +1 g.
    {
        const float accel[3] = {0.0f, 0.0f, imu::kGravity};
        float world[3];
        rotateBodyToWorld(0.0f, 0.0f, 0.0f, accel, world);
        checkNear(world[2], imu::kGravity, 1e-4f, "rest gravity in world Z");
    }
}

// ---------------------------------------------------------------------------
// Estimator initialization must recover a known attitude exactly
// ---------------------------------------------------------------------------
void testEstimatorInitialization()
{
    section("estimator initialization");

    // Build the body-frame accel/mag for a known body->world attitude, then
    // check that initialize() recovers that attitude. This is the test that
    // catches a frame-handedness mistake (e.g. using PX4's east = down x north
    // in a Y-left frame, which yields a 180 deg roll error).
    struct Case
    {
        float roll, pitch, yaw;
    };
    static const Case cases[] = {
        {0.0f, 0.0f, 0.0f},     {30.0f, 0.0f, 0.0f},   {0.0f, 20.0f, 0.0f},
        {0.0f, 0.0f, 45.0f},    {15.0f, -10.0f, 120.0f}, {-25.0f, 35.0f, -70.0f},
        {0.0f, 0.0f, -179.0f},  {60.0f, 45.0f, 10.0f},
    };

    const imu::Vec3 worldAccel{0.0f, 0.0f, imu::kGravity};
    const imu::Vec3 worldMag{25.0f, 0.0f, -40.0f};

    for (const Case &c : cases)
    {
        const imu::Mat3 rotation = bodyToWorldMatrix(c.roll, c.pitch, c.yaw);
        const imu::Mat3 inverse = rotation.transposed();
        const imu::Vec3 bodyAccel = inverse * worldAccel;
        const imu::Vec3 bodyMag = inverse * worldMag;

        imu::AttitudeEstimator estimator;
        imu::AttitudeEstimatorConfig config;
        estimator.configure(config);
        check(estimator.initialize(bodyAccel, bodyMag), "initialize from known attitude");

        float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
        estimator.eulerDegrees(roll, pitch, yaw);
        checkNear(roll, c.roll, 0.05f, "init roll matches truth");
        checkNear(pitch, c.pitch, 0.05f, "init pitch matches truth");
        // Compare yaw on the circle so -179 and 181 are the same angle.
        checkNear(imu::wrapDeg(yaw - c.yaw), 0.0f, 0.05f, "init yaw matches truth");
    }

    // Gravity-only fallback (no usable magnetometer) must at least level.
    {
        imu::AttitudeEstimator estimator;
        imu::AttitudeEstimatorConfig config;
        estimator.configure(config);
        const imu::Vec3 bodyAccel{0.0f, imu::kGravity * sinf(0.4f),
                                  imu::kGravity * cosf(0.4f)};
        check(estimator.initialize(bodyAccel, imu::Vec3{}), "initialize without mag");
        float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
        estimator.eulerDegrees(roll, pitch, yaw);
        checkNear(roll, 0.4f * imu::kRadToDeg, 0.5f, "gravity-only fallback levels roll");
        checkNear(pitch, 0.0f, 0.5f, "gravity-only fallback levels pitch");
    }
}

// ---------------------------------------------------------------------------
// Filters
// ---------------------------------------------------------------------------
void testFilters()
{
    section("filters");

    // Low-pass: DC gain must be 1.
    {
        imu::LowPassFilter2p filter;
        filter.setCutoffFrequency(200.0f, 30.0f);
        check(filter.enabled(), "low-pass enabled");
        float output = 0.0f;
        for (int i = 0; i < 2000; ++i)
            output = filter.apply(1.0f);
        checkNear(output, 1.0f, 1e-3f, "low-pass DC gain");
    }

    // Low-pass: a 100 Hz tone at a 30 Hz cutoff must be strongly attenuated.
    {
        imu::LowPassFilter2p filter;
        filter.setCutoffFrequency(200.0f, 30.0f);
        float peak = 0.0f;
        for (int i = 0; i < 4000; ++i)
        {
            const float sample = sinf(2.0f * imu::kPi * 100.0f * i / 200.0f);
            const float output = filter.apply(sample);
            if (i > 2000)
                peak = fmaxf(peak, fabsf(output));
        }
        check(peak < 0.15f, "low-pass attenuates 100 Hz");
    }

    // Invalid cutoff disables the filter (pass-through), like PX4.
    {
        imu::LowPassFilter2p filter;
        filter.setCutoffFrequency(200.0f, 0.0f);
        check(!filter.enabled(), "zero cutoff disables low-pass");
        checkNear(filter.apply(3.5f), 3.5f, 1e-6f, "disabled low-pass is pass-through");
    }

    // Median(3) rejects a single spike.
    {
        imu::MedianFilter3 median;
        median.apply(1.0f);
        median.apply(1.0f);
        const float out = median.apply(100.0f);
        checkNear(out, 1.0f, 1e-6f, "median rejects spike");
    }

    // Notch: attenuates its centre frequency, passes DC.
    {
        imu::NotchFilter notch;
        check(notch.setParameters(200.0f, 50.0f, 20.0f), "notch parameters accepted");
        float peak = 0.0f;
        for (int i = 0; i < 4000; ++i)
        {
            const float sample = sinf(2.0f * imu::kPi * 50.0f * i / 200.0f);
            const float output = notch.apply(sample);
            if (i > 2000)
                peak = fmaxf(peak, fabsf(output));
        }
        check(peak < 0.2f, "notch attenuates centre frequency");

        imu::NotchFilter dc;
        dc.setParameters(200.0f, 50.0f, 20.0f);
        float output = 0.0f;
        for (int i = 0; i < 2000; ++i)
            output = dc.apply(1.0f);
        checkNear(output, 1.0f, 1e-2f, "notch passes DC");
    }

    // Alpha filter converges to the input.
    {
        imu::AlphaFilter alpha;
        alpha.setTimeConstant(0.1f);
        alpha.reset(0.0f);
        float output = 0.0f;
        for (int i = 0; i < 1000; ++i)
            output = alpha.apply(5.0f, 0.005f);
        checkNear(output, 5.0f, 1e-2f, "alpha filter converges");
    }
}

// ---------------------------------------------------------------------------
// Estimator
// ---------------------------------------------------------------------------
void testEstimator()
{
    section("attitude estimator");

    // Level, at rest, with a north-pointing mag: attitude must stay level.
    {
        imu::AttitudeEstimator estimator;
        imu::AttitudeEstimatorConfig config;
        estimator.configure(config);

        const imu::Vec3 accel{0.0f, 0.0f, imu::kGravity};
        const imu::Vec3 mag{25.0f, 0.0f, -40.0f};
        check(estimator.initialize(accel, mag), "estimator initializes level");

        float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
        estimator.eulerDegrees(roll, pitch, yaw);
        checkNear(roll, 0.0f, 1.0f, "level roll");
        checkNear(pitch, 0.0f, 1.0f, "level pitch");
        checkNear(yaw, 0.0f, 1.0f, "level yaw");

        // Hold still for 2 s: nothing should drift.
        for (int i = 0; i < 400; ++i)
            estimator.update(accel, imu::Vec3{}, mag, 0.005f);
        estimator.eulerDegrees(roll, pitch, yaw);
        checkNear(roll, 0.0f, 1.0f, "level roll after 2 s");
        checkNear(pitch, 0.0f, 1.0f, "level pitch after 2 s");
        checkNear(yaw, 0.0f, 1.0f, "level yaw after 2 s");
    }

    // Rolled 30 deg: the accelerometer alone must pull the estimate to 30 deg.
    {
        imu::AttitudeEstimator estimator;
        imu::AttitudeEstimatorConfig config;
        estimator.configure(config);

        // Seed level, then feed a 30 deg roll gravity vector.
        const imu::Vec3 levelAccel{0.0f, 0.0f, imu::kGravity};
        const imu::Vec3 mag{25.0f, 0.0f, -40.0f};
        estimator.initialize(levelAccel, mag);

        const float angle = 30.0f * imu::kDegToRad;
        const imu::Vec3 rolledAccel{0.0f, imu::kGravity * sinf(angle),
                                    imu::kGravity * cosf(angle)};

        for (int i = 0; i < 4000; ++i)
            estimator.update(rolledAccel, imu::Vec3{}, mag, 0.005f);

        float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
        estimator.eulerDegrees(roll, pitch, yaw);
        checkNear(roll, 30.0f, 2.0f, "accel pulls roll to 30 deg");
        checkNear(pitch, 0.0f, 2.0f, "pitch stays level");
    }

    // Gyro integration: a constant yaw rate must produce the right heading.
    {
        imu::AttitudeEstimator estimator;
        imu::AttitudeEstimatorConfig config;
        config.magWeight = 0.0f; // isolate the gyro path
        config.accelWeight = 0.0f;
        config.gyroBiasWeight = 0.0f;
        estimator.configure(config);

        const imu::Vec3 accel{0.0f, 0.0f, imu::kGravity};
        estimator.initialize(accel, imu::Vec3{});

        const float rate = 45.0f * imu::kDegToRad; // 45 deg/s about Z
        const float dt = 0.005f;
        const int steps = 200; // 1 s
        for (int i = 0; i < steps; ++i)
            estimator.update(accel, imu::Vec3{0.0f, 0.0f, rate}, imu::Vec3{}, dt);

        float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
        estimator.eulerDegrees(roll, pitch, yaw);
        checkNear(yaw, 45.0f, 1.0f, "gyro integrates 45 deg/s for 1 s");
    }

    // Gyro bias rejection: a constant gyro bias must NOT make the attitude
    // drift away, because the accelerometer correction holds it. (The bias
    // integrator itself only learns from an existing attitude error, which is
    // the same property PX4 has - a perfectly level, perfectly still vehicle
    // produces no error signal.)
    {
        imu::AttitudeEstimator estimator;
        imu::AttitudeEstimatorConfig config;
        config.magWeight = 0.0f;
        estimator.configure(config);

        const imu::Vec3 accel{0.0f, 0.0f, imu::kGravity};
        estimator.initialize(accel, imu::Vec3{});

        const imu::Vec3 bias{0.02f, -0.01f, 0.03f}; // rad/s
        for (int i = 0; i < 4000; ++i)
            estimator.update(accel, bias, imu::Vec3{}, 0.005f);

        float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
        estimator.eulerDegrees(roll, pitch, yaw);
        check(fabsf(roll) < 5.0f, "gyro bias does not drift roll");
        check(fabsf(pitch) < 5.0f, "gyro bias does not drift pitch");
        check(estimator.gyroBiasRad().norm() < 0.2f, "gyro bias state stays bounded");
    }

    // Gyro bias estimation: with a real attitude error present, the integral
    // state must absorb the bias and the estimate must converge.
    {
        imu::AttitudeEstimator estimator;
        imu::AttitudeEstimatorConfig config;
        config.magWeight = 0.0f;
        estimator.configure(config);

        const imu::Vec3 accel{0.0f, 0.0f, imu::kGravity};
        estimator.initialize(accel, imu::Vec3{});

        // 20 deg of roll error plus a gyro bias: the estimator has to fix both.
        const float angle = 20.0f * imu::kDegToRad;
        const imu::Vec3 rolledAccel{0.0f, imu::kGravity * sinf(angle),
                                    imu::kGravity * cosf(angle)};
        const imu::Vec3 bias{0.0f, 0.0f, 0.02f};

        for (int i = 0; i < 8000; ++i)
            estimator.update(rolledAccel, bias, imu::Vec3{}, 0.005f);

        float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
        estimator.eulerDegrees(roll, pitch, yaw);
        checkNear(roll, 20.0f, 2.0f, "converges with a gyro bias present");
        check(estimator.gyroBiasRad().norm() > 0.001f, "gyro bias state is learning");
    }

    // Degenerate input must not corrupt the state.
    {
        imu::AttitudeEstimator estimator;
        imu::AttitudeEstimatorConfig config;
        estimator.configure(config);
        estimator.initialize(imu::Vec3{0.0f, 0.0f, imu::kGravity}, imu::Vec3{25.0f, 0.0f, -40.0f});

        const imu::Vec3 nanVec{NAN, NAN, NAN};
        estimator.update(nanVec, imu::Vec3{}, imu::Vec3{}, 0.005f);
        check(estimator.quaternion().isFinite(), "NaN input keeps quaternion finite");
    }
}

// ---------------------------------------------------------------------------
// Calibration math
// ---------------------------------------------------------------------------
void testAccelerometerCalibration()
{
    section("accelerometer calibration");

    // Synthesize six-position data from a known bias + matrix, then check the
    // solver recovers it.
    const float trueBias[3] = {0.03f, -0.02f, 0.05f};
    const float trueMatrix[3][3] = {{1.02f, 0.01f, -0.005f},
                                    {0.008f, 0.98f, 0.012f},
                                    {-0.004f, 0.006f, 1.03f}};

    // Invert the model: raw = bias + matrix^-1 * reference
    imu::Mat3 matrix;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            matrix.m[r][c] = trueMatrix[r][c];
    imu::Mat3 inverse;
    check(matrix.invert(inverse), "true matrix invertible");

    static const float reference[imu::kAccelCalibrationSides][3] = {
        {1.0f, 0.0f, 0.0f},  {-1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f},
        {0.0f, -1.0f, 0.0f}, {0.0f, 0.0f, 1.0f},  {0.0f, 0.0f, -1.0f}};

    float raw[imu::kAccelCalibrationSides][3];
    for (int side = 0; side < imu::kAccelCalibrationSides; ++side)
    {
        const imu::Vec3 corrected = inverse * imu::Vec3{reference[side]};
        raw[side][0] = corrected.x + trueBias[0];
        raw[side][1] = corrected.y + trueBias[1];
        raw[side][2] = corrected.z + trueBias[2];
    }

    float bias[3];
    float solved[3][3];
    float rms = 0.0f;
    check(imu::calibration::fitAccelerometerSixPosition(raw, bias, solved, rms),
          "six-position solve succeeds");

    for (int i = 0; i < 3; ++i)
        checkNear(bias[i], trueBias[i], 1e-4f, "recovered accel bias");

    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            checkNear(solved[r][c], trueMatrix[r][c], 1e-3f, "recovered accel matrix");

    // PX4's six-position solve only uses three of the six faces (X+, Y+, Z+) to
    // determine the matrix, so the opposite faces carry the cross-axis residual.
    // A few milli-g is expected and is exactly what PX4 reports too.
    check(rms < 0.02f, "accel fit residual is small");

    // A degenerate face pair must be rejected.
    float badRaw[imu::kAccelCalibrationSides][3];
    memcpy(badRaw, raw, sizeof(badRaw));
    badRaw[1][0] = badRaw[0][0]; // X+ and X- identical
    check(!imu::calibration::fitAccelerometerSixPosition(badRaw, bias, solved, rms),
          "degenerate face pair rejected");
}

void testMagnetometerCalibration()
{
    section("magnetometer calibration");

    // Synthesize a sphere of samples around a known offset, then check the
    // sphere fit recovers the offset.
    const float trueOffset[3] = {12.0f, -8.0f, 5.0f};
    const float radius = 45.0f;

    static float samples[600 * 3];
    int count = 0;
    for (int i = 0; i < 600; ++i)
    {
        // Fibonacci sphere: even coverage, which is what the fit needs.
        const float t = (i + 0.5f) / 600.0f;
        const float inclination = acosf(1.0f - 2.0f * t);
        const float azimuth = imu::kPi * (1.0f + sqrtf(5.0f)) * i;
        samples[3 * count + 0] = trueOffset[0] + radius * sinf(inclination) * cosf(azimuth);
        samples[3 * count + 1] = trueOffset[1] + radius * sinf(inclination) * sinf(azimuth);
        samples[3 * count + 2] = trueOffset[2] + radius * cosf(inclination);
        ++count;
    }

    float offset[3] = {};
    float matrix[3][3];
    float fittedRadius = 0.0f;
    float rms = 0.0f;
    check(imu::calibration::fitMagnetometerSphere(samples, count, offset, matrix,
                                                  fittedRadius, rms),
          "sphere fit succeeds");

    for (int i = 0; i < 3; ++i)
        checkNear(offset[i], trueOffset[i], 1.0f, "recovered mag offset");
    checkNear(fittedRadius, radius, 1.0f, "recovered mag radius");
    check(rms < 1.0f, "sphere fit residual is small");

    // Ellipsoid refinement on a deliberately squashed cloud.
    static float squashed[600 * 3];
    for (int i = 0; i < count; ++i)
    {
        squashed[3 * i + 0] = samples[3 * i + 0] * 1.15f;
        squashed[3 * i + 1] = samples[3 * i + 1] * 0.90f;
        squashed[3 * i + 2] = samples[3 * i + 2] * 1.05f;
    }

    float ellipsoidOffset[3] = {};
    float ellipsoidMatrix[3][3];
    float ellipsoidRadius = 0.0f;
    float ellipsoidRms = 0.0f;
    const bool sphereOk = imu::calibration::fitMagnetometerSphere(
        squashed, count, ellipsoidOffset, ellipsoidMatrix, ellipsoidRadius, ellipsoidRms);
    check(sphereOk, "sphere fit on squashed cloud succeeds");

    const float sphereRms = ellipsoidRms;
    const bool ellipsoidOk = imu::calibration::fitMagnetometerEllipsoid(
        squashed, count, ellipsoidOffset, ellipsoidMatrix, ellipsoidRadius, ellipsoidRms);
    check(ellipsoidOk, "ellipsoid fit succeeds");
    if (ellipsoidOk)
        check(ellipsoidRms <= sphereRms + 1e-3f, "ellipsoid fit is not worse than sphere");
}

// ---------------------------------------------------------------------------
// Still detection
// ---------------------------------------------------------------------------
void testStillDetection()
{
    section("still detection");

    float accelNormSumG = 0.0f;
    float accelVectorVarianceSum = 0.0f;
    float accelMean[3] = {};
    float accelSquaresMean[3] = {};

    for (int i = 0; i < 40; ++i)
    {
        const float zG = 1.025f + ((i % 3) - 1) * 0.0001f;
        const float normG = sqrtf(zG * zG);
        accelNormSumG += normG;
        accelMean[2] += zG / 40.0f;
        accelSquaresMean[2] += zG * zG / 40.0f;
    }

    const float meanNormG = accelNormSumG / 40.0f;
    for (int axis = 0; axis < 3; ++axis)
        accelVectorVarianceSum += fmaxf(0.0f,
                                        accelSquaresMean[axis] -
                                            accelMean[axis] * accelMean[axis]);
    const float stdDevG = sqrtf(accelVectorVarianceSum);

    bool still = false;
    const float gyroNormDps = 0.0f;
    const float accelErrorG = fabsf(meanNormG - 1.0f);
    const bool accelNormPass =
        still
            ? accelErrorG <= STILL_ACCEL_LEAVE_THRESHOLD_G
            : accelErrorG < STILL_ACCEL_ENTER_THRESHOLD_G;
    const uint8_t mask =
        (!accelNormPass ? (1u << 0) : 0u) |
        (gyroNormDps >= 0.5f ? (1u << 1) : 0u) |
        (stdDevG >= 0.004f ? (1u << 2) : 0u);

    still = (mask == 0);
    check(still, "constant 1.025 g accel, zero gyro, and low noise become still");
    check((mask & (1u << 0)) == 0, "1.025 g mean norm passes still accel bit0");
    checkNear(meanNormG, 1.025f, 0.001f, "40-sample mean accel norm");
}

// ---------------------------------------------------------------------------
// End-to-end: a synthetic rotation must be tracked by the estimator
// ---------------------------------------------------------------------------
void testEndToEndTracking()
{
    section("end-to-end tracking");

    imu::AttitudeEstimator estimator;
    imu::AttitudeEstimatorConfig config;
    estimator.configure(config);

    // Ground truth: yaw at 30 deg/s, level.
    const float rate = 30.0f * imu::kDegToRad;
    const float dt = 0.005f;

    const imu::Vec3 accel{0.0f, 0.0f, imu::kGravity};
    const imu::Vec3 mag{25.0f, 0.0f, -40.0f};
    estimator.initialize(accel, mag);

    float truthYaw = 0.0f;
    for (int i = 0; i < 400; ++i) // 2 s
    {
        truthYaw += rate * dt;
        // Rotate the mag vector into the body frame for the current heading.
        const float c = cosf(truthYaw), s = sinf(truthYaw);
        const imu::Vec3 bodyMag{c * mag.x + s * mag.y, -s * mag.x + c * mag.y, mag.z};
        estimator.update(accel, imu::Vec3{0.0f, 0.0f, rate}, bodyMag, dt);
    }

    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
    estimator.eulerDegrees(roll, pitch, yaw);
    checkNear(yaw, truthYaw * imu::kRadToDeg, 3.0f, "tracked yaw matches truth");
    checkNear(roll, 0.0f, 2.0f, "tracked roll stays level");
    checkNear(pitch, 0.0f, 2.0f, "tracked pitch stays level");
}

} // namespace

int main()
{
    printf("IMU host tests\n==============\n");

    testQuaternionBasics();
    testRotationContract();
    testEstimatorInitialization();
    testFilters();
    testEstimator();
    testAccelerometerCalibration();
    testMagnetometerCalibration();
    testStillDetection();
    testEndToEndTracking();

    printf("\n%d checks, %d failures\n", gChecks, gFailures);
    return gFailures == 0 ? 0 : 1;
}
