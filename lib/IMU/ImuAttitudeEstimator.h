#ifndef IMU_ATTITUDE_ESTIMATOR_H
#define IMU_ATTITUDE_ESTIMATOR_H

// Quaternion complementary attitude estimator.
//
// This is a port of PX4's `attitude_estimator_q`
// (src/modules/attitude_estimator_q/attitude_estimator_q_main.cpp), keeping the
// algorithm and its tuning structure intact:
//
//   * quaternion state, propagated with the body rate (feed-forward gyro)
//   * accelerometer correction that is only applied while |a| is near 1 g
//   * magnetometer heading correction with a spin-rate gain boost
//   * integral gyro-bias state, frozen while the vehicle is spinning fast
//   * initial attitude built from the accel + mag vectors (TRIAD)
//
// Differences from PX4, all deliberate:
//   * no uORB / parameters / GPS / vision / mocap inputs - this is a standalone
//     IMU on a ground robot, so those paths do not exist here
//   * the frame is the project's FLU body / ENU-like world (X forward, Y left,
//     Z up) instead of PX4's FRD/NED. The estimator math is frame agnostic as
//     long as `worldDownInBody` and the specific-force sign are consistent, so
//     the equations are unchanged. See docs/imu/ARCHITECTURE.md.
//   * magnetic declination is a plain constant (no GPS lookup)

#include "ImuMath.h"

namespace imu
{

struct AttitudeEstimatorConfig
{
    // PX4 ATT_W_ACC / ATT_W_MAG defaults.
    float accelWeight = 0.2f;
    float magWeight = 0.1f;

    // PX4's ATT_W_GYRO_BIAS default is 0.1, which is tuned for a flying vehicle
    // where the accelerometer correction is transient. On a ground robot that
    // sits still for minutes, a persistent attitude error is fed into the bias
    // integrator and the (accel, bias) loop becomes lightly damped: the estimate
    // overshoots and takes ~1 minute to settle. 0.02 keeps the same structure
    // but makes the loop well damped (both poles real and negative).
    float gyroBiasWeight = 0.02f;

    // PX4 ATT_BIAS_MAX default is 0.05 rad/s. The MPU9250 in a GY-91 can have a
    // larger residual bias after a short calibration, so the default here is
    // wider; the calibration routine normally leaves far less than this.
    float gyroBiasLimitRad = 0.15f;

    // Magnetic declination in radians (east positive). Zero means "yaw is
    // relative to magnetic north", which is what this robot uses.
    float magneticDeclinationRad = 0.0f;

    // PX4 fuses accel only while |a| is within +-10% of 1 g. Same here.
    float accelNormLowerG = 0.9f;
    float accelNormUpperG = 1.1f;

    // PX4 freezes the bias integrator above 0.175 rad/s (10 deg/s).
    float gyroBiasFreezeRateRad = 0.175f;

    // PX4 boosts the mag gain above 0.873 rad/s (50 deg/s) to keep up with fast
    // yaw rotation, capped at 10x.
    float magGainBoostRateRad = 0.873f;
    float magGainBoostMax = 10.0f;
};

class AttitudeEstimator
{
public:
    void configure(const AttitudeEstimatorConfig &config);
    const AttitudeEstimatorConfig &config() const { return _config; }

    // Drop the state; the next update() with valid data re-initializes.
    void reset();

    // Build the initial attitude from one accel + mag sample (PX4 init_attitude_q).
    // `accel` is specific force in m/s^2 (body), `mag` is in uT (body).
    // Returns false when the inputs are degenerate.
    bool initialize(const Vec3 &accel, const Vec3 &mag);

    // One estimator step.
    //   accel : specific force, m/s^2, body frame (measures +1 g "up" at rest)
    //   gyro  : body rate, rad/s, bias already removed by the caller? No - pass
    //           the *raw* bias-corrected-by-calibration rate; the estimator adds
    //           its own integral bias state on top.
    //   mag   : magnetic field, uT, body frame. Pass a zero vector to skip.
    //   dt    : seconds
    // Returns false when the state had to be rejected (non-finite).
    bool update(const Vec3 &accel, const Vec3 &gyro, const Vec3 &mag, float dt);

    bool initialized() const { return _initialized; }
    const Quat &quaternion() const { return _q; }
    Vec3 gyroBiasRad() const { return _gyroBias; }
    Vec3 correctedRateRad() const { return _rates; }

    // Euler angles in degrees using the project convention
    // R = Rz(yaw) * Ry(pitch) * Rx(roll).
    void eulerDegrees(float &roll, float &pitch, float &yaw) const;

private:
    AttitudeEstimatorConfig _config{};
    Quat _q{};
    Vec3 _gyroBias{};
    Vec3 _rates{};
    bool _initialized = false;
};

} // namespace imu

#endif // IMU_ATTITUDE_ESTIMATOR_H
