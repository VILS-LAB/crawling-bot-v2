#include "ImuAttitudeEstimator.h"

namespace imu
{

void AttitudeEstimator::configure(const AttitudeEstimatorConfig &config)
{
    _config = config;
}

void AttitudeEstimator::reset()
{
    _q = Quat{};
    _gyroBias = Vec3{};
    _rates = Vec3{};
    _initialized = false;
}

bool AttitudeEstimator::initialize(const Vec3 &accel, const Vec3 &mag)
{
    // PX4 init_attitude_q(), adapted to the project's FLU/ENU-like frame.
    //
    // PX4 builds the rotation matrix from the earth axes expressed in the body
    // frame: row 0 = earth X, row 1 = earth Y, row 2 = earth Z. That matrix is
    // body->world, which is exactly what our quaternion represents.
    //
    // In PX4's NED frame the accelerometer reads -g along earth Z (down), so
    // `k = -accel` is earth Z in body. In our frame earth Z points up and the
    // accelerometer reads +g along it, so `k = -accel.normalized()` is still
    // earth Z (down) in body. The expression is therefore unchanged.

    Vec3 down = accel * -1.0f;
    if (!down.normalize())
        return false;

    Vec3 north = mag - down * mag.dot(down);
    if (!north.normalize())
    {
        // No usable magnetometer: level the attitude from gravity alone and
        // leave the heading at zero. Yaw then comes from the gyro only.
        const Vec3 up = -down;
        // `up` is the world-up direction expressed in the body frame, so the
        // body->world rotation must map `up` onto world +Z. The smallest such
        // rotation has axis = up x Z and angle = angle(up, Z).
        const Vec3 axis = up.cross(Vec3{0.0f, 0.0f, 1.0f});
        const float sinAngle = axis.norm();
        const float cosAngle = clampf(up.dot(Vec3{0.0f, 0.0f, 1.0f}), -1.0f, 1.0f);
        const float angle = atan2f(sinAngle, cosAngle);
        if (sinAngle < 1e-6f)
        {
            _q = (cosAngle > 0.0f) ? Quat{} : Quat{0.0f, 1.0f, 0.0f, 0.0f};
        }
        else
        {
            const Vec3 unitAxis = axis * (1.0f / sinAngle);
            const float half = 0.5f * angle;
            const float s = sinf(half);
            _q = Quat{cosf(half), unitAxis.x * s, unitAxis.y * s, unitAxis.z * s};
        }
        _q.normalize();
        _gyroBias = Vec3{};
        _rates = Vec3{};
        _initialized = _q.isFinite();
        return _initialized;
    }

    // Rows of a body->world matrix are the world axes expressed in the body
    // frame. Our world frame is X = heading, Y = 90 deg to the LEFT of X,
    // Z = up, so:
    //   row 0 = north (mag direction) in body
    //   row 1 = west  (north x down)  in body
    //   row 2 = up    (-down)         in body
    //
    // PX4 uses `j = k % i` (down x north) because its NED frame has Y = east.
    // With Y = left the cross product order flips; using PX4's order here would
    // produce a 180 deg roll error at initialization.
    const Vec3 west = north.cross(down);
    const Vec3 up = -down;

    Mat3 rotation;
    rotation.setRow(0, north);
    rotation.setRow(1, west);
    rotation.setRow(2, up);

    _q = Quat::fromRotationMatrix(rotation);

    // Compensate for magnetic declination (PX4 does the same with a Z rotation).
    if (fabsf(_config.magneticDeclinationRad) > 1e-6f)
    {
        const float half = 0.5f * _config.magneticDeclinationRad;
        const Quat declination{cosf(half), 0.0f, 0.0f, sinf(half)};
        _q = _q * declination;
        _q.normalize();
    }

    _gyroBias = Vec3{};
    _rates = Vec3{};
    _initialized = _q.isFinite() && _q.norm() > 0.95f && _q.norm() < 1.05f;
    return _initialized;
}

bool AttitudeEstimator::update(const Vec3 &accel, const Vec3 &gyro, const Vec3 &mag, float dt)
{
    if (!_initialized)
        return false;
    if (!isFinitef(dt) || dt <= 0.0f)
        return false;
    if (!accel.isFinite() || !gyro.isFinite())
        return false;

    const Quat qLast = _q;

    // --- angular rate of correction -------------------------------------
    Vec3 correction{};
    const float spinRate = gyro.norm();

    // --- magnetometer heading correction --------------------------------
    const bool magUsable = _config.magWeight > 0.0f && mag.isFinite() && mag.norm() > 1.0f;
    if (magUsable)
    {
        const Vec3 magEarth = _q.rotate(mag);
        const float magError =
            wrapPi(atan2f(magEarth.y, magEarth.x) - _config.magneticDeclinationRad);

        float gainMultiplier = 1.0f;
        if (spinRate > _config.magGainBoostRateRad)
        {
            gainMultiplier = fminf(spinRate / _config.magGainBoostRateRad, _config.magGainBoostMax);
        }

        correction += _q.rotateInverse(Vec3{0.0f, 0.0f, -magError}) *
                      (_config.magWeight * gainMultiplier);
    }

    _q.normalize();

    // --- accelerometer correction ---------------------------------------
    // `k` is earth Z (down) expressed in the body frame, exactly as in PX4.
    const Vec3 k = _q.worldDownInBody();

    const float accelNorm = accel.norm();
    const float lowerLimit = _config.accelNormLowerG * kGravity;
    const float upperLimit = _config.accelNormUpperG * kGravity;

    if (accelNorm > lowerLimit && accelNorm < upperLimit)
    {
        const Vec3 accelDirection = accel * (1.0f / accelNorm);
        correction += k.cross(accelDirection) * _config.accelWeight;
    }

    // --- gyro bias estimation -------------------------------------------
    if (spinRate < _config.gyroBiasFreezeRateRad)
    {
        _gyroBias += correction * (_config.gyroBiasWeight * dt);
        const float limit = _config.gyroBiasLimitRad;
        _gyroBias.x = clampf(_gyroBias.x, -limit, limit);
        _gyroBias.y = clampf(_gyroBias.y, -limit, limit);
        _gyroBias.z = clampf(_gyroBias.z, -limit, limit);
    }

    _rates = gyro + _gyroBias;

    // --- feed-forward gyro + correction, integrate ----------------------
    correction += _rates;
    _q = _q + _q.derivative1(correction) * dt;
    _q.normalize();

    if (!_q.isFinite())
    {
        _q = qLast;
        _rates = Vec3{};
        _gyroBias = Vec3{};
        return false;
    }

    return true;
}

void AttitudeEstimator::eulerDegrees(float &roll, float &pitch, float &yaw) const
{
    _q.toEulerDegrees(roll, pitch, yaw);
}

} // namespace imu
