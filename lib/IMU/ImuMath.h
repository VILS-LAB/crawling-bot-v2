#ifndef IMU_MATH_H
#define IMU_MATH_H

// Small, dependency-free 3D math used by the IMU stack.
//
// Deliberately header-only and free of Arduino/ESP-IDF includes so the exact
// same code can be compiled and unit-tested on the host (see test/host).
//
// Frame conventions used everywhere in this project (see docs/imu/ARCHITECTURE.md):
//   body  : X forward, Y left, Z up   (FLU, right handed)
//   world : X forward, Y left, Z up   (ENU-like, right handed)
//   Euler : R = Rz(yaw) * Ry(pitch) * Rx(roll)   (ZYX intrinsic)
//   quat  : body -> world, v_world = q * v_body * q^-1

#include <math.h>
#include <string.h>

namespace imu
{

constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 2.0f * kPi;
constexpr float kDegToRad = kPi / 180.0f;
constexpr float kRadToDeg = 180.0f / kPi;
constexpr float kGravity = 9.80665f;

inline float clampf(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

inline bool isFinitef(float value)
{
    return isfinite(value) != 0;
}

// Wrap an angle in radians to (-pi, pi].
inline float wrapPi(float angle)
{
    if (!isFinitef(angle))
        return 0.0f;
    angle = fmodf(angle + kPi, kTwoPi);
    if (angle < 0.0f)
        angle += kTwoPi;
    return angle - kPi;
}

// Wrap an angle in degrees to (-180, 180].
inline float wrapDeg(float angle)
{
    return wrapPi(angle * kDegToRad) * kRadToDeg;
}

struct Vec3
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Vec3() = default;
    Vec3(float xValue, float yValue, float zValue) : x(xValue), y(yValue), z(zValue) {}
    explicit Vec3(const float values[3]) : x(values[0]), y(values[1]), z(values[2]) {}

    float operator[](int index) const { return index == 0 ? x : (index == 1 ? y : z); }
    float &operator[](int index) { return index == 0 ? x : (index == 1 ? y : z); }

    void toArray(float out[3]) const
    {
        out[0] = x;
        out[1] = y;
        out[2] = z;
    }

    Vec3 operator+(const Vec3 &other) const { return {x + other.x, y + other.y, z + other.z}; }
    Vec3 operator-(const Vec3 &other) const { return {x - other.x, y - other.y, z - other.z}; }
    Vec3 operator*(float scale) const { return {x * scale, y * scale, z * scale}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3 &operator+=(const Vec3 &other)
    {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }
    Vec3 &operator-=(const Vec3 &other)
    {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }
    Vec3 &operator*=(float scale)
    {
        x *= scale;
        y *= scale;
        z *= scale;
        return *this;
    }

    float dot(const Vec3 &other) const { return x * other.x + y * other.y + z * other.z; }
    Vec3 cross(const Vec3 &other) const
    {
        return {y * other.z - z * other.y, z * other.x - x * other.z,
                x * other.y - y * other.x};
    }
    float normSquared() const { return x * x + y * y + z * z; }
    float norm() const { return sqrtf(normSquared()); }
    bool isFinite() const { return isFinitef(x) && isFinitef(y) && isFinitef(z); }

    // Returns false when the vector is too short to normalize safely.
    bool normalize()
    {
        const float length = norm();
        if (!isFinitef(length) || length < 1e-6f)
            return false;
        const float inverse = 1.0f / length;
        x *= inverse;
        y *= inverse;
        z *= inverse;
        return true;
    }

    Vec3 normalized() const
    {
        Vec3 copy = *this;
        copy.normalize();
        return copy;
    }
};

inline Vec3 operator*(float scale, const Vec3 &vector) { return vector * scale; }

struct Mat3
{
    float m[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

    static Mat3 identity() { return Mat3{}; }

    static Mat3 diagonal(float x, float y, float z)
    {
        Mat3 result;
        result.m[0][0] = x;
        result.m[1][1] = y;
        result.m[2][2] = z;
        return result;
    }

    Vec3 row(int index) const { return {m[index][0], m[index][1], m[index][2]}; }
    Vec3 column(int index) const { return {m[0][index], m[1][index], m[2][index]}; }

    void setRow(int index, const Vec3 &value)
    {
        m[index][0] = value.x;
        m[index][1] = value.y;
        m[index][2] = value.z;
    }

    Vec3 operator*(const Vec3 &vector) const
    {
        return {m[0][0] * vector.x + m[0][1] * vector.y + m[0][2] * vector.z,
                m[1][0] * vector.x + m[1][1] * vector.y + m[1][2] * vector.z,
                m[2][0] * vector.x + m[2][1] * vector.y + m[2][2] * vector.z};
    }

    Mat3 operator*(const Mat3 &other) const
    {
        Mat3 result;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                result.m[r][c] = m[r][0] * other.m[0][c] + m[r][1] * other.m[1][c] +
                                 m[r][2] * other.m[2][c];
        return result;
    }

    Mat3 transposed() const
    {
        Mat3 result;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                result.m[r][c] = m[c][r];
        return result;
    }

    bool isFinite() const
    {
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                if (!isFinitef(m[r][c]))
                    return false;
        return true;
    }

    // Gauss-Jordan inverse with partial pivoting. Returns false when singular.
    bool invert(Mat3 &out) const
    {
        float a[3][6];
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
            {
                a[r][c] = m[r][c];
                a[r][c + 3] = (r == c) ? 1.0f : 0.0f;
            }
        for (int c = 0; c < 3; ++c)
        {
            int pivot = c;
            for (int r = c + 1; r < 3; ++r)
                if (fabsf(a[r][c]) > fabsf(a[pivot][c]))
                    pivot = r;
            if (fabsf(a[pivot][c]) < 1e-9f)
                return false;
            if (pivot != c)
                for (int i = 0; i < 6; ++i)
                {
                    const float swap = a[c][i];
                    a[c][i] = a[pivot][i];
                    a[pivot][i] = swap;
                }
            const float divisor = a[c][c];
            for (int i = 0; i < 6; ++i)
                a[c][i] /= divisor;
            for (int r = 0; r < 3; ++r)
            {
                if (r == c)
                    continue;
                const float factor = a[r][c];
                if (factor == 0.0f)
                    continue;
                for (int i = 0; i < 6; ++i)
                    a[r][i] -= factor * a[c][i];
            }
        }
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                out.m[r][c] = a[r][c + 3];
        return out.isFinite();
    }
};

struct Quat
{
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Quat() = default;
    Quat(float wValue, float xValue, float yValue, float zValue)
        : w(wValue), x(xValue), y(yValue), z(zValue) {}

    float operator[](int index) const
    {
        return index == 0 ? w : (index == 1 ? x : (index == 2 ? y : z));
    }
    float &operator[](int index)
    {
        return index == 0 ? w : (index == 1 ? x : (index == 2 ? y : z));
    }

    void toArray(float out[4]) const
    {
        out[0] = w;
        out[1] = x;
        out[2] = y;
        out[3] = z;
    }

    bool isFinite() const
    {
        return isFinitef(w) && isFinitef(x) && isFinitef(y) && isFinitef(z);
    }

    float normSquared() const { return w * w + x * x + y * y + z * z; }
    float norm() const { return sqrtf(normSquared()); }

    bool normalize()
    {
        const float length = norm();
        if (!isFinitef(length) || length < 1e-6f)
            return false;
        const float inverse = 1.0f / length;
        w *= inverse;
        x *= inverse;
        y *= inverse;
        z *= inverse;
        return true;
    }

    Quat normalized() const
    {
        Quat copy = *this;
        copy.normalize();
        return copy;
    }

    Quat conjugated() const { return {w, -x, -y, -z}; }

    Quat operator*(const Quat &other) const
    {
        return {w * other.w - x * other.x - y * other.y - z * other.z,
                w * other.x + x * other.w + y * other.z - z * other.y,
                w * other.y - x * other.z + y * other.w + z * other.x,
                w * other.z + x * other.y - y * other.x + z * other.w};
    }

    Quat operator+(const Quat &other) const
    {
        return {w + other.w, x + other.x, y + other.y, z + other.z};
    }

    Quat operator*(float scale) const { return {w * scale, x * scale, y * scale, z * scale}; }

    // Rotate a body-frame vector into the world frame: q * v * q^-1.
    Vec3 rotate(const Vec3 &vector) const
    {
        const Quat pure{0.0f, vector.x, vector.y, vector.z};
        const Quat result = (*this) * pure * conjugated();
        return {result.x, result.y, result.z};
    }

    // Rotate a world-frame vector into the body frame: q^-1 * v * q.
    Vec3 rotateInverse(const Vec3 &vector) const
    {
        const Quat pure{0.0f, vector.x, vector.y, vector.z};
        const Quat result = conjugated() * pure * (*this);
        return {result.x, result.y, result.z};
    }

    // dq/dt for a body-frame angular rate, as used by PX4's attitude_estimator_q:
    //   dq = 0.5 * q * (0, w)
    Quat derivative1(const Vec3 &bodyRate) const
    {
        return {0.5f * (-x * bodyRate.x - y * bodyRate.y - z * bodyRate.z),
                0.5f * (w * bodyRate.x + y * bodyRate.z - z * bodyRate.y),
                0.5f * (w * bodyRate.y - x * bodyRate.z + z * bodyRate.x),
                0.5f * (w * bodyRate.z + x * bodyRate.y - y * bodyRate.x)};
    }

    Mat3 toRotationMatrix() const
    {
        const float xx = x * x, yy = y * y, zz = z * z;
        const float xy = x * y, xz = x * z, yz = y * z;
        const float wx = w * x, wy = w * y, wz = w * z;
        Mat3 result;
        result.m[0][0] = 1.0f - 2.0f * (yy + zz);
        result.m[0][1] = 2.0f * (xy - wz);
        result.m[0][2] = 2.0f * (xz + wy);
        result.m[1][0] = 2.0f * (xy + wz);
        result.m[1][1] = 1.0f - 2.0f * (xx + zz);
        result.m[1][2] = 2.0f * (yz - wx);
        result.m[2][0] = 2.0f * (xz - wy);
        result.m[2][1] = 2.0f * (yz + wx);
        result.m[2][2] = 1.0f - 2.0f * (xx + yy);
        return result;
    }

    // Build a quaternion from a body->world rotation matrix.
    static Quat fromRotationMatrix(const Mat3 &rotation)
    {
        const float trace = rotation.m[0][0] + rotation.m[1][1] + rotation.m[2][2];
        Quat result;
        if (trace > 0.0f)
        {
            const float s = sqrtf(trace + 1.0f) * 2.0f;
            result.w = 0.25f * s;
            result.x = (rotation.m[2][1] - rotation.m[1][2]) / s;
            result.y = (rotation.m[0][2] - rotation.m[2][0]) / s;
            result.z = (rotation.m[1][0] - rotation.m[0][1]) / s;
        }
        else if (rotation.m[0][0] > rotation.m[1][1] && rotation.m[0][0] > rotation.m[2][2])
        {
            const float s = sqrtf(1.0f + rotation.m[0][0] - rotation.m[1][1] - rotation.m[2][2]) * 2.0f;
            result.w = (rotation.m[2][1] - rotation.m[1][2]) / s;
            result.x = 0.25f * s;
            result.y = (rotation.m[0][1] + rotation.m[1][0]) / s;
            result.z = (rotation.m[0][2] + rotation.m[2][0]) / s;
        }
        else if (rotation.m[1][1] > rotation.m[2][2])
        {
            const float s = sqrtf(1.0f + rotation.m[1][1] - rotation.m[0][0] - rotation.m[2][2]) * 2.0f;
            result.w = (rotation.m[0][2] - rotation.m[2][0]) / s;
            result.x = (rotation.m[0][1] + rotation.m[1][0]) / s;
            result.y = 0.25f * s;
            result.z = (rotation.m[1][2] + rotation.m[2][1]) / s;
        }
        else
        {
            const float s = sqrtf(1.0f + rotation.m[2][2] - rotation.m[0][0] - rotation.m[1][1]) * 2.0f;
            result.w = (rotation.m[1][0] - rotation.m[0][1]) / s;
            result.x = (rotation.m[0][2] + rotation.m[2][0]) / s;
            result.y = (rotation.m[1][2] + rotation.m[2][1]) / s;
            result.z = 0.25f * s;
        }
        result.normalize();
        return result;
    }

    // Euler angles in degrees, matching rotateBodyToWorld():
    //   R = Rz(yaw) * Ry(pitch) * Rx(roll)
    void toEulerDegrees(float &rollDeg, float &pitchDeg, float &yawDeg) const
    {
        const Mat3 rotation = toRotationMatrix();
        const float sinPitch = clampf(-rotation.m[2][0], -1.0f, 1.0f);
        pitchDeg = asinf(sinPitch) * kRadToDeg;
        rollDeg = atan2f(rotation.m[2][1], rotation.m[2][2]) * kRadToDeg;
        yawDeg = atan2f(rotation.m[1][0], rotation.m[0][0]) * kRadToDeg;
    }

    // Body-frame direction of world "down" (0, 0, -1).
    // This is the quantity PX4 calls `k` in attitude_estimator_q.
    Vec3 worldDownInBody() const
    {
        return rotateInverse(Vec3{0.0f, 0.0f, -1.0f});
    }

    // Body-frame direction of world "up" (0, 0, 1).
    Vec3 worldUpInBody() const
    {
        return rotateInverse(Vec3{0.0f, 0.0f, 1.0f});
    }
};

} // namespace imu

#endif // IMU_MATH_H
