#include "ImuRotation.h"

void rotateBodyToWorld(float roll, float pitch, float yaw, const float body[3],
                       float world[3])
{
    // Kept byte-for-byte equivalent to the original implementation: this is the
    // rotation contract the hardware and both visualizers were validated with.
    const float x = roll * imu::kDegToRad;
    const float y = pitch * imu::kDegToRad;
    const float z = yaw * imu::kDegToRad;
    const float cx = cosf(x), sx = sinf(x);
    const float cy = cosf(y), sy = sinf(y);
    const float cz = cosf(z), sz = sinf(z);

    const float ax1 = body[0];
    const float ay1 = body[1] * cx - body[2] * sx;
    const float az1 = body[1] * sx + body[2] * cx;

    const float ax2 = ax1 * cy + az1 * sy;
    const float az2 = -ax1 * sy + az1 * cy;

    world[0] = ax2 * cz - ay1 * sz;
    world[1] = ax2 * sz + ay1 * cz;
    world[2] = az2;
}

imu::Mat3 bodyToWorldMatrix(float roll, float pitch, float yaw)
{
    const float x = roll * imu::kDegToRad;
    const float y = pitch * imu::kDegToRad;
    const float z = yaw * imu::kDegToRad;
    const float cx = cosf(x), sx = sinf(x);
    const float cy = cosf(y), sy = sinf(y);
    const float cz = cosf(z), sz = sinf(z);

    imu::Mat3 matrix;
    matrix.m[0][0] = cy * cz;
    matrix.m[0][1] = sx * sy * cz - cx * sz;
    matrix.m[0][2] = cx * sy * cz + sx * sz;
    matrix.m[1][0] = cy * sz;
    matrix.m[1][1] = sx * sy * sz + cx * cz;
    matrix.m[1][2] = cx * sy * sz - sx * cz;
    matrix.m[2][0] = -sy;
    matrix.m[2][1] = sx * cy;
    matrix.m[2][2] = cx * cy;
    return matrix;
}
