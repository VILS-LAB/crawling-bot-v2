#include "ImuRotation.h"
#include <math.h>

void rotateBodyToWorld(float roll, float pitch, float yaw,
                       const float body[3], float world[3])
{
    const float x = radians(roll), y = radians(pitch), z = radians(yaw);
    const float cx = cosf(x), sx = sinf(x), cy = cosf(y), sy = sinf(y);
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

void ComplementaryAttitude::reset()
{
    initialized = false;
    roll = pitch = yaw = 0;
}

void ComplementaryAttitude::update(const float accel[3], const float gyro[3],
                                   float dt, float accelTrust,
                                   float &outRoll, float &outPitch,
                                   float &outYaw)
{
    const float norm = sqrtf(sq(accel[0]) + sq(accel[1]) + sq(accel[2]));
    const float accelRoll = atan2f(accel[1], accel[2]) * 180.0f / PI;
    const float accelPitch = -atan2f(accel[0], sqrtf(sq(accel[1]) + sq(accel[2]))) *
                             180.0f / PI;
    if (!initialized)
    {
        roll = accelRoll;
        pitch = accelPitch;
        yaw = 0;
        initialized = true;
    }
    else
    {
        roll += gyro[0] * dt;
        pitch += gyro[1] * dt;
        yaw += gyro[2] * dt;
        const float gyroNorm = sqrtf(sq(gyro[0]) + sq(gyro[1]) + sq(gyro[2]));
        // Accelerometer correction is valid only while the sensor is still.
        // During translation, a norm near 1g does not prove that gravity is
        // the only acceleration present.
        if (norm > 0.90f && norm < 1.10f && gyroNorm < 3.0f)
        {
            const float correction = 1.0f - expf(-dt / 0.50f);
            roll += accelTrust * correction * (accelRoll - roll);
            pitch += accelTrust * correction * (accelPitch - pitch);
        }
    }
    outRoll = roll;
    outPitch = pitch;
    outYaw = yaw;
}
