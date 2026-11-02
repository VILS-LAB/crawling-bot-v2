#ifndef IMU_ROTATION_H
#define IMU_ROTATION_H

#include <Arduino.h>

void rotateBodyToWorld(float roll, float pitch, float yaw,
                       const float body[3], float world[3]);

class ComplementaryAttitude
{
public:
    void reset();
    void update(const float accel[3], const float gyro[3], float dt,
                float &roll, float &pitch, float &yaw);
private:
    bool initialized = false;
    float roll = 0;
    float pitch = 0;
    float yaw = 0;
};

#endif
