#ifndef HEALTH_CHECK_H
#define HEALTH_CHECK_H

#include <Arduino.h>
#include <Display.h>
#include <AHRS.h>
#include <ServoControl.h>
#include <ServoHealth.h>

struct HealthReport
{
    ServoHealthCode servoDown = SERVO_HEALTH_DISABLED;
    ServoHealthCode servoUp = SERVO_HEALTH_DISABLED;

    // A disabled servo is not a failure: it was never asked to run.
    bool ok() const
    {
        return (servoDown == SERVO_HEALTH_OK || servoDown == SERVO_HEALTH_DISABLED) &&
               (servoUp == SERVO_HEALTH_OK || servoUp == SERVO_HEALTH_DISABLED);
    }
};

class HealthCheck
{
public:
    HealthCheck(Display *display, AHRS *ahrs, ServoControl *servoControl);
    HealthReport run();

private:
    Display *display;
    AHRS *ahrs;
    ServoControl *servoControl;

    void reportServo(const char *name, uint8_t pin, ServoHealthCode code);
    ServoHealthCode checkMotion(bool down);
    float gyroMagnitude();
    float measureBaseline(uint32_t windowMs);
    float sweep(bool down, int fromAngle, int toAngle);
};

#endif // HEALTH_CHECK_H
