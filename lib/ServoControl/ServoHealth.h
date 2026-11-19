#ifndef SERVO_HEALTH_H
#define SERVO_HEALTH_H

#include <Arduino.h>

// Numeric codes for the servo health check.
//
// These numbers are a serial contract: the firmware prints them, host tools and
// the docs table read them. Append new codes, never renumber existing ones.
enum ServoHealthCode : uint8_t
{
    SERVO_HEALTH_OK = 0,             // attached, pulse round-trip matches, IMU saw the move
    SERVO_HEALTH_ATTACH_FAILED = 1,  // attach() rejected the pin / no free LEDC channel
    SERVO_HEALTH_NOT_ATTACHED = 2,   // attached earlier, but the channel is gone now
    SERVO_HEALTH_PULSE_MISMATCH = 3, // PWM read-back does not match the angle written
    SERVO_HEALTH_NO_MOTION = 4,      // swept the servo, the IMU saw no reaction
    SERVO_HEALTH_DISABLED = 5,       // ENABLE_SERVOS is 0, nothing was tested
};

inline const char *servoHealthName(ServoHealthCode code)
{
    switch (code)
    {
    case SERVO_HEALTH_OK:
        return "OK";
    case SERVO_HEALTH_ATTACH_FAILED:
        return "ATTACH_FAILED";
    case SERVO_HEALTH_NOT_ATTACHED:
        return "NOT_ATTACHED";
    case SERVO_HEALTH_PULSE_MISMATCH:
        return "PULSE_MISMATCH";
    case SERVO_HEALTH_NO_MOTION:
        return "NO_MOTION";
    case SERVO_HEALTH_DISABLED:
        return "DISABLED";
    }
    return "UNKNOWN";
}

#endif // SERVO_HEALTH_H
