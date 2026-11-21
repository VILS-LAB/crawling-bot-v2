#ifndef SERVO_MOTION_H
#define SERVO_MOTION_H

// Pure motion profile for one servo axis.
//
// This file has no Arduino dependency, so the ramp can be unit-tested on the
// host (see test/host/test_servo_motion.cpp). ServoControl owns one
// ServoMotionState per servo and feeds it the elapsed time from millis().
//
// The profile is a trapezoid: accelerate at `accel` up to `maxSpeed`, cruise,
// then decelerate so the axis lands exactly on the target. The braking point is
// derived from the remaining distance, so retargeting mid-move never produces a
// jump: the axis simply brakes and heads for the new target.

struct ServoMotionConfig
{
    float maxSpeed = 180.0f; // degrees per second
    float accel = 600.0f;    // degrees per second squared
};

struct ServoMotionState
{
    float angle = 0.0f;    // commanded angle, degrees
    float target = 0.0f;   // where the axis is heading, degrees
    float velocity = 0.0f; // degrees per second
    float minAngle = 0.0f; // soft limit, degrees
    float maxAngle = 180.0f;
};

// Advances the axis by dt seconds and returns the new commanded angle.
// dt <= 0 is a no-op. The caller is responsible for clamping dt to a sane
// maximum so a stalled loop cannot produce a huge step.
float servoMotionStep(ServoMotionState &state, const ServoMotionConfig &config, float dt);

// True when the axis has reached its target and is not moving.
bool servoMotionIdle(const ServoMotionState &state);

// Clamps `angle` into the state's soft limits.
float servoMotionClamp(const ServoMotionState &state, float angle);

// Estimated time in milliseconds to travel `distance` degrees from rest.
float servoMotionEstimateMs(const ServoMotionConfig &config, float distance);

#endif // SERVO_MOTION_H
