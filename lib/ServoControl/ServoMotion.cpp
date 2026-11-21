#include "ServoMotion.h"

#include <math.h>

namespace
{
// "At the target" tolerance. Kept tiny on purpose: the profile snaps the angle
// exactly onto the target when it lands, so a loose epsilon here would let
// servoMotionIdle() report "done" while the axis is still short of the target.
constexpr float kAngleEpsilon = 1.0e-4f; // degrees
constexpr float kVelocityEpsilon = 0.5f; // degrees per second
} // namespace

float servoMotionClamp(const ServoMotionState &state, float angle)
{
    if (angle < state.minAngle)
        return state.minAngle;
    if (angle > state.maxAngle)
        return state.maxAngle;
    return angle;
}

bool servoMotionIdle(const ServoMotionState &state)
{
    return fabsf(state.target - state.angle) <= kAngleEpsilon &&
           fabsf(state.velocity) <= kVelocityEpsilon;
}

float servoMotionStep(ServoMotionState &state, const ServoMotionConfig &config, float dt)
{
    if (!(dt > 0.0f))
        return state.angle;

    // Guard against a zero/negative config: a zero ramp would divide by zero
    // and a zero speed would leave the axis stuck forever.
    const float maxSpeed = fmaxf(config.maxSpeed, 1.0f);
    const float accel = fmaxf(config.accel, 1.0f);
    const float maxDeltaV = accel * dt;

    const float error = state.target - state.angle;
    const float distance = fabsf(error);

    if (distance <= kAngleEpsilon)
    {
        // At the target. Bleed off any leftover velocity at the acceleration
        // limit instead of zeroing it in one step: an instant stop would be an
        // infinite deceleration and a visible jerk on the leg.
        state.angle = state.target;
        if (fabsf(state.velocity) <= maxDeltaV)
            state.velocity = 0.0f;
        else
            state.velocity -= (state.velocity > 0.0f) ? maxDeltaV : -maxDeltaV;
        return state.angle;
    }

    // Discrete-time braking limit.
    //
    // The continuous limit sqrt(2*a*d) assumes the velocity can drop
    // instantaneously, so a discrete profile that uses it always brakes too
    // late and lands with residual speed. Solving the discrete stopping
    // distance (v^2/2a + v*dt/2 = d) for v gives the limit below, which makes
    // the axis arrive with ~zero speed instead of snapping to a stop.
    const float speedLimit =
        (-accel * dt + sqrtf(accel * accel * dt * dt + 8.0f * accel * distance)) * 0.5f;

    const float direction = (error > 0.0f) ? 1.0f : -1.0f;
    const float desiredSpeed = direction * fminf(maxSpeed, speedLimit);

    // Ramp the velocity toward the desired speed, never faster than `accel`.
    float deltaV = desiredSpeed - state.velocity;
    if (deltaV > maxDeltaV)
        deltaV = maxDeltaV;
    else if (deltaV < -maxDeltaV)
        deltaV = -maxDeltaV;
    state.velocity += deltaV;

    const float step = state.velocity * dt;
    if (fabsf(step) >= distance)
    {
        // The next step would reach or pass the target: land exactly on it and
        // let the at-target branch above decelerate the residual velocity.
        state.angle = state.target;
    }
    else
    {
        state.angle += step;
    }

    // Soft limits win over the profile: never command past them.
    if (state.angle < state.minAngle)
    {
        state.angle = state.minAngle;
        state.velocity = 0.0f;
    }
    else if (state.angle > state.maxAngle)
    {
        state.angle = state.maxAngle;
        state.velocity = 0.0f;
    }

    return state.angle;
}

float servoMotionEstimateMs(const ServoMotionConfig &config, float distance)
{
    distance = fabsf(distance);
    if (distance <= 0.0f)
        return 0.0f;

    const float maxSpeed = fmaxf(config.maxSpeed, 1.0f);
    const float accel = fmaxf(config.accel, 1.0f);

    // Distance covered while accelerating from 0 to maxSpeed.
    const float rampDistance = (maxSpeed * maxSpeed) / (2.0f * accel);

    float seconds;
    if (distance <= 2.0f * rampDistance)
    {
        // Triangular profile: accelerate then decelerate, never reaching maxSpeed.
        seconds = 2.0f * sqrtf(distance / accel);
    }
    else
    {
        seconds = distance / maxSpeed + maxSpeed / accel;
    }

    return seconds * 1000.0f;
}
