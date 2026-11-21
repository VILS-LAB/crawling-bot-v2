#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include "ServoHealth.h"
#include "ServoMotion.h"

// Safe servo driver for the two leg axes.
//
// Every public move goes through a trapezoidal motion profile (see
// ServoMotion.h): the axis accelerates, cruises and decelerates into the
// target, so a student can never command a full-speed jump by accident. The
// driver also remembers where each axis is, so a new target always starts from
// the real commanded position instead of snapping.
//
// Two ways to use it:
//
//   Non-blocking (preferred in loop()):
//       servos.moveDownTo(120);
//       ...
//       servos.update();          // call every loop()
//
//   Blocking (fine in setup() or a simple sketch):
//       servos.moveDownSmooth(120);
//
// Both paths are ramped. `moveDown()` / `moveUp()` are kept for compatibility
// and now behave like the blocking smooth versions.
class ServoControl {
public:
    ServoControl(uint8_t pinDown, uint8_t pinUp);

    // Attaches both PWM channels and parks the servos at their initial angle.
    // Returns false if either channel could not be attached; the per-servo
    // result is available from checkDown()/checkUp().
    //
    // The park write is the one unavoidable instant write: at power-on the
    // firmware cannot know where the horn physically is. Place the legs at the
    // park pose before switching the robot on.
    bool begin();

    // ---- Non-blocking API (preferred) -------------------------------------
    // Sets a new target. The axis ramps toward it from its current position;
    // calling this again mid-move simply retargets without a jump.
    void moveDownTo(int angle);
    void moveUpTo(int angle);
    void moveBothTo(int downAngle, int upAngle);

    // Advances both motion profiles and writes the PWM channels. Call this
    // from loop() as often as possible; it is cheap and never blocks.
    void update();

    // True while either axis is still travelling.
    bool isMoving() const;

    // Freezes both axes where they are (target := current position).
    void stop();

    // ---- Blocking helpers (still ramped) ----------------------------------
    // Blocks until the axis reaches the target. Returns false only if the
    // safety timeout expired (which means something is mechanically stuck).
    //
    // `stepDelayMs` is the legacy tuning knob from the old step-and-delay
    // implementation: it is converted to a cruise speed of 2 deg per
    // stepDelayMs, so existing sketches keep the same feel. Leave it at 0 to
    // use the speed configured with setSpeed() / SERVO_MAX_SPEED_DPS.
    bool moveDownSmooth(int targetAngle, int stepDelayMs = 0);
    bool moveUpSmooth(int targetAngle, int stepDelayMs = 0);
    bool moveBothSmooth(int downAngle, int upAngle, int stepDelayMs = 0);

    // Blocks until both axes are idle. timeoutMs == 0 estimates the time from
    // the motion profile plus a margin.
    bool waitUntilIdle(uint32_t timeoutMs = 0);

    // ---- Legacy names, now safe -------------------------------------------
    void moveDown(int angle) { moveDownSmooth(angle); }
    void moveUp(int angle) { moveUpSmooth(angle); }
    void setInitialPosition();
    void setTestPosition();

    // ---- Limits and tuning ------------------------------------------------
    // Soft limits are enforced on every target and every profile step, so a
    // bad angle from a student's policy is clamped instead of slamming the leg
    // into the ground.
    void setDownLimits(int minAngle, int maxAngle);
    void setUpLimits(int minAngle, int maxAngle);
    void setSpeed(float degreesPerSecond);
    void setAccel(float degreesPerSecondSquared);

    int getCurrentDownAngle() const;
    int getCurrentUpAngle() const;
    int getDownTarget() const;
    int getUpTarget() const;

    uint8_t getPinDown() const { return pinDown; }
    uint8_t getPinUp() const { return pinUp; }
    bool isDownAttached();
    bool isUpAttached();

    // Electrical self-test: writes the current angle and reads the pulse width
    // back through the PWM peripheral. This proves the firmware/PWM path is
    // alive; it cannot prove the servo itself moved (see HealthCheck).
    ServoHealthCode checkDown();
    ServoHealthCode checkUp();

private:
    Servo servoDown;
    Servo servoUp;
    uint8_t pinDown;
    uint8_t pinUp;
    bool downAttached;
    bool upAttached;

    ServoMotionState downState;
    ServoMotionState upState;
    ServoMotionConfig motionConfig;

    uint32_t lastUpdateMs;
    int lastWrittenDown;
    int lastWrittenUp;

    static const int INITIAL_DOWN_ANGLE = 180;
    static const int INITIAL_UP_ANGLE = 180;

    void writeDown(int angle);
    void writeUp(int angle);
    void applyLegacyStepDelay(int stepDelayMs);
    uint32_t estimateTimeoutMs() const;
    ServoHealthCode checkServo(Servo &servo, int angle);
};

#endif // SERVO_CONTROL_H
