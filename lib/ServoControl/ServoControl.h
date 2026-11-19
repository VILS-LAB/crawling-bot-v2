#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include "ServoHealth.h"

class ServoControl {
public:
    ServoControl(uint8_t pinDown, uint8_t pinUp);

    // Attaches both PWM channels and parks the servos at their initial angle.
    // Returns false if either channel could not be attached; the per-servo
    // result is available from checkDown()/checkUp().
    bool begin();

    void moveDown(int angle);
    void moveUp(int angle);
    void moveDownSmooth(int targetAngle, int stepDelay = 10);
    void moveUpSmooth(int targetAngle, int stepDelay = 10);
    void setInitialPosition();
    void setTestPosition();
    int getCurrentDownAngle();
    int getCurrentUpAngle();

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
    int currentDownAngle;
    int currentUpAngle;
    bool downAttached;
    bool upAttached;

    static const int INITIAL_DOWN_ANGLE = 180;
    static const int INITIAL_UP_ANGLE = 180;

    void moveServoSmooth(Servo &servo, int &currentAngle, int targetAngle, int stepDelay);
    ServoHealthCode checkServo(Servo &servo, int angle);
};

#endif // SERVO_CONTROL_H
