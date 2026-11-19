#include "HealthCheck.h"
#include "../../include/config.h"

HealthCheck::HealthCheck(Display *display, AHRS *ahrs, ServoControl *servoControl)
    : display(display), ahrs(ahrs), servoControl(servoControl)
{
}

HealthReport HealthCheck::run()
{
    HealthReport report;

    display->clear();
    display->setCursor(0, 0);
    display->print("Health Check");
    display->refresh();
    delay(300);

    // Check AHRS
    ahrs->update();
    display->setCursor(0, 16);
    display->print("IMU: ");
    display->print(ahrs->isMoving() ? "moving" : "still");
    display->refresh();
    delay(300);

    // Servo hardware can be disabled while the mechanics are disconnected.
#if ENABLE_SERVOS
    Serial.println("HEALTH,SERVO,START");

    report.servoDown = servoControl->checkDown();
    report.servoUp = servoControl->checkUp();

#if ENABLE_SERVO_MOTION_CHECK
    if (report.servoDown == SERVO_HEALTH_OK)
        report.servoDown = checkMotion(true);
    if (report.servoUp == SERVO_HEALTH_OK)
        report.servoUp = checkMotion(false);
#endif

    reportServo("down", servoControl->getPinDown(), report.servoDown);
    reportServo("up", servoControl->getPinUp(), report.servoUp);

    servoControl->setInitialPosition();
#else
    Serial.println("HEALTH,SERVO,SKIPPED,reason=ENABLE_SERVOS=0");
#endif

    Serial.print("HEALTH,SUMMARY,ok=");
    Serial.print(report.ok() ? 1 : 0);
    Serial.print(",down=");
    Serial.print(static_cast<unsigned>(report.servoDown));
    Serial.print(",up=");
    Serial.println(static_cast<unsigned>(report.servoUp));

    display->clear();
    display->setCursor(0, 0);
    display->print("Health Check");
    display->setCursor(0, 16);
    display->print(report.ok() ? "Servos: OK" : "Servos: FAIL");
    display->refresh();
    delay(500);
    display->clear();

    return report;
}

void HealthCheck::reportServo(const char *name, uint8_t pin, ServoHealthCode code)
{
    Serial.print("HEALTH,SERVO,");
    Serial.print(name);
    Serial.print(",pin=");
    Serial.print(pin);
    Serial.print(",code=");
    Serial.print(static_cast<unsigned>(code));
    Serial.print(",status=");
    Serial.println(servoHealthName(code));
}

// Without current or position feedback the only proof that a servo actually
// moved is the reaction the IMU sees. Sweep a few degrees and compare the peak
// gyro rate against the noise floor measured just before the sweep.
ServoHealthCode HealthCheck::checkMotion(bool down)
{
    const int from = down ? servoControl->getCurrentDownAngle()
                          : servoControl->getCurrentUpAngle();
    const int to = constrain(from - SERVO_HEALTH_SWEEP_DEG, 0, 180);

    const float baseline = measureBaseline(SERVO_HEALTH_BASELINE_MS);
    const float peak = sweep(down, from, to);
    sweep(down, to, from);

    return (peak > baseline + SERVO_HEALTH_MOTION_DPS) ? SERVO_HEALTH_OK
                                                       : SERVO_HEALTH_NO_MOTION;
}

float HealthCheck::gyroMagnitude()
{
    ahrs->update();
    const float x = ahrs->getGyroX();
    const float y = ahrs->getGyroY();
    const float z = ahrs->getGyroZ();
    return sqrtf(x * x + y * y + z * z);
}

float HealthCheck::measureBaseline(uint32_t windowMs)
{
    float peak = 0.0f;
    const uint32_t start = millis();
    while (millis() - start < windowMs)
    {
        const float magnitude = gyroMagnitude();
        if (magnitude > peak)
            peak = magnitude;
        delay(5);
    }
    return peak;
}

float HealthCheck::sweep(bool down, int fromAngle, int toAngle)
{
    const int step = (toAngle >= fromAngle) ? 2 : -2;
    float peak = 0.0f;
    int angle = fromAngle;

    while (true)
    {
        if (down)
            servoControl->moveDown(angle);
        else
            servoControl->moveUp(angle);

        const float magnitude = gyroMagnitude();
        if (magnitude > peak)
            peak = magnitude;

        if (angle == toAngle)
            break;

        angle += step;
        if ((step > 0 && angle > toAngle) || (step < 0 && angle < toAngle))
            angle = toAngle;
        delay(SERVO_HEALTH_SWEEP_STEP_DELAY_MS);
    }

    return peak;
}
