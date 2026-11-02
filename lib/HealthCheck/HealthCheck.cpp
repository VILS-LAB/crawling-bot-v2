#include "HealthCheck.h"
#include "../../include/config.h"

HealthCheck::HealthCheck(Display *display, AHRS *ahrs, ServoControl *servoControl)
    : display(display), ahrs(ahrs), servoControl(servoControl)
{
}

void HealthCheck::run()
{
    display->clear();
    display->setCursor(0, 0);
    display->print("Health Check");
    display->refresh();
    delay(500);

    // Check AHRS
    display->clear();
    display->setCursor(0, 0);
    if (ahrs->isMoving())
    {
        display->print("AHRS: Moving");
    }
    else
    {
        display->print("AHRS: Still");
    }

    // Display orientation
    display->setCursor(0, 16);
    display->print("R:");
    display->print((int)ahrs->getRoll());
    display->print(" P:");
    display->print((int)ahrs->getPitch());
    display->refresh();
    delay(500);

    // Servo hardware can be disabled while the mechanics are disconnected.
#if ENABLE_SERVOS
    // Test servos
    display->clear();
    display->setCursor(0, 0);
    display->print("Testing Servos");
    display->refresh();
    servoControl->setTestPosition();
    delay(500);
#else
    display->setCursor(0, 32);
    display->print("Servos: disabled");
    display->refresh();
    delay(500);
#endif

#if ENABLE_SERVOS
    servoControl->setInitialPosition();
    display->clear();
    display->setCursor(0, 0);
    display->print("Servos Reset");
    display->refresh();
    delay(500);
#endif

    display->clear();
    display->setCursor(0, 0);
    display->print("Health Check");
    display->setCursor(0, 16);
    display->print("Complete");
    display->refresh();
    delay(500);
    display->clear();
}
