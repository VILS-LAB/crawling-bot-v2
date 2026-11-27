#include <Arduino.h>
#include <stdlib.h>
#include <Display.h>
#include <AHRS.h>
#include <ServoControl.h>
#include <Network.h>
#include <HealthCheck.h>
#include <CommandServer.h>
#include <SystemWatchdog.h>
#include "../include/config.h"

namespace
{
Display display;
AHRS ahrs;
ServoControl servos(SERVO_PIN_DOWN, SERVO_PIN_UP);
Network network(&display);
HealthCheck healthCheck(&display, &ahrs, &servos);
CommandServer commands(&ahrs, &servos, &network);

uint32_t lastScreenMs = 0;

void showBootMessage(const char *line)
{
    display.clear();
    display.print("Crawling Bot V2", 0, 0);
    display.print(line, 0, 16);
    display.refresh();
}

void showTelemetry()
{
    const ImuTelemetry &imu = ahrs.getTelemetry();
    display.clear();
    display.print("IMU ", 0, 0);
    display.print(ahrs.isMoving() ? "MOVING" : "STILL");
    display.setCursor(0, 16);
    display.print("R:");
    display.printFloat(imu.roll, 1);
    display.print(" P:");
    display.printFloat(imu.pitch, 1);
    display.print(" Y:");
    display.printFloat(imu.yaw, 1);
    display.setCursor(0, 32);
    display.print("A:");
    display.printFloat(imu.filteredLinearAccel[0], 2);
    display.print(",");
    display.printFloat(imu.filteredLinearAccel[1], 2);
    display.print(",");
    display.printFloat(imu.filteredLinearAccel[2], 2);
    display.setCursor(0, 48);
    if (!ahrs.isCalibrated())
        display.print("Uncalibrated: CAL");
    else
    {
        display.print("V:");
        display.printFloat(ahrs.getSpeed(), 2);
        display.print(" T:");
        display.printFloat(imu.temperature, 1);
        display.print("C");
    }
    display.refresh();
}

void showUncalibratedScreen()
{
    display.clear();
    display.print("Crawling Bot V2", 0, 0);
    display.print("Uncalibrated - run CAL", 0, 20);
    display.print("serial 115200", 0, 36);
    display.print("or TCP :4242", 0, 48);
    display.refresh();
}

} // namespace

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);
    delay(100);

    // The watchdog is armed first so a hang anywhere in the boot path below is
    // recovered by a reset instead of leaving the servos powered and frozen.
    // Guided calibration unsubscribes the loop task explicitly.
    watchdog::begin(WATCHDOG_TIMEOUT_S, true);

    display.begin();
    ahrs.setDisplay(&display);
    showBootMessage("Starting...");

    network.begin();
    network.startOTATask();

    showBootMessage("Initializing IMU...");
    if (!ahrs.begin(network.getRobotNumber()))
    {
        // An IMU that cannot be reached is not recovered by a watchdog reset,
        // so report it and keep the command server alive: PING/ID still work
        // and the board is reachable over OTA for a fix.
        showBootMessage("IMU FAILED");
        Serial.println("ERROR,IMU_INIT_FAILED");
    }

#if ENABLE_SERVOS
    // Attach and park before anything moves: the legs must be in a known, still
    // pose for calibration and for the health check.
    showBootMessage("Attaching servos...");
    if (!servos.begin())
        Serial.println("ERROR,SERVO_ATTACH_FAILED");
#endif

    // Boot never blocks on a human. A stored calibration is verified with a
    // short self-check; if it is missing or fails, the robot keeps running with
    // the servos parked and tells the operator to run CAL.
    const bool calibrated = ahrs.calibrate();
    if (!calibrated)
    {
        showUncalibratedScreen();
        Serial.println("ROBOT,UNCALIBRATED,send_CAL_to_calibrate");
    }

    if (!ahrs.startTask())
    {
        // The sampling task is what the watchdog protects, so there is nothing
        // safe to do without it. Report and stop here rather than feed a
        // watchdog over a dead estimator.
        showBootMessage("IMU TASK FAILED");
        Serial.println("ERROR,IMU_TASK_FAILED");
        watchdog::removeCurrentTask();
        while (true)
            delay(1000);
    }

    // The servo test moves the legs, so it only runs with a valid calibration:
    // moving during the still window would corrupt the gyro bias and the
    // six-position accelerometer fit.
    if (calibrated)
    {
        showBootMessage("Health check...");
        const HealthReport health = healthCheck.run();
        if (!health.ok())
            Serial.println("ERROR,HEALTH_CHECK_FAILED");
    }
    else
    {
        Serial.println("HEALTH,SERVO,SKIPPED,reason=uncalibrated");
    }

    commands.begin();
    commands.update();

    if (calibrated)
        showBootMessage("Ready");
    else
        showUncalibratedScreen();

    lastScreenMs = millis();
}

void loop()
{
    // The hardware watchdog resets the board if this task stops feeding it for
    // WATCHDOG_TIMEOUT_S, so every iteration must reach the feed at the bottom.
    watchdog::feed();

    // The IMU sampling + estimation runs in its own task; this only copies the
    // latest snapshot, so the loop stays responsive for display/servo/OTA work.
    ahrs.update();

#if ENABLE_SERVOS
    // Drives the ramped motion profiles. Must run every loop so a target set
    // with moveDownTo()/moveUpTo() is reached smoothly instead of in one jump.
    servos.update();
#endif

    // Serial + TCP commands, and the STREAM frames.
    commands.update();

    const uint32_t now = millis();

    if (now - lastScreenMs >= DISPLAY_PAGE_INTERVAL_MS)
    {
        lastScreenMs = now;
        if (ahrs.isCalibrated())
            showTelemetry();
        else
            showUncalibratedScreen();
    }
}
