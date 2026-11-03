#include <Arduino.h>
#include <Display.h>
#include <AHRS.h>
#include <ImuTelemetry.h>
#include <ServoControl.h>
#include <Network.h>
#include <HealthCheck.h>
#include "../include/config.h"

namespace
{
Display display;
AHRS ahrs;
ServoControl servos(SERVO_PIN_DOWN, SERVO_PIN_UP);
Network network(&display);
HealthCheck healthCheck(&display, &ahrs, &servos);

uint32_t lastScreenMs = 0;
ImuTelemetryOutput telemetryOutput;

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
    display.print("V:");
    display.printFloat(ahrs.getSpeed(), 2);
    display.print(" T:");
    display.printFloat(imu.temperature, 1);
    display.print("C");
    display.refresh();
}

} // namespace

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);
    delay(100);

    display.begin();
    ahrs.setDisplay(&display);
    showBootMessage("Starting...");

    network.begin();
    network.startOTATask();

    showBootMessage("Initializing IMU...");
    if (!ahrs.begin(network.getRobotNumber()))
    {
        showBootMessage("IMU FAILED");
        Serial.println("ERROR,IMU_INIT_FAILED");
        while (true)
            delay(1000);
    }

    showBootMessage("Keep still...");
    if (!ahrs.calibrate())
    {
        showBootMessage("CAL FAILED - OLED");
        Serial.println("ERROR,IMU_CALIBRATION_FAILED,OLED_AVAILABLE");
    }

#if ENABLE_SERVOS
    servos.begin();
#endif

    healthCheck.run();
    showBootMessage("Ready");
    telemetryOutput.begin();
    lastScreenMs = millis();
}

void loop()
{
    ahrs.update();
    ahrs.serviceConsole();
    telemetryOutput.update(ahrs);
    const uint32_t now = millis();

    if (now - lastScreenMs >= DISPLAY_PAGE_INTERVAL_MS)
    {
        lastScreenMs = now;
        showTelemetry();
    }
}
