#include <Arduino.h>
#include <stdlib.h>
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

// ---------------------------------------------------------------------------
// Servo test console
//
// Type commands in the serial monitor (115200 baud, newline terminated):
//
//   d <angle>   move the down leg to <angle> (0-180)
//   u <angle>   move the up leg to <angle>
//   b <d> <u>   move both legs
//   s           stop both legs where they are
//   p           go to the park (initial) pose
//   t           go to the test pose (90/90)
//   ?           print the current angles and targets
//
// Every move goes through the ramped profile, so even a typo like "d 180"
// cannot slam the leg into the ground. The IMU console menu ('q') still works;
// this one ignores input while that menu is open.
// ---------------------------------------------------------------------------
constexpr size_t kServoCommandLength = 32;
char servoCommand[kServoCommandLength];
size_t servoCommandLength = 0;

void printServoHelp()
{
    Serial.println("SERVO,HELP,d <angle> | u <angle> | b <down> <up> | s | p | t | ?");
}

void printServoState()
{
    Serial.print("SERVO,STATE,down=");
    Serial.print(servos.getCurrentDownAngle());
    Serial.print(",downTarget=");
    Serial.print(servos.getDownTarget());
    Serial.print(",up=");
    Serial.print(servos.getCurrentUpAngle());
    Serial.print(",upTarget=");
    Serial.print(servos.getUpTarget());
    Serial.print(",moving=");
    Serial.println(servos.isMoving() ? 1 : 0);
}

void handleServoCommand(char *line)
{
    // strtok skips repeated spaces, so "d   90" works too.
    char *verb = strtok(line, " \t");
    if (verb == nullptr)
        return;

    switch (verb[0])
    {
    case 'd':
    case 'D':
    {
        char *arg = strtok(nullptr, " \t");
        if (arg == nullptr)
        {
            Serial.println("SERVO,ERR,missing angle");
            return;
        }
        servos.moveDownTo(atoi(arg));
        Serial.print("SERVO,DOWN,target=");
        Serial.println(servos.getDownTarget());
        break;
    }
    case 'u':
    case 'U':
    {
        char *arg = strtok(nullptr, " \t");
        if (arg == nullptr)
        {
            Serial.println("SERVO,ERR,missing angle");
            return;
        }
        servos.moveUpTo(atoi(arg));
        Serial.print("SERVO,UP,target=");
        Serial.println(servos.getUpTarget());
        break;
    }
    case 'b':
    case 'B':
    {
        char *downArg = strtok(nullptr, " \t");
        char *upArg = strtok(nullptr, " \t");
        if (downArg == nullptr || upArg == nullptr)
        {
            Serial.println("SERVO,ERR,usage: b <down> <up>");
            return;
        }
        servos.moveBothTo(atoi(downArg), atoi(upArg));
        Serial.print("SERVO,BOTH,downTarget=");
        Serial.print(servos.getDownTarget());
        Serial.print(",upTarget=");
        Serial.println(servos.getUpTarget());
        break;
    }
    case 's':
    case 'S':
        servos.stop();
        Serial.println("SERVO,STOP");
        break;
    case 'p':
    case 'P':
        servos.moveBothTo(180, 180);
        Serial.println("SERVO,PARK");
        break;
    case 't':
    case 'T':
        servos.moveBothTo(90, 90);
        Serial.println("SERVO,TEST_POSE");
        break;
    case '?':
        printServoHelp();
        printServoState();
        break;
    default:
        Serial.print("SERVO,ERR,unknown command: ");
        Serial.println(verb);
        printServoHelp();
        break;
    }
}

// Must run BEFORE ahrs.serviceConsole(): that function drains the whole serial
// buffer every loop, so anything left here would be swallowed. We peek first
// and leave the IMU menu key ('q') untouched for AHRS to handle.
void serviceServoConsole()
{
    // The IMU menu owns the serial port while it is open; stay out of its way.
    if (!ahrs.isSerialTelemetryEnabled())
    {
        servoCommandLength = 0;
        return;
    }

    while (Serial.available())
    {
        const char character = static_cast<char>(Serial.peek());

        // 'q' opens the IMU calibration menu; let AHRS consume it.
        if (character == 'q' || character == 'Q')
            return;

        Serial.read();

        if (character == '\r' || character == '\n')
        {
            if (servoCommandLength == 0)
                continue;
            servoCommand[servoCommandLength] = '\0';
            handleServoCommand(servoCommand);
            servoCommandLength = 0;
            continue;
        }

        if (servoCommandLength + 1 < kServoCommandLength)
            servoCommand[servoCommandLength++] = character;
        else
            servoCommandLength = 0; // overlong line: drop it
    }
}

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

#if ENABLE_SERVOS
    // Attach and park the servos before calibration: the legs must be in a
    // known, still pose while the IMU is calibrated, and the attach result is
    // what the health check reports later.
    showBootMessage("Attaching servos...");
    if (!servos.begin())
        Serial.println("ERROR,SERVO_ATTACH_FAILED");
#endif

    // Calibration runs before the sampling task starts: it needs exclusive
    // access to the sensor and it drives the OLED.
    showBootMessage("Keep still...");
    if (!ahrs.calibrate())
    {
        showBootMessage("CAL FAILED - OLED");
        Serial.println("ERROR,IMU_CALIBRATION_FAILED,OLED_AVAILABLE");
    }

    if (!ahrs.startTask())
    {
        showBootMessage("IMU TASK FAILED");
        Serial.println("ERROR,IMU_TASK_FAILED");
        while (true)
            delay(1000);
    }

    // The servo test moves the legs, so it runs after calibration: moving
    // during the still window would corrupt the gyro bias and the six-position
    // accelerometer fit.
    const HealthReport health = healthCheck.run();
    if (!health.ok())
        Serial.println("ERROR,HEALTH_CHECK_FAILED");

    showBootMessage("Ready");
    telemetryOutput.begin();
    lastScreenMs = millis();

#if ENABLE_SERVOS
    Serial.println("SERVO,READY,type ? for commands");
#endif
}

void loop()
{
    // The IMU sampling + estimation runs in its own task; this only copies the
    // latest snapshot, so the loop stays responsive for display/servo/OTA work.
    ahrs.update();

#if ENABLE_SERVOS
    // Drives the ramped motion profiles. Must run every loop so a target set
    // with moveDownTo()/moveUpTo() is reached smoothly instead of in one jump.
    servos.update();
    // Before ahrs.serviceConsole(), which drains the serial buffer.
    serviceServoConsole();
#endif

    ahrs.serviceConsole();
    telemetryOutput.update(ahrs);

    const uint32_t now = millis();

    if (now - lastScreenMs >= DISPLAY_PAGE_INTERVAL_MS)
    {
        lastScreenMs = now;
        showTelemetry();
    }
}
