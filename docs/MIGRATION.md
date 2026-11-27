# History and migration notes

Three eras, oldest first, plus what changed between them. Read this if you are
looking for something that used to exist and no longer does, or if you are
porting material written for an older version of the course.

| Era | Student worked by | Robot ran |
|---|---|---|
| Arduino template (`archive/old-template.ino`) | editing one `.ino` file | the student's sketch |
| PlatformIO firmware (previous) | filling in `lib/Training` and flashing | the student's algorithm |
| **Current** | writing a client that talks over WiFi/USB | identical lab-wide firmware |

The rest of this page details the first two transitions; the third is the one
that matters going forward.

## Arduino → PlatformIO

### Hardware changes

| Component | Old (Arduino) | New (PlatformIO) |
|-----------|---------------|------------------|
| Display | LCD 16x2 (I2C) | OLED 0.96" SSD1306 |
| Distance Sensor | SRF-05 Ultrasonic | MPU9250 AHRS (9-DOF IMU) |
| Board | ESP32 Dev Module | ESP32-S3-DevKitM-1 |
| Servos | 2x Servo Motors | 2x Servo Motors (same) |

### Code architecture

#### Old structure (monolithic)
```
old-template.ino
└── All code in single file
    ├── Global variables
    ├── Helper functions
    ├── setup()
    └── loop()
```

#### New structure (library-based)
```
src/main.cpp              # Main application
lib/
├── Display/              # OLED display management
├── IMU/                  # MPU9250 + AHRS pipeline
├── ServoControl/         # Servo motor control
├── Network/              # WiFi AP & OTA updates
├── CommandServer/        # serial + TCP command protocol
├── I2CBus/               # shared I2C mutex (OLED + IMU)
└── SystemWatchdog/       # esp_task_wdt wrapper
```

### API changes

#### Display

**Old (LCD):**
```cpp
LiquidCrystal_I2C lcd(0x27, 16, 2);
lcd.init();
lcd.backlight();
lcd.clear();
lcd.print("Hello");
lcd.setCursor(0, 1);
lcd.print("World");
```

**New (OLED):**
```cpp
Display display;
display.begin();
display.clear();
display.print("Hello", 0, 0);
display.print("World", 0, 16);
```

#### Distance/motion sensing

**Old (SRF-05):**
```cpp
NewPing sonar(triggerPin, echoPin, MAX_DISTANCE);
float distance = sonar.ping_cm();
if (distance == 0) {
    // No object detected
}
```

**New (MPU9250 AHRS):**
```cpp
AHRS ahrs;
ahrs.begin();
ahrs.update();  // Call in loop

// Motion detection
bool moving = ahrs.isMoving();

// Orientation
float roll = ahrs.getRoll();
float pitch = ahrs.getPitch();
float yaw = ahrs.getYaw();

// Displacement tracking
float dx = ahrs.getDisplacementX();
float dy = ahrs.getDisplacementY();

// Raw sensor data
float accelX = ahrs.getAccelX();
float gyroZ = ahrs.getGyroZ();
float magX = ahrs.getMagX();
```

#### Servo control

**Old:**
```cpp
Servo servodown;
Servo servoup;
servodown.attach(control_servo_down, 600, 2400);
servoup.attach(control_servo_up, 600, 2400);
servodown.write(90);

// Smooth movement
moveServoSmooth(servodown, 0, 90, 10);
```

**New:**
```cpp
ServoControl servos(SERVO_PIN_DOWN, SERVO_PIN_UP);
servos.begin();
servos.moveDown(90);

// Smooth movement
servos.moveDownSmooth(90, 10);
servos.setInitialPosition();
```

#### Network and OTA

**Old:**
```cpp
// Separate functions
setup_ap();
setup_ota();

// Manual task creation
xTaskCreatePinnedToCore(otaTask, "OTATask", ...);
```

**New:**
```cpp
Network network(&display);
network.begin();          // Sets up AP and OTA
network.startOTATask();   // Starts FreeRTOS task
uint8_t num = network.getRobotNumber();
```

#### Pin definitions

**Old:**
```cpp
const int triggerPin = 5;        // SRF-05 trigger
const int echoPin = 17;          // SRF-05 echo
const int control_servo_down = 32;
const int control_servo_up = 33;
```

**New:**
```cpp
// config.h
#define SERVO_PIN_DOWN 7
#define SERVO_PIN_UP 6
// I2C pins are defaults (SDA: 21, SCL: 22)
// No ultrasonic sensor pins needed
```

## PlatformIO → protocol-driven firmware

### Feature improvements

#### Enhanced sensing

The MPU9250 provides far more data than the SRF-05:

**Old capabilities:**
- Distance measurement only
- Limited to line-of-sight

**New capabilities:**
- 9-DOF motion sensing (accel, gyro, mag)
- Orientation (roll, pitch, yaw)
- Quaternion representation
- Displacement tracking
- Motion detection
- Temperature sensing
- No line-of-sight required

#### Better display

**Old (LCD 16x2):**
- 16 characters × 2 lines
- Text only
- Backlight control

**New (OLED 0.96"):**
- 128×64 pixels
- Graphics capable
- Progress bars
- Better contrast
- More information density

#### Modular architecture

**Benefits:**
- Easier to test individual components
- Better code organization
- Reusable libraries
- Simpler maintenance
- Clear separation of concerns

### The student interface: protocol instead of a code template

**Old:** students filled in `lib/Training/Training.cpp` and reflashed the robot.

**New:** the robot is a service. Student code lives outside this repository and
talks to it over serial or TCP:

```text
GET        -> OK SNAP,...        observe
ACT 120 60 -> OK ACT d=120 u=60  act
STREAM 50  -> OK stream=50       log
```

Why this changed:

| Old model | New model |
|---|---|
| a bug in student code bricks the robot | a bug disconnects a client; the robot keeps running |
| every student needs the toolchain | any language, any machine on the WiFi |
| results are not comparable | every robot runs identical firmware |
| the firmware must compile the learner | the learner runs where it is easy to debug |

`lib/Training` was removed accordingly. See [PROTOCOL.md](PROTOCOL.md) and
[languages/](languages/README.md).

### Migration steps for developers

1. **Update platformio.ini** with new dependencies
2. **Create library structure** in `lib/` directory
3. **Port display code** from LCD to OLED API
4. **Replace distance sensing** with AHRS motion tracking
5. **Wrap servo control** in dedicated class
6. **Encapsulate networking** in Network class
7. **Update main.cpp** to use new libraries
8. **Test each module** independently
9. **Add your policy module** when ready (see EXAMPLES.md)

### Backward compatibility

This is a **breaking change** - the old code will not run on the new hardware without modification because:

1. Different display driver (LCD → OLED)
2. Different sensor (ultrasonic → IMU)
3. Different library dependencies
4. Different architecture

However, all **functionality** has been preserved or enhanced:
- ✅ Display output
- ✅ Motion/distance detection (improved)
- ✅ Servo control
- ✅ WiFi AP
- ✅ OTA updates
- ✅ Robot number configuration
- ✅ Health checks
- ✅ Serial + TCP command protocol (new)
- ✅ Hardware watchdog and shared I2C bus (new)

### Build process changes

**Old:**
- Arduino IDE
- Manual library installation
- Board selection in IDE
- Upload via USB

**New:**
- PlatformIO (CLI or IDE)
- Automatic dependency management
- Board configured in platformio.ini
- Upload via USB or OTA

## Next steps

For the firmware platform:
1. Test all hardware components
2. Calibrate the MPU9250 (see [CALIBRATION.md](CALIBRATION.md))
3. Fine-tune servo speed and limits for your mechanics
4. Customize the display pages

For a student project (no firmware change required):
1. Write a client against [PROTOCOL.md](PROTOCOL.md) in any language
2. Start from [examples/python](../examples/python/README.md)

## Questions?

| Topic | Document |
|---|---|
| the interface | [PROTOCOL.md](PROTOCOL.md) |
| building and flashing | [BUILD.md](BUILD.md) |
| firmware internals | [ARCHITECTURE.md](ARCHITECTURE.md) |
| hardware | [HARDWARE.md](HARDWARE.md) |
| working on the firmware | [DEVELOPMENT.md](DEVELOPMENT.md) |
