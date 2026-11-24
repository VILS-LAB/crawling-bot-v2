#ifndef CONFIG_H
#define CONFIG_H

// Pin Configuration
#define SERVO_PIN_DOWN 7
#define SERVO_PIN_UP 6

// I2C Configuration (using default ESP32 pins)
// SDA - GPIO 21 (default)
// SCL - GPIO 22 (default)

// Display Configuration
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_ADDRESS 0x3C

// MPU9250 Configuration
#define MPU9250_ADDRESS 0x68
#define IMU_ACCEL_RANGE ACCEL_FS_SEL::A4G
#define IMU_GYRO_RANGE GYRO_FS_SEL::G500DPS

// Network Configuration
#define AP_PASSWORD "12345678"
#define SERIAL_BAUD_RATE 115200
#define ENABLE_SERVOS 1

// Servo motion safety.
//
// Every move is ramped (trapezoidal profile) instead of jumping to the target,
// so a student's policy cannot slam the legs into the ground or strip a gear by
// commanding a large step. Tune these once on the real mechanics:
//
//   SERVO_MAX_SPEED_DPS  cruise speed, degrees per second
//   SERVO_ACCEL_DPS2     acceleration, degrees per second squared
//
// A full 180 degree move at these defaults takes about 1.3 s. Lower the speed
// if the legs still hit the ground, raise it if the robot feels sluggish.
#define SERVO_MAX_SPEED_DPS 180.0f
#define SERVO_ACCEL_DPS2 600.0f

// Soft limits, in degrees. The driver clamps every target and every profile
// step to these, so a bad angle from a policy is ignored instead of driving the
// leg past the mechanical stop. Narrow them to the real safe range of the
// assembled robot.
#define SERVO_DOWN_MIN_ANGLE 0
#define SERVO_DOWN_MAX_ANGLE 180
#define SERVO_UP_MIN_ANGLE 0
#define SERVO_UP_MAX_ANGLE 180

// Longest time step the motion profile will integrate in one update(). If the
// loop stalls (OTA, display, a student's delay()), the profile advances by at
// most this much instead of jumping, so the servo keeps a sane speed.
#define SERVO_UPDATE_MAX_DT_MS 50

// Servo health check.
//
// The electrical check writes the current angle and reads the pulse width back
// through the PWM peripheral. The motion check sweeps the servo a few degrees
// and watches the IMU for a reaction: without current or position feedback that
// is the only way to tell a dead or disconnected servo from a working one.
#define SERVO_HEALTH_PULSE_TOLERANCE_US 60
#define SERVO_HEALTH_SETTLE_MS 250
#define SERVO_HEALTH_SWEEP_DEG 20
#define SERVO_HEALTH_SWEEP_STEP_DELAY_MS 12
#define SERVO_HEALTH_BASELINE_MS 300
#define SERVO_HEALTH_MOTION_DPS 6.0f
#define ENABLE_SERVO_MOTION_CHECK 1
#define TELEMETRY_INTERVAL_MS 100
#define DISPLAY_PAGE_INTERVAL_MS 3000
#define IMU_CALIBRATION_TIMEOUT_MS 90000
#define DEADBAND_MODE 2
#define DEADBAND_MS2 0.01f

// Still-detection accelerometer norm hysteresis, in g.
#define STILL_ACCEL_ENTER_THRESHOLD_G 0.03f
#define STILL_ACCEL_LEAVE_THRESHOLD_G 0.05f

// IMU sampling task.
//
// The IMU runs in its own FreeRTOS task so the estimator sees a stable dt even
// while the Arduino loop is busy with the OLED, servos or OTA. The ESP32-S3 has
// two cores: core 0 is where the Arduino loopTask and the WiFi/OTA stack live,
// so the IMU task is pinned to core 1 to keep it out of their way.
//
// If you ever need to free core 1 (e.g. for a heavier OTA path), set
// IMU_TASK_CORE to 0: the task then shares core 0 with the Arduino loop and the
// WiFi stack, which still works but gives a noisier dt.
#define IMU_TASK_CORE 1
#define IMU_TASK_PRIORITY 5
#define IMU_TASK_STACK 4096

// EEPROM Configuration
#define EEPROM_SIZE 1
#define ROBOT_NUM_ADDR 0
#define MIN_ROBOT_NUM 1
#define MAX_ROBOT_NUM 8

#endif // CONFIG_H
