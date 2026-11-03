#ifndef CONFIG_H
#define CONFIG_H

// Pin Configuration
#define SERVO_PIN_DOWN 32
#define SERVO_PIN_UP 33

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
#define ENABLE_SERVOS 0
#define TELEMETRY_INTERVAL_MS 100
#define DISPLAY_PAGE_INTERVAL_MS 3000
#define IMU_CALIBRATION_TIMEOUT_MS 90000
#define DEADBAND_MODE 2
#define DEADBAND_MS2 0.01f

// EEPROM Configuration
#define EEPROM_SIZE 1
#define ROBOT_NUM_ADDR 0
#define MIN_ROBOT_NUM 1
#define MAX_ROBOT_NUM 8

#endif // CONFIG_H
