#ifndef AHRS_H
#define AHRS_H

#include <Arduino.h>
#include <Wire.h>
#include <MPU9250.h>
#include <Preferences.h>
#include <Math.h>
#include "ImuCalibration.h"
#include "ImuFilters.h"
#include "ImuRotation.h"

class Display;

struct ImuTelemetry
{
    float accel[3];       // Calibrated accelerometer, m/s^2, body frame
    float linearAccel[3]; // Gravity-free acceleration, m/s^2, world frame
    float filteredLinearAccel[3]; // Low-pass filtered gravity-free acceleration
    float filteredGyro[3]; // Low-pass filtered gyro, deg/s
    float filteredMag[3]; // Median + Butterworth + adaptive Kalman, uT
    float gyro[3];        // Gyroscope, deg/s, body frame
    float mag[3];         // Magnetometer, uT, body frame
    float quaternion[4];  // w, x, y, z
    float roll;
    float pitch;
    float yaw;
    float temperature;
    float dt;
    bool still;
    uint8_t stillFailureMask;
};

class AHRS
{
public:
    explicit AHRS(Display *display = nullptr);
    ~AHRS();
    bool begin(uint8_t robotNumber = 1);
    void setDisplay(Display *display) { statusDisplay = display; }
    void update();
    bool calibrate();
    void serviceConsole();
    bool isSerialTelemetryEnabled() const { return serialTelemetryEnabled; }

    // Motion detection
    bool isMoving();
    void resetPosition();

    // Position (m)
    float getPositionX() { return position[0]; }
    float getPositionY() { return position[1]; }
    float getPositionZ() { return position[2]; }

    // Velocity (m/s)
    float getVelocityX() { return velocity[0]; }
    float getVelocityY() { return velocity[1]; }
    float getVelocityZ() { return velocity[2]; }
    float getSpeed() { return sqrt(velocity[0] * velocity[0] + velocity[1] * velocity[1] + velocity[2] * velocity[2]); }

    // Linear acceleration (m/s^2) - gravity removed
    float getLinearAccelX() { return linearAccel[0]; }
    float getLinearAccelY() { return linearAccel[1]; }
    float getLinearAccelZ() { return linearAccel[2]; }
    const ImuTelemetry &getTelemetry() const { return telemetry; }

    // Orientation (Euler angles)
    float getRoll() { return initialized ? mpu->getRoll() : 0; }
    float getPitch() { return initialized ? mpu->getPitch() : 0; }
    float getYaw() { return initialized ? mpu->getYaw() : 0; }

    // Accelerometer data (g)
    float getAccelX() { return initialized ? mpu->getAccX() * G_CONST : 0; }
    float getAccelY() { return initialized ? mpu->getAccY() * G_CONST : 0; }
    float getAccelZ() { return initialized ? mpu->getAccZ() * G_CONST : 0; }

    // Gyroscope data (deg/s)
    float getGyroX() { return initialized ? mpu->getGyroX() : 0; }
    float getGyroY() { return initialized ? mpu->getGyroY() : 0; }
    float getGyroZ() { return initialized ? mpu->getGyroZ() : 0; }

    // Magnetometer data (uT)
    float getMagX() { return initialized ? mpu->getMagX() : 0; }
    float getMagY() { return initialized ? mpu->getMagY() : 0; }
    float getMagZ() { return initialized ? mpu->getMagZ() : 0; }

    // Quaternion
    float getQuatW() { return initialized ? mpu->getQuaternionW() : 1; }
    float getQuatX() { return initialized ? mpu->getQuaternionX() : 0; }
    float getQuatY() { return initialized ? mpu->getQuaternionY() : 0; }
    float getQuatZ() { return initialized ? mpu->getQuaternionZ() : 0; }

    // Temperature (Celsius)
    float getTemperature() { return initialized ? mpu->getTemperature() : 0; }

private:
    void updateTelemetry(float dt);
    bool calibrateAccelerometerCloud();
    bool calibrateGyroscope();
    bool calibrateMagnetometer();
    bool checkCalibrationAtBoot();
    bool runCalibration(bool gyro, bool accelerometer, bool magnetometer);
    void printCalibrationMenu() const;
    void printCalibrationStatus() const;
    bool consumeCalibrationControl();
    bool saveCalibration();
    void clearStoredCalibration();
    bool loadCalibration();
    void showCalibrationScreen(const char *title, const char *line1 = nullptr,
                               const char *line2 = nullptr);
    void correctAccelerometer(const float input[3], float output[3]) const;
    void filterTelemetry();
    MPU9250 *mpu;
    bool initialized;
    bool isStatic;
    unsigned long stationaryStartTime;
    unsigned long lastUpdateTime;

    float velocity[3];    // m/s
    float position[3];    // m
    float linearAccel[3]; // m/s^2 (gravity removed)
    float gravityReference; // Estimated world-frame gravity magnitude
    ImuCalibrationData calibration{};
    uint8_t robotNumber;
    Preferences preferences;
    bool calibrationLoaded;
    bool serialTelemetryEnabled;
    ImuFilterPipeline accelFilter;
    ImuFilterPipeline gyroFilter;
    ImuFilterPipeline magFilter;
    ComplementaryAttitude attitude;
    ImuTelemetry telemetry{};
    Display *statusDisplay;

    // Constants
    static constexpr float G_CONST = 9.80665f;                // m/s^2
    static constexpr unsigned long STATIONARY_TIME_MS = 200; // ms - time to confirm stationary
    static constexpr uint16_t STILL_ACCEL_WINDOW_SAMPLES = 40;

    float accelNormHistory[STILL_ACCEL_WINDOW_SAMPLES] = {};
    uint16_t accelNormHistoryIndex = 0;
    uint16_t accelNormHistoryCount = 0;
    float accelNormHistorySum = 0;
    float accelNormHistorySumSquares = 0;
};

#endif // AHRS_H
