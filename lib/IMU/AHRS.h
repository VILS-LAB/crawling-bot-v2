#ifndef AHRS_H
#define AHRS_H

// IMU / attitude stack for the crawling bot.
//
// Pipeline (see docs/imu/ARCHITECTURE.md for the full story):
//
//   MPU9250 (200 Hz)
//     -> calibration (PX4 model: matrix/bias per sensor)
//     -> filters (PX4 ports: median -> notch -> 2nd order Butterworth)
//     -> attitude estimator (PX4 attitude_estimator_q port, quaternion + bias)
//     -> gravity removal via rotateBodyToWorld()  [project rotation contract]
//     -> still detection + ZUPT + dead-reckoning integration
//
// The sampling loop runs in its own FreeRTOS task pinned to IMU_TASK_CORE so
// the estimator sees a stable dt even while the Arduino loop is busy with the
// display, servos or OTA. update() only copies the latest snapshot.

#include <Arduino.h>
#include <Wire.h>
#include <MPU9250.h>

#include "ImuCalibration.h"
#include "ImuFilters.h"
#include "ImuAttitudeEstimator.h"
#include "ImuRotation.h"

class Display;

struct ImuTelemetry
{
    // --- existing columns (order is part of the CSV contract) --------------
    float accel[3];               // calibrated accelerometer, m/s^2, body frame
    float linearAccel[3];         // gravity-free acceleration, m/s^2, world frame
    float filteredLinearAccel[3]; // low-pass filtered gravity-free acceleration
    float filteredGyro[3];        // filtered gyro, deg/s
    float filteredMag[3];         // filtered magnetometer, uT
    float gyro[3];                // bias-corrected gyro, deg/s, body frame
    float mag[3];                 // calibrated magnetometer, uT, body frame
    float quaternion[4];          // w, x, y, z  (body -> world)
    float roll;                   // deg
    float pitch;                  // deg
    float yaw;                    // deg
    float temperature;            // deg C
    float dt;                     // s
    float stdA;                   // accel vector std-dev over the still window, g
    float trust;                  // 0..1, how much accel is currently fused
    float rollG;                  // gyro-only shadow roll, deg
    float pitchG;                 // gyro-only shadow pitch, deg
    bool still;
    uint8_t stillFailureMask;
    float gyroBias[3];            // calibration gyro bias, deg/s

    // --- appended columns (new in the PX4-based stack) ---------------------
    float estimatorBias[3];       // estimator integral gyro bias, deg/s
    float accelNormG;             // |accel| in g
    float magNorm;                // |mag| in uT
    float imuRateHz;              // measured sampling rate of the IMU task
    float accelCalRms;            // accel calibration residual, g
    float magCalRms;              // mag calibration residual, uT
    uint8_t magFitKind;           // 0 none, 1 sphere, 2 ellipsoid
    uint8_t estimatorInitialized; // 0/1

    // Dead-reckoning state, published so the getters never race the IMU task.
    float velocity[3];
    float position[3];
};

class AHRS
{
public:
    explicit AHRS(Display *display = nullptr);
    ~AHRS();

    bool begin(uint8_t robotNumber = 1);

    // Start / stop the dedicated sampling task (core IMU_TASK_CORE).
    bool startTask();
    void stopTask();

    void setDisplay(Display *display) { statusDisplay = display; }

    // Copy the latest snapshot produced by the sampling task. Cheap and
    // non-blocking; safe to call from the Arduino loop.
    void update();

    bool calibrate();
    void serviceConsole();
    bool isSerialTelemetryEnabled() const { return serialTelemetryEnabled; }

    // Motion detection
    bool isMoving() { return !telemetry.still; }
    void resetPosition();

    // Position (m)
    float getPositionX() { return telemetry.position[0]; }
    float getPositionY() { return telemetry.position[1]; }
    float getPositionZ() { return telemetry.position[2]; }

    // Velocity (m/s)
    float getVelocityX() { return telemetry.velocity[0]; }
    float getVelocityY() { return telemetry.velocity[1]; }
    float getVelocityZ() { return telemetry.velocity[2]; }
    float getSpeed()
    {
        return sqrtf(telemetry.velocity[0] * telemetry.velocity[0] +
                     telemetry.velocity[1] * telemetry.velocity[1] +
                     telemetry.velocity[2] * telemetry.velocity[2]);
    }

    // Linear acceleration (m/s^2), gravity removed
    float getLinearAccelX() { return telemetry.linearAccel[0]; }
    float getLinearAccelY() { return telemetry.linearAccel[1]; }
    float getLinearAccelZ() { return telemetry.linearAccel[2]; }

    const ImuTelemetry &getTelemetry() const { return telemetry; }

    // Orientation (Euler angles, deg, project convention)
    float getRoll() { return telemetry.roll; }
    float getPitch() { return telemetry.pitch; }
    float getYaw() { return telemetry.yaw; }

    // Accelerometer data (m/s^2)
    float getAccelX() { return telemetry.accel[0]; }
    float getAccelY() { return telemetry.accel[1]; }
    float getAccelZ() { return telemetry.accel[2]; }

    // Gyroscope data (deg/s)
    float getGyroX() { return telemetry.gyro[0]; }
    float getGyroY() { return telemetry.gyro[1]; }
    float getGyroZ() { return telemetry.gyro[2]; }

    // Magnetometer data (uT)
    float getMagX() { return telemetry.mag[0]; }
    float getMagY() { return telemetry.mag[1]; }
    float getMagZ() { return telemetry.mag[2]; }

    // Quaternion (body -> world)
    float getQuatW() { return telemetry.quaternion[0]; }
    float getQuatX() { return telemetry.quaternion[1]; }
    float getQuatY() { return telemetry.quaternion[2]; }
    float getQuatZ() { return telemetry.quaternion[3]; }

    float getTemperature() { return telemetry.temperature; }

    // Diagnostics
    const imu::ImuCalibrationData &getCalibration() const { return calibration; }
    bool isCalibrationLoaded() const { return calibrationLoaded; }

private:
    // --- sampling task -----------------------------------------------------
    static void taskEntry(void *argument);
    void taskLoop();
    void processSample(float dt);

    // --- pipeline stages ---------------------------------------------------
    void readAndCalibrate();
    void runFilters();
    void runEstimator(float dt);
    void updateStillDetection(float dt);
    void updateDeadReckoning(float dt);
    void publishSnapshot(float dt);

    // --- calibration -------------------------------------------------------
    bool runCalibration(bool gyro, bool accelerometer, bool magnetometer);
    bool calibrateGyroscope();
    bool calibrateAccelerometer();
    bool calibrateMagnetometer();
    bool checkCalibrationAtBoot();
    bool saveCalibration();
    void clearStoredCalibration();
    bool loadCalibration();
    void printCalibrationMenu() const;
    void printCalibrationStatus() const;
    bool consumeCalibrationControl();
    void showCalibrationScreen(const char *title, const char *line1 = nullptr,
                               const char *line2 = nullptr);

    // --- state -------------------------------------------------------------
    MPU9250 *mpu = nullptr;
    bool initialized = false;
    bool calibrationLoaded = false;
    uint8_t robotNumber = 1;
    Display *statusDisplay = nullptr;
    bool serialTelemetryEnabled = true;

    imu::ImuCalibrationData calibration{};

    // Raw + calibrated sample of the current iteration.
    float rawAccelG[3] = {};   // g
    float rawGyroDps[3] = {};  // deg/s
    float rawMagUt[3] = {};    // uT
    float accelMs2[3] = {};    // m/s^2, calibrated
    float gyroDps[3] = {};     // deg/s, bias corrected
    float initialGyroBias[3] = {}; // bias as calibrated, the runtime tracker's anchor
    float magUt[3] = {};       // uT, calibrated

    // Filters
    imu::GyroFilterPipeline gyroFilter;
    imu::AccelFilterPipeline accelFilter;
    imu::MagFilterPipeline magFilter;
    imu::Vec3 filteredGyroDps{};
    imu::Vec3 filteredAccelMs2{};
    imu::Vec3 filteredMagUt{};
    imu::LowPassFilter2pVec3 linearAccelFilter;
    float filteredLinearAccel[3] = {};

    // Estimator
    imu::AttitudeEstimator estimator;
    bool estimatorSeeded = false;

    // Still detection (the visualizers depend on these fields)
    bool isStatic = true;
    unsigned long stationaryStartTime = 0;
    unsigned long biasStillStartTime = 0;
    unsigned long lastStillTime = 0;
    float accelHistory[40][3] = {};
    uint16_t accelHistoryIndex = 0;
    uint16_t accelHistoryCount = 0;
    float accelHistorySum[3] = {};
    float accelHistorySumSquares[3] = {};

    // Dead reckoning
    float velocity[3] = {};
    float position[3] = {};
    float linearAccel[3] = {};
    float gravityReference = imu::kGravity;

    // Gyro-only shadow attitude (drift diagnostic)
    float shadowRoll = 0.0f;
    float shadowPitch = 0.0f;
    bool shadowInitialized = false;

    // Trust smoothing
    float trust = 0.0f;

    // Timing
    unsigned long lastSampleUs = 0;
    float measuredRateHz = 0.0f;

    // Task plumbing. `taskHandle` is only ever written by the creator
    // (startTask/stopTask); the task signals its own exit through taskExited so
    // the two never race on the handle.
    TaskHandle_t taskHandle = nullptr;
    volatile bool taskRunning = false;
    volatile bool taskPaused = false;
    volatile bool taskPauseAck = false;  // task has observed taskPaused
    volatile bool taskExited = false;
    portMUX_TYPE snapshotMux = portMUX_INITIALIZER_UNLOCKED;
    ImuTelemetry telemetry{};          // read by the Arduino loop
    ImuTelemetry pendingTelemetry{};   // written by the IMU task, guarded by snapshotMux
    volatile bool snapshotReady = false;

    // Still-detection results for the current sample. Task-local: only the IMU
    // task touches these, so they need no locking.
    struct StillState
    {
        float stdA = 0.0f;
        uint8_t failureMask = 0;
        bool still = true;
        float trust = 0.0f;
        float accelNormG = 1.0f;
    } stillState;

    // Constants
    static constexpr float G_CONST = imu::kGravity;
    static constexpr unsigned long STATIONARY_TIME_MS = 200;
    static constexpr unsigned long ACCEL_TRUST_TRANSITION_MS = 100;
    static constexpr unsigned long ACCEL_TRUST_FALLBACK_MS = 5000;
    static constexpr uint16_t STILL_ACCEL_WINDOW_SAMPLES = 40;
    static constexpr bool PLANAR_MODE = true;

    // Filter tuning (PX4 defaults where they exist)
    static constexpr float FILTER_ACCEL_CUTOFF_HZ = 30.0f;  // PX4 IMU_ACCEL_CUTOFF
    static constexpr float FILTER_GYRO_CUTOFF_HZ = 40.0f;   // PX4 IMU_GYRO_CUTOFF
    static constexpr float FILTER_GYRO_NOTCH_HZ = 0.0f;     // 0 disables the notch
    static constexpr float FILTER_GYRO_NOTCH_BW_HZ = 20.0f; // PX4 IMU_GYRO_NF1_BW
    static constexpr float FILTER_MAG_CUTOFF_HZ = 5.0f;
    static constexpr float FILTER_LINEAR_ACCEL_CUTOFF_HZ = 7.5f;
    static constexpr float NOMINAL_SAMPLE_HZ = 200.0f;
};

#endif // AHRS_H
