#include "AHRS.h"
#include "../../include/config.h"
#include <Display.h>

namespace
{
// --- calibration tuning ----------------------------------------------------
constexpr float GYRO_CAL_ACCEL_MIN_G = 0.85f;
constexpr float GYRO_CAL_ACCEL_MAX_G = 1.15f;
constexpr float GYRO_CAL_MAX_RATE_DPS = 8.0f;
constexpr uint32_t GYRO_CAL_SETTLE_MS = 1500;
constexpr uint16_t GYRO_CAL_SAMPLES = 2500;
constexpr uint16_t GYRO_CAL_MIN_SAMPLES = 2000;
constexpr float GYRO_CAL_MAX_SIGMA_DPS = 0.20f;
// PX4 gyro_calibration.cpp: max |median - mean| before the run is rejected.
constexpr float GYRO_CAL_MAX_MEDIAN_ERROR_DPS = 0.6f;
constexpr uint8_t GYRO_CAL_MAX_RETRIES = 5;

constexpr uint16_t CAL_FACE_SAMPLES = 180;
constexpr uint32_t CAL_FACE_TIMEOUT_MS = 120000;
constexpr uint32_t CAL_SAMPLE_INTERVAL_MS = 10;

constexpr uint32_t MAG_CAL_DURATION_MS = 20000;
constexpr uint16_t MAG_CAL_MAX_SAMPLES = 1200;
constexpr float MAG_CAL_MIN_SAMPLE_DISTANCE_UT = 3.0f;

// --- still detection -------------------------------------------------------
constexpr float STILL_GYRO_NORM_MAX_DPS = 0.5f;
constexpr float STILL_ACCEL_VECTOR_STDDEV_MAX_G = 0.004f;
constexpr uint8_t STILL_FAILURE_ACCEL_NORM = 1u << 0;
constexpr uint8_t STILL_FAILURE_GYRO_NORM = 1u << 1;
constexpr uint8_t STILL_FAILURE_ACCEL_STDDEV = 1u << 2;

// --- runtime gyro bias tracking -------------------------------------------
constexpr uint32_t GYRO_BIAS_UPDATE_HOLD_MS = 1000;
constexpr float GYRO_BIAS_UPDATE_ALPHA = 0.001f;
constexpr float GYRO_BIAS_MAX_DRIFT_DPS = 0.5f;

// --- boot self-check -------------------------------------------------------
constexpr uint16_t CHECK_SAMPLES = 200;
constexpr float MAX_ACCEL_NORM_ERROR_G = 0.05f;
constexpr float MAX_GYRO_NORM_DPS = 1.0f;
constexpr float MAX_MAG_FIELD_DELTA_PERCENT = 0.20f;
constexpr float MAX_TEMPERATURE_DELTA_C = 15.0f;

// AK8963 output is milliGauss; 1 uT = 10 mG.
constexpr float MG_TO_UT = 0.1f;

constexpr float DEG_TO_RAD_F = imu::kDegToRad;
constexpr float RAD_TO_DEG_F = imu::kRadToDeg;
} // namespace

AHRS::AHRS(Display *display) : statusDisplay(display)
{
    mpu = new MPU9250();
    calibration.robotNumber = 1;
    calibration.accelSides = 0;
    calibration.magFitKind = 0;

    imu::AttitudeEstimatorConfig estimatorConfig;
    estimator.configure(estimatorConfig);

    gyroFilter.configure(NOMINAL_SAMPLE_HZ, FILTER_GYRO_CUTOFF_HZ, FILTER_GYRO_NOTCH_HZ,
                         FILTER_GYRO_NOTCH_BW_HZ);
    accelFilter.configure(NOMINAL_SAMPLE_HZ, FILTER_ACCEL_CUTOFF_HZ);
    magFilter.configure(100.0f, FILTER_MAG_CUTOFF_HZ);
    linearAccelFilter.setCutoffFrequency(NOMINAL_SAMPLE_HZ, FILTER_LINEAR_ACCEL_CUTOFF_HZ);
}

AHRS::~AHRS()
{
    stopTask();
    delete mpu;
}

bool AHRS::begin(uint8_t requestedRobotNumber)
{
    robotNumber = (requestedRobotNumber >= 1 && requestedRobotNumber <= 8)
                      ? requestedRobotNumber
                      : 1;

    Wire.begin();

    MPU9250Setting setting;
    setting.accel_fs_sel = IMU_ACCEL_RANGE;
    setting.gyro_fs_sel = IMU_GYRO_RANGE;
    setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
    setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
    setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

    if (!mpu->setup(MPU9250_ADDRESS, setting))
        return false;

    // The library ships its own Madgwick filter and its own mag calibration.
    // Both are replaced by this stack, so switch them off to save CPU and to
    // keep the raw magnetometer values available for our own calibration.
    mpu->selectFilter(QuatFilterSel::NONE);
    mpu->setMagBias(0.0f, 0.0f, 0.0f);
    mpu->setMagScale(1.0f, 1.0f, 1.0f);

    initialized = true;
    lastSampleUs = micros();

    if (loadCalibration())
    {
        for (int i = 0; i < 3; ++i)
            initialGyroBias[i] = calibration.gyroBias[i];
        Serial.print("CAL,LOADED,robot=");
        Serial.println(robotNumber);
    }
    else
    {
        Serial.println("CAL,NONE,using_identity");
    }

    return true;
}

// ---------------------------------------------------------------------------
// Sampling task
// ---------------------------------------------------------------------------
bool AHRS::startTask()
{
    if (taskHandle != nullptr)
        return true;

    taskExited = false;
    taskRunning = true;
    const BaseType_t created = xTaskCreatePinnedToCore(
        taskEntry, "imu", IMU_TASK_STACK, this, IMU_TASK_PRIORITY, &taskHandle,
        IMU_TASK_CORE);

    if (created != pdPASS)
    {
        taskHandle = nullptr;
        taskRunning = false;
        Serial.println("ERROR,IMU_TASK_CREATE_FAILED");
        return false;
    }
    return true;
}

void AHRS::stopTask()
{
    if (taskHandle == nullptr)
        return;

    taskRunning = false;

    // Wait for the task to actually leave its loop before the caller is allowed
    // to tear down anything it touches (the sensor object, this instance).
    for (int i = 0; i < 250 && !taskExited; ++i)
        delay(2);

    if (!taskExited)
    {
        // The task is wedged (should not happen). Deleting it from here would
        // leave the sensor in an unknown state, so report and keep the handle.
        Serial.println("ERROR,IMU_TASK_STOP_TIMEOUT");
        return;
    }

    taskHandle = nullptr;
}

void AHRS::taskEntry(void *argument)
{
    static_cast<AHRS *>(argument)->taskLoop();
}

void AHRS::taskLoop()
{
    while (taskRunning)
    {
        if (taskPaused)
        {
            // Keep the timestamp fresh so the first sample after a pause does
            // not look like a multi-second stall.
            lastSampleUs = micros();
            taskPauseAck = true;
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        taskPauseAck = false;

        if (!mpu->update())
        {
            vTaskDelay(1);
            continue;
        }

        const unsigned long nowUs = micros();
        float dt = (nowUs - lastSampleUs) * 1e-6f;
        lastSampleUs = nowUs;

        // Reject absurd dt (first sample, or a long stall) instead of
        // integrating garbage into the estimator.
        if (dt <= 0.0f || dt > 0.1f)
            continue;

        // Track the real sampling rate with a slow alpha filter so the
        // telemetry shows what the sensor is actually doing.
        const float instantRate = 1.0f / dt;
        measuredRateHz = (measuredRateHz <= 0.0f)
                             ? instantRate
                             : measuredRateHz + 0.01f * (instantRate - measuredRateHz);

        processSample(dt);
    }

    taskExited = true;
    vTaskDelete(nullptr);
}

void AHRS::processSample(float dt)
{
    readAndCalibrate();
    runFilters();
    runEstimator(dt);
    updateStillDetection(dt);
    updateDeadReckoning(dt);
    publishSnapshot(dt);
}

// ---------------------------------------------------------------------------
// Stage 1: read + calibrate
// ---------------------------------------------------------------------------
void AHRS::readAndCalibrate()
{
    rawAccelG[0] = mpu->getAccX();
    rawAccelG[1] = mpu->getAccY();
    rawAccelG[2] = mpu->getAccZ();

    rawGyroDps[0] = mpu->getGyroX();
    rawGyroDps[1] = mpu->getGyroY();
    rawGyroDps[2] = mpu->getGyroZ();

    rawMagUt[0] = mpu->getMagX() * MG_TO_UT;
    rawMagUt[1] = mpu->getMagY() * MG_TO_UT;
    rawMagUt[2] = mpu->getMagZ() * MG_TO_UT;

    imu::calibration::applyAccelCorrection(rawAccelG, calibration, accelMs2);
    accelMs2[0] *= G_CONST;
    accelMs2[1] *= G_CONST;
    accelMs2[2] *= G_CONST;

    for (int i = 0; i < 3; ++i)
        gyroDps[i] = rawGyroDps[i] - calibration.gyroBias[i];

    imu::calibration::applyMagCorrection(rawMagUt, calibration, magUt);
}

// ---------------------------------------------------------------------------
// Stage 2: filters
// ---------------------------------------------------------------------------
void AHRS::runFilters()
{
    filteredGyroDps = gyroFilter.apply(imu::Vec3{gyroDps});
    filteredAccelMs2 = accelFilter.apply(imu::Vec3{accelMs2});
    filteredMagUt = magFilter.apply(imu::Vec3{magUt});
}

// ---------------------------------------------------------------------------
// Stage 3: attitude estimator
// ---------------------------------------------------------------------------
void AHRS::runEstimator(float dt)
{
    const imu::Vec3 accel{accelMs2};
    const imu::Vec3 gyroRad{gyroDps[0] * DEG_TO_RAD_F, gyroDps[1] * DEG_TO_RAD_F,
                            gyroDps[2] * DEG_TO_RAD_F};
    const imu::Vec3 mag{magUt};

    if (!estimatorSeeded)
    {
        // Wait for a plausible gravity vector before seeding, otherwise the
        // initial attitude is built from noise.
        const float accelNorm = accel.norm();
        if (accelNorm > 0.5f * G_CONST && accelNorm < 1.5f * G_CONST)
        {
            estimatorSeeded = estimator.initialize(accel, mag);
            if (estimatorSeeded)
                Serial.println("IMU,ESTIMATOR,initialized");
        }
        if (!estimatorSeeded)
            return;
    }

    estimator.update(accel, gyroRad, mag, dt);
}

// ---------------------------------------------------------------------------
// Stage 4: still detection
// ---------------------------------------------------------------------------
void AHRS::updateStillDetection(float dt)
{
    const float accelNormG =
        sqrtf(accelMs2[0] * accelMs2[0] + accelMs2[1] * accelMs2[1] +
              accelMs2[2] * accelMs2[2]) /
        G_CONST;
    const float gyroNorm =
        sqrtf(gyroDps[0] * gyroDps[0] + gyroDps[1] * gyroDps[1] + gyroDps[2] * gyroDps[2]);

    const uint16_t index = accelHistoryIndex;
    if (accelHistoryCount == STILL_ACCEL_WINDOW_SAMPLES)
    {
        for (uint8_t axis = 0; axis < 3; ++axis)
        {
            accelHistorySum[axis] -= accelHistory[index][axis];
            accelHistorySumSquares[axis] -=
                accelHistory[index][axis] * accelHistory[index][axis];
        }
    }
    for (uint8_t axis = 0; axis < 3; ++axis)
    {
        const float value = accelMs2[axis] / G_CONST;
        accelHistory[index][axis] = value;
        accelHistorySum[axis] += value;
        accelHistorySumSquares[axis] += value * value;
    }
    accelHistoryIndex = (accelHistoryIndex + 1) % STILL_ACCEL_WINDOW_SAMPLES;
    if (accelHistoryCount < STILL_ACCEL_WINDOW_SAMPLES)
        ++accelHistoryCount;

    float accelMeanNormG = 0.0f;
    if (accelHistoryCount > 0)
    {
        for (uint16_t i = 0; i < accelHistoryCount; ++i)
        {
            const float x = accelHistory[i][0];
            const float y = accelHistory[i][1];
            const float z = accelHistory[i][2];
            accelMeanNormG += sqrtf(x * x + y * y + z * z);
        }
        accelMeanNormG /= accelHistoryCount;
    }

    uint8_t stillFailureMask = 0;
    const float accelNormErrorG = fabsf(accelMeanNormG - 1.0f);
    const bool accelNormPass =
        isStatic
            ? accelNormErrorG <= STILL_ACCEL_LEAVE_THRESHOLD_G
            : accelNormErrorG < STILL_ACCEL_ENTER_THRESHOLD_G;
    if (!accelNormPass)
        stillFailureMask |= STILL_FAILURE_ACCEL_NORM;
    if (gyroNorm >= STILL_GYRO_NORM_MAX_DPS)
        stillFailureMask |= STILL_FAILURE_GYRO_NORM;

    float accelNormStdDevG = 0.0f;
    if (accelHistoryCount < STILL_ACCEL_WINDOW_SAMPLES)
    {
        stillFailureMask |= STILL_FAILURE_ACCEL_STDDEV;
    }
    else
    {
        float varianceSum = 0.0f;
        for (uint8_t axis = 0; axis < 3; ++axis)
        {
            const float mean = accelHistorySum[axis] / STILL_ACCEL_WINDOW_SAMPLES;
            varianceSum += fmaxf(0.0f, accelHistorySumSquares[axis] /
                                              STILL_ACCEL_WINDOW_SAMPLES -
                                          mean * mean);
        }
        accelNormStdDevG = sqrtf(varianceSum);
        if (accelNormStdDevG >= STILL_ACCEL_VECTOR_STDDEV_MAX_G)
            stillFailureMask |= STILL_FAILURE_ACCEL_STDDEV;
    }

    const bool stationary = stillFailureMask == 0;
    const unsigned long now = millis();

    if (stationary)
    {
        if (stationaryStartTime == 0)
            stationaryStartTime = now;
        if (now - stationaryStartTime >= STATIONARY_TIME_MS)
        {
            isStatic = true;
            velocity[0] = velocity[1] = velocity[2] = 0;
        }
    }
    else
    {
        stationaryStartTime = 0;
        isStatic = false;
    }

    if (isStatic)
        lastStillTime = now;

    // Runtime gyro bias tracking: only while genuinely still, and clamped to a
    // small window around the calibrated value so a bad "still" detection can
    // never walk the bias away.
    if (isStatic)
    {
        if (biasStillStartTime == 0)
            biasStillStartTime = now;
        if (now - biasStillStartTime >= GYRO_BIAS_UPDATE_HOLD_MS)
        {
            for (int i = 0; i < 3; ++i)
            {
                calibration.gyroBias[i] +=
                    GYRO_BIAS_UPDATE_ALPHA * (rawGyroDps[i] - calibration.gyroBias[i]);
                // Anchor the tracker to the calibrated value so a false
                // "still" detection can never walk the bias away.
                calibration.gyroBias[i] =
                    imu::clampf(calibration.gyroBias[i],
                                initialGyroBias[i] - GYRO_BIAS_MAX_DRIFT_DPS,
                                initialGyroBias[i] + GYRO_BIAS_MAX_DRIFT_DPS);
            }
        }
    }
    else
    {
        biasStillStartTime = 0;
    }

    // Trust: how much the accelerometer is allowed to correct the attitude.
    const float trustTarget =
        isStatic ? 1.0f
                 : ((lastStillTime != 0 &&
                     now - lastStillTime >= ACCEL_TRUST_FALLBACK_MS)
                        ? 0.05f
                        : 0.0f);
    const float trustAlpha =
        fminf(1.0f, dt / (ACCEL_TRUST_TRANSITION_MS * 0.001f));
    trust += trustAlpha * (trustTarget - trust);

    stillState.stdA = accelNormStdDevG;
    stillState.failureMask = stillFailureMask;
    stillState.still = isStatic;
    stillState.trust = trust;
    stillState.accelNormG = accelNormG;
}

// ---------------------------------------------------------------------------
// Stage 5: gravity removal + dead reckoning
// ---------------------------------------------------------------------------
void AHRS::updateDeadReckoning(float dt)
{
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
    estimator.eulerDegrees(roll, pitch, yaw);

    // The project's rotation contract is kept verbatim: body -> world through
    // Rz(yaw) * Ry(pitch) * Rx(roll). See docs/imu/ROTATION_MATRIX.md.
    float worldAccel[3];
    rotateBodyToWorld(roll, pitch, 0.0f, accelMs2, worldAccel);

    linearAccel[0] = worldAccel[0];
    linearAccel[1] = worldAccel[1];

    const float gyroNorm =
        sqrtf(gyroDps[0] * gyroDps[0] + gyroDps[1] * gyroDps[1] + gyroDps[2] * gyroDps[2]);
    const float worldAccelNorm =
        sqrtf(worldAccel[0] * worldAccel[0] + worldAccel[1] * worldAccel[1] +
              worldAccel[2] * worldAccel[2]);

    if (isStatic && gyroNorm < 3.0f && fabsf(worldAccelNorm - G_CONST) < 0.15f)
    {
        constexpr float GRAVITY_REFERENCE_ALPHA = 0.02f;
        gravityReference +=
            GRAVITY_REFERENCE_ALPHA * (worldAccel[2] - gravityReference);
    }
    linearAccel[2] = worldAccel[2] - gravityReference;

    if (PLANAR_MODE)
    {
        velocity[2] = 0.0f;
        position[2] = 0.0f;
    }

    if (!isStatic)
    {
        for (int i = 0; i < 3; ++i)
        {
            if (PLANAR_MODE && i == 2)
                continue;
            const float magnitude = fabsf(linearAccel[i]);
            float acceleration = linearAccel[i];
            if (DEADBAND_MODE == 1)
                acceleration = magnitude < DEADBAND_MS2 ? 0.0f : linearAccel[i];
            else if (DEADBAND_MODE == 2)
                acceleration = copysignf(fmaxf(magnitude - DEADBAND_MS2, 0.0f), linearAccel[i]);
            velocity[i] += acceleration * dt;
            position[i] += velocity[i] * dt;
        }
    }

    // Low-pass the gravity-free acceleration for the "filtered" telemetry
    // column. The integration above deliberately uses the unfiltered value so
    // the dead-reckoning path keeps its original phase response.
    const imu::Vec3 filtered = linearAccelFilter.apply(imu::Vec3{linearAccel});
    filteredLinearAccel[0] = filtered.x;
    filteredLinearAccel[1] = filtered.y;
    filteredLinearAccel[2] = filtered.z;

    // Gyro-only shadow attitude: integrates the gyro with no correction at all,
    // so the visualizers can show how much the estimator is actually helping.
    if (!shadowInitialized)
    {
        shadowRoll = roll;
        shadowPitch = pitch;
        shadowInitialized = true;
    }
    else if (isStatic)
    {
        shadowRoll = roll;
        shadowPitch = pitch;
    }
    else
    {
        shadowRoll += gyroDps[0] * dt;
        shadowPitch += gyroDps[1] * dt;
    }
}

// ---------------------------------------------------------------------------
// Stage 6: publish a consistent snapshot for the Arduino loop
// ---------------------------------------------------------------------------
void AHRS::publishSnapshot(float dt)
{
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
    estimator.eulerDegrees(roll, pitch, yaw);

    ImuTelemetry snapshot{};
    snapshot.dt = dt;
    for (int i = 0; i < 3; ++i)
    {
        snapshot.accel[i] = accelMs2[i];
        snapshot.linearAccel[i] = linearAccel[i];
        snapshot.filteredLinearAccel[i] = filteredLinearAccel[i];
        snapshot.filteredGyro[i] = filteredGyroDps[i];
        snapshot.filteredMag[i] = filteredMagUt[i];
        snapshot.gyro[i] = gyroDps[i];
        snapshot.mag[i] = magUt[i];
        snapshot.gyroBias[i] = calibration.gyroBias[i];
        snapshot.estimatorBias[i] = estimator.gyroBiasRad()[i] * RAD_TO_DEG_F;
    }
    estimator.quaternion().toArray(snapshot.quaternion);
    snapshot.roll = roll;
    snapshot.pitch = pitch;
    snapshot.yaw = yaw;
    snapshot.temperature = mpu->getTemperature();
    snapshot.stdA = stillState.stdA;
    snapshot.trust = stillState.trust;
    snapshot.rollG = shadowRoll;
    snapshot.pitchG = shadowPitch;
    snapshot.still = stillState.still;
    snapshot.stillFailureMask = stillState.failureMask;
    snapshot.accelNormG = stillState.accelNormG;
    for (int i = 0; i < 3; ++i)
    {
        snapshot.velocity[i] = velocity[i];
        snapshot.position[i] = position[i];
    }
    snapshot.magNorm = sqrtf(magUt[0] * magUt[0] + magUt[1] * magUt[1] +
                             magUt[2] * magUt[2]);
    snapshot.imuRateHz = measuredRateHz;
    snapshot.accelCalRms = calibration.accelFitRms;
    snapshot.magCalRms = calibration.magFitRms;
    snapshot.magFitKind = calibration.magFitKind;
    snapshot.estimatorInitialized = estimator.initialized() ? 1 : 0;

    portENTER_CRITICAL(&snapshotMux);
    pendingTelemetry = snapshot;
    snapshotReady = true;
    portEXIT_CRITICAL(&snapshotMux);
}

void AHRS::update()
{
    if (!snapshotReady)
        return;

    portENTER_CRITICAL(&snapshotMux);
    telemetry = pendingTelemetry;
    portEXIT_CRITICAL(&snapshotMux);
}

// ---------------------------------------------------------------------------
// Calibration
// ---------------------------------------------------------------------------
bool AHRS::calibrate()
{
    if (!initialized)
        return false;

    Serial.println("CAL,STARTUP,warmup_seconds=5");
    showCalibrationScreen("Starting IMU", "Sensor warm-up", "Please wait...");
    delay(5000);

    if (calibrationLoaded && checkCalibrationAtBoot())
    {
        Serial.println("CAL,SELF_CHECK,ok");
        showCalibrationScreen("Calibration OK", "Saved data is valid", "Starting robot...");
        delay(700);
        return true;
    }
    if (calibrationLoaded)
        Serial.println("CAL,SELF_CHECK,recalibration_required");

    showCalibrationScreen("Calibration needed", "Starting guided setup", "Follow the screen");
    delay(1000);

    const bool calibrated = runCalibration(true, true, true);
    if (!calibrated)
    {
        showCalibrationScreen("Calibration failed", "Robot is not ready", "Press q for menu");
        delay(1500);
    }
    return calibrated;
}

bool AHRS::runCalibration(bool gyro, bool accelerometer, bool magnetometer)
{
    // The sampling task must not touch the sensor while we are calibrating.
    // Wait for it to acknowledge the pause so we never interleave an I2C
    // transaction with a calibration read.
    taskPauseAck = false;
    taskPaused = true;
    for (int i = 0; i < 100 && !taskPauseAck; ++i)
        delay(2);

    Serial.print("CAL,START,robot=");
    Serial.println(robotNumber);

    bool ok = true;

    if (gyro)
    {
        showCalibrationScreen("Gyro calibration", "Keep robot still", "Starting in 3 sec");
        delay(3000);
        Serial.println("CAL,gyro,keep_the_robot_completely_still");
        ok = calibrateGyroscope();
        if (!ok)
            Serial.println("CAL,FAIL,gyro_calibration_rejected");
    }

    if (ok && accelerometer)
    {
        Serial.println("CAL,accelerometer,guided_six_position");
        ok = calibrateAccelerometer();
        if (!ok)
            Serial.println("CAL,FAIL,accelerometer_calibration_rejected");
    }

    if (ok && magnetometer)
    {
        ok = calibrateMagnetometer();
        if (!ok)
            Serial.println("CAL,FAIL,magnetometer_calibration_rejected");
    }

    if (ok)
    {
        ok = saveCalibration();
        if (!ok)
            Serial.println("CAL,FAIL,storage");
        else
            calibrationLoaded = true;
    }

    if (ok)
    {
        for (int i = 0; i < 3; ++i)
            initialGyroBias[i] = calibration.gyroBias[i];
        showCalibrationScreen("Calibration complete", "All sensor data saved", "Starting robot...");
        delay(1000);
        Serial.println("CAL,DONE");
    }

    // Force a fresh estimator seed with the new calibration.
    estimator.reset();
    estimatorSeeded = false;
    shadowInitialized = false;
    accelHistoryCount = 0;
    accelHistoryIndex = 0;
    memset(accelHistorySum, 0, sizeof(accelHistorySum));
    memset(accelHistorySumSquares, 0, sizeof(accelHistorySumSquares));

    taskPaused = false;
    return ok;
}

bool AHRS::calibrateGyroscope()
{
    // Port of PX4's gyro_calibration_worker + the median/mean consistency check
    // from do_gyro_calibration().
    for (uint8_t attempt = 0; attempt < GYRO_CAL_MAX_RETRIES; ++attempt)
    {
        Serial.println("CAL,gyro,settling");
        delay(GYRO_CAL_SETTLE_MS);

        float sum[3] = {}, sumSquared[3] = {};
        imu::MedianFilter3 median[3];
        uint16_t samples = 0;
        const uint32_t started = millis();

        while (samples < GYRO_CAL_SAMPLES && millis() - started < 30000)
        {
            if (consumeCalibrationControl())
                return false;
            if (!mpu->update())
            {
                delay(2);
                continue;
            }

            const float ax = mpu->getAccX();
            const float ay = mpu->getAccY();
            const float az = mpu->getAccZ();
            const float accelNorm = sqrtf(ax * ax + ay * ay + az * az);
            const float gyro[3] = {mpu->getGyroX(), mpu->getGyroY(), mpu->getGyroZ()};
            const float gyroNorm =
                sqrtf(gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]);

            if (accelNorm < GYRO_CAL_ACCEL_MIN_G || accelNorm > GYRO_CAL_ACCEL_MAX_G ||
                gyroNorm > GYRO_CAL_MAX_RATE_DPS)
                continue;

            for (uint8_t i = 0; i < 3; ++i)
            {
                sum[i] += gyro[i];
                sumSquared[i] += gyro[i] * gyro[i];
                median[i].apply(gyro[i]);
            }
            ++samples;
        }

        if (samples < GYRO_CAL_MIN_SAMPLES)
        {
            Serial.println("CAL,gyro,not_enough_samples");
            return false;
        }

        float bias[3];
        bool rejected = false;
        for (uint8_t i = 0; i < 3; ++i)
        {
            bias[i] = sum[i] / samples;
            const float variance = sumSquared[i] / samples - bias[i] * bias[i];
            const float sigma = sqrtf(fmaxf(0.0f, variance));
            const float medianError = fabsf(median[i].median() - bias[i]);

            Serial.print("CAL,gyro,axis=");
            Serial.print(i);
            Serial.print(",sigma=");
            Serial.print(sigma, 5);
            Serial.print(",median_error=");
            Serial.println(medianError, 5);

            if (sigma > GYRO_CAL_MAX_SIGMA_DPS ||
                medianError > GYRO_CAL_MAX_MEDIAN_ERROR_DPS)
                rejected = true;
        }

        if (rejected)
        {
            Serial.println("CAL,gyro,motion_detected_retrying");
            continue;
        }

        for (uint8_t i = 0; i < 3; ++i)
            calibration.gyroBias[i] = bias[i];

        Serial.print("CAL,gyro,bias=");
        Serial.print(bias[0], 5);
        Serial.print(',');
        Serial.print(bias[1], 5);
        Serial.print(',');
        Serial.println(bias[2], 5);
        return true;
    }

    Serial.println("CAL,gyro,too_many_retries");
    return false;
}

bool AHRS::calibrateAccelerometer()
{
    // Port of PX4's do_accel_calibration(): guide the user through the six
    // orientations, average each one, then solve the full 3x3 transform.
    static const imu::AccelSide sides[imu::kAccelCalibrationSides] = {
        imu::AccelSide::TailDown, imu::AccelSide::NoseDown, imu::AccelSide::Left,
        imu::AccelSide::Right, imu::AccelSide::UpsideDown, imu::AccelSide::RightSideUp};

    float raw[imu::kAccelCalibrationSides][3] = {};

    Serial.println("CAL,accelerometer,local_g_not_needed_sensor_units_are_g");
    Serial.println("CAL,accelerometer,keep_robot_still_during_each_step");

    for (uint8_t sideIndex = 0; sideIndex < imu::kAccelCalibrationSides; ++sideIndex)
    {
        const imu::AccelSide side = sides[sideIndex];
        const uint8_t axis = sideIndex / 2;
        const bool positiveSide = (sideIndex % 2) == 0;

        char requested[24];
        snprintf(requested, sizeof(requested), "Put %s up",
                 imu::accelSideName(side));

        Serial.print("CAL,accelerometer,place_axis=");
        Serial.print(imu::accelSideName(side));
        Serial.print(",side=");
        Serial.println(positiveSide ? "UP" : "DOWN");
        Serial.println("CAL,accelerometer,waiting_for_stable");

        showCalibrationScreen("Accel calibration", requested, "Hold still - starting");
        delay(1500);

        uint16_t count = 0;
        double sum[3] = {};
        uint32_t lastAcceptedAt = 0;
        uint32_t lastDiagnosticAt = 0;
        const uint32_t phaseStarted = millis();

        while (count < CAL_FACE_SAMPLES && millis() - phaseStarted < CAL_FACE_TIMEOUT_MS)
        {
            if (consumeCalibrationControl())
            {
                Serial.println("CAL,CANCELLED,accelerometer");
                return false;
            }
            if (!mpu->update())
            {
                delay(2);
                continue;
            }
            if (count > 0 && millis() - lastAcceptedAt < CAL_SAMPLE_INTERVAL_MS)
                continue;

            const float sample[3] = {mpu->getAccX(), mpu->getAccY(), mpu->getAccZ()};
            const float norm = sqrtf(sample[0] * sample[0] + sample[1] * sample[1] +
                                     sample[2] * sample[2]);
            const float correctedGyro[3] = {
                mpu->getGyroX() - calibration.gyroBias[0],
                mpu->getGyroY() - calibration.gyroBias[1],
                mpu->getGyroZ() - calibration.gyroBias[2]};
            const float gyroNorm = sqrtf(correctedGyro[0] * correctedGyro[0] +
                                         correctedGyro[1] * correctedGyro[1] +
                                         correctedGyro[2] * correctedGyro[2]);

            uint8_t dominantAxis = 0;
            if (fabsf(sample[1]) > fabsf(sample[dominantAxis]))
                dominantAxis = 1;
            if (fabsf(sample[2]) > fabsf(sample[dominantAxis]))
                dominantAxis = 2;

            const bool expectedSide = positiveSide ? sample[axis] >= 0.0f : sample[axis] < 0.0f;

            if (norm < 0.85f || norm > 1.15f || gyroNorm > 3.0f || dominantAxis != axis ||
                fabsf(sample[axis]) < 0.75f || !expectedSide)
            {
                if (count == 0 && millis() - lastDiagnosticAt >= 1000)
                {
                    Serial.print("CAL,accelerometer,waiting,acc_norm_g=");
                    Serial.print(norm, 4);
                    Serial.print(",gyro_norm_dps=");
                    Serial.print(gyroNorm, 4);
                    Serial.print(",dominant=");
                    Serial.print("XYZ"[dominantAxis]);
                    Serial.print(sample[dominantAxis] >= 0.0f ? '+' : '-');
                    Serial.print(",expected=");
                    Serial.print(imu::accelSideName(side));
                    Serial.print(",rejection=");
                    if (norm < 0.85f)
                        Serial.println("accel_norm_low");
                    else if (norm > 1.15f)
                        Serial.println("accel_norm_high");
                    else if (gyroNorm > 3.0f)
                        Serial.println("motion");
                    else if (dominantAxis != axis)
                        Serial.println("dominant_axis");
                    else if (fabsf(sample[axis]) < 0.75f)
                        Serial.println("axis_magnitude");
                    else
                        Serial.println("axis_sign");
                    lastDiagnosticAt = millis();
                }
                if (count == 0 && millis() - phaseStarted > 1200)
                {
                    char current[24];
                    snprintf(current, sizeof(current), "Current: %c%c",
                             "XYZ"[dominantAxis], sample[dominantAxis] >= 0.0f ? '+' : '-');
                    char needed[24];
                    snprintf(needed, sizeof(needed), "Need: %s",
                             imu::accelSideName(side));
                    showCalibrationScreen("Wrong position", current, needed);
                }
                continue;
            }

            if (count == 0)
                showCalibrationScreen("Correct position", "Hold still", "Collecting...");

            for (uint8_t i = 0; i < 3; ++i)
                sum[i] += sample[i];
            ++count;
            lastAcceptedAt = millis();

            if ((count % 30) == 0)
            {
                Serial.print("CAL,accelerometer,side=");
                Serial.print(imu::accelSideName(side));
                Serial.print(",samples=");
                Serial.println(count);
                char progress[24];
                snprintf(progress, sizeof(progress), "Samples %u/%u", count, CAL_FACE_SAMPLES);
                showCalibrationScreen("Correct position", progress, "Keep still");
            }
        }

        if (count < CAL_FACE_SAMPLES)
        {
            showCalibrationScreen("Still waiting", "Position not stable", "Press q to cancel");
            delay(700);
            Serial.print("CAL,FAIL,accelerometer,side=");
            Serial.print(imu::accelSideName(side));
            Serial.print(",samples=");
            Serial.println(count);
            return false;
        }

        for (uint8_t i = 0; i < 3; ++i)
            raw[sideIndex][i] = static_cast<float>(sum[i] / count);

        Serial.print("CAL,accelerometer,result=");
        Serial.print(imu::accelSideName(side));
        Serial.print(',');
        Serial.print(raw[sideIndex][0], 4);
        Serial.print(',');
        Serial.print(raw[sideIndex][1], 4);
        Serial.print(',');
        Serial.println(raw[sideIndex][2], 4);
    }

    float bias[3];
    float matrix[3][3];
    float rms = 0.0f;
    if (!imu::calibration::fitAccelerometerSixPosition(raw, bias, matrix, rms))
    {
        Serial.println("CAL,FAIL,accelerometer,solve_failed");
        return false;
    }

    for (uint8_t i = 0; i < 3; ++i)
    {
        calibration.accelBias[i] = bias[i];
        for (uint8_t j = 0; j < 3; ++j)
            calibration.accelMatrix[i][j] = matrix[i][j];
    }
    calibration.accelFitRms = rms;
    calibration.accelSides = imu::kAccelCalibrationSides;

    Serial.print("CAL,accelerometer,bias=");
    Serial.print(bias[0], 5);
    Serial.print(',');
    Serial.print(bias[1], 5);
    Serial.print(',');
    Serial.println(bias[2], 5);
    Serial.print("CAL,accelerometer,matrix=");
    for (uint8_t i = 0; i < 3; ++i)
    {
        Serial.print(matrix[i][0], 5);
        Serial.print(',');
        Serial.print(matrix[i][1], 5);
        Serial.print(',');
        Serial.print(matrix[i][2], 5);
        if (i < 2)
            Serial.print(',');
    }
    Serial.println();
    Serial.print("CAL,accelerometer,rms_g=");
    Serial.println(rms, 5);
    return true;
}

bool AHRS::calibrateMagnetometer()
{
    // Port of PX4's mag calibration: collect a rotating sample cloud, fit a
    // sphere, then refine with a full ellipsoid fit.
    Serial.println("CAL,magnetometer,rotate_slowly_in_all_directions_for_20_seconds");
    showCalibrationScreen("Mag calibration", "Rotate slowly", "All directions - 20 sec");

    static float samples[MAG_CAL_MAX_SAMPLES * 3];
    uint16_t count = 0;
    const uint32_t started = millis();

    while (millis() - started < MAG_CAL_DURATION_MS && count < MAG_CAL_MAX_SAMPLES)
    {
        if (consumeCalibrationControl())
        {
            Serial.println("CAL,CANCELLED,magnetometer");
            return false;
        }
        if (!mpu->update())
        {
            delay(2);
            continue;
        }

        const float mx = mpu->getMagX() * MG_TO_UT;
        const float my = mpu->getMagY() * MG_TO_UT;
        const float mz = mpu->getMagZ() * MG_TO_UT;
        if (mx == 0.0f && my == 0.0f && mz == 0.0f)
            continue;

        // PX4 enforces a minimum distance between samples so the fit is not
        // dominated by a dense cluster in one orientation.
        if (count > 0)
        {
            const float dx = mx - samples[3 * (count - 1) + 0];
            const float dy = my - samples[3 * (count - 1) + 1];
            const float dz = mz - samples[3 * (count - 1) + 2];
            if (sqrtf(dx * dx + dy * dy + dz * dz) < MAG_CAL_MIN_SAMPLE_DISTANCE_UT)
                continue;
        }

        samples[3 * count + 0] = mx;
        samples[3 * count + 1] = my;
        samples[3 * count + 2] = mz;
        ++count;

        if ((count % 100) == 0)
        {
            char progress[24];
            snprintf(progress, sizeof(progress), "Samples %u", count);
            showCalibrationScreen("Mag calibration", progress, "Keep rotating");
        }
        delay(4);
    }

    Serial.print("CAL,magnetometer,samples=");
    Serial.println(count);

    if (count < 200)
    {
        Serial.println("CAL,FAIL,magnetometer,not_enough_samples");
        return false;
    }

    float offset[3] = {};
    float matrix[3][3];
    float radius = 0.0f;
    float rms = 0.0f;

    if (!imu::calibration::fitMagnetometerSphere(samples, count, offset, matrix, radius, rms))
    {
        Serial.println("CAL,FAIL,magnetometer,sphere_fit_failed");
        return false;
    }
    Serial.print("CAL,magnetometer,sphere_radius=");
    Serial.print(radius, 4);
    Serial.print(",rms=");
    Serial.println(rms, 4);

    uint8_t fitKind = 1;
    float ellipsoidOffset[3] = {offset[0], offset[1], offset[2]};
    float ellipsoidMatrix[3][3];
    float ellipsoidRadius = radius;
    float ellipsoidRms = rms;

    if (imu::calibration::fitMagnetometerEllipsoid(samples, count, ellipsoidOffset,
                                                   ellipsoidMatrix, ellipsoidRadius,
                                                   ellipsoidRms))
    {
        // Only accept the ellipsoid when it actually improves the residual.
        if (ellipsoidRms <= rms)
        {
            for (int i = 0; i < 3; ++i)
            {
                offset[i] = ellipsoidOffset[i];
                for (int j = 0; j < 3; ++j)
                    matrix[i][j] = ellipsoidMatrix[i][j];
            }
            radius = ellipsoidRadius;
            rms = ellipsoidRms;
            fitKind = 2;
        }
        Serial.print("CAL,magnetometer,ellipsoid_rms=");
        Serial.println(ellipsoidRms, 4);
    }

    for (int i = 0; i < 3; ++i)
    {
        calibration.magOffset[i] = offset[i];
        for (int j = 0; j < 3; ++j)
            calibration.magMatrix[i][j] = matrix[i][j];
    }
    calibration.magFieldNorm = radius;
    calibration.magFitRms = rms;
    calibration.magFitKind = fitKind;

    Serial.print("CAL,magnetometer,offset=");
    Serial.print(offset[0], 3);
    Serial.print(',');
    Serial.print(offset[1], 3);
    Serial.print(',');
    Serial.println(offset[2], 3);
    Serial.print("CAL,magnetometer,fit=");
    Serial.println(fitKind == 2 ? "ellipsoid" : "sphere");
    return true;
}

bool AHRS::checkCalibrationAtBoot()
{
    float accelNormSum = 0.0f;
    float gyroNormSum = 0.0f;
    float magNormSum = 0.0f;
    uint16_t accepted = 0;

    for (uint16_t i = 0; i < CHECK_SAMPLES; ++i)
    {
        if (!mpu->update())
        {
            delay(2);
            continue;
        }

        const float raw[3] = {mpu->getAccX(), mpu->getAccY(), mpu->getAccZ()};
        float corrected[3];
        imu::calibration::applyAccelCorrection(raw, calibration, corrected);
        const float accelNorm =
            sqrtf(corrected[0] * corrected[0] + corrected[1] * corrected[1] +
                  corrected[2] * corrected[2]);

        const float gyroNorm =
            sqrtf((mpu->getGyroX() - calibration.gyroBias[0]) *
                      (mpu->getGyroX() - calibration.gyroBias[0]) +
                  (mpu->getGyroY() - calibration.gyroBias[1]) *
                      (mpu->getGyroY() - calibration.gyroBias[1]) +
                  (mpu->getGyroZ() - calibration.gyroBias[2]) *
                      (mpu->getGyroZ() - calibration.gyroBias[2]));

        const float rawMag[3] = {mpu->getMagX() * MG_TO_UT, mpu->getMagY() * MG_TO_UT,
                                 mpu->getMagZ() * MG_TO_UT};
        float correctedMag[3];
        imu::calibration::applyMagCorrection(rawMag, calibration, correctedMag);
        const float magNorm = sqrtf(correctedMag[0] * correctedMag[0] +
                                    correctedMag[1] * correctedMag[1] +
                                    correctedMag[2] * correctedMag[2]);

        accelNormSum += accelNorm;
        gyroNormSum += gyroNorm;
        magNormSum += magNorm;
        ++accepted;
        delay(4);
    }

    if (accepted < CHECK_SAMPLES)
        return false;

    const float accelError = fabsf(accelNormSum / accepted - 1.0f);
    const float gyroAverage = gyroNormSum / accepted;
    const float magAverage = magNormSum / accepted;
    const float currentTemperature = mpu->getTemperature();
    const float temperatureDelta =
        fabsf(currentTemperature - calibration.calibrationTemperature);

    const bool magValid =
        calibration.magFieldNorm <= 0.0f ||
        fabsf(magAverage - calibration.magFieldNorm) /
                fmaxf(calibration.magFieldNorm, 1e-3f) <=
            MAX_MAG_FIELD_DELTA_PERCENT;

    Serial.print("CAL,SELF_CHECK,accel_error_g=");
    Serial.print(accelError, 4);
    Serial.print(",gyro_norm_dps=");
    Serial.print(gyroAverage, 3);
    Serial.print(",mag_norm=");
    Serial.print(magAverage, 3);
    Serial.print(",temp_c=");
    Serial.print(currentTemperature, 2);
    Serial.print(",temp_delta_c=");
    Serial.println(temperatureDelta, 2);

    return accelError <= MAX_ACCEL_NORM_ERROR_G && gyroAverage <= MAX_GYRO_NORM_DPS &&
           magValid &&
           (calibration.calibrationTemperature <= 0.0f ||
            temperatureDelta <= MAX_TEMPERATURE_DELTA_C);
}

bool AHRS::loadCalibration()
{
    if (!imu::ImuCalibrationStore::load(robotNumber, calibration))
        return false;
    calibrationLoaded = true;
    return true;
}

bool AHRS::saveCalibration()
{
    calibration.robotNumber = robotNumber;
    calibration.calibrationTemperature = mpu->getTemperature();
    return imu::ImuCalibrationStore::save(calibration);
}

void AHRS::clearStoredCalibration()
{
    const bool removed = imu::ImuCalibrationStore::clear(robotNumber);
    calibrationLoaded = false;
    Serial.println(removed ? "IMU,CLEAR,done" : "IMU,CLEAR,nothing_to_remove");
}

bool AHRS::consumeCalibrationControl()
{
    bool cancelled = false;
    while (Serial.available())
    {
        const char character = static_cast<char>(Serial.read());
        if (character == 'x' || character == 'X')
            cancelled = true;
        else if (character == '?')
            Serial.println("CAL,HELP,send x to cancel; keep still or move slowly as requested");
    }
    if (cancelled)
        Serial.println("CAL,CANCELLED,user");
    return cancelled;
}

void AHRS::showCalibrationScreen(const char *title, const char *line1, const char *line2)
{
    if (statusDisplay == nullptr)
        return;
    statusDisplay->clear();
    statusDisplay->print(title, 0, 0);
    if (line1 != nullptr)
    {
        statusDisplay->print(line1, 0, 20);
        if (line2 != nullptr)
            statusDisplay->print(line2, 0, 36);
    }
    statusDisplay->refresh();
}

void AHRS::printCalibrationMenu() const
{
    Serial.println();
    Serial.println(
        "IMU MENU: 1=gyro 2=accel 3=mag 4=all 5=status 6=clear 0=close");
}

void AHRS::printCalibrationStatus() const
{
    Serial.print("IMU,STATUS,robot=");
    Serial.print(robotNumber);
    Serial.print(",saved=");
    Serial.print(calibrationLoaded ? 1 : 0);
    Serial.print(",acc_norm_g=");
    Serial.print(telemetry.accelNormG, 4);
    Serial.print(",gyro_norm_dps=");
    Serial.print(sqrtf(telemetry.gyro[0] * telemetry.gyro[0] +
                       telemetry.gyro[1] * telemetry.gyro[1] +
                       telemetry.gyro[2] * telemetry.gyro[2]),
                 4);
    Serial.print(",mag_norm_ut=");
    Serial.print(telemetry.magNorm, 3);
    Serial.print(",still=");
    Serial.print(telemetry.still ? 1 : 0);
    Serial.print(",stdA=");
    Serial.print(telemetry.stdA, 5);
    Serial.print(",trust=");
    Serial.print(telemetry.trust, 3);
    Serial.print(",rate_hz=");
    Serial.print(telemetry.imuRateHz, 1);
    Serial.print(",accel_rms_g=");
    Serial.print(telemetry.accelCalRms, 5);
    Serial.print(",mag_rms_ut=");
    Serial.print(telemetry.magCalRms, 4);
    Serial.print(",mag_fit=");
    Serial.println(telemetry.magFitKind);
}

void AHRS::serviceConsole()
{
    static bool menuOpen = false;
    while (Serial.available())
    {
        const char character = static_cast<char>(Serial.read());
        if (!menuOpen && (character == 'q' || character == 'Q'))
        {
            menuOpen = true;
            serialTelemetryEnabled = false;
            printCalibrationMenu();
            continue;
        }
        if (!menuOpen)
            continue;

        if (character == '0')
        {
            menuOpen = false;
            serialTelemetryEnabled = true;
            Serial.println("IMU MENU:CLOSED,stream=on");
        }
        else if (character == '1')
        {
            menuOpen = false;
            runCalibration(true, false, false);
            serialTelemetryEnabled = true;
        }
        else if (character == '2')
        {
            menuOpen = false;
            runCalibration(false, true, false);
            serialTelemetryEnabled = true;
        }
        else if (character == '3')
        {
            menuOpen = false;
            runCalibration(false, false, true);
            serialTelemetryEnabled = true;
        }
        else if (character == '4')
        {
            menuOpen = false;
            runCalibration(true, true, true);
            serialTelemetryEnabled = true;
        }
        else if (character == '5')
        {
            printCalibrationStatus();
        }
        else if (character == '6')
        {
            clearStoredCalibration();
        }
    }
}

void AHRS::resetPosition()
{
    for (int i = 0; i < 3; ++i)
        velocity[i] = position[i] = 0.0f;
}
