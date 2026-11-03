#include "AHRS.h"
#include "../../include/config.h"
#include <Display.h>

namespace
{
constexpr float FILTER_ACCEL_CUTOFF_HZ = 7.5f;
constexpr float FILTER_GYRO_CUTOFF_HZ = 25.0f;
constexpr float FILTER_MAG_CUTOFF_HZ = 5.0f;
constexpr float GYRO_CAL_ACCEL_MIN_G = 0.85f;
constexpr float GYRO_CAL_ACCEL_MAX_G = 1.15f;
constexpr float GYRO_CAL_MAX_RATE_DPS = 8.0f;
constexpr uint32_t GYRO_CAL_SETTLE_MS = 1500;
constexpr uint16_t CAL_FACE_SAMPLES = 180;
constexpr uint32_t CAL_FACE_TIMEOUT_MS = 120000;
constexpr uint32_t CAL_SAMPLE_INTERVAL_MS = 10;
constexpr float STILL_ACCEL_NORM_TOLERANCE_G = 0.02f;
constexpr float STILL_GYRO_NORM_MAX_DPS = 0.5f;
constexpr float STILL_ACCEL_NORM_STDDEV_MAX_G = 0.004f;
constexpr uint8_t STILL_FAILURE_ACCEL_NORM = 1u << 0;
constexpr uint8_t STILL_FAILURE_GYRO_NORM = 1u << 1;
constexpr uint8_t STILL_FAILURE_ACCEL_STDDEV = 1u << 2;
constexpr uint32_t GYRO_BIAS_UPDATE_HOLD_MS = 1000;
constexpr float GYRO_BIAS_UPDATE_ALPHA = 0.001f;
constexpr float GYRO_BIAS_MAX_DRIFT_DPS = 0.5f;
}

AHRS::AHRS(Display *display) : initialized(false), isStatic(true), stationaryStartTime(0),
               biasStillStartTime(0),
               robotNumber(1), calibrationLoaded(false),
               serialTelemetryEnabled(true), deadbandMode(DEADBAND_MODE),
               gravityReference(G_CONST),
               statusDisplay(display)
{
    mpu = new MPU9250();
    ImuCalibration::setIdentity(calibration.accelMatrix);
    ImuCalibration::setIdentity(calibration.magMatrix);
    calibration.robotNumber = 1;
    for (int i = 0; i < 3; i++)
    {
        velocity[i] = position[i] = linearAccel[i] = 0;
        calibration.accelBias[i] = 0;
        calibration.accelScale[i] = 1;
        calibration.gyroBias[i] = 0;
        calibration.magBias[i] = 0;
        calibration.magScale[i] = 1;
    }
    calibration.calibrationTemperature = 0;
    calibration.magFieldNorm = 0;
    accelFilter.configure(FILTER_ACCEL_CUTOFF_HZ, 200.0f, 0.03f, 0.10f);
    gyroFilter.configure(FILTER_GYRO_CUTOFF_HZ, 200.0f, 0.02f, 0.08f);
    magFilter.configure(FILTER_MAG_CUTOFF_HZ, 100.0f, 0.02f, 0.50f);
}

AHRS::~AHRS()
{
    delete mpu;
}

bool AHRS::begin(uint8_t requestedRobotNumber)
{
    robotNumber = requestedRobotNumber >= 1 && requestedRobotNumber <= 8 ? requestedRobotNumber : 1;
    Wire.begin();
    MPU9250Setting setting;
    setting.accel_fs_sel = IMU_ACCEL_RANGE;
    setting.gyro_fs_sel = IMU_GYRO_RANGE;
    setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
    setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
    setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

    if (!mpu->setup(MPU9250_ADDRESS, setting))
        return false;

    initialized = true;
    lastUpdateTime = millis();
    if (loadCalibration())
    {
        for (int i = 0; i < 3; ++i)
            initialGyroBias[i] = calibration.gyroBias[i];
        mpu->setMagBias(calibration.magBias[0], calibration.magBias[1], calibration.magBias[2]);
        mpu->setMagScale(calibration.magScale[0], calibration.magScale[1], calibration.magScale[2]);
        Serial.print("CAL,LOADED,robot=");
        Serial.println(robotNumber);
    }
    else
    {
        for (int i = 0; i < 3; ++i)
            initialGyroBias[i] = calibration.gyroBias[i];
    }
    return true;
}

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
    Serial.print("CAL,START,robot=");
    Serial.println(robotNumber);

    if (gyro)
    {
        showCalibrationScreen("Gyro calibration", "Keep robot still", "Starting in 3 sec");
        delay(3000);
        Serial.println("CAL,gyro,keep the robot completely still");
        if (!calibrateGyroscope())
        {
            Serial.println("CAL,FAIL,gyro calibration rejected");
            return false;
        }
    }

    if (accelerometer)
    {
        Serial.println("CAL,accelerometer,guided_six_position");
        if (!calibrateAccelerometerCloud())
        {
            Serial.println("CAL,FAIL,accelerometer calibration rejected");
            return false;
        }
    }

    if (magnetometer)
    {
        if (!calibrateMagnetometer())
        {
            Serial.println("CAL,FAIL,magnetometer calibration rejected");
            return false;
        }
    }

    if (!saveCalibration())
    {
        Serial.println("CAL,FAIL,storage");
        return false;
    }
    calibrationLoaded = true;
    showCalibrationScreen("Calibration complete", "All sensor data saved", "Starting robot...");
    delay(1000);
    Serial.println("CAL,DONE");
    return true;
}

void AHRS::printCalibrationMenu() const
{
    Serial.println();
    Serial.println("IMU MENU: 1=gyro 2=accel 3=mag 4=all 5=status 6=clear 7=cycle_deadband 0=close");
}

void AHRS::printCalibrationStatus() const
{
    const ImuTelemetry &imu = telemetry;
    Serial.print("IMU,STATUS,robot=");
    Serial.print(robotNumber);
    Serial.print(",saved=");
    Serial.print(calibrationLoaded ? 1 : 0);
    Serial.print(",acc_norm=");
    Serial.print(sqrtf(sq(imu.accel[0]) + sq(imu.accel[1]) + sq(imu.accel[2])) / G_CONST, 4);
    Serial.print(",gyro_norm=");
    Serial.print(sqrtf(sq(imu.gyro[0]) + sq(imu.gyro[1]) + sq(imu.gyro[2])), 4);
    Serial.print(",deadband_mode=");
    Serial.print(deadbandMode);
    Serial.print(",deadband_ms2=");
    Serial.println(DEADBAND_MS2, 4);
}

void AHRS::clearStoredCalibration()
{
    const bool removed = ImuCalibration::clear(preferences, robotNumber);
    calibrationLoaded = false;
    Serial.println(removed ? "IMU,CLEAR,done" : "IMU,CLEAR,nothing_to_remove");
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
            Serial.println("IMU MENU: calibration cleared; reboot before using");
        }
        else if (character == '7')
        {
            deadbandMode = (deadbandMode + 1) % 3;
            Serial.print("IMU,DEADBAND,mode=");
            Serial.println(deadbandMode);
        }
    }
}

bool AHRS::checkCalibrationAtBoot()
{
    Serial.println("CAL,SELF_CHECK,keep_still,starting_in=3");
    showCalibrationScreen("Checking calibration", "Keep robot still", "Starting in 3 sec");
    delay(3000);

    constexpr uint16_t CHECK_SAMPLES = 250;
    constexpr float MAX_GYRO_NORM_DPS = 1.5f;
    constexpr float MAX_ACCEL_NORM_ERROR_G = 0.035f;
    constexpr float MAX_TEMPERATURE_DELTA_C = 8.0f;
    constexpr float MAX_MAG_FIELD_DELTA_PERCENT = 0.25f;
    double accelNormSum = 0;
    double gyroNormSum = 0;
    double magNormSum = 0;
    uint16_t accepted = 0;
    const uint32_t started = millis();
    while (accepted < CHECK_SAMPLES && millis() - started < 8000)
    {
        if (!mpu->update())
        {
            delay(2);
            continue;
        }
        float raw[3] = {mpu->getAccX(), mpu->getAccY(), mpu->getAccZ()};
        float corrected[3];
        correctAccelerometer(raw, corrected);
        const float accelNorm = sqrtf(sq(corrected[0]) + sq(corrected[1]) + sq(corrected[2]));
        const float gyroNorm = sqrtf(sq(mpu->getGyroX() - calibration.gyroBias[0]) +
                                     sq(mpu->getGyroY() - calibration.gyroBias[1]) +
                                     sq(mpu->getGyroZ() - calibration.gyroBias[2]));
        const float magNorm = sqrtf(sq(mpu->getMagX()) + sq(mpu->getMagY()) +
                                    sq(mpu->getMagZ()));
        accelNormSum += accelNorm;
        gyroNormSum += gyroNorm;
        magNormSum += magNorm;
        ++accepted;
        delay(4);
    }
    if (accepted < CHECK_SAMPLES)
        return false;
    const float accelError = fabsf(static_cast<float>(accelNormSum / accepted) - 1.0f);
    const float gyroAverage = static_cast<float>(gyroNormSum / accepted);
    const float magAverage = static_cast<float>(magNormSum / accepted);
    const float currentTemperature = mpu->getTemperature();
    const float temperatureDelta = fabsf(currentTemperature -
                                         calibration.calibrationTemperature);
    const bool magValid = calibration.magFieldNorm <= 0 ||
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
    return accelError <= MAX_ACCEL_NORM_ERROR_G &&
           gyroAverage <= MAX_GYRO_NORM_DPS &&
           magValid &&
           (calibration.calibrationTemperature <= 0 ||
            temperatureDelta <= MAX_TEMPERATURE_DELTA_C);
}

bool AHRS::calibrateMagnetometer()
{
    Serial.println("CAL,magnetometer,rotate slowly in all directions for 15 seconds");
    showCalibrationScreen("Mag calibration", "Rotate slowly", "All directions - 15 sec");
    mpu->calibrateMag();
    for (uint8_t i = 0; i < 3; ++i)
    {
        calibration.magBias[i] = mpu->getMagBias(i);
        calibration.magScale[i] = mpu->getMagScale(i);
    }
    double fieldSum = 0;
    uint16_t fieldSamples = 0;
    const uint32_t started = millis();
    while (fieldSamples < 50 && millis() - started < 3000)
    {
        if (mpu->update())
        {
            const float mx = mpu->getMagX();
            const float my = mpu->getMagY();
            const float mz = mpu->getMagZ();
            fieldSum += sqrtf(sq(mx) + sq(my) + sq(mz));
            ++fieldSamples;
        }
        delay(4);
    }
    calibration.magFieldNorm = fieldSamples > 0
                                   ? static_cast<float>(fieldSum / fieldSamples)
                                   : 0;
    return true;
}

bool AHRS::loadCalibration()
{
    if (!ImuCalibration::load(preferences, robotNumber, calibration))
        return false;
    calibrationLoaded = true;
    return true;
}

bool AHRS::saveCalibration()
{
    calibration.robotNumber = robotNumber;
    calibration.calibrationTemperature = mpu->getTemperature();
    return ImuCalibration::save(preferences, calibration);
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

void AHRS::showCalibrationScreen(const char *title, const char *line1,
                                 const char *line2)
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

bool AHRS::calibrateGyroscope()
{
    float sum[3] = {}, sumSquared[3] = {};
    uint16_t samples = 0;
    Serial.println("CAL,gyro,settling");
    delay(GYRO_CAL_SETTLE_MS);
    const uint32_t started = millis();
    while (samples < 2500 && millis() - started < 30000)
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
        const float gyroNorm = sqrtf(gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]);
        if (accelNorm < GYRO_CAL_ACCEL_MIN_G || accelNorm > GYRO_CAL_ACCEL_MAX_G ||
            gyroNorm > GYRO_CAL_MAX_RATE_DPS)
            continue;
        for (uint8_t i = 0; i < 3; ++i)
        {
            const float value = gyro[i];
            sum[i] += value;
            sumSquared[i] += value * value;
        }
        ++samples;
    }
    if (samples < 2000)
        return false;
    for (uint8_t i = 0; i < 3; ++i)
    {
        calibration.gyroBias[i] = sum[i] / samples;
        const float variance = sumSquared[i] / samples -
                               calibration.gyroBias[i] * calibration.gyroBias[i];
        const float sigma = sqrtf(fmaxf(0.0f, variance));
        Serial.print("CAL,gyro,sigma=");
        Serial.print(i);
        Serial.print(',');
        Serial.println(sigma, 5);
        if (sigma > 0.20f)
            return false;
    }
    Serial.print("CAL,gyro,bias=");
    Serial.print(calibration.gyroBias[0], 5); Serial.print(',');
    Serial.print(calibration.gyroBias[1], 5); Serial.print(',');
    Serial.println(calibration.gyroBias[2], 5);
    return true;
}

bool AHRS::calibrateAccelerometerCloud()
{
    const char *axisNames[3] = {"X", "Y", "Z"};
    float positive[3] = {}, negative[3] = {};
    Serial.println("CAL,accelerometer,guided_six_position");
    Serial.println("CAL,accelerometer,local_g_not_needed_sensor_units_are_g");
    Serial.println("CAL,accelerometer,keep_robot_still_during_each_step");

    for (uint8_t axis = 0; axis < 3; ++axis)
    {
        float *faces[2] = {&positive[axis], &negative[axis]};
        for (uint8_t side = 0; side < 2; ++side)
        {
            Serial.print("CAL,accelerometer,place_axis=");
            Serial.print(axisNames[axis]);
            Serial.print(",side=");
            Serial.println(side == 0 ? "UP" : "DOWN");
            Serial.println("CAL,accelerometer,waiting_for_stable");
            char requested[24];
            snprintf(requested, sizeof(requested), "Put %s%c up",
                     axisNames[axis], side == 0 ? '+' : '-');
            showCalibrationScreen("Accel calibration", requested, "Hold still - starting");
            delay(1500);
            uint16_t count = 0;
            double sum = 0;
            uint32_t lastAcceptedAt = 0;
            const uint32_t phaseStarted = millis();
            while (count < CAL_FACE_SAMPLES &&
                   millis() - phaseStarted < CAL_FACE_TIMEOUT_MS)
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
                const float norm = sqrtf(sq(sample[0]) + sq(sample[1]) + sq(sample[2]));
                const float gyroNorm = sqrtf(sq(mpu->getGyroX()) + sq(mpu->getGyroY()) +
                                             sq(mpu->getGyroZ()));
                uint8_t dominantAxis = 0;
                if (fabsf(sample[1]) > fabsf(sample[dominantAxis])) dominantAxis = 1;
                if (fabsf(sample[2]) > fabsf(sample[dominantAxis])) dominantAxis = 2;
                const bool expectedSide = side == 0 ? sample[axis] >= 0 : sample[axis] < 0;
                if (norm < 0.85f || norm > 1.15f || gyroNorm > 3.0f ||
                    dominantAxis != axis || fabsf(sample[axis]) < 0.75f || !expectedSide)
                {
                    if (count == 0 && millis() - phaseStarted > 1200)
                    {
                        char current[24];
                        snprintf(current, sizeof(current), "Current: %c%c",
                                 "XYZ"[dominantAxis], sample[dominantAxis] >= 0 ? '+' : '-');
                        char needed[24];
                        snprintf(needed, sizeof(needed), "Need: %s%c",
                                 axisNames[axis], side == 0 ? '+' : '-');
                        showCalibrationScreen("Wrong position", current, needed);
                    }
                    continue;
                }
                if (count == 0)
                    showCalibrationScreen("Correct position", "Hold still", "Collecting...");
                sum += sample[axis];
                ++count;
                lastAcceptedAt = millis();
                if ((count % 30) == 0)
                {
                    Serial.print("CAL,accelerometer,axis=");
                    Serial.print(axisNames[axis]);
                    Serial.print(",side=");
                    Serial.print(side == 0 ? "UP" : "DOWN");
                    Serial.print(",samples=");
                    Serial.println(count);
                    char progress[24];
                    snprintf(progress, sizeof(progress), "Samples %u/%u", count,
                             CAL_FACE_SAMPLES);
                    showCalibrationScreen("Correct position", progress, "Keep still");
                }
            }
            if (count < CAL_FACE_SAMPLES)
            {
                showCalibrationScreen("Still waiting", "Position not stable", "Press q to cancel");
                delay(700);
                Serial.print("CAL,FAIL,accelerometer,axis=");
                Serial.print(axisNames[axis]);
                Serial.print(",side=");
                Serial.print(side == 0 ? "UP" : "DOWN");
                Serial.print(",samples=");
                Serial.println(count);
                return false;
            }
            *faces[side] = static_cast<float>(sum / count);
        }
    }

    ImuCalibration::setIdentity(calibration.accelMatrix);
    for (uint8_t axis = 0; axis < 3; ++axis)
    {
        const float span = positive[axis] - negative[axis];
        if (fabsf(span) < 0.50f)
        {
            Serial.print("CAL,FAIL,accelerometer,axis_span=");
            Serial.print(axisNames[axis]);
            Serial.println(",too_small");
            return false;
        }
        calibration.accelBias[axis] = 0.5f * (positive[axis] + negative[axis]);
        calibration.accelMatrix[axis][axis] = 2.0f / span;
        calibration.accelScale[axis] = 1.0f;
    }
    Serial.print("CAL,accelerometer,bias=");
    Serial.print(calibration.accelBias[0], 5); Serial.print(',');
    Serial.print(calibration.accelBias[1], 5); Serial.print(',');
    Serial.println(calibration.accelBias[2], 5);
    Serial.println("CAL,accelerometer,six_position=ready");
    return true;
}

void AHRS::correctAccelerometer(const float input[3], float output[3]) const
{
    ImuCalibration::applyMatrixCorrection(input, calibration.accelBias,
                                          calibration.accelMatrix, output);
}

void AHRS::update()
{
    if (!initialized || !mpu->update())
        return;

    const unsigned long now = millis();
    const float dt = (now - lastUpdateTime) * 0.001f;
    lastUpdateTime = now;
    if (dt <= 0 || dt > 0.1f)
        return;

    updateTelemetry(dt);

    const float accelNormG = sqrtf(sq(telemetry.accel[0]) +
                                   sq(telemetry.accel[1]) +
                                   sq(telemetry.accel[2])) /
                             G_CONST;
    const float gyroNorm = sqrtf(sq(telemetry.gyro[0]) +
                                 sq(telemetry.gyro[1]) +
                                 sq(telemetry.gyro[2]));
    const float oldAccelNormG = accelNormHistoryCount == STILL_ACCEL_WINDOW_SAMPLES
                                    ? accelNormHistory[accelNormHistoryIndex]
                                    : 0.0f;
    accelNormHistorySum -= oldAccelNormG;
    accelNormHistorySumSquares -= oldAccelNormG * oldAccelNormG;
    accelNormHistory[accelNormHistoryIndex] = accelNormG;
    accelNormHistorySum += accelNormG;
    accelNormHistorySumSquares += accelNormG * accelNormG;
    accelNormHistoryIndex =
        (accelNormHistoryIndex + 1) % STILL_ACCEL_WINDOW_SAMPLES;
    if (accelNormHistoryCount < STILL_ACCEL_WINDOW_SAMPLES)
        ++accelNormHistoryCount;

    uint8_t stillFailureMask = 0;
    if (fabsf(accelNormG - 1.0f) >= STILL_ACCEL_NORM_TOLERANCE_G)
        stillFailureMask |= STILL_FAILURE_ACCEL_NORM;
    if (gyroNorm >= STILL_GYRO_NORM_MAX_DPS)
        stillFailureMask |= STILL_FAILURE_GYRO_NORM;

    float accelNormStdDevG = 0.0f;
    if (accelNormHistoryCount < STILL_ACCEL_WINDOW_SAMPLES)
    {
        stillFailureMask |= STILL_FAILURE_ACCEL_STDDEV;
    }
    else
    {
        const float mean = accelNormHistorySum / STILL_ACCEL_WINDOW_SAMPLES;
        const float variance = fmaxf(
            0.0f, accelNormHistorySumSquares / STILL_ACCEL_WINDOW_SAMPLES -
                       mean * mean);
        accelNormStdDevG = sqrtf(variance);
        if (accelNormStdDevG >= STILL_ACCEL_NORM_STDDEV_MAX_G)
            stillFailureMask |= STILL_FAILURE_ACCEL_STDDEV;
    }
    telemetry.stillFailureMask = stillFailureMask;
    const bool stationary = stillFailureMask == 0;

    if (stationary)
    {
        if (stationaryStartTime == 0)
            stationaryStartTime = now;

        if (now - stationaryStartTime > STATIONARY_TIME_MS)
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
    telemetry.still = isStatic;

    if (telemetry.still)
    {
        if (biasStillStartTime == 0)
            biasStillStartTime = now;
        if (now - biasStillStartTime >= GYRO_BIAS_UPDATE_HOLD_MS)
        {
            const float gyroRaw[3] = {
                mpu->getGyroX(), mpu->getGyroY(), mpu->getGyroZ()};
            for (int i = 0; i < 3; ++i)
            {
                calibration.gyroBias[i] +=
                    GYRO_BIAS_UPDATE_ALPHA *
                    (gyroRaw[i] - calibration.gyroBias[i]);
                calibration.gyroBias[i] = initialGyroBias[i] +
                    fmaxf(-GYRO_BIAS_MAX_DRIFT_DPS,
                          fminf(GYRO_BIAS_MAX_DRIFT_DPS,
                                calibration.gyroBias[i] - initialGyroBias[i]));
                telemetry.gyroBias[i] = calibration.gyroBias[i];
            }
        }
    }
    else
    {
        biasStillStartTime = 0;
    }

    if (isStatic)
        return;

    for (int i = 0; i < 3; i++)
    {
        const float inputAcceleration = telemetry.filteredLinearAccel[i];
        const float magnitude = fabsf(inputAcceleration);
        float acceleration = inputAcceleration;
        if (deadbandMode == 1)
            acceleration = magnitude < DEADBAND_MS2 ? 0.0f : inputAcceleration;
        else if (deadbandMode == 2)
            acceleration = copysignf(fmaxf(magnitude - DEADBAND_MS2, 0.0f),
                                     inputAcceleration);
        velocity[i] += acceleration * dt;
        position[i] += velocity[i] * dt;
    }
}

void AHRS::updateTelemetry(float dt)
{
    telemetry.dt = dt;
    const float rawAccel[3] = {mpu->getAccX(), mpu->getAccY(), mpu->getAccZ()};
    float correctedAccel[3];
    correctAccelerometer(rawAccel, correctedAccel);
    telemetry.accel[0] = correctedAccel[0] * G_CONST;
    telemetry.accel[1] = correctedAccel[1] * G_CONST;
    telemetry.accel[2] = correctedAccel[2] * G_CONST;
    telemetry.gyro[0] = mpu->getGyroX() - calibration.gyroBias[0];
    telemetry.gyro[1] = mpu->getGyroY() - calibration.gyroBias[1];
    telemetry.gyro[2] = mpu->getGyroZ() - calibration.gyroBias[2];
    for (int i = 0; i < 3; ++i)
        telemetry.gyroBias[i] = calibration.gyroBias[i];
    telemetry.mag[0] = mpu->getMagX();
    telemetry.mag[1] = mpu->getMagY();
    telemetry.mag[2] = mpu->getMagZ();
    gyroFilter.update(telemetry.gyro, telemetry.filteredGyro);
    float medianGyro[3];
    gyroMedianFilter.update(telemetry.gyro, medianGyro);
    telemetry.quaternion[0] = mpu->getQuaternionW();
    telemetry.quaternion[1] = mpu->getQuaternionX();
    telemetry.quaternion[2] = mpu->getQuaternionY();
    telemetry.quaternion[3] = mpu->getQuaternionZ();
    attitude.update(correctedAccel, medianGyro, dt,
                    telemetry.roll, telemetry.pitch, telemetry.yaw);
    telemetry.temperature = mpu->getTemperature();

    float worldAccel[3];
    rotateBodyToWorld(telemetry.roll, telemetry.pitch, 0.0f,
                      telemetry.accel, worldAccel);
    linearAccel[0] = worldAccel[0];
    linearAccel[1] = worldAccel[1];
    const float gyroNorm = sqrtf(sq(telemetry.gyro[0]) +
                                 sq(telemetry.gyro[1]) +
                                 sq(telemetry.gyro[2]));
    const float worldAccelNorm = sqrtf(sq(worldAccel[0]) +
                                      sq(worldAccel[1]) +
                                      sq(worldAccel[2]));
    if (isStatic && gyroNorm < 3.0f && fabsf(worldAccelNorm - G_CONST) < 0.15f)
    {
        constexpr float GRAVITY_REFERENCE_ALPHA = 0.02f;
        gravityReference += GRAVITY_REFERENCE_ALPHA *
                            (worldAccel[2] - gravityReference);
    }
    linearAccel[2] = worldAccel[2] - gravityReference;
    telemetry.linearAccel[0] = linearAccel[0];
    telemetry.linearAccel[1] = linearAccel[1];
    telemetry.linearAccel[2] = linearAccel[2];

    filterTelemetry();
}

void AHRS::filterTelemetry()
{
    // Each pipeline is: median spike rejection -> Butterworth low-pass ->
    // scalar adaptive Kalman smoothing.
    accelFilter.update(linearAccel, telemetry.filteredLinearAccel);
    magFilter.update(telemetry.mag, telemetry.filteredMag);
}

bool AHRS::isMoving()
{
    return !isStatic;
}

void AHRS::resetPosition()
{
    for (int i = 0; i < 3; i++)
        velocity[i] = position[i] = 0;
}
