#include "AHRS.h"
#include "../../include/config.h"

namespace
{
constexpr float FILTER_ACCEL_CUTOFF_HZ = 12.0f;
constexpr float FILTER_GYRO_CUTOFF_HZ = 25.0f;
constexpr float FILTER_MAG_CUTOFF_HZ = 5.0f;
constexpr float GYRO_CAL_ACCEL_MIN_G = 0.85f;
constexpr float GYRO_CAL_ACCEL_MAX_G = 1.15f;
constexpr float GYRO_CAL_MAX_RATE_DPS = 8.0f;
constexpr uint32_t GYRO_CAL_SETTLE_MS = 1500;
constexpr uint16_t CAL_FACE_SAMPLES = 180;
constexpr uint32_t CAL_FACE_TIMEOUT_MS = 12000;
constexpr uint32_t CAL_SAMPLE_INTERVAL_MS = 10;
}

AHRS::AHRS() : initialized(false), isStatic(true), stationaryStartTime(0),
               robotNumber(1), calibrationLoaded(false),
               serialTelemetryEnabled(false), gravityReference(G_CONST)
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
    accelFilter.configure(FILTER_ACCEL_CUTOFF_HZ, 200.0f, 0.03f, 0.10f);
    gyroFilter.configure(FILTER_GYRO_CUTOFF_HZ, 200.0f, 0.02f, 0.08f);
    magFilter.configure(FILTER_MAG_CUTOFF_HZ, 100.0f, 0.02f, 0.50f);
    pinMode(IMU_STATUS_LED_PIN, OUTPUT);
    digitalWrite(IMU_STATUS_LED_PIN, LOW);
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
        mpu->setMagBias(calibration.magBias[0], calibration.magBias[1], calibration.magBias[2]);
        mpu->setMagScale(calibration.magScale[0], calibration.magScale[1], calibration.magScale[2]);
        Serial.print("CAL,LOADED,robot=");
        Serial.println(robotNumber);
    }
    return true;
}

bool AHRS::calibrate()
{
    if (!initialized)
        return false;
    if (calibrationLoaded)
    {
        Serial.print("CAL,USING_SAVED,robot=");
        Serial.println(robotNumber);
        return true;
    }

    return runCalibration(true, true, true);
}

bool AHRS::runCalibration(bool gyro, bool accelerometer, bool magnetometer)
{
    Serial.print("CAL,START,robot=");
    Serial.println(robotNumber);

    if (gyro)
    {
        Serial.println("CAL,gyro,keep the robot completely still");
        if (!calibrateGyroscope())
        {
            Serial.println("CAL,FAIL,gyro calibration rejected");
            setCalibrationLed(false);
            return false;
        }
    }

    if (accelerometer)
    {
        Serial.println("CAL,accelerometer,move slowly through all orientations");
        if (!calibrateAccelerometerCloud())
        {
            Serial.println("CAL,FAIL,accelerometer calibration rejected");
            setCalibrationLed(false);
            return false;
        }
    }

    if (magnetometer)
    {
        if (!calibrateMagnetometer())
        {
            Serial.println("CAL,FAIL,magnetometer calibration rejected");
            setCalibrationLed(false);
            return false;
        }
    }

    if (!saveCalibration())
    {
        Serial.println("CAL,FAIL,storage");
        setCalibrationLed(false);
        return false;
    }
    calibrationLoaded = true;
    Serial.println("CAL,magnetometer,done");
    blinkCalibrationLed(3);
    Serial.println("CAL,DONE");
    setCalibrationLed(false);
    return true;
}

void AHRS::printCalibrationMenu() const
{
    Serial.println();
    Serial.println("IMU PANEL");
    Serial.println("  menu       show this menu");
    Serial.println("  status     show calibration and live values");
    Serial.println("  data       print one calibrated sensor snapshot");
    Serial.println("  stream on  start continuous CSV telemetry");
    Serial.println("  stream off stop continuous CSV telemetry");
    Serial.println("  cal gyro   calibrate gyro only");
    Serial.println("  cal accel  calibrate accelerometer only");
    Serial.println("  cal mag    calibrate magnetometer only");
    Serial.println("  cal all    calibrate all sensors");
    Serial.println("  clear      erase this robot calibration");
    Serial.println("During calibration: send x to cancel, ? for help");
    Serial.println("LED: solid=active, short flashes=progress, 3 flashes=success");
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
    Serial.println(sqrtf(sq(imu.gyro[0]) + sq(imu.gyro[1]) + sq(imu.gyro[2])), 4);
}

void AHRS::clearStoredCalibration()
{
    const bool removed = ImuCalibration::clear(preferences, robotNumber);
    calibrationLoaded = false;
    Serial.println(removed ? "IMU,CLEAR,done" : "IMU,CLEAR,nothing_to_remove");
}

void AHRS::serviceConsole()
{
    static String command;
    static bool announced = false;
    if (!announced)
    {
        printCalibrationMenu();
        announced = true;
    }
    while (Serial.available())
    {
        const char character = static_cast<char>(Serial.read());
        if (character == '\r')
            continue;
        if (character == '\n')
        {
            command.trim();
            command.toLowerCase();
            if (command == "menu" || command == "help")
                printCalibrationMenu();
            else if (command == "status")
                printCalibrationStatus();
            else if (command == "data")
            {
                const ImuTelemetry &imu = telemetry;
                Serial.print("IMU,DATA,accel=");
                Serial.print(imu.accel[0], 4); Serial.print(',');
                Serial.print(imu.accel[1], 4); Serial.print(',');
                Serial.print(imu.accel[2], 4);
                Serial.print(",gyro=");
                Serial.print(imu.gyro[0], 4); Serial.print(',');
                Serial.print(imu.gyro[1], 4); Serial.print(',');
                Serial.println(imu.gyro[2], 4);
            }
            else if (command == "stream on")
            {
                serialTelemetryEnabled = true;
                Serial.println("IMU,STREAM,on");
                Serial.println("IMU,t_ms,dt,ax,ay,az,lax,lay,laz,flax,flay,flaz,gx,gy,gz,fgx,fgy,fgz,magx,magy,magz,roll,pitch,yaw,vx,vy,vz,px,py,pz,temp_c,moving");
            }
            else if (command == "stream off")
            {
                serialTelemetryEnabled = false;
                Serial.println("IMU,STREAM,off");
            }
            else if (command == "cal gyro")
                runCalibration(true, false, false);
            else if (command == "cal accel")
                runCalibration(false, true, false);
            else if (command == "cal mag")
                runCalibration(false, false, true);
            else if (command == "cal all")
                runCalibration(true, true, true);
            else if (command == "clear")
                clearStoredCalibration();
            else if (command.length() > 0)
                Serial.println("IMU,ERROR,unknown_command");
            command = "";
        }
        else if (command.length() < 48)
            command += character;
    }
}

bool AHRS::calibrateMagnetometer()
{
    Serial.println("CAL,magnetometer,rotate slowly in all directions for 15 seconds");
    setCalibrationLed(true);
    mpu->calibrateMag();
    for (uint8_t i = 0; i < 3; ++i)
    {
        calibration.magBias[i] = mpu->getMagBias(i);
        calibration.magScale[i] = mpu->getMagScale(i);
    }
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

void AHRS::setCalibrationLed(bool on)
{
    digitalWrite(IMU_STATUS_LED_PIN, on ? HIGH : LOW);
}

void AHRS::blinkCalibrationLed(uint8_t count)
{
    for (uint8_t i = 0; i < count; ++i)
    {
        setCalibrationLed(false);
        delay(120);
        setCalibrationLed(true);
        delay(120);
    }
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
        if ((samples % 250) == 0)
            blinkCalibrationLed(1);
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
    int8_t firstSideSign[3] = {};
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
                const bool expectedSide = side == 0 ||
                                          (firstSideSign[axis] > 0 ? sample[axis] < 0
                                                                   : sample[axis] >= 0);
                if (norm < 0.85f || norm > 1.15f || gyroNorm > 3.0f ||
                    dominantAxis != axis || fabsf(sample[axis]) < 0.75f || !expectedSide)
                    continue;
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
                }
            }
            if (count < CAL_FACE_SAMPLES)
            {
                Serial.print("CAL,FAIL,accelerometer,axis=");
                Serial.print(axisNames[axis]);
                Serial.print(",side=");
                Serial.print(side == 0 ? "UP" : "DOWN");
                Serial.print(",samples=");
                Serial.println(count);
                return false;
            }
            *faces[side] = static_cast<float>(sum / count);
            if (side == 0)
                firstSideSign[axis] = *faces[side] >= 0 ? 1 : -1;
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

    const float calibratedAccelNorm = sqrtf(sq(telemetry.accel[0]) +
                                            sq(telemetry.accel[1]) +
                                            sq(telemetry.accel[2]));
    const float accelError = fabsf(calibratedAccelNorm - G_CONST);
    const float gyroNorm = sqrt(sq(telemetry.gyro[0]) +
                                sq(telemetry.gyro[1]) +
                                sq(telemetry.gyro[2]));

    if (accelError < MOTION_THRESHOLD && gyroNorm < GYRO_THRESHOLD)
    {
        if (stationaryStartTime == 0)
            stationaryStartTime = now;

        if (now - stationaryStartTime > STATIONARY_TIME_MS)
        {
            isStatic = true;
            velocity[0] = velocity[1] = velocity[2] = 0;
            return;
        }
    }
    else
    {
        stationaryStartTime = 0;
        isStatic = false;
    }

    if (!isStatic)
    {
        for (int i = 0; i < 3; i++)
        {
            velocity[i] += telemetry.filteredLinearAccel[i] * dt;
            position[i] += velocity[i] * dt;
        }
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
    telemetry.mag[0] = mpu->getMagX();
    telemetry.mag[1] = mpu->getMagY();
    telemetry.mag[2] = mpu->getMagZ();
    gyroFilter.update(telemetry.gyro, telemetry.filteredGyro);
    telemetry.quaternion[0] = mpu->getQuaternionW();
    telemetry.quaternion[1] = mpu->getQuaternionX();
    telemetry.quaternion[2] = mpu->getQuaternionY();
    telemetry.quaternion[3] = mpu->getQuaternionZ();
    attitude.update(correctedAccel, telemetry.filteredGyro, dt,
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
