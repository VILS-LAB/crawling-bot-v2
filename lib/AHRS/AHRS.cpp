#include "AHRS.h"
#include "../../include/config.h"
#include <new>

namespace
{
constexpr float FACE_MIN_NORM_G = 0.80f;
constexpr float FACE_MAX_NORM_G = 1.20f;
constexpr float FACE_MIN_DOMINANCE_G = 0.72f;
constexpr float FACE_MAX_CROSS_AXIS_G = 0.42f;
constexpr float FACE_MAX_GYRO_DPS = 3.0f;
constexpr float FACE_MIN_SEPARATION_G = 1.35f;
constexpr float FACE_MAX_RESIDUAL_G = 0.08f;
constexpr uint8_t FACE_SAMPLE_COUNT = 48;
constexpr uint32_t FACE_TIMEOUT_MS = 90000;
constexpr uint32_t CALIBRATION_SETTLE_MS = 1200;
constexpr uint32_t CALIBRATION_MAGIC = 0x494D5532;
constexpr uint16_t CALIBRATION_VERSION = 3;
constexpr float FILTER_ACCEL_CUTOFF_HZ = 12.0f;
constexpr float FILTER_GYRO_CUTOFF_HZ = 25.0f;
constexpr uint16_t CAL_SAMPLE_COUNT = 1200;
constexpr uint32_t CAL_SAMPLE_TIMEOUT_MS = IMU_CALIBRATION_TIMEOUT_MS;
constexpr float CAL_GYRO_STILL_LIMIT_DPS = 15.0f;
constexpr float GYRO_CAL_ACCEL_MIN_G = 0.85f;
constexpr float GYRO_CAL_ACCEL_MAX_G = 1.15f;
constexpr float GYRO_CAL_MAX_RATE_DPS = 8.0f;
constexpr uint32_t GYRO_CAL_SETTLE_MS = 1500;
constexpr uint8_t CAL_MIN_SAMPLES_PER_OCTANT = 8;

struct CalibrationRecord
{
    uint32_t magic;
    uint16_t version;
    uint8_t robotNumber;
    uint8_t reserved;
    float accelBias[3];
    float accelScale[3];
    float accelMatrix[3][3];
    float gyroBias[3];
    float magBias[3];
    float magScale[3];
    float magMatrix[3][3];
};

void setIdentity(float matrix[3][3])
{
    memset(matrix, 0, sizeof(float) * 9);
    matrix[0][0] = matrix[1][1] = matrix[2][2] = 1.0f;
}

bool eigenSymmetric3(float input[3][3], float values[3], float vectors[3][3])
{
    float d[3][3];
    memcpy(d, input, sizeof(d));
    setIdentity(vectors);
    for (uint8_t i = 0; i < 3; ++i)
        values[i] = 0;
    for (uint8_t iteration = 0; iteration < 40; ++iteration)
    {
        uint8_t p = 0, q = 1;
        if (fabsf(d[0][2]) > fabsf(d[p][q]))
            p = 0, q = 2;
        if (fabsf(d[1][2]) > fabsf(d[p][q]))
            p = 1, q = 2;
        if (fabsf(d[p][q]) < 1e-7f)
            break;
        const float angle = 0.5f * atan2f(2.0f * d[p][q], d[q][q] - d[p][p]);
        const float c = cosf(angle), s = sinf(angle);
        const float app = c * c * d[p][p] - 2 * s * c * d[p][q] + s * s * d[q][q];
        const float aqq = s * s * d[p][p] + 2 * s * c * d[p][q] + c * c * d[q][q];
        for (uint8_t k = 0; k < 3; ++k)
        {
            if (k == p || k == q)
                continue;
            const float dkp = d[k][p], dkq = d[k][q];
            d[k][p] = d[p][k] = c * dkp - s * dkq;
            d[k][q] = d[q][k] = s * dkp + c * dkq;
        }
        d[p][p] = app;
        d[q][q] = aqq;
        d[p][q] = d[q][p] = 0;
        for (uint8_t k = 0; k < 3; ++k)
        {
            const float vkp = vectors[k][p], vkq = vectors[k][q];
            vectors[k][p] = c * vkp - s * vkq;
            vectors[k][q] = s * vkp + c * vkq;
        }
    }
    for (uint8_t i = 0; i < 3; ++i)
        values[i] = d[i][i];
    return true;
}

bool solveLinear9(float matrix[9][9], float rhs[9], float result[9])
{
    float augmented[9][10] = {};
    for (uint8_t row = 0; row < 9; ++row)
    {
        for (uint8_t column = 0; column < 9; ++column)
            augmented[row][column] = matrix[row][column];
        augmented[row][9] = rhs[row];
    }
    for (uint8_t column = 0; column < 9; ++column)
    {
        uint8_t pivot = column;
        for (uint8_t row = column + 1; row < 9; ++row)
            if (fabsf(augmented[row][column]) > fabsf(augmented[pivot][column]))
                pivot = row;
        if (fabsf(augmented[pivot][column]) < 1e-10f)
            return false;
        for (uint8_t index = column; index < 10; ++index)
        {
            const float value = augmented[column][index];
            augmented[column][index] = augmented[pivot][index];
            augmented[pivot][index] = value;
        }
        const float divisor = augmented[column][column];
        for (uint8_t index = column; index < 10; ++index)
            augmented[column][index] /= divisor;
        for (uint8_t row = 0; row < 9; ++row)
        {
            if (row == column)
                continue;
            const float factor = augmented[row][column];
            for (uint8_t index = column; index < 10; ++index)
                augmented[row][index] -= factor * augmented[column][index];
        }
    }
    for (uint8_t i = 0; i < 9; ++i)
        result[i] = augmented[i][9];
    return true;
}

bool inverse3(const float input[3][3], float output[3][3])
{
    float augmented[3][6] = {};
    for (uint8_t row = 0; row < 3; ++row)
        for (uint8_t column = 0; column < 3; ++column)
        {
            augmented[row][column] = input[row][column];
            augmented[row][column + 3] = row == column ? 1.0f : 0.0f;
        }
    for (uint8_t column = 0; column < 3; ++column)
    {
        uint8_t pivot = column;
        for (uint8_t row = column + 1; row < 3; ++row)
            if (fabsf(augmented[row][column]) > fabsf(augmented[pivot][column]))
                pivot = row;
        if (fabsf(augmented[pivot][column]) < 1e-9f)
            return false;
        for (uint8_t index = 0; index < 6; ++index)
        {
            const float value = augmented[column][index];
            augmented[column][index] = augmented[pivot][index];
            augmented[pivot][index] = value;
        }
        const float divisor = augmented[column][column];
        for (uint8_t index = 0; index < 6; ++index)
            augmented[column][index] /= divisor;
        for (uint8_t row = 0; row < 3; ++row)
        {
            if (row == column)
                continue;
            const float factor = augmented[row][column];
            for (uint8_t index = 0; index < 6; ++index)
                augmented[row][index] -= factor * augmented[column][index];
        }
    }
    for (uint8_t row = 0; row < 3; ++row)
        for (uint8_t column = 0; column < 3; ++column)
            output[row][column] = augmented[row][column + 3];
    return true;
}

bool fitAccelerometerEllipsoid(const float (*samples)[3], uint16_t count,
                               float bias[3], float correction[3][3], float &rms)
{
    if (count < 1200)
        return false;
    double mean[3] = {};
    for (uint16_t i = 0; i < count; ++i)
        for (uint8_t axis = 0; axis < 3; ++axis)
            mean[axis] += samples[i][axis];
    float scale[3];
    for (uint8_t axis = 0; axis < 3; ++axis)
    {
        mean[axis] /= count;
        double variance = 0;
        for (uint16_t i = 0; i < count; ++i)
        {
            const double delta = samples[i][axis] - mean[axis];
            variance += delta * delta;
        }
        scale[axis] = sqrtf(variance / count);
        if (scale[axis] < 1e-5f)
            return false;
    }

    double normal[9][9] = {}, right[9] = {};
    for (uint16_t i = 0; i < count; ++i)
    {
        const double x = (samples[i][0] - mean[0]) / scale[0];
        const double y = (samples[i][1] - mean[1]) / scale[1];
        const double z = (samples[i][2] - mean[2]) / scale[2];
        const double feature[9] = {x * x, y * y, z * z, 2 * x * y,
                                   2 * x * z, 2 * y * z, 2 * x, 2 * y, 2 * z};
        for (uint8_t row = 0; row < 9; ++row)
        {
            right[row] += feature[row];
            for (uint8_t column = 0; column < 9; ++column)
                normal[row][column] += feature[row] * feature[column];
        }
    }
    float normalFloat[9][9], rightFloat[9], coefficients[9];
    for (uint8_t row = 0; row < 9; ++row)
    {
        rightFloat[row] = right[row];
        for (uint8_t column = 0; column < 9; ++column)
            normalFloat[row][column] = normal[row][column];
    }
    if (!solveLinear9(normalFloat, rightFloat, coefficients))
        return false;

    float quadratic[3][3] = {
        {coefficients[0], coefficients[3], coefficients[4]},
        {coefficients[3], coefficients[1], coefficients[5]},
        {coefficients[4], coefficients[5], coefficients[2]}};
    const float linear[3] = {coefficients[6], coefficients[7], coefficients[8]};
    float quadraticInverse[3][3];
    if (!inverse3(quadratic, quadraticInverse))
        return false;

    float centerNormalized[3] = {};
    for (uint8_t row = 0; row < 3; ++row)
        for (uint8_t column = 0; column < 3; ++column)
            centerNormalized[row] -= quadraticInverse[row][column] * linear[column];
    float linearQuadraticLinear = 0;
    for (uint8_t row = 0; row < 3; ++row)
        for (uint8_t column = 0; column < 3; ++column)
            linearQuadraticLinear += linear[row] * quadraticInverse[row][column] * linear[column];
    const float normalization = 1.0f + linearQuadraticLinear;
    if (!isfinite(normalization) || normalization <= 1e-8f)
        return false;
    float normalizedQuadratic[3][3];
    for (uint8_t row = 0; row < 3; ++row)
        for (uint8_t column = 0; column < 3; ++column)
            normalizedQuadratic[row][column] = quadratic[row][column] / normalization;

    float eigenValues[3], eigenVectors[3][3];
    eigenSymmetric3(normalizedQuadratic, eigenValues, eigenVectors);
    for (uint8_t i = 0; i < 3; ++i)
        if (!isfinite(eigenValues[i]) || eigenValues[i] <= 1e-8f)
            return false;

    for (uint8_t axis = 0; axis < 3; ++axis)
        bias[axis] = mean[axis] + scale[axis] * centerNormalized[axis];
    const float target = 1.0f;
    float symmetricRoot[3][3] = {};
    for (uint8_t row = 0; row < 3; ++row)
        for (uint8_t column = 0; column < 3; ++column)
            for (uint8_t k = 0; k < 3; ++k)
                symmetricRoot[row][column] += eigenVectors[row][k] *
                    sqrtf(eigenValues[k]) * eigenVectors[column][k];
    for (uint8_t row = 0; row < 3; ++row)
        for (uint8_t column = 0; column < 3; ++column)
            correction[row][column] = target * symmetricRoot[row][column] / scale[column];

    double errorSquared = 0;
    for (uint16_t i = 0; i < count; ++i)
    {
        float centered[3], corrected[3] = {};
        for (uint8_t axis = 0; axis < 3; ++axis)
            centered[axis] = samples[i][axis] - bias[axis];
        for (uint8_t row = 0; row < 3; ++row)
            for (uint8_t column = 0; column < 3; ++column)
                corrected[row] += correction[row][column] * centered[column];
        const float error = sqrtf(sq(corrected[0]) + sq(corrected[1]) + sq(corrected[2])) - target;
        errorSquared += error * error;
    }
    rms = sqrtf(errorSquared / count);
    return isfinite(rms);
}
} // namespace

AHRS::AHRS() : initialized(false), isStatic(true), stationaryStartTime(0),
               robotNumber(1), calibrationLoaded(false), filtersInitialized(false),
               serialTelemetryEnabled(false), attitudeInitialized(false),
               attitudeRoll(0), attitudePitch(0), attitudeYaw(0)
{
    mpu = new MPU9250();
    for (int i = 0; i < 3; i++)
    {
        velocity[i] = position[i] = linearAccel[i] = 0;
        accelBias[i] = 0;
        accelScale[i] = 1;
        gyroBias[i] = 0;
        magBias[i] = 0;
        magScale[i] = 1;
    }
    setIdentity(accelMatrix);
    setIdentity(magMatrix);
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
        mpu->setMagBias(magBias[0], magBias[1], magBias[2]);
        mpu->setMagScale(magScale[0], magScale[1], magScale[2]);
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
    char key[8];
    snprintf(key, sizeof(key), "r%u", robotNumber);
    Preferences storage;
    if (!storage.begin("imu-cal", false))
    {
        Serial.println("IMU,ERROR,storage_open");
        return;
    }
    const bool removed = storage.remove(key);
    storage.end();
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
                Serial.println("IMU,t_ms,dt,ax,ay,az,lax,lay,laz,flax,flay,flaz,gx,gy,gz,fgx,fgy,fgz,roll,pitch,yaw,vx,vy,vz,px,py,pz,temp_c,moving");
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
        magBias[i] = mpu->getMagBias(i);
        magScale[i] = mpu->getMagScale(i);
    }
    return true;
}

bool AHRS::loadCalibration()
{
    CalibrationRecord record{};
    char key[8];
    snprintf(key, sizeof(key), "r%u", robotNumber);
    preferences.begin("imu-cal", true);
    const size_t length = preferences.getBytes(key, &record, sizeof(record));
    preferences.end();
    if (length != sizeof(record) || record.magic != CALIBRATION_MAGIC ||
        record.version != CALIBRATION_VERSION || record.robotNumber != robotNumber)
        return false;

    memcpy(accelBias, record.accelBias, sizeof(accelBias));
    memcpy(accelScale, record.accelScale, sizeof(accelScale));
    memcpy(accelMatrix, record.accelMatrix, sizeof(accelMatrix));
    memcpy(gyroBias, record.gyroBias, sizeof(gyroBias));
    memcpy(magBias, record.magBias, sizeof(magBias));
    memcpy(magScale, record.magScale, sizeof(magScale));
    memcpy(magMatrix, record.magMatrix, sizeof(magMatrix));
    calibrationLoaded = true;
    return true;
}

bool AHRS::saveCalibration()
{
    CalibrationRecord record{};
    record.magic = CALIBRATION_MAGIC;
    record.version = CALIBRATION_VERSION;
    record.robotNumber = robotNumber;
    memcpy(record.accelBias, accelBias, sizeof(accelBias));
    memcpy(record.accelScale, accelScale, sizeof(accelScale));
    memcpy(record.accelMatrix, accelMatrix, sizeof(accelMatrix));
    memcpy(record.gyroBias, gyroBias, sizeof(gyroBias));
    memcpy(record.magBias, magBias, sizeof(magBias));
    memcpy(record.magScale, magScale, sizeof(magScale));
    memcpy(record.magMatrix, magMatrix, sizeof(magMatrix));

    char key[8];
    snprintf(key, sizeof(key), "r%u", robotNumber);
    Preferences storage;
    if (!storage.begin("imu-cal", false))
        return false;
    const size_t written = storage.putBytes(key, &record, sizeof(record));
    storage.end();
    return written == sizeof(record);
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
        gyroBias[i] = sum[i] / samples;
        const float variance = sumSquared[i] / samples - gyroBias[i] * gyroBias[i];
        const float sigma = sqrtf(fmaxf(0.0f, variance));
        Serial.print("CAL,gyro,sigma=");
        Serial.print(i);
        Serial.print(',');
        Serial.println(sigma, 5);
        if (sigma > 0.20f)
            return false;
    }
    Serial.print("CAL,gyro,bias=");
    Serial.print(gyroBias[0], 5); Serial.print(',');
    Serial.print(gyroBias[1], 5); Serial.print(',');
    Serial.println(gyroBias[2], 5);
    return true;
}

bool AHRS::calibrateAccelerometerCloud()
{
    float (*samples)[3] = new (std::nothrow) float[CAL_SAMPLE_COUNT][3];
    if (samples == nullptr)
    {
        Serial.println("CAL,FAIL,accelerometer,memory");
        return false;
    }
    uint16_t octants[8] = {};
    uint16_t accepted = 0;
    const uint32_t started = millis();

    Serial.println("CAL,accelerometer,rotate and tilt slowly; cover all directions");
    while (accepted < CAL_SAMPLE_COUNT && millis() - started < CAL_SAMPLE_TIMEOUT_MS)
    {
        if (consumeCalibrationControl())
        {
            Serial.println("CAL,CANCELLED,accelerometer");
            delete[] samples;
            return false;
        }
        if (!mpu->update())
        {
            delay(2);
            continue;
        }
        const float sample[3] = {mpu->getAccX(), mpu->getAccY(), mpu->getAccZ()};
        const float gyroNorm = sqrtf(sq(mpu->getGyroX()) + sq(mpu->getGyroY()) + sq(mpu->getGyroZ()));
        const float norm = sqrtf(sq(sample[0]) + sq(sample[1]) + sq(sample[2]));
        if (norm < 0.70f || norm > 1.30f || gyroNorm > CAL_GYRO_STILL_LIMIT_DPS)
            continue;
        for (uint8_t i = 0; i < 3; ++i)
            samples[accepted][i] = sample[i];
        const uint8_t octant = (sample[0] >= 0 ? 1 : 0) |
                               (sample[1] >= 0 ? 2 : 0) |
                               (sample[2] >= 0 ? 4 : 0);
        ++octants[octant];
        ++accepted;
        if ((accepted % 100) == 0)
        {
            blinkCalibrationLed(1);
            Serial.print("CAL,accelerometer,samples=");
            Serial.print(accepted);
            Serial.print(",elapsed_ms=");
            Serial.print(millis() - started);
            Serial.print(",remaining_ms=");
            const uint32_t elapsed = static_cast<uint32_t>(millis() - started);
            Serial.println(CAL_SAMPLE_TIMEOUT_MS -
                           (elapsed < CAL_SAMPLE_TIMEOUT_MS ? elapsed : CAL_SAMPLE_TIMEOUT_MS));
        }
    }
    if (accepted < CAL_SAMPLE_COUNT)
    {
        Serial.print("CAL,FAIL,accelerometer_timeout,samples=");
        Serial.println(accepted);
        delete[] samples;
        return false;
    }
    for (uint8_t i = 0; i < 8; ++i)
    {
        if (octants[i] < CAL_MIN_SAMPLES_PER_OCTANT)
        {
            Serial.print("CAL,FAIL,accelerometer_octant=");
            Serial.print(i);
            Serial.print(",samples=");
            Serial.println(octants[i]);
            delete[] samples;
            return false;
        }
    }
    float fitRms = 0;
    if (!fitAccelerometerEllipsoid(samples, accepted, accelBias, accelMatrix, fitRms))
    {
        Serial.println("CAL,FAIL,accelerometer,ellipsoid_fit");
        delete[] samples;
        return false;
    }
    delete[] samples;
    memset(accelScale, 0, sizeof(accelScale));
    accelScale[0] = accelScale[1] = accelScale[2] = 1.0f;
    Serial.print("CAL,accelerometer,bias=");
    Serial.print(accelBias[0], 5); Serial.print(',');
    Serial.print(accelBias[1], 5); Serial.print(',');
    Serial.println(accelBias[2], 5);
    Serial.print("CAL,accelerometer,fit_rms=");
    Serial.println(fitRms, 6);
    Serial.println("CAL,accelerometer,full_matrix=ready");
    return true;
}

void AHRS::correctAccelerometer(const float input[3], float output[3]) const
{
    const float centered[3] = {input[0] - accelBias[0], input[1] - accelBias[1],
                               input[2] - accelBias[2]};
    for (uint8_t i = 0; i < 3; ++i)
        output[i] = accelMatrix[i][0] * centered[0] +
                    accelMatrix[i][1] * centered[1] +
                    accelMatrix[i][2] * centered[2];
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
    telemetry.gyro[0] = mpu->getGyroX() - gyroBias[0];
    telemetry.gyro[1] = mpu->getGyroY() - gyroBias[1];
    telemetry.gyro[2] = mpu->getGyroZ() - gyroBias[2];
    telemetry.mag[0] = mpu->getMagX();
    telemetry.mag[1] = mpu->getMagY();
    telemetry.mag[2] = mpu->getMagZ();
    telemetry.quaternion[0] = mpu->getQuaternionW();
    telemetry.quaternion[1] = mpu->getQuaternionX();
    telemetry.quaternion[2] = mpu->getQuaternionY();
    telemetry.quaternion[3] = mpu->getQuaternionZ();
    const float accelNorm = sqrtf(sq(correctedAccel[0]) + sq(correctedAccel[1]) +
                                  sq(correctedAccel[2]));
    const float accelRoll = atan2f(correctedAccel[1], correctedAccel[2]) * 180.0f / PI;
    const float accelPitch = -atan2f(correctedAccel[0],
                                     sqrtf(sq(correctedAccel[1]) + sq(correctedAccel[2]))) *
                             180.0f / PI;
    if (!attitudeInitialized)
    {
        attitudeRoll = accelRoll;
        attitudePitch = accelPitch;
        attitudeYaw = 0;
        attitudeInitialized = true;
    }
    else
    {
        const float gyroRollRate = telemetry.gyro[0];
        const float gyroPitchRate = telemetry.gyro[1];
        const float gyroYawRate = telemetry.gyro[2];
        attitudeRoll += gyroRollRate * dt;
        attitudePitch += gyroPitchRate * dt;
        attitudeYaw += gyroYawRate * dt;
        if (accelNorm > 0.85f && accelNorm < 1.15f)
        {
            const float correctionAlpha = 1.0f - expf(-dt / 0.50f);
            attitudeRoll += correctionAlpha * (accelRoll - attitudeRoll);
            attitudePitch += correctionAlpha * (accelPitch - attitudePitch);
        }
    }
    telemetry.roll = attitudeRoll;
    telemetry.pitch = attitudePitch;
    telemetry.yaw = attitudeYaw;
    telemetry.temperature = mpu->getTemperature();

    float worldAccel[3];
    rotateBodyToWorld(telemetry.roll, telemetry.pitch, 0.0f,
                      telemetry.accel, worldAccel);
    linearAccel[0] = worldAccel[0];
    linearAccel[1] = worldAccel[1];
    linearAccel[2] = worldAccel[2] - G_CONST;
    telemetry.linearAccel[0] = linearAccel[0];
    telemetry.linearAccel[1] = linearAccel[1];
    telemetry.linearAccel[2] = linearAccel[2];

    const float filterDt = telemetry.dt;
    const float accelAlpha = 1.0f - expf(-2.0f * PI * FILTER_ACCEL_CUTOFF_HZ * filterDt);
    const float gyroAlpha = 1.0f - expf(-2.0f * PI * FILTER_GYRO_CUTOFF_HZ * filterDt);
    for (uint8_t i = 0; i < 3; ++i)
    {
        if (!filtersInitialized)
        {
            telemetry.filteredLinearAccel[i] = linearAccel[i];
            telemetry.filteredGyro[i] = telemetry.gyro[i];
        }
        else
        {
            telemetry.filteredLinearAccel[i] += accelAlpha * (linearAccel[i] - telemetry.filteredLinearAccel[i]);
            telemetry.filteredGyro[i] += gyroAlpha * (telemetry.gyro[i] - telemetry.filteredGyro[i]);
        }
    }
    filtersInitialized = true;
}

void AHRS::rotateBodyToWorld(float roll, float pitch, float yaw,
                             const float body[3], float world[3])
{
    const float x = radians(roll);
    const float y = radians(pitch);
    const float z = radians(yaw);
    const float cx = cosf(x), sx = sinf(x);
    const float cy = cosf(y), sy = sinf(y);
    const float cz = cosf(z), sz = sinf(z);

    const float ax1 = body[0];
    const float ay1 = body[1] * cx - body[2] * sx;
    const float az1 = body[1] * sx + body[2] * cx;
    const float ax2 = ax1 * cy + az1 * sy;
    const float ay2 = ay1;
    const float az2 = -ax1 * sy + az1 * cy;
    world[0] = ax2 * cz - ay2 * sz;
    world[1] = ax2 * sz + ay2 * cz;
    world[2] = az2;
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
