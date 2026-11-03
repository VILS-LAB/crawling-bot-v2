#ifndef IMU_CALIBRATION_H
#define IMU_CALIBRATION_H

#include <Arduino.h>
#include <Preferences.h>
#include <MPU9250.h>

struct ImuCalibrationData
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
    float calibrationTemperature;
    float magFieldNorm;
};

namespace ImuCalibration
{
void setIdentity(float matrix[3][3]);
bool fitAccelerometerEllipsoid(const float (*samples)[3], uint16_t count,
                               float bias[3], float correction[3][3], float &rms);
void applyMatrixCorrection(const float input[3], const float bias[3],
                           const float matrix[3][3], float output[3]);
bool load(Preferences &preferences, uint8_t robotNumber, ImuCalibrationData &data);
bool save(Preferences &preferences, const ImuCalibrationData &data);
bool clear(Preferences &preferences, uint8_t robotNumber);
}

#endif
