#ifndef IMU_CALIBRATION_H
#define IMU_CALIBRATION_H

// Sensor calibration model and fitting math.
//
// The model is the one PX4 uses (src/lib/sensor_calibration/*.hpp):
//
//   accel_corrected = accelMatrix * (accel_raw - accelBias)
//   gyro_corrected  = gyro_raw - gyroBias
//   mag_corrected   = magMatrix * (mag_raw - magOffset)
//
// `accelMatrix` is a full 3x3 transform (offset + gain + cross-axis), which is
// what PX4's six-position solve produces. PX4 then throws away the off-diagonal
// terms and keeps only the diagonal; we keep the full matrix because the GY-91
// breakout is hand-soldered and its axes are not perfectly orthogonal.
//
// The fitting routines are ports of:
//   * src/modules/commander/accelerometer_calibration.cpp  (six-position solve)
//   * src/modules/commander/lm_fit.cpp                     (LM sphere/ellipsoid)
//
// Everything here is free of Arduino/ESP-IDF dependencies except the
// Preferences-based persistence at the bottom, so the math can be host-tested.

#include <math.h>
#include <string.h>

#include "ImuMath.h"

namespace imu
{

// Number of guided accelerometer positions (PX4 detect_orientation_side_count).
constexpr uint8_t kAccelCalibrationSides = 6;

// Order matches PX4's detect_orientation enum so the reference vectors line up.
enum class AccelSide : uint8_t
{
    TailDown = 0,     // +X up   -> reference ( g, 0, 0)
    NoseDown = 1,     // -X up   -> reference (-g, 0, 0)
    Left = 2,         // +Y up   -> reference ( 0, g, 0)
    Right = 3,        // -Y up   -> reference ( 0,-g, 0)
    UpsideDown = 4,   // +Z up   -> reference ( 0, 0, g)
    RightSideUp = 5,  // -Z up   -> reference ( 0, 0,-g)
};

const char *accelSideName(AccelSide side);

struct ImuCalibrationData
{
    uint32_t magic = 0;
    uint16_t version = 0;
    uint8_t robotNumber = 0;
    uint8_t reserved = 0;

    // Accelerometer: corrected = accelMatrix * (raw - accelBias), units of g.
    float accelBias[3] = {0.0f, 0.0f, 0.0f};
    float accelMatrix[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

    // Gyroscope: corrected = raw - gyroBias, units of deg/s.
    float gyroBias[3] = {0.0f, 0.0f, 0.0f};

    // Magnetometer: corrected = magMatrix * (raw - magOffset), units of uT.
    float magOffset[3] = {0.0f, 0.0f, 0.0f};
    float magMatrix[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

    // Diagnostics / self-check.
    float calibrationTemperature = 0.0f;
    float magFieldNorm = 0.0f;      // uT, expected field magnitude
    float accelFitRms = 0.0f;       // g, residual of the six-position fit
    float magFitRms = 0.0f;         // uT, residual of the sphere/ellipsoid fit
    uint8_t accelSides = 0;         // how many positions were actually used
    uint8_t magFitKind = 0;         // 0 = none, 1 = sphere, 2 = ellipsoid
};

namespace calibration
{

// --- accelerometer ---------------------------------------------------------

// Solve the PX4 six-position model from the six averaged raw readings.
// `raw[side][axis]` is in g (sensor units), indexed by AccelSide.
// Fills `bias` and `matrix` (corrected = matrix * (raw - bias)).
// Returns false when the data is degenerate (a face pair too close together).
bool fitAccelerometerSixPosition(const float raw[kAccelCalibrationSides][3],
                                 float bias[3], float matrix[3][3], float &rms);

// --- magnetometer ----------------------------------------------------------

// Levenberg-Marquardt sphere fit (PX4 lm_mag_fit with full_ellipsoid = false).
// `samples` is a flat array of `count` xyz triples in uT.
// Produces offset + a diagonal scale that normalizes the field to `radius`.
bool fitMagnetometerSphere(const float *samples, uint32_t count, float offset[3],
                           float matrix[3][3], float &radius, float &rms);

// Levenberg-Marquardt ellipsoid fit (PX4 lm_mag_fit with full_ellipsoid = true).
// Must be seeded from a successful sphere fit, exactly like PX4.
bool fitMagnetometerEllipsoid(const float *samples, uint32_t count, float offset[3],
                              float matrix[3][3], float &radius, float &rms);

// --- application -----------------------------------------------------------

void applyAccelCorrection(const float raw[3], const ImuCalibrationData &data,
                          float corrected[3]);
void applyMagCorrection(const float raw[3], const ImuCalibrationData &data,
                        float corrected[3]);

// --- helpers ---------------------------------------------------------------

void setIdentity(float matrix[3][3]);
bool invertMatrix(const float input[3][3], float output[3][3]);

} // namespace calibration

// --- persistence (Arduino only) --------------------------------------------
// Declared here so callers do not need to know about Preferences; the
// implementation is compiled only when Arduino is available.
struct ImuCalibrationStore
{
    // Returns true when a record for `robotNumber` was found and is valid.
    static bool load(uint8_t robotNumber, ImuCalibrationData &data);
    static bool save(const ImuCalibrationData &data);
    static bool clear(uint8_t robotNumber);
};

} // namespace imu

#endif // IMU_CALIBRATION_H
