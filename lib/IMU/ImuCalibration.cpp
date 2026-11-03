#include "ImuCalibration.h"
#include <math.h>

namespace
{
constexpr uint32_t CALIBRATION_MAGIC = 0x494D5532;
constexpr uint16_t CALIBRATION_VERSION = 4;

bool eigenSymmetric3(float input[3][3], float values[3], float vectors[3][3])
{
    float d[3][3];
    memcpy(d, input, sizeof(d));
    ImuCalibration::setIdentity(vectors);
    for (uint8_t iteration = 0; iteration < 40; ++iteration)
    {
        uint8_t p = 0, q = 1;
        if (fabsf(d[0][2]) > fabsf(d[p][q])) p = 0, q = 2;
        if (fabsf(d[1][2]) > fabsf(d[p][q])) p = 1, q = 2;
        if (fabsf(d[p][q]) < 1e-7f) break;
        const float angle = 0.5f * atan2f(2.0f * d[p][q], d[q][q] - d[p][p]);
        const float c = cosf(angle), s = sinf(angle);
        const float app = c*c*d[p][p] - 2*s*c*d[p][q] + s*s*d[q][q];
        const float aqq = s*s*d[p][p] + 2*s*c*d[p][q] + c*c*d[q][q];
        for (uint8_t k = 0; k < 3; ++k)
        {
            if (k == p || k == q) continue;
            const float dkp = d[k][p], dkq = d[k][q];
            d[k][p] = d[p][k] = c*dkp - s*dkq;
            d[k][q] = d[q][k] = s*dkp + c*dkq;
        }
        d[p][p] = app; d[q][q] = aqq; d[p][q] = d[q][p] = 0;
        for (uint8_t k = 0; k < 3; ++k)
        {
            const float vkp = vectors[k][p], vkq = vectors[k][q];
            vectors[k][p] = c*vkp - s*vkq;
            vectors[k][q] = s*vkp + c*vkq;
        }
    }
    for (uint8_t i = 0; i < 3; ++i) values[i] = d[i][i];
    return true;
}

bool solveLinear9(float matrix[9][9], float rhs[9], float result[9])
{
    float a[9][10] = {};
    for (uint8_t r = 0; r < 9; ++r)
    {
        for (uint8_t c = 0; c < 9; ++c) a[r][c] = matrix[r][c];
        a[r][9] = rhs[r];
    }
    for (uint8_t c = 0; c < 9; ++c)
    {
        uint8_t pivot = c;
        for (uint8_t r = c + 1; r < 9; ++r)
            if (fabsf(a[r][c]) > fabsf(a[pivot][c])) pivot = r;
        if (fabsf(a[pivot][c]) < 1e-10f) return false;
        for (uint8_t i = c; i < 10; ++i) {
            const float value = a[c][i]; a[c][i] = a[pivot][i]; a[pivot][i] = value;
        }
        const float divisor = a[c][c];
        for (uint8_t i = c; i < 10; ++i) a[c][i] /= divisor;
        for (uint8_t r = 0; r < 9; ++r)
        {
            if (r == c) continue;
            const float factor = a[r][c];
            for (uint8_t i = c; i < 10; ++i) a[r][i] -= factor * a[c][i];
        }
    }
    for (uint8_t i = 0; i < 9; ++i) result[i] = a[i][9];
    return true;
}

bool inverse3(const float input[3][3], float output[3][3])
{
    float a[3][6] = {};
    for (uint8_t r = 0; r < 3; ++r)
        for (uint8_t c = 0; c < 3; ++c)
        {
            a[r][c] = input[r][c];
            a[r][c + 3] = r == c ? 1.0f : 0.0f;
        }
    for (uint8_t c = 0; c < 3; ++c)
    {
        uint8_t pivot = c;
        for (uint8_t r = c + 1; r < 3; ++r)
            if (fabsf(a[r][c]) > fabsf(a[pivot][c])) pivot = r;
        if (fabsf(a[pivot][c]) < 1e-9f) return false;
        for (uint8_t i = 0; i < 6; ++i) {
            const float value = a[c][i]; a[c][i] = a[pivot][i]; a[pivot][i] = value;
        }
        const float divisor = a[c][c];
        for (uint8_t i = 0; i < 6; ++i) a[c][i] /= divisor;
        for (uint8_t r = 0; r < 3; ++r)
        {
            if (r == c) continue;
            const float factor = a[r][c];
            for (uint8_t i = 0; i < 6; ++i) a[r][i] -= factor * a[c][i];
        }
    }
    for (uint8_t r = 0; r < 3; ++r)
        for (uint8_t c = 0; c < 3; ++c) output[r][c] = a[r][c + 3];
    return true;
}
}

namespace ImuCalibration
{
void setIdentity(float matrix[3][3])
{
    memset(matrix, 0, sizeof(float) * 9);
    matrix[0][0] = matrix[1][1] = matrix[2][2] = 1.0f;
}

bool fitAccelerometerEllipsoid(const float (*samples)[3], uint16_t count,
                               float bias[3], float correction[3][3], float &rms)
{
    if (count < 1200) return false;
    double mean[3] = {};
    float scale[3];
    for (uint16_t i = 0; i < count; ++i)
        for (uint8_t axis = 0; axis < 3; ++axis) mean[axis] += samples[i][axis];
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
        if (scale[axis] < 1e-5f) return false;
    }
    double normal[9][9] = {}, right[9] = {};
    for (uint16_t i = 0; i < count; ++i)
    {
        const double x = (samples[i][0] - mean[0]) / scale[0];
        const double y = (samples[i][1] - mean[1]) / scale[1];
        const double z = (samples[i][2] - mean[2]) / scale[2];
        const double f[9] = {x*x, y*y, z*z, 2*x*y, 2*x*z, 2*y*z, 2*x, 2*y, 2*z};
        for (uint8_t r = 0; r < 9; ++r)
        {
            right[r] += f[r];
            for (uint8_t c = 0; c < 9; ++c) normal[r][c] += f[r] * f[c];
        }
    }
    float n[9][9], b[9], coefficients[9];
    for (uint8_t r = 0; r < 9; ++r) {
        b[r] = right[r];
        for (uint8_t c = 0; c < 9; ++c) n[r][c] = normal[r][c];
    }
    if (!solveLinear9(n, b, coefficients)) return false;
    float quadratic[3][3] = {
        {coefficients[0], coefficients[3], coefficients[4]},
        {coefficients[3], coefficients[1], coefficients[5]},
        {coefficients[4], coefficients[5], coefficients[2]}};
    const float linear[3] = {coefficients[6], coefficients[7], coefficients[8]};
    float quadraticInverse[3][3];
    if (!inverse3(quadratic, quadraticInverse)) return false;
    float center[3] = {};
    for (uint8_t r = 0; r < 3; ++r)
        for (uint8_t c = 0; c < 3; ++c) center[r] -= quadraticInverse[r][c] * linear[c];
    float linearQuadraticLinear = 0;
    for (uint8_t r = 0; r < 3; ++r)
        for (uint8_t c = 0; c < 3; ++c)
            linearQuadraticLinear += linear[r] * quadraticInverse[r][c] * linear[c];
    const float normalization = 1.0f + linearQuadraticLinear;
    if (!isfinite(normalization) || normalization <= 1e-8f) return false;
    float normalizedQuadratic[3][3];
    for (uint8_t r = 0; r < 3; ++r)
        for (uint8_t c = 0; c < 3; ++c) normalizedQuadratic[r][c] = quadratic[r][c] / normalization;
    float eigenValues[3], eigenVectors[3][3];
    eigenSymmetric3(normalizedQuadratic, eigenValues, eigenVectors);
    for (uint8_t i = 0; i < 3; ++i)
        if (!isfinite(eigenValues[i]) || eigenValues[i] <= 1e-8f) return false;
    for (uint8_t axis = 0; axis < 3; ++axis) bias[axis] = mean[axis] + scale[axis] * center[axis];
    float root[3][3] = {};
    for (uint8_t r = 0; r < 3; ++r)
        for (uint8_t c = 0; c < 3; ++c)
            for (uint8_t k = 0; k < 3; ++k)
                root[r][c] += eigenVectors[r][k] * sqrtf(eigenValues[k]) * eigenVectors[c][k];
    for (uint8_t r = 0; r < 3; ++r)
        for (uint8_t c = 0; c < 3; ++c) correction[r][c] = root[r][c] / scale[c];
    double errorSquared = 0;
    for (uint16_t i = 0; i < count; ++i)
    {
        float corrected[3] = {};
        for (uint8_t r = 0; r < 3; ++r)
            for (uint8_t c = 0; c < 3; ++c)
                corrected[r] += correction[r][c] * (samples[i][c] - bias[c]);
        const float error = sqrtf(sq(corrected[0]) + sq(corrected[1]) + sq(corrected[2])) - 1.0f;
        errorSquared += error * error;
    }
    rms = sqrtf(errorSquared / count);
    return isfinite(rms);
}

void applyMatrixCorrection(const float input[3], const float bias[3],
                           const float matrix[3][3], float output[3])
{
    const float centered[3] = {input[0] - bias[0], input[1] - bias[1], input[2] - bias[2]};
    for (uint8_t r = 0; r < 3; ++r)
        output[r] = matrix[r][0] * centered[0] + matrix[r][1] * centered[1] +
                    matrix[r][2] * centered[2];
}

bool load(Preferences &preferences, uint8_t robotNumber, ImuCalibrationData &data)
{
    char key[8];
    snprintf(key, sizeof(key), "r%u", robotNumber);
    if (!preferences.begin("imu-cal", true)) return false;
    const size_t length = preferences.getBytes(key, &data, sizeof(data));
    preferences.end();
    return length == sizeof(data) && data.magic == CALIBRATION_MAGIC &&
           data.version == CALIBRATION_VERSION && data.robotNumber == robotNumber;
}

bool save(Preferences &preferences, const ImuCalibrationData &data)
{
    char key[8];
    snprintf(key, sizeof(key), "r%u", data.robotNumber);
    ImuCalibrationData record = data;
    record.magic = CALIBRATION_MAGIC;
    record.version = CALIBRATION_VERSION;
    if (!preferences.begin("imu-cal", false)) return false;
    const size_t length = preferences.putBytes(key, &record, sizeof(record));
    preferences.end();
    return length == sizeof(data);
}

bool clear(Preferences &preferences, uint8_t robotNumber)
{
    char key[8];
    snprintf(key, sizeof(key), "r%u", robotNumber);
    if (!preferences.begin("imu-cal", false)) return false;
    const bool removed = preferences.remove(key);
    preferences.end();
    return removed;
}
}
