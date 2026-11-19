#include "ImuCalibration.h"

#if defined(ARDUINO)
#include <Arduino.h>
#include <Preferences.h>
#endif

namespace imu
{

namespace
{
#if defined(ARDUINO)
constexpr uint32_t kCalibrationMagic = 0x494D5533; // "IMU3"
constexpr uint16_t kCalibrationVersion = 5;
#endif

// PX4 lm_mag_fit() tuning (src/modules/commander/lm_fit.cpp).
constexpr int kMagFitMaxIterations = 100;
constexpr int kMagFitMinIterations = 10;
// PX4 works in Gauss (radius 0.2 .. 0.7 G, cost 0.01 G). This project keeps the
// magnetometer in uT, and 1 G = 100 uT, so the same limits become 20 .. 70 uT
// and 1 uT. Earth's field is ~25 .. 65 uT, so the window is right.
constexpr float kMagFitCostThreshold = 1.0f;    // uT
constexpr float kMagFitStepThreshold = 0.001f;  // dimensionless damping
constexpr float kMagFitMinRadius = 20.0f;       // uT  (0.2 G)
constexpr float kMagFitMaxRadius = 70.0f;       // uT  (0.7 G)
constexpr float kLmDamping = 10.0f;

// Generic dense inverse with partial pivoting. `n` is small (4 or 9).
template <int N>
bool invertDense(float matrix[N][N], float out[N][N])
{
    float a[N][2 * N];
    for (int r = 0; r < N; ++r)
        for (int c = 0; c < N; ++c)
        {
            a[r][c] = matrix[r][c];
            a[r][c + N] = (r == c) ? 1.0f : 0.0f;
        }

    for (int c = 0; c < N; ++c)
    {
        int pivot = c;
        for (int r = c + 1; r < N; ++r)
            if (fabsf(a[r][c]) > fabsf(a[pivot][c]))
                pivot = r;
        if (fabsf(a[pivot][c]) < 1e-12f)
            return false;
        if (pivot != c)
            for (int i = 0; i < 2 * N; ++i)
            {
                const float swap = a[c][i];
                a[c][i] = a[pivot][i];
                a[pivot][i] = swap;
            }
        const float divisor = a[c][c];
        for (int i = 0; i < 2 * N; ++i)
            a[c][i] /= divisor;
        for (int r = 0; r < N; ++r)
        {
            if (r == c)
                continue;
            const float factor = a[r][c];
            if (factor == 0.0f)
                continue;
            for (int i = 0; i < 2 * N; ++i)
                a[r][i] -= factor * a[c][i];
        }
    }

    for (int r = 0; r < N; ++r)
        for (int c = 0; c < N; ++c)
        {
            out[r][c] = a[r][c + N];
            if (!isFinitef(out[r][c]))
                return false;
        }
    return true;
}

// Symmetric 3x3 matrix built from PX4's diag/offdiag representation.
struct Symmetric3
{
    float diag[3] = {1.0f, 1.0f, 1.0f};
    float offdiag[3] = {0.0f, 0.0f, 0.0f}; // xy, xz, yz

    Vec3 multiply(const Vec3 &v) const
    {
        return {diag[0] * v.x + offdiag[0] * v.y + offdiag[1] * v.z,
                offdiag[0] * v.x + diag[1] * v.y + offdiag[2] * v.z,
                offdiag[1] * v.x + offdiag[2] * v.y + diag[2] * v.z};
    }

    void toMatrix(float matrix[3][3]) const
    {
        matrix[0][0] = diag[0];
        matrix[0][1] = offdiag[0];
        matrix[0][2] = offdiag[1];
        matrix[1][0] = offdiag[0];
        matrix[1][1] = diag[1];
        matrix[1][2] = offdiag[2];
        matrix[2][0] = offdiag[1];
        matrix[2][1] = offdiag[2];
        matrix[2][2] = diag[2];
    }
};

struct MagFitState
{
    Vec3 offset{};
    Symmetric3 scale{};
    float radius = 0.2f;
    float cost = 1e30f;
    float damping = 1.0f;
    bool ok = false;
};

// One Levenberg-Marquardt iteration of PX4's sphere fit (scale fixed to I).
bool lmSphereIteration(const float *samples, uint32_t count, MagFitState &state)
{
    float jtj[4][4] = {};
    float jtfi[4] = {};

    for (uint32_t k = 0; k < count; ++k)
    {
        const float x = samples[3 * k + 0] - state.offset.x;
        const float y = samples[3 * k + 1] - state.offset.y;
        const float z = samples[3 * k + 2] - state.offset.z;
        const float length = sqrtf(x * x + y * y + z * z);
        if (length < 1e-6f)
            continue;

        const float jacobian[4] = {1.0f, x / length, y / length, z / length};
        const float residual = state.radius - length;

        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 4; ++j)
                jtj[i][j] += jacobian[i] * jacobian[j];
            jtfi[i] += jacobian[i] * residual;
        }
    }

    // Symmetrize to fight numerical drift, exactly like PX4.
    for (int i = 0; i < 4; ++i)
        for (int j = i + 1; j < 4; ++j)
        {
            const float mean = 0.5f * (jtj[i][j] + jtj[j][i]);
            jtj[i][j] = jtj[j][i] = mean;
        }

    float damped[4][4];
    float dampedHalf[4][4];
    memcpy(damped, jtj, sizeof(damped));
    memcpy(dampedHalf, jtj, sizeof(dampedHalf));
    for (int i = 0; i < 4; ++i)
    {
        damped[i][i] += state.damping;
        dampedHalf[i][i] += state.damping / kLmDamping;
    }

    float inverse[4][4];
    float inverseHalf[4][4];
    if (!invertDense<4>(damped, inverse) || !invertDense<4>(dampedHalf, inverseHalf))
        return false;

    float params1[4] = {state.radius, state.offset.x, state.offset.y, state.offset.z};
    float params2[4];
    memcpy(params2, params1, sizeof(params1));

    for (int row = 0; row < 4; ++row)
        for (int col = 0; col < 4; ++col)
        {
            params1[row] -= jtfi[col] * inverse[row][col];
            params2[row] -= jtfi[col] * inverseHalf[row][col];
        }

    auto costOf = [&](const float params[4]) {
        float sum = 0.0f;
        for (uint32_t k = 0; k < count; ++k)
        {
            const float dx = samples[3 * k + 0] - params[1];
            const float dy = samples[3 * k + 1] - params[2];
            const float dz = samples[3 * k + 2] - params[3];
            const float residual = params[0] - sqrtf(dx * dx + dy * dy + dz * dz);
            sum += residual * residual;
        }
        return sqrtf(sum) / static_cast<float>(count);
    };

    const float cost1 = costOf(params1);
    const float cost2 = costOf(params2);

    float fitness = state.cost;
    if (cost1 > state.cost && cost2 > state.cost)
    {
        state.damping *= kLmDamping;
    }
    else if (cost2 < state.cost && cost2 < cost1)
    {
        state.damping /= kLmDamping;
        memcpy(params1, params2, sizeof(params1));
        fitness = cost2;
    }
    else if (cost1 < state.cost)
    {
        fitness = cost1;
    }

    if (!isFinitef(fitness) || fitness > state.cost)
        return false;

    state.cost = fitness;
    state.radius = params1[0];
    state.offset = Vec3{params1[1], params1[2], params1[3]};
    return true;
}

// One Levenberg-Marquardt iteration of PX4's full ellipsoid fit.
bool lmEllipsoidIteration(const float *samples, uint32_t count, MagFitState &state)
{
    float jtj[9][9] = {};
    float jtfi[9] = {};

    for (uint32_t k = 0; k < count; ++k)
    {
        const float dx = samples[3 * k + 0] - state.offset.x;
        const float dy = samples[3 * k + 1] - state.offset.y;
        const float dz = samples[3 * k + 2] - state.offset.z;

        const float a = state.scale.diag[0] * dx + state.scale.offdiag[0] * dy +
                        state.scale.offdiag[1] * dz;
        const float b = state.scale.offdiag[0] * dx + state.scale.diag[1] * dy +
                        state.scale.offdiag[2] * dz;
        const float c = state.scale.offdiag[1] * dx + state.scale.offdiag[2] * dy +
                        state.scale.diag[2] * dz;
        const float length = sqrtf(a * a + b * b + c * c);
        if (length < 1e-6f)
            continue;

        float jacobian[9];
        jacobian[0] = (state.scale.diag[0] * a + state.scale.offdiag[0] * b +
                       state.scale.offdiag[1] * c) / length;
        jacobian[1] = (state.scale.offdiag[0] * a + state.scale.diag[1] * b +
                       state.scale.offdiag[2] * c) / length;
        jacobian[2] = (state.scale.offdiag[1] * a + state.scale.offdiag[2] * b +
                       state.scale.diag[2] * c) / length;
        jacobian[3] = -(dx * a) / length;
        jacobian[4] = -(dy * b) / length;
        jacobian[5] = -(dz * c) / length;
        jacobian[6] = -((dy * a) + (dx * b)) / length;
        jacobian[7] = -((dz * a) + (dx * c)) / length;
        jacobian[8] = -((dz * b) + (dy * c)) / length;

        const float residual = state.radius - length;

        for (int i = 0; i < 9; ++i)
        {
            for (int j = 0; j < 9; ++j)
                jtj[i][j] += jacobian[i] * jacobian[j];
            jtfi[i] += jacobian[i] * residual;
        }
    }

    for (int i = 0; i < 9; ++i)
        for (int j = i + 1; j < 9; ++j)
        {
            const float mean = 0.5f * (jtj[i][j] + jtj[j][i]);
            jtj[i][j] = jtj[j][i] = mean;
        }

    float damped[9][9];
    float dampedHalf[9][9];
    memcpy(damped, jtj, sizeof(damped));
    memcpy(dampedHalf, jtj, sizeof(dampedHalf));
    for (int i = 0; i < 9; ++i)
    {
        damped[i][i] += state.damping;
        dampedHalf[i][i] += state.damping / kLmDamping;
    }

    float inverse[9][9];
    float inverseHalf[9][9];
    if (!invertDense<9>(damped, inverse) || !invertDense<9>(dampedHalf, inverseHalf))
        return false;

    float params1[9] = {state.offset.x, state.offset.y, state.offset.z,
                        state.scale.diag[0], state.scale.diag[1], state.scale.diag[2],
                        state.scale.offdiag[0], state.scale.offdiag[1], state.scale.offdiag[2]};
    float params2[9];
    memcpy(params2, params1, sizeof(params1));

    for (int row = 0; row < 9; ++row)
        for (int col = 0; col < 9; ++col)
        {
            params1[row] -= jtfi[col] * inverse[row][col];
            params2[row] -= jtfi[col] * inverseHalf[row][col];
        }

    auto costOf = [&](const float params[9]) {
        Symmetric3 scale;
        scale.diag[0] = params[3];
        scale.diag[1] = params[4];
        scale.diag[2] = params[5];
        scale.offdiag[0] = params[6];
        scale.offdiag[1] = params[7];
        scale.offdiag[2] = params[8];
        float sum = 0.0f;
        for (uint32_t k = 0; k < count; ++k)
        {
            const Vec3 delta{samples[3 * k + 0] - params[0],
                             samples[3 * k + 1] - params[1],
                             samples[3 * k + 2] - params[2]};
            const float residual = state.radius - scale.multiply(delta).norm();
            sum += residual * residual;
        }
        return sqrtf(sum) / static_cast<float>(count);
    };

    const float cost1 = costOf(params1);
    const float cost2 = costOf(params2);

    float fitness = state.cost;
    if (cost1 > state.cost && cost2 > state.cost)
    {
        state.damping *= kLmDamping;
    }
    else if (cost2 < state.cost && cost2 < cost1)
    {
        state.damping /= kLmDamping;
        memcpy(params1, params2, sizeof(params1));
        fitness = cost2;
    }
    else if (cost1 < state.cost)
    {
        fitness = cost1;
    }

    if (!isFinitef(fitness) || fitness > state.cost)
        return false;

    state.cost = fitness;
    state.offset = Vec3{params1[0], params1[1], params1[2]};
    state.scale.diag[0] = params1[3];
    state.scale.diag[1] = params1[4];
    state.scale.diag[2] = params1[5];
    state.scale.offdiag[0] = params1[6];
    state.scale.offdiag[1] = params1[7];
    state.scale.offdiag[2] = params1[8];
    return true;
}

// Proper RMS residual in the units of the samples, for reporting.
float magResidualRms(const float *samples, uint32_t count, const MagFitState &state)
{
    if (count == 0)
        return 0.0f;
    float sum = 0.0f;
    for (uint32_t k = 0; k < count; ++k)
    {
        const Vec3 delta{samples[3 * k + 0] - state.offset.x,
                         samples[3 * k + 1] - state.offset.y,
                         samples[3 * k + 2] - state.offset.z};
        const float residual = state.radius - state.scale.multiply(delta).norm();
        sum += residual * residual;
    }
    return sqrtf(sum / static_cast<float>(count));
}

} // namespace

const char *accelSideName(AccelSide side)
{
    switch (side)
    {
    case AccelSide::TailDown:
        return "X+";
    case AccelSide::NoseDown:
        return "X-";
    case AccelSide::Left:
        return "Y+";
    case AccelSide::Right:
        return "Y-";
    case AccelSide::UpsideDown:
        return "Z+";
    case AccelSide::RightSideUp:
        return "Z-";
    }
    return "?";
}

namespace calibration
{

void setIdentity(float matrix[3][3])
{
    memset(matrix, 0, sizeof(float) * 9);
    matrix[0][0] = matrix[1][1] = matrix[2][2] = 1.0f;
}

bool invertMatrix(const float input[3][3], float output[3][3])
{
    Mat3 matrix;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            matrix.m[r][c] = input[r][c];
    Mat3 inverse;
    if (!matrix.invert(inverse))
        return false;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            output[r][c] = inverse.m[r][c];
    return true;
}

// ---------------------------------------------------------------------------
// Accelerometer: PX4 six-position solve
// (src/modules/commander/accelerometer_calibration.cpp)
//
//   offset[i] = (raw[2i][i] + raw[2i+1][i]) / 2
//   A.row(0)  = raw[TailDown]    - offset
//   A.row(1)  = raw[Left]        - offset
//   A.row(2)  = raw[UpsideDown]  - offset
//   accel_T   = A^-1 * 1g
//
// PX4 works in m/s^2 so it multiplies by CONSTANTS_ONE_G. Our raw samples are
// already in g, so the reference magnitude is 1.0 and accel_T comes out
// dimensionless, which is what the correction step expects.
// ---------------------------------------------------------------------------
bool fitAccelerometerSixPosition(const float raw[kAccelCalibrationSides][3],
                                 float bias[3], float matrix[3][3], float &rms)
{
    for (int axis = 0; axis < 3; ++axis)
    {
        const float positive = raw[2 * axis][axis];
        const float negative = raw[2 * axis + 1][axis];
        bias[axis] = 0.5f * (positive + negative);

        // PX4 rejects a face pair that is too close together: the span is what
        // carries the gain information, so a small span means a bad placement.
        if (fabsf(positive - negative) < 0.5f)
            return false;
    }

    Mat3 a;
    a.setRow(0, Vec3{raw[0][0], raw[0][1], raw[0][2]} - Vec3{bias[0], bias[1], bias[2]});
    a.setRow(1, Vec3{raw[2][0], raw[2][1], raw[2][2]} - Vec3{bias[0], bias[1], bias[2]});
    a.setRow(2, Vec3{raw[4][0], raw[4][1], raw[4][2]} - Vec3{bias[0], bias[1], bias[2]});

    Mat3 aInverse;
    if (!a.invert(aInverse))
        return false;

    // PX4 computes accel_T = A^-1 * 1g and then only keeps diag(accel_T), so
    // the transpose below is invisible there. We keep the full matrix, and for
    // the model `corrected = M * (raw - bias)` the correct M is (A^-1)^T:
    //   A.row(i) = raw[side_i] - bias = M^-1 * e_i  =>  A = (M^-1)^T
    //   =>  M = (A^-1)^T
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            matrix[r][c] = aInverse.m[c][r];

    if (!isFinitef(matrix[0][0]) || !isFinitef(matrix[1][1]) || !isFinitef(matrix[2][2]))
        return false;

    // Residual over all six positions against the ideal reference vectors.
    static const float reference[kAccelCalibrationSides][3] = {
        {1.0f, 0.0f, 0.0f},  {-1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f},
        {0.0f, -1.0f, 0.0f}, {0.0f, 0.0f, 1.0f},  {0.0f, 0.0f, -1.0f}};

    float sum = 0.0f;
    for (int side = 0; side < kAccelCalibrationSides; ++side)
    {
        const Vec3 centered{raw[side][0] - bias[0], raw[side][1] - bias[1],
                            raw[side][2] - bias[2]};
        const Vec3 corrected = aInverse * centered;
        const Vec3 expected{reference[side][0], reference[side][1], reference[side][2]};
        const Vec3 error = corrected - expected;
        sum += error.normSquared();
    }
    rms = sqrtf(sum / static_cast<float>(kAccelCalibrationSides));
    return isFinitef(rms);
}

// ---------------------------------------------------------------------------
// Magnetometer: PX4 Levenberg-Marquardt sphere fit
// ---------------------------------------------------------------------------
bool fitMagnetometerSphere(const float *samples, uint32_t count, float offset[3],
                           float matrix[3][3], float &radius, float &rms)
{
    if (samples == nullptr || count < 30)
        return false;

    MagFitState state;
    state.scale = Symmetric3{}; // identity
    state.cost = 1e30f;
    state.damping = 1.0f;

    // Seed the offset with the sample mean; PX4 seeds from the previous
    // calibration, which we do not have on first run.
    Vec3 mean{};
    for (uint32_t k = 0; k < count; ++k)
        mean += Vec3{samples[3 * k + 0], samples[3 * k + 1], samples[3 * k + 2]};
    mean *= 1.0f / static_cast<float>(count);
    state.offset = mean;

    // Seed the radius with the mean distance from the centroid, which is a much
    // better starting point than PX4's fixed 0.2 G (it has a previous
    // calibration to seed from; we do not).
    float meanDistance = 0.0f;
    for (uint32_t k = 0; k < count; ++k)
    {
        const Vec3 delta{samples[3 * k + 0] - mean.x, samples[3 * k + 1] - mean.y,
                         samples[3 * k + 2] - mean.z};
        meanDistance += delta.norm();
    }
    state.radius = meanDistance / static_cast<float>(count);
    if (!(state.radius > kMagFitMinRadius && state.radius < kMagFitMaxRadius))
        state.radius = 0.5f * (kMagFitMinRadius + kMagFitMaxRadius);

    bool converged = false;
    for (int iteration = 0; iteration < kMagFitMaxIterations; ++iteration)
    {
        if (!lmSphereIteration(samples, count, state))
            break;

        if (iteration > kMagFitMinIterations &&
            (state.cost < kMagFitCostThreshold || state.damping < kMagFitStepThreshold))
        {
            converged = true;
            break;
        }
    }

    if (!converged && state.cost >= kMagFitCostThreshold)
        return false;

    if (!(state.radius > kMagFitMinRadius && state.radius < kMagFitMaxRadius))
        return false;

    offset[0] = state.offset.x;
    offset[1] = state.offset.y;
    offset[2] = state.offset.z;
    setIdentity(matrix);
    radius = state.radius;
    rms = magResidualRms(samples, count, state);
    return isFinitef(rms);
}

// ---------------------------------------------------------------------------
// Magnetometer: PX4 Levenberg-Marquardt ellipsoid fit
// Must be seeded from a successful sphere fit, exactly like PX4.
// ---------------------------------------------------------------------------
bool fitMagnetometerEllipsoid(const float *samples, uint32_t count, float offset[3],
                              float matrix[3][3], float &radius, float &rms)
{
    if (samples == nullptr || count < 30)
        return false;

    MagFitState state;
    state.offset = Vec3{offset[0], offset[1], offset[2]};
    state.radius = radius;
    state.cost = 1e30f;
    state.damping = 1.0f;

    bool converged = false;
    for (int iteration = 0; iteration < kMagFitMaxIterations; ++iteration)
    {
        if (!lmEllipsoidIteration(samples, count, state))
            break;

        if (iteration > kMagFitMinIterations &&
            (state.cost < kMagFitCostThreshold || state.damping < kMagFitStepThreshold))
        {
            converged = true;
            break;
        }
    }

    if (!converged && state.cost >= kMagFitCostThreshold)
        return false;

    if (!(state.radius > kMagFitMinRadius && state.radius < kMagFitMaxRadius))
        return false;

    offset[0] = state.offset.x;
    offset[1] = state.offset.y;
    offset[2] = state.offset.z;
    state.scale.toMatrix(matrix);
    radius = state.radius;
    rms = magResidualRms(samples, count, state);
    return isFinitef(rms);
}

// ---------------------------------------------------------------------------
// Application
// ---------------------------------------------------------------------------
void applyAccelCorrection(const float raw[3], const ImuCalibrationData &data,
                          float corrected[3])
{
    const Vec3 centered{raw[0] - data.accelBias[0], raw[1] - data.accelBias[1],
                        raw[2] - data.accelBias[2]};
    Mat3 matrix;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            matrix.m[r][c] = data.accelMatrix[r][c];
    const Vec3 result = matrix * centered;
    result.toArray(corrected);
}

void applyMagCorrection(const float raw[3], const ImuCalibrationData &data,
                        float corrected[3])
{
    const Vec3 centered{raw[0] - data.magOffset[0], raw[1] - data.magOffset[1],
                        raw[2] - data.magOffset[2]};
    Mat3 matrix;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            matrix.m[r][c] = data.magMatrix[r][c];
    const Vec3 result = matrix * centered;
    result.toArray(corrected);
}

} // namespace calibration

// ---------------------------------------------------------------------------
// Persistence
// ---------------------------------------------------------------------------
#if defined(ARDUINO)
namespace
{
constexpr const char *kNamespace = "imu-cal";

void makeKey(uint8_t robotNumber, char *buffer, size_t length)
{
    snprintf(buffer, length, "r%u", static_cast<unsigned>(robotNumber));
}
} // namespace

bool ImuCalibrationStore::load(uint8_t robotNumber, ImuCalibrationData &data)
{
    Preferences preferences;
    char key[8];
    makeKey(robotNumber, key, sizeof(key));
    if (!preferences.begin(kNamespace, true))
        return false;
    const size_t length = preferences.getBytes(key, &data, sizeof(data));
    preferences.end();
    return length == sizeof(data) && data.magic == kCalibrationMagic &&
           data.version == kCalibrationVersion && data.robotNumber == robotNumber;
}

bool ImuCalibrationStore::save(const ImuCalibrationData &data)
{
    Preferences preferences;
    char key[8];
    makeKey(data.robotNumber, key, sizeof(key));
    ImuCalibrationData record = data;
    record.magic = kCalibrationMagic;
    record.version = kCalibrationVersion;
    if (!preferences.begin(kNamespace, false))
        return false;
    const size_t length = preferences.putBytes(key, &record, sizeof(record));
    preferences.end();
    return length == sizeof(record);
}

bool ImuCalibrationStore::clear(uint8_t robotNumber)
{
    Preferences preferences;
    char key[8];
    makeKey(robotNumber, key, sizeof(key));
    if (!preferences.begin(kNamespace, false))
        return false;
    const bool removed = preferences.remove(key);
    preferences.end();
    return removed;
}
#else
bool ImuCalibrationStore::load(uint8_t, ImuCalibrationData &) { return false; }
bool ImuCalibrationStore::save(const ImuCalibrationData &) { return false; }
bool ImuCalibrationStore::clear(uint8_t) { return false; }
#endif

} // namespace imu
