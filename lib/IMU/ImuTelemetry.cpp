#include "ImuTelemetry.h"
#include "../../include/config.h"

namespace
{
// Keep this in sync with formatLine(). Appended columns go at the end.
constexpr const char *kHeader =
    "IMU,t_ms,dt,ax,ay,az,lax,lay,laz,flax,flay,flaz,gx,gy,gz,gbx,gby,gbz,"
    "fgx,fgy,fgz,magx,magy,magz,roll,pitch,yaw,vx,vy,vz,px,py,pz,temp_c,"
    "moving,still,still_failure_mask,stdA,trust,rollG,pitchG,"
    // --- appended in the PX4-based stack ---
    "ebx,eby,ebz,acc_norm_g,mag_norm,rate_hz,accel_rms_g,mag_rms_ut,mag_fit,"
    "est_init";
} // namespace

const char *ImuTelemetryOutput::headerLine()
{
    return kHeader;
}

int ImuTelemetryOutput::formatLine(AHRS &ahrs, char *out, size_t size)
{
    if (out == nullptr || size == 0)
        return 0;

    const ImuTelemetry &imu = ahrs.getTelemetry();

    // One buffer, one write: keeps the line atomic so a client never sees half
    // a line, and avoids ~50 individual Serial.print() calls per sample.
    const int length = snprintf(
        out, size,
        "IMU,%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
        "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
        "%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%d,%d,%u,"
        "%.5f,%.3f,%.2f,%.2f,"
        "%.3f,%.3f,%.3f,%.4f,%.3f,%.1f,%.5f,%.4f,%u,%u",
        static_cast<unsigned long>(millis()), imu.dt,
        imu.accel[0], imu.accel[1], imu.accel[2],
        imu.linearAccel[0], imu.linearAccel[1], imu.linearAccel[2],
        imu.filteredLinearAccel[0], imu.filteredLinearAccel[1], imu.filteredLinearAccel[2],
        imu.gyro[0], imu.gyro[1], imu.gyro[2],
        imu.gyroBias[0], imu.gyroBias[1], imu.gyroBias[2],
        imu.filteredGyro[0], imu.filteredGyro[1], imu.filteredGyro[2],
        imu.mag[0], imu.mag[1], imu.mag[2],
        imu.roll, imu.pitch, imu.yaw,
        ahrs.getVelocityX(), ahrs.getVelocityY(), ahrs.getVelocityZ(),
        ahrs.getPositionX(), ahrs.getPositionY(), ahrs.getPositionZ(),
        imu.temperature,
        ahrs.isMoving() ? 1 : 0,
        imu.still ? 1 : 0,
        static_cast<unsigned>(imu.stillFailureMask),
        imu.stdA, imu.trust, imu.rollG, imu.pitchG,
        imu.estimatorBias[0], imu.estimatorBias[1], imu.estimatorBias[2],
        imu.accelNormG, imu.magNorm, imu.imuRateHz,
        imu.accelCalRms, imu.magCalRms,
        static_cast<unsigned>(imu.magFitKind),
        static_cast<unsigned>(imu.estimatorInitialized));

    if (length <= 0)
        return 0;
    if (length >= static_cast<int>(size))
        return static_cast<int>(size) - 1;
    return length;
}
