#include "ImuTelemetry.h"
#include "../../include/config.h"

namespace
{
// Keep this in sync with printCsv(). Appended columns go at the end.
constexpr const char *kHeader =
    "IMU,t_ms,dt,ax,ay,az,lax,lay,laz,flax,flay,flaz,gx,gy,gz,gbx,gby,gbz,"
    "fgx,fgy,fgz,magx,magy,magz,roll,pitch,yaw,vx,vy,vz,px,py,pz,temp_c,"
    "moving,still,still_failure_mask,stdA,trust,rollG,pitchG,"
    // --- appended in the PX4-based stack ---
    "ebx,eby,ebz,acc_norm_g,mag_norm,rate_hz,accel_rms_g,mag_rms_ut,mag_fit,"
    "est_init";

constexpr uint32_t kHeaderRepeatMs = 5000;
} // namespace

const char *ImuTelemetryOutput::headerLine()
{
    return kHeader;
}

void ImuTelemetryOutput::begin()
{
    printHeader();
    lastOutputMs = millis();
    lastHeaderMs = lastOutputMs;
}

void ImuTelemetryOutput::update(AHRS &ahrs)
{
    if (!ahrs.isSerialTelemetryEnabled())
        return;

    const uint32_t now = millis();

    // Re-emit the header periodically so a visualizer attached later can still
    // learn the column layout.
    if (now - lastHeaderMs >= kHeaderRepeatMs)
    {
        lastHeaderMs = now;
        printHeader();
    }

    if (now - lastOutputMs < TELEMETRY_INTERVAL_MS)
        return;

    lastOutputMs = now;
    printCsv(ahrs);
}

void ImuTelemetryOutput::printHeader() const
{
    Serial.println(kHeader);
}

void ImuTelemetryOutput::printCsv(AHRS &ahrs) const
{
    const ImuTelemetry &imu = ahrs.getTelemetry();

    // One buffer, one write: keeps the line atomic so the visualizer never sees
    // a half line, and avoids ~50 individual Serial.print() calls per sample.
    char line[512];
    int length = snprintf(
        line, sizeof(line),
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
        return;
    if (length >= static_cast<int>(sizeof(line)))
        length = static_cast<int>(sizeof(line)) - 1;

    line[length++] = '\n';
    Serial.write(reinterpret_cast<const uint8_t *>(line), length);
}
