#include "ImuTelemetry.h"
#include "../../include/config.h"

void ImuTelemetryOutput::begin()
{
    printHeader();
    lastOutputMs = millis();
}

void ImuTelemetryOutput::update(AHRS &ahrs)
{
    if (!ahrs.isSerialTelemetryEnabled())
        return;

    const uint32_t now = millis();
    if (now - lastOutputMs < TELEMETRY_INTERVAL_MS)
        return;

    lastOutputMs = now;
    printCsv(ahrs);
}

void ImuTelemetryOutput::printHeader() const
{
    Serial.println("IMU,t_ms,dt,ax,ay,az,lax,lay,laz,flax,flay,flaz,gx,gy,gz,gbx,gby,gbz,fgx,fgy,fgz,magx,magy,magz,roll,pitch,yaw,vx,vy,vz,px,py,pz,temp_c,moving,still,still_failure_mask,stdA,trust,rollG,pitchG");
}

void ImuTelemetryOutput::printCsv(AHRS &ahrs) const
{
    const ImuTelemetry &imu = ahrs.getTelemetry();
    Serial.print("IMU,");
    Serial.print(millis());
    Serial.print(',');
    Serial.print(imu.dt, 4);
    for (uint8_t i = 0; i < 3; ++i)
    {
        Serial.print(',');
        Serial.print(imu.accel[i], 4);
    }
    for (uint8_t i = 0; i < 3; ++i)
    {
        Serial.print(',');
        Serial.print(imu.linearAccel[i], 4);
    }
    for (uint8_t i = 0; i < 3; ++i)
    {
        Serial.print(',');
        Serial.print(imu.filteredLinearAccel[i], 4);
    }
    for (uint8_t i = 0; i < 3; ++i)
    {
        Serial.print(',');
        Serial.print(imu.gyro[i], 3);
    }
    for (uint8_t i = 0; i < 3; ++i)
    {
        Serial.print(',');
        Serial.print(imu.gyroBias[i], 3);
    }
    for (uint8_t i = 0; i < 3; ++i)
    {
        Serial.print(',');
        Serial.print(imu.filteredGyro[i], 3);
    }
    for (uint8_t i = 0; i < 3; ++i)
    {
        Serial.print(',');
        Serial.print(imu.filteredMag[i], 3);
    }
    Serial.print(',');
    Serial.print(imu.roll, 2);
    Serial.print(',');
    Serial.print(imu.pitch, 2);
    Serial.print(',');
    Serial.print(imu.yaw, 2);
    Serial.print(',');
    Serial.print(ahrs.getVelocityX(), 4);
    Serial.print(',');
    Serial.print(ahrs.getVelocityY(), 4);
    Serial.print(',');
    Serial.print(ahrs.getVelocityZ(), 4);
    Serial.print(',');
    Serial.print(ahrs.getPositionX(), 4);
    Serial.print(',');
    Serial.print(ahrs.getPositionY(), 4);
    Serial.print(',');
    Serial.print(ahrs.getPositionZ(), 4);
    Serial.print(',');
    Serial.print(imu.temperature, 2);
    Serial.print(',');
    Serial.print(ahrs.isMoving() ? 1 : 0);
    Serial.print(',');
    Serial.print(imu.still ? 1 : 0);
    Serial.print(',');
    Serial.print(imu.stillFailureMask);
    Serial.print(',');
    Serial.print(imu.stdA, 5);
    Serial.print(',');
    Serial.print(imu.trust, 3);
    Serial.print(',');
    Serial.print(imu.rollG, 2);
    Serial.print(',');
    Serial.println(imu.pitchG, 2);
}
