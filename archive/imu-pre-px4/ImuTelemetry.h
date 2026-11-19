#ifndef IMU_TELEMETRY_H
#define IMU_TELEMETRY_H

#include <Arduino.h>
#include "AHRS.h"

class ImuTelemetryOutput
{
public:
    void begin();
    void update(AHRS &ahrs);

private:
    void printHeader() const;
    void printCsv(AHRS &ahrs) const;
    uint32_t lastOutputMs = 0;
};

#endif
