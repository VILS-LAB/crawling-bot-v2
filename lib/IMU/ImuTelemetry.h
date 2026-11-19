#ifndef IMU_TELEMETRY_H
#define IMU_TELEMETRY_H

#include <Arduino.h>
#include "AHRS.h"

// CSV telemetry for the IMU stack.
//
// The column order is a contract with tools/imu_visualizer.py and
// tools/imu_visualizer_v2.py. New columns are only ever APPENDED so an older
// visualizer keeps working (it ignores unknown trailing columns) and a newer
// one can still read an older firmware (missing columns show as NaN).
//
// A header line is printed at boot and then periodically, so the visualizers
// can map columns by name instead of by position.
class ImuTelemetryOutput
{
public:
    void begin();
    void update(AHRS &ahrs);

    // Column names, in order, without the leading "IMU," prefix.
    static const char *headerLine();

private:
    void printHeader() const;
    void printCsv(AHRS &ahrs) const;
    uint32_t lastOutputMs = 0;
    uint32_t lastHeaderMs = 0;
};

#endif
