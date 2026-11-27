#ifndef IMU_TELEMETRY_H
#define IMU_TELEMETRY_H

#include <Arduino.h>
#include "AHRS.h"

// CSV formatter for the IMU stack.
//
// The column order is a contract with tools/imu_visualizer.py and
// tools/imu_visualizer_v2.py. New columns are only ever APPENDED so an older
// visualizer keeps working (it ignores unknown trailing columns) and a newer
// one can still read an older firmware (missing columns show as NaN).
//
// A header line is printed at boot and then periodically, so the visualizers
// can map columns by name instead of by position.
//
// Formatting is separate from transmission: the command server decides the rate
// and writes the same line to USB serial and to any TCP client that asked for
// STREAM.
class ImuTelemetryOutput
{
public:
    // Column names, in order, without the leading "IMU," prefix.
    static const char *headerLine();

    // Writes one "IMU,..." line (no trailing newline) into `out`.
    // Returns the number of characters written, 0 on failure.
    static int formatLine(AHRS &ahrs, char *out, size_t size);
};

#endif
