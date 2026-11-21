#!/usr/bin/env bash
# Build and run the host-side IMU tests.
#
# These compile the real firmware math sources with the system C++ compiler, so
# a regression in the estimator, filters or calibration shows up in seconds
# instead of requiring a flash + bench test.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd "$here/../.." && pwd)"
build="$here/build"
mkdir -p "$build"

CXX="${CXX:-c++}"
CXXFLAGS="${CXXFLAGS:--std=c++17 -O2 -Wall -Wextra -Wno-unused-parameter}"

echo "compiling host tests..."
# shellcheck disable=SC2086
"$CXX" $CXXFLAGS \
    -I"$root/lib/IMU" \
    "$here/test_imu_math.cpp" \
    "$root/lib/IMU/ImuFilters.cpp" \
    "$root/lib/IMU/ImuAttitudeEstimator.cpp" \
    "$root/lib/IMU/ImuCalibration.cpp" \
    "$root/lib/IMU/ImuRotation.cpp" \
    -o "$build/test_imu_math"

# shellcheck disable=SC2086
"$CXX" $CXXFLAGS \
    -I"$root/lib/ServoControl" \
    "$here/test_servo_motion.cpp" \
    "$root/lib/ServoControl/ServoMotion.cpp" \
    -o "$build/test_servo_motion"

echo "running..."
"$build/test_imu_math"
"$build/test_servo_motion"
