# What was ported from PX4, and what was not

A precise map from PX4 source to this project, so the behaviour can be audited
against upstream. Full design rationale: [ARCHITECTURE.md](./ARCHITECTURE.md).
For how this compares to the two earlier in-house implementations, see
[IMPLEMENTATION_HISTORY.md](./IMPLEMENTATION_HISTORY.md).

PX4 revision used: the checkout in `github-reference-repositories/PX4-Autopilot`.

## Ported

| PX4 source | Ported to | Notes |
|---|---|---|
| `src/modules/attitude_estimator_q/attitude_estimator_q_main.cpp` | [`ImuAttitudeEstimator.cpp`](../../lib/IMU/ImuAttitudeEstimator.cpp) | `update()`, `init_attitude_q()`, `update_mag_declination()` |
| `src/lib/mathlib/math/filter/LowPassFilter2p.hpp` | [`ImuFilters.cpp`](../../lib/IMU/ImuFilters.cpp) | Direct Form II, same coefficient math |
| `src/lib/mathlib/math/filter/NotchFilter.hpp` | [`ImuFilters.cpp`](../../lib/IMU/ImuFilters.cpp) | Direct Form I, same coefficient math and reset seeding |
| `src/lib/mathlib/math/filter/MedianFilter.hpp` | [`ImuFilters.cpp`](../../lib/IMU/ImuFilters.cpp) | Window 3 |
| `src/lib/mathlib/math/filter/AlphaFilter.hpp` | [`ImuFilters.cpp`](../../lib/IMU/ImuFilters.cpp) | Time-constant form |
| `src/modules/commander/gyro_calibration.cpp` | `AHRS::calibrateGyroscope()` | Worker + median/mean quality gate + retries |
| `src/modules/commander/accelerometer_calibration.cpp` | `AHRS::calibrateAccelerometer()` + `calibration::fitAccelerometerSixPosition()` | Six positions, same solve |
| `src/modules/commander/mag_calibration.cpp` | `AHRS::calibrateMagnetometer()` | Sample collection + min-distance rule |
| `src/modules/commander/lm_fit.cpp` | `calibration::fitMagnetometerSphere/Ellipsoid()` | LM sphere then ellipsoid |
| `src/lib/sensor_calibration/Accelerometer.hpp` (`Correct`) | `calibration::applyAccelCorrection()` | Same model |
| `src/lib/sensor_calibration/Gyroscope.hpp` (`Correct`) | `AHRS::readAndCalibrate()` | Same model |
| `src/lib/sensor_calibration/Magnetometer.hpp` (`Correct`) | `calibration::applyMagCorrection()` | Same model |
| `src/modules/sensors/vehicle_imu/VehicleIMU.cpp` | `AHRS::taskLoop()` | Sampling/interval handling, not the uORB plumbing |
| `src/modules/sensors/vehicle_acceleration/imu_accel_parameters.yaml` | `FILTER_ACCEL_CUTOFF_HZ = 30` | `IMU_ACCEL_CUTOFF` default |
| `src/modules/sensors/vehicle_angular_velocity/imu_gyro_parameters.yaml` | `FILTER_GYRO_CUTOFF_HZ = 40`, `FILTER_GYRO_NOTCH_BW_HZ = 20` | `IMU_GYRO_CUTOFF`, `IMU_GYRO_NF1_BW` defaults |
| `attitude_estimator_q_params.yaml` | `AttitudeEstimatorConfig` | `ATT_W_ACC`, `ATT_W_MAG`, `ATT_BIAS_MAX`, `ATT_MAG_DECL` |

## Deliberately different

| Item | PX4 | Here | Reason |
|---|---|---|---|
| Body/world frame | FRD / NED | FLU / ENU-like | Project convention; see ARCHITECTURE §2 |
| `ATT_W_GYRO_BIAS` | 0.1 | 0.02 | 0.1 is under-damped for a robot that sits still for minutes |
| `ATT_BIAS_MAX` | 0.05 rad/s | 0.15 rad/s | GY-91 residual bias after a short calibration |
| Accel matrix | diagonal only | full 3×3 | Hand-soldered GY-91 has real cross-axis terms |
| Mag units | Gauss | µT | Project convention; LM thresholds scaled ×100 |
| Mag fit seeding | previous calibration | sample centroid + mean distance | No previous calibration on first run |
| Mag fit radius window | 0.2–0.7 G | 20–70 µT | Same physical window, different units |
| Notch filter | dynamic notch bank (ESC RPM / FFT) | single optional static notch, disabled | No RPM telemetry, no FFT on this MCU budget |
| Thermal calibration | full offset model | boot-time drift *detection* only | Out of scope; detection forces a recalibration |
| Gyro bias tracking | estimator only | estimator **and** a clamped runtime tracker | The runtime tracker handles slow bias drift between calibrations |

## Not ported (no matching input on this robot)

* GPS velocity fusion (`ATT_ACC_COMP`, `update_vehicle_local_position`)
* Vision odometry heading (`ATT_EXT_HDG_M = 1`)
* Motion-capture odometry heading (`ATT_EXT_HDG_M = 2`)
* GPS-derived magnetic declination (`ATT_MAG_DECL_A`)
* uORB topics, `ModuleBase`, `WorkItem`, parameter framework
* `matrix` library (replaced by the small `ImuMath.h`)
* `sensor_correction` thermal offsets
* Factory calibration storage (`factory_calibration_storage.cpp`)

## Verification

`test/host/test_imu_math.cpp` exercises the ported math directly:

* filter DC gain, attenuation, and pass-through-when-disabled,
* estimator leveling, convergence, gyro integration, bias behaviour,
* six-position solve recovering a known bias + matrix,
* LM sphere/ellipsoid fits recovering a known offset + radius,
* end-to-end yaw tracking against synthetic ground truth.

Run it with `test/host/run.sh`.
