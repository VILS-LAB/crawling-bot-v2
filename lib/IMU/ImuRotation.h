#ifndef IMU_ROTATION_H
#define IMU_ROTATION_H

// The project's body -> world rotation contract.
//
// This is intentionally NOT replaced by the PX4 estimator's internal math: the
// estimator produces a quaternion, but the gravity-removal step below is the
// exact sequential rotation that has been verified against the hardware and
// the visualizers. Keeping it means the numbers the tools plot do not change
// meaning when the estimator changes.
//
//   body acceleration -> rotate X (roll) -> rotate Y (pitch) -> rotate Z (yaw)
//
// which is R = Rz(yaw) * Ry(pitch) * Rx(roll), the same convention the
// estimator reports Euler angles in. See docs/imu/ROTATION_MATRIX.md.

#include "ImuMath.h"

// Rotate a body-frame vector into the world frame using the project's
// sequential X -> Y -> Z rotation. Angles are in degrees.
void rotateBodyToWorld(float roll, float pitch, float yaw, const float body[3],
                       float world[3]);

// Same rotation, expressed as a matrix (body -> world).
imu::Mat3 bodyToWorldMatrix(float roll, float pitch, float yaw);

#endif // IMU_ROTATION_H
