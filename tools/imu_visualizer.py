#!/usr/bin/env python3
"""Plot the CSV telemetry emitted by the crawling bot and show its attitude."""

import argparse
import math
import sys
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


# Column order of the firmware CSV, without the leading "IMU," prefix.
# The firmware also prints this list as a header line, so a mismatch here is
# recoverable at runtime; this copy is only the fallback for old firmware.
FIELDS = [
    "t_ms", "dt", "ax", "ay", "az", "lax", "lay", "laz",
    "flax", "flay", "flaz", "gx", "gy", "gz", "gbx", "gby", "gbz",
    "fgx", "fgy", "fgz",
    "magx", "magy", "magz",
    "roll", "pitch", "yaw", "vx", "vy", "vz",
    "px", "py", "pz", "temp_c", "moving", "still", "still_failure_mask",
    "stdA", "trust", "rollG", "pitchG",
    # --- appended by the PX4-based IMU stack ---
    "ebx", "eby", "ebz",          # estimator integral gyro bias (deg/s)
    "acc_norm_g",                 # |accel| in g
    "mag_norm",                   # |mag| in uT
    "rate_hz",                    # measured IMU sampling rate
    "accel_rms_g",                # accel calibration residual
    "mag_rms_ut",                 # mag calibration residual
    "mag_fit",                    # 0 none / 1 sphere / 2 ellipsoid
    "est_init",                   # estimator initialized flag
]

# Columns this visualizer actually needs. Anything else is optional, so the
# tool keeps working against older firmware that does not send them.
REQUIRED_FIELDS = {
    "t_ms", "lax", "lay", "laz", "flax", "flay", "flaz",
    "gx", "gy", "gz", "fgx", "fgy", "fgz",
    "vx", "vy", "vz", "px", "py", "pz",
    "roll", "pitch", "yaw", "rollG", "pitchG",
    "temp_c", "still", "trust", "stdA",
}


# The firmware world frame is X = north, Y = west, Z = up, and yaw is measured
# counter-clockwise from +X. The firmware body frame is X = forward, Y = left,
# Z = up (FLU). The robot is mounted with body +Y forward and body +X right, so
# its nose direction in world is R @ (0,1,0) = (-sin yaw, cos yaw, 0) in
# (north, west, up), and the compass heading is
#
#     heading = atan2(east, north) = atan2(-cos yaw, -sin yaw) = (270 - yaw) mod 360
HEADING_OFFSET_DEG = 270.0


def compass_heading(yaw_deg):
    """Compass heading (0 = north, clockwise) from the firmware yaw (degrees)."""
    return (HEADING_OFFSET_DEG - yaw_deg) % 360.0


def rotation_matrix(roll, pitch, yaw):
    """Body -> display rotation.

    Body is the firmware's FLU frame (X forward, Y left, Z up). Display is a
    compass frame (east, north, up) so the 3D view matches the top-down map:
    north up, east right. The firmware world is (north, west, up), so
    east = -west.
    """
    rx, ry, rz = map(math.radians, (roll, pitch, yaw))
    cr, sr = math.cos(rx), math.sin(rx)
    cp, sp = math.cos(ry), math.sin(ry)
    cy, sy = math.cos(rz), math.sin(rz)
    body_to_world = (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )
    # world (north, west, up) -> display (east, north, up)
    return (
        tuple(-body_to_world[1][column] for column in range(3)),
        body_to_world[0],
        body_to_world[2],
    )


def rotated_axes(roll, pitch, yaw):
    matrix = rotation_matrix(roll, pitch, yaw)
    origin = (0, 0, 0)
    return [
        (origin, tuple(matrix[row][0] for row in range(3))),
        (origin, tuple(matrix[row][1] for row in range(3))),
        (origin, tuple(matrix[row][2] for row in range(3))),
    ]


def rotate_point(matrix, point):
    return tuple(
        sum(matrix[row][column] * point[column] for column in range(3))
        for row in range(3)
    )


def robot_body(roll, pitch, yaw):
    """Return a body whose forward direction is local +Y."""
    matrix = rotation_matrix(roll, pitch, yaw)
    half_x, half_y, half_z = 0.45, 0.7, 0.2
    corners = [
        (-half_x, -half_y, -half_z), (half_x, -half_y, -half_z),
        (half_x, half_y, -half_z), (-half_x, half_y, -half_z),
        (-half_x, -half_y, half_z), (half_x, -half_y, half_z),
        (half_x, half_y, half_z), (-half_x, half_y, half_z),
    ]
    transformed = [rotate_point(matrix, corner) for corner in corners]
    faces = (
        (0, 1, 2, 3), (4, 5, 6, 7),
        (0, 1, 5, 4), (2, 3, 7, 6),
        (1, 2, 6, 5), (0, 3, 7, 4),
    )
    return [[transformed[index] for index in face] for face in faces]


def rotate_vector(matrix, vector):
    return rotate_point(matrix, vector)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("port", help="Serial port, e.g. /dev/cu.usbmodemXXXX")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--samples", type=int, default=100,
                        help="Number of recent samples to keep (default: 100)")
    args = parser.parse_args()

    try:
        import serial
    except ImportError:
        raise SystemExit("Install dependencies first: python3 -m pip install pyserial matplotlib")

    serial_port = serial.Serial(args.port, args.baud, timeout=0.1)
    samples = deque(maxlen=max(1, args.samples))
    figure = plt.figure(figsize=(14, 10))
    grid = figure.add_gridspec(3, 4)
    accel_axes = [
        figure.add_subplot(grid[0, 0]),
        figure.add_subplot(grid[0, 1]),
        figure.add_subplot(grid[1, 0]),
    ]
    axes = [
        figure.add_subplot(grid[1, 1]),
        figure.add_subplot(grid[2, 0]),
        figure.add_subplot(grid[2, 1]),
    ]
    temperature_text = figure.text(
        0.02, 0.02, "Temperature: -- °C", fontsize=12,
        bbox={"facecolor": "white", "alpha": 0.8, "edgecolor": "0.7"},
    )
    attitude = figure.add_subplot(grid[:2, 2:], projection="3d")
    trajectory = figure.add_subplot(grid[2, 2:])
    field_names = FIELDS

    def parse_telemetry(line):
        nonlocal field_names
        values = line.split(",")
        # Header line: learn the real column layout from the firmware.
        if len(values) > 1 and values[1] == "t_ms":
            field_names = values[1:]
            return None
        if len(values) < 2:
            return None
        payload = values[1:]
        # Accept both an exact match and a longer line (newer firmware with
        # appended columns): map by name, ignore the extras.
        if len(payload) < len(field_names):
            return None
        try:
            sample = dict(zip(field_names, map(float, payload)))
        except ValueError:
            return None
        return sample if REQUIRED_FIELDS <= sample.keys() else None

    def read_serial(_frame):
        while serial_port.in_waiting:
            line = serial_port.readline().decode("utf-8", errors="ignore").strip()
            if not line.startswith("IMU,"):
                continue
            sample = parse_telemetry(line)
            if sample is None:
                continue
            samples.append(sample)

        if not samples or not REQUIRED_FIELDS <= samples[-1].keys():
            return
        data = list(samples)
        time = [(item["t_ms"] - data[0]["t_ms"]) / 1000 for item in data]
        for axis in (*accel_axes, *axes):
            axis.clear()
        for axis, raw_name, filtered_name, label in zip(
            accel_axes,
            ("lax", "lay", "laz"),
            ("flax", "flay", "flaz"),
            ("X", "Y", "Z"),
        ):
            axis.plot(time, [item[raw_name] for item in data],
                      alpha=0.35, label=f"raw {label}")
            axis.plot(time, [item[filtered_name] for item in data],
                      label=f"filtered {label}")
            axis.set_title(f"Linear acceleration {label} (m/s²)")
            axis.set_ylabel("m/s²")
            axis.legend(loc="upper left")
            axis.grid(alpha=0.25)

        axes[0].plot(time, [item["gx"] for item in data], alpha=0.3, label="raw x")
        axes[0].plot(time, [item["gy"] for item in data], alpha=0.3, label="raw y")
        axes[0].plot(time, [item["gz"] for item in data], alpha=0.3, label="raw z")
        axes[0].plot(time, [item["fgx"] for item in data], label="filtered x")
        axes[0].plot(time, [item["fgy"] for item in data], label="filtered y")
        axes[0].plot(time, [item["fgz"] for item in data], label="filtered z")
        axes[0].set_title("Gyroscope: raw + filtered (deg/s)")
        axes[0].legend(loc="upper left")

        axes[1].plot(time, [item["vx"] for item in data], label="vx")
        axes[1].plot(time, [item["vy"] for item in data], label="vy")
        axes[1].plot(time, [item["vz"] for item in data], label="vz")
        axes[1].set_title("Velocity (m/s)")
        axes[1].set_ylabel("m/s")
        axes[1].legend(loc="upper left")

        axes[2].plot(time, [item["roll"] for item in data],
                     label="attitude roll")
        axes[2].plot(time, [item["pitch"] for item in data],
                     label="attitude pitch")
        axes[2].plot(time, [item["rollG"] for item in data],
                     "--", label="gyro-only roll")
        axes[2].plot(time, [item["pitchG"] for item in data],
                     "--", label="gyro-only pitch")
        axes[2].set_title("Attitude vs gyro-only shadow (deg)")
        axes[2].set_ylabel("degrees")
        axes[2].legend(loc="upper left", fontsize="small")
        axes[2].grid(alpha=0.25)
        last = data[-1]
        rate = last.get("rate_hz")
        rate_text = f"   Rate: {rate:.0f} Hz" if rate is not None else ""
        temperature_text.set_text(
            f"Temperature: {last['temp_c']:.1f} °C   "
            f"Trust: {last['trust']:.2f}   "
            f"stdA: {last['stdA']:.5f} g   "
            f"Still: {'yes' if last['still'] else 'no'}"
            f"{rate_text}"
        )

        attitude.clear()
        attitude.set_title("Robot 3D orientation")
        attitude.set_xlim(-1.25, 1.25)
        attitude.set_ylim(-1.25, 1.25)
        attitude.set_zlim(-1.25, 1.25)
        attitude.set_xlabel("X (east)")
        attitude.set_ylabel("Y (north)")
        attitude.set_zlabel("Z (up)")
        attitude.add_collection3d(
            Poly3DCollection(
                robot_body(last["roll"], last["pitch"], last["yaw"]),
                facecolors="tab:orange",
                edgecolors="black",
                alpha=0.65,
            )
        )
        matrix = rotation_matrix(last["roll"], last["pitch"], last["yaw"])
        nose = rotate_vector(matrix, (0.0, 0.95, 0.0))
        attitude.plot(
            (0.0, nose[0]), (0.0, nose[1]), (0.0, nose[2]),
            color="darkorange", linewidth=5, label="forward (+Y)",
        )
        attitude.scatter(nose[0], nose[1], nose[2], color="darkorange", s=45)

        colors = ("r", "g", "b")
        for (start, end), color in zip(
            rotated_axes(last["roll"], last["pitch"], last["yaw"]), colors
        ):
            attitude.plot(
                (start[0], end[0]), (start[1], end[1]), (start[2], end[2]),
                color=color, linewidth=3,
            )
        attitude.text2D(
            0.02, 0.96,
            f"Forward: body +Y   R {last['roll']:.1f}°  "
            f"P {last['pitch']:.1f}°  Y {last['yaw']:.1f}°\n"
            f"heading {compass_heading(last['yaw']):.1f}° (0 = north, clockwise)",
            transform=attitude.transAxes,
        )

        trajectory.clear()
        trajectory.set_title("Top-down displacement (dead reckoning)")
        # The firmware world is X = north, Y = west. Plot a compass-style map
        # instead: north up, east right.
        trajectory.set_xlabel("Δ east (m)")
        trajectory.set_ylabel("Δ north (m)")
        # px = world X = north, py = world Y = west  ->  east = -west
        x_positions = [-item["py"] for item in data]
        y_positions = [item["px"] for item in data]
        x_current, y_current = x_positions[-1], y_positions[-1]
        trajectory.plot(
            x_positions, y_positions, color="tab:purple", linewidth=2,
            label="path",
        )
        trajectory.scatter(
            0, 0, color="tab:green", s=55, marker="o", label="start",
            zorder=3,
        )
        trajectory.scatter(
            x_current, y_current, color="tab:red", s=60, marker="x",
            label="current", zorder=3,
        )
        trajectory.plot(
            (0, x_current), (0, y_current), color="tab:red",
            linestyle=":", alpha=0.7,
        )
        trajectory.axhline(0, color="0.75", linewidth=0.8)
        trajectory.axvline(0, color="0.75", linewidth=0.8)
        distance = math.hypot(x_current, y_current)
        trajectory.text(
            0.02, 0.96,
            f"Δ east {x_current:.3f} m   Δ north {y_current:.3f} m   "
            f"distance {distance:.3f} m",
            transform=trajectory.transAxes,
            verticalalignment="top",
            bbox={"facecolor": "white", "alpha": 0.8, "edgecolor": "0.7"},
        )
        trajectory.legend(loc="lower left", fontsize="small")
        extent = max(
            max(max(x_positions) - min(x_positions),
                max(y_positions) - min(y_positions), 0.1),
            0.2,
        )
        center_x = 0.5 * (max(x_positions) + min(x_positions))
        center_y = 0.5 * (max(y_positions) + min(y_positions))
        half_extent = 0.6 * extent
        trajectory.set_xlim(center_x - half_extent, center_x + half_extent)
        trajectory.set_ylim(center_y - half_extent, center_y + half_extent)
        trajectory.set_aspect("equal", adjustable="box")

        figure.tight_layout()

    animation = FuncAnimation(figure, read_serial, interval=100, cache_frame_data=False)
    try:
        plt.show()
    finally:
        animation.event_source.stop()
        serial_port.close()


if __name__ == "__main__":
    sys.exit(main())
