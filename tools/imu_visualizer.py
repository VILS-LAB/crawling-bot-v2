#!/usr/bin/env python3
"""Plot the CSV telemetry emitted by the crawling bot and show its attitude."""

import argparse
import math
import sys
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


FIELDS = [
    "t_ms", "dt", "ax", "ay", "az", "lax", "lay", "laz",
    "flax", "flay", "flaz", "gx", "gy", "gz", "fgx", "fgy", "fgz",
    "magx", "magy", "magz",
    "roll", "pitch", "yaw", "vx", "vy", "vz",
    "px", "py", "pz", "temp_c", "moving", "still", "still_failure_mask",
]


def rotation_matrix(roll, pitch, yaw):
    rx, ry, rz = map(math.radians, (roll, pitch, yaw))
    cr, sr = math.cos(rx), math.sin(rx)
    cp, sp = math.cos(ry), math.sin(ry)
    cy, sy = math.cos(rz), math.sin(rz)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
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
    attitude = figure.add_subplot(grid[:2, 2:], projection="3d")
    trajectory = figure.add_subplot(grid[2, 2:], projection="3d")

    def read_serial(_frame):
        while serial_port.in_waiting:
            line = serial_port.readline().decode("utf-8", errors="ignore").strip()
            if not line.startswith("IMU,"):
                continue
            values = line.split(",")[1:]
            if len(values) != len(FIELDS):
                continue
            try:
                samples.append(dict(zip(FIELDS, map(float, values))))
            except ValueError:
                continue

        if not samples:
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

        axes[2].plot(time, [item["temp_c"] for item in data], color="tab:red")
        axes[2].set_title("Temperature (°C)")
        axes[2].set_ylabel("°C")

        attitude.clear()
        attitude.set_title("Robot 3D orientation")
        attitude.set_xlim(-1.25, 1.25)
        attitude.set_ylim(-1.25, 1.25)
        attitude.set_zlim(-1.25, 1.25)
        attitude.set_xlabel("X")
        attitude.set_ylabel("Y")
        attitude.set_zlabel("Z")
        last = data[-1]
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
            f"Forward: +Y   R {last['roll']:.1f}°  "
            f"P {last['pitch']:.1f}°  Y {last['yaw']:.1f}°",
            transform=attitude.transAxes,
        )

        trajectory.clear()
        trajectory.set_title("Estimated 3D position (dead reckoning)")
        trajectory.set_xlabel("X (m)")
        trajectory.set_ylabel("Y (m)")
        trajectory.set_zlabel("Z (m)")
        positions = (
            [item["px"] for item in data],
            [item["py"] for item in data],
            [item["pz"] for item in data],
        )
        trajectory.plot(*positions, color="tab:purple", linewidth=2)
        trajectory.scatter(
            positions[0][-1], positions[1][-1], positions[2][-1],
            color="black", s=25, label="current",
        )
        trajectory.legend(loc="upper left")
        extent = max(
            max(max(values) - min(values), 0.1) for values in positions
        )
        center = [0.5 * (max(values) + min(values)) for values in positions]
        half_extent = 0.55 * extent
        trajectory.set_xlim(center[0] - half_extent, center[0] + half_extent)
        trajectory.set_ylim(center[1] - half_extent, center[1] + half_extent)
        trajectory.set_zlim(center[2] - half_extent, center[2] + half_extent)

        figure.tight_layout()

    animation = FuncAnimation(figure, read_serial, interval=100, cache_frame_data=False)
    try:
        plt.show()
    finally:
        animation.event_source.stop()
        serial_port.close()


if __name__ == "__main__":
    sys.exit(main())
