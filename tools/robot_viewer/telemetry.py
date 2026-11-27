"""Data format, line parsing and the history buffer.

The firmware streams one CSV line per sample as ``IMU,<t_ms>,<values...>``. Column
order is fixed by ``lib/IMU/ImuTelemetry.cpp`` and is **append-only**, so this
module carries the canonical list and degrades gracefully when a robot sends
more columns (ignored) or fewer (NaN).

Everything here is GUI-free, so it can be tested headlessly.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence

import numpy as np

# ---------------------------------------------------------------------------
# Column contract (mirror of lib/IMU/ImuTelemetry.cpp)
# ---------------------------------------------------------------------------
DEFAULT_FIELDS: tuple[str, ...] = (
    "t_ms", "dt",
    "ax", "ay", "az",                 # body-frame accel, m/s^2
    "lax", "lay", "laz",              # linear accel (gravity removed)
    "flax", "flay", "flaz",           # linear accel, filtered
    "gx", "gy", "gz",                 # gyro, deg/s
    "gbx", "gby", "gbz",              # calibration gyro bias
    "fgx", "fgy", "fgz",              # gyro, filtered
    "magx", "magy", "magz",           # magnetometer, uT
    "roll", "pitch", "yaw",           # attitude, deg
    "vx", "vy", "vz",                 # velocity, m/s
    "px", "py", "pz",                 # position, m
    "temp_c", "moving", "still",
    "still_failure_mask",
    "stdA", "trust",
    "rollG", "pitchG",                # gyro-only shadow attitude
    # --- appended by the PX4-based stack ---
    "ebx", "eby", "ebz",              # estimator integral gyro bias
    "acc_norm_g",
    "mag_norm",
    "rate_hz",
    "accel_rms_g",
    "mag_rms_ut",
    "mag_fit",
    "est_init",
)

# Bit meanings of still_failure_mask, from lib/IMU/AHRS.cpp.
STILL_FAIL_BITS = {
    0: "accel norm",
    1: "gyro rate",
    2: "accel spread",
}

# Channels the viewer derives itself (not sent by the robot).
DERIVED = (
    "speed",        # |velocity|
    "a_norm",       # |accel|
    "drift_roll",   # roll - rollG (positive = accel pulling up)
    "drift_pitch",
    "still_on",     # 1 - still, so 'still' reads as a positive indicator
    "bias_norm",    # |estimator gyro bias|
    "fail_any",     # 1 when any still-check is failing (detail is in the text)
) + tuple(f"fb{i}" for i in STILL_FAIL_BITS)

CHANNELS: tuple[str, ...] = ("t",) + DEFAULT_FIELDS[1:] + DERIVED
CH = {name: index for index, name in enumerate(CHANNELS)}
N_CH = len(CHANNELS)


def make_colmap(names: Sequence[str]):
    """Map incoming column position -> (channel index, scale) or None."""
    colmap = []
    for name in names:
        name = name.strip().lstrip("#")
        if name == "t_ms":
            colmap.append((CH["t"], 1e-3))          # ms -> s
        elif name in CH and name not in ("t",) + DERIVED:
            colmap.append((CH[name], 1.0))
        else:
            colmap.append(None)
    return colmap


def finish_row(row: np.ndarray) -> np.ndarray:
    """Fill the derived channels for one sample row."""
    velocity = row[[CH["vx"], CH["vy"], CH["vz"]]]
    row[CH["speed"]] = math.sqrt(float(np.nansum(velocity * velocity)))

    accel = row[[CH["ax"], CH["ay"], CH["az"]]]
    row[CH["a_norm"]] = (
        math.sqrt(float(np.sum(accel * accel))) if np.isfinite(accel).all() else float("nan")
    )

    for shadow, attitude, target in (
        ("rollG", "roll", "drift_roll"),
        ("pitchG", "pitch", "drift_pitch"),
    ):
        # Positive = the estimator sits above the gyro-only projection, i.e.
        # the accelerometer is currently pulling the attitude up.
        delta = row[CH[attitude]] - row[CH[shadow]]
        row[CH[target]] = ((delta + 180.0) % 360.0) - 180.0 if np.isfinite(delta) else float("nan")

    mask = row[CH["still_failure_mask"]]
    if np.isfinite(mask):
        for bit in STILL_FAIL_BITS:
            row[CH[f"fb{bit}"]] = float((int(mask) >> bit) & 1)
        row[CH["fail_any"]] = 1.0 if int(mask) else 0.0

    still = row[CH["still"]]
    row[CH["still_on"]] = (1.0 - still) if np.isfinite(still) else float("nan")

    bias = row[[CH["ebx"], CH["eby"], CH["ebz"]]]
    row[CH["bias_norm"]] = (
        math.sqrt(float(np.sum(bias * bias))) if np.isfinite(bias).all() else float("nan")
    )
    return row


class LineParser:
    """Turns one raw line into a data row, a header update, or nothing.

    ``last_reason`` explains the most recent rejection so the UI can show *why*
    data is not appearing instead of silently plotting nothing.
    """

    def __init__(self, prefix: str = "IMU") -> None:
        self.prefix = prefix
        self.columns: tuple[str, ...] = DEFAULT_FIELDS
        self.colmap = make_colmap(self.columns)
        self.last_reason: Optional[str] = None

    def is_data_line(self, line: str) -> bool:
        """True when ``line`` is telemetry rather than a reply or a diagnostic.

        The firmware interleaves ``OK``/``ERR`` replies and ``HEALTH,``/``CAL,``
        diagnostics with the stream, so the caller must decide *before* parsing:
        a reply that reaches ``feed()`` looks like a malformed data row and gets
        discarded, which silently swallows every command acknowledgment.
        """
        text = line.strip()
        if not text:
            return False
        if self.prefix and text.startswith(self.prefix + ","):
            return True
        # A header is telemetry metadata, not data, but it is still ours to read.
        return text.startswith("#") or text.startswith("t_ms")

    def feed(self, line: str, host_t: float) -> Optional[np.ndarray]:
        self.last_reason = None
        line = line.strip()
        if not line:
            return None

        if self.prefix and not line.startswith("#"):
            if not line.startswith(self.prefix + ","):
                self.last_reason = f"line does not start with '{self.prefix},'"
                return None
            line = line[len(self.prefix) + 1 :]

        if line.startswith("#") or line.startswith("t_ms"):
            self._set_header(line.lstrip("#"))
            return None

        parts = line.split(",")
        if len(parts) < len(self.colmap):
            self.last_reason = f"got {len(parts)} values, expected at least {len(self.colmap)}"
            return None

        try:
            values = [float(p) for p in parts[: len(self.colmap)]]
        except ValueError:
            self.last_reason = "a value is not a number"
            return None

        row = np.full(N_CH, np.nan)
        for value, mapping in zip(values, self.colmap):
            if mapping is not None:
                row[mapping[0]] = value * mapping[1]
        if not np.isfinite(row[CH["t"]]):
            row[CH["t"]] = host_t
        return finish_row(row)

    def _set_header(self, header: str) -> None:
        names = tuple(n.strip() for n in header.split(","))
        if "t_ms" not in names:
            return
        self.columns = names
        self.colmap = make_colmap(names)


class Ring:
    """Fixed-capacity history, shaped (channels, capacity).

    Only the GUI thread touches it, so no locking is needed.
    """

    def __init__(self, capacity: int) -> None:
        self.capacity = max(2, int(capacity))
        self.buf = np.full((N_CH, self.capacity), np.nan)
        self.n = 0

    def clear(self) -> None:
        self.buf[:] = np.nan
        self.n = 0

    def push(self, row: np.ndarray) -> None:
        self.buf[:, self.n % self.capacity] = row
        self.n += 1

    def last_time(self) -> Optional[float]:
        if self.n == 0:
            return None
        return float(self.buf[CH["t"], (self.n - 1) % self.capacity])

    def last_row(self) -> Optional[np.ndarray]:
        if self.n == 0:
            return None
        return self.buf[:, (self.n - 1) % self.capacity].copy()

    def _ordered_index(self) -> np.ndarray:
        size = min(self.n, self.capacity)
        return np.arange(self.n - size, self.n) % self.capacity

    def window(self, t_min: float, rows: Optional[Iterable[int]] = None):
        """Samples with ``t >= t_min``, shape (channels, n) or (len(rows), n)."""
        if self.n == 0:
            return None
        index = self._ordered_index()
        start = int(np.searchsorted(self.buf[CH["t"], index], t_min, side="left"))
        start = min(start, len(index) - 1)
        selected = index[start:]
        if len(selected) == 0:
            return None
        if rows is None:
            return self.buf[:, selected]
        return self.buf[np.ix_(list(rows), selected)]

    def decimate(self, max_points: int, channels: Sequence[int]):
        """Min/max envelope decimation that never drops a spike.

        Returns ``(t, values)`` where ``t`` is relative to the newest sample and
        ``values`` has one row per requested channel, each twice the bucket count
        (low then high per bucket, in time order). Returns ``(None, None)`` when
        there is nothing buffered.
        """
        if self.n == 0 or not channels:
            return None, None

        index = self._ordered_index()
        t = self.buf[CH["t"], index] - self.last_time()
        count = len(t)
        per = max(1, count // max(1, max_points // 2))

        if count <= max_points or per < 2:
            return t, self.buf[np.ix_(list(channels), index)]

        buckets = count // per
        used = slice(count - buckets * per, count)
        t_out = np.repeat(t[used].reshape(buckets, per).mean(axis=1), 2)

        out = np.empty((len(channels), 2 * buckets))
        for row_index, channel_index in enumerate(channels):
            block = self.buf[channel_index, index[used]].reshape(buckets, per)
            low = np.fmin.reduce(block, axis=1)
            high = np.fmax.reduce(block, axis=1)
            swapped = np.argmin(block, axis=1) > np.argmax(block, axis=1)
            out[row_index, 0::2] = np.where(swapped, high, low)
            out[row_index, 1::2] = np.where(swapped, low, high)
        return t_out, out


def channel(window: np.ndarray, name: str) -> np.ndarray:
    return window[CH[name]]
