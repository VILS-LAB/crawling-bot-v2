"""Parsing of the robot's ASCII protocol into typed records.

The firmware writes one reply per command (``OK ...`` / ``ERR ...``) and may
interleave unsolicited *events* — live telemetry (``IMU,...``) and calibration
progress (``CAL,...``). Callers that only want a reply must therefore skip
event lines, which is what :func:`parse_reply` helps with.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional


class RobotError(RuntimeError):
    """The robot answered a command with an ``ERR ...`` line."""

    def __init__(self, code: str, detail: str = "") -> None:
        self.code = code
        self.detail = detail
        message = code if not detail else f"{code}: {detail}"
        super().__init__(message)


class RobotTimeout(TimeoutError):
    """The robot did not answer within the caller's deadline."""


# ---------------------------------------------------------------------------
# Reply / event classification
# ---------------------------------------------------------------------------

REPLY_PREFIXES = ("OK", "ERR")
TELEMETRY_PREFIX = "IMU,"
CAL_PREFIX = "CAL,"
CAL_RESULT_PREFIX = "CALRESULT,"


def is_reply(line: str) -> bool:
    return line.startswith(REPLY_PREFIXES)


def is_event(line: str) -> bool:
    return line.startswith((TELEMETRY_PREFIX, CAL_PREFIX, CAL_RESULT_PREFIX))


# ---------------------------------------------------------------------------
# Key/value replies
# ---------------------------------------------------------------------------


def split_kv(payload: str) -> Dict[str, str]:
    """Split ``a=1,b=2`` into a dict.

    Tokens without ``=`` (for example the ``PONG`` in ``OK PONG 123``) are kept
    as positional entries under ``_positional`` so nothing is silently lost.
    """
    values: Dict[str, str] = {}
    positional: List[str] = []
    for token in payload.replace("\t", " ").split(","):
        token = token.strip()
        if not token:
            continue
        for part in token.split():
            if "=" in part:
                key, _, value = part.partition("=")
                values[key.strip()] = value.strip()
            else:
                positional.append(part)
    if positional:
        values["_positional"] = " ".join(positional)
    return values


# ---------------------------------------------------------------------------
# Typed records
# ---------------------------------------------------------------------------


def _f(values: Dict[str, str], key: str, default: float = 0.0) -> float:
    try:
        return float(values[key])
    except (KeyError, ValueError):
        return default


def _i(values: Dict[str, str], key: str, default: int = 0) -> int:
    try:
        return int(float(values[key]))
    except (KeyError, ValueError):
        return default


def _b(values: Dict[str, str], key: str, default: bool = False) -> bool:
    return _i(values, key, 1 if default else 0) != 0


@dataclass(frozen=True)
class Snapshot:
    """One ``GET`` reply: attitude, sensors and servo state in one record."""

    robot_id: int
    uptime_ms: int
    roll: float
    pitch: float
    yaw: float
    accel: tuple[float, float, float]
    gyro: tuple[float, float, float]
    mag: tuple[float, float, float]
    temperature_c: float
    still: bool
    trust: float
    rate_hz: float
    calibrated: bool
    down: int
    down_target: int
    up: int
    up_target: int
    servo_moving: bool
    raw: Dict[str, str] = field(default_factory=dict, repr=False)

    @classmethod
    def parse(cls, line: str) -> "Snapshot":
        payload = line[3:] if line.startswith("OK ") else line
        payload = payload[len("SNAP,") :] if payload.startswith("SNAP,") else payload
        v = split_kv(payload)
        return cls(
            robot_id=_i(v, "id"),
            uptime_ms=_i(v, "t"),
            roll=_f(v, "roll"),
            pitch=_f(v, "pitch"),
            yaw=_f(v, "yaw"),
            accel=(_f(v, "ax"), _f(v, "ay"), _f(v, "az")),
            gyro=(_f(v, "gx"), _f(v, "gy"), _f(v, "gz")),
            mag=(_f(v, "mx"), _f(v, "my"), _f(v, "mz")),
            temperature_c=_f(v, "temp_c"),
            still=_b(v, "still"),
            trust=_f(v, "trust"),
            rate_hz=_f(v, "rate_hz"),
            calibrated=_b(v, "cal"),
            down=_i(v, "d"),
            down_target=_i(v, "dt"),
            up=_i(v, "u"),
            up_target=_i(v, "ut"),
            servo_moving=_b(v, "act_moving"),
            raw=v,
        )


# Column order of the STREAM frames. Kept in sync with lib/IMU/ImuTelemetry.cpp;
# a mismatch is survivable because the firmware re-sends the header line, and
# ImuStream tracks it.
TELEMETRY_COLUMNS = (
    "t_ms", "dt",
    "ax", "ay", "az",
    "lax", "lay", "laz",
    "flax", "flay", "flaz",
    "gx", "gy", "gz",
    "gbx", "gby", "gbz",
    "fgx", "fgy", "fgz",
    "magx", "magy", "magz",
    "roll", "pitch", "yaw",
    "vx", "vy", "vz",
    "px", "py", "pz",
    "temp_c", "moving", "still", "still_failure_mask", "stdA", "trust", "rollG", "pitchG",
    "ebx", "eby", "ebz",
    "acc_norm_g", "mag_norm", "rate_hz",
    "accel_rms_g", "mag_rms_ut", "mag_fit", "est_init",
)


@dataclass(frozen=True)
class ImuFrame:
    """One telemetry frame from ``STREAM``.

    Only the columns a client usually needs are lifted into attributes; the
    complete mapping is available as :attr:`values`.
    """

    timestamp_ms: int
    dt: float
    accel: tuple[float, float, float]        # m/s^2, body frame
    linear_accel: tuple[float, float, float]  # m/s^2, gravity removed
    gyro: tuple[float, float, float]          # deg/s
    mag: tuple[float, float, float]           # uT
    roll: float
    pitch: float
    yaw: float
    velocity: tuple[float, float, float]      # m/s
    position: tuple[float, float, float]      # m
    temperature_c: float
    still: bool
    trust: float
    values: Dict[str, float] = field(default_factory=dict, repr=False)

    @classmethod
    def parse(
        cls, line: str, columns: tuple[str, ...] = TELEMETRY_COLUMNS
    ) -> "ImuFrame":
        body = line[4:] if line.startswith(TELEMETRY_PREFIX) else line
        parts = body.split(",")
        numeric: List[float] = []
        for part in parts:
            try:
                numeric.append(float(part))
            except ValueError:
                numeric.append(float("nan"))

        # If a firmware revision added columns, map positionally over the
        # columns we know; `columns` may have been updated from a header line.
        values = {
            name: numeric[index]
            for index, name in enumerate(columns)
            if index < len(numeric)
        }
        return cls(
            timestamp_ms=int(values.get("t_ms", 0)),
            dt=values.get("dt", 0.0),
            accel=(values.get("ax", 0.0), values.get("ay", 0.0), values.get("az", 0.0)),
            linear_accel=(
                values.get("lax", 0.0),
                values.get("lay", 0.0),
                values.get("laz", 0.0),
            ),
            gyro=(values.get("gx", 0.0), values.get("gy", 0.0), values.get("gz", 0.0)),
            mag=(
                values.get("magx", 0.0),
                values.get("magy", 0.0),
                values.get("magz", 0.0),
            ),
            roll=values.get("roll", 0.0),
            pitch=values.get("pitch", 0.0),
            yaw=values.get("yaw", 0.0),
            velocity=(values.get("vx", 0.0), values.get("vy", 0.0), values.get("vz", 0.0)),
            position=(values.get("px", 0.0), values.get("py", 0.0), values.get("pz", 0.0)),
            temperature_c=values.get("temp_c", 0.0),
            still=values.get("still", 0.0) != 0.0,
            trust=values.get("trust", 0.0),
            values=values,
        )


def parse_header(line: str) -> Optional[tuple[str, ...]]:
    """Return the column names if `line` is a telemetry header, else None."""
    stripped = line.lstrip("#")
    if stripped.startswith("IMU,"):
        stripped = stripped[len("IMU,") :]
    if not stripped.startswith("t_ms,"):
        return None
    return tuple(name.strip() for name in stripped.split(","))


def parse_reply(line: str) -> Dict[str, str]:
    """Parse an ``OK ...`` line into key/values, raising on ``ERR ...``."""
    if line.startswith("ERR "):
        body = line[4:]
        code, _, detail = body.partition(",")
        raise RobotError(code.strip(), detail.strip())
    if not line.startswith("OK"):
        raise RobotError("unexpected_reply", line)
    return split_kv(line[3:] if line.startswith("OK ") else line[2:])
