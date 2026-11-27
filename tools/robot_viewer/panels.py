"""What is plotted, and where.

The previous IMU visualizer spent six charts on data that fits in three, which
left no room for the things you actually watch while testing a robot. Each
consolidation below is a deliberate trade:

| Before (imu_visualizer_v2)                    | Now                                        | Why |
|-----------------------------------------------|--------------------------------------------|-----|
| 3 separate charts, linear acceleration X/Y/Z   | 1 chart, 3 axes + filtered overlay         | same information, one third of the space; the axes share a unit and scale |
| gyro chart with 6 lines (raw + filtered)       | same chart, filtered solid + faint raw     | unchanged, but the raw lines no longer dominate |
| attitude + gyro-only shadow                    | attitude with shadow and a *drift* line    | the drift (attitude - shadow) is the signal you look for, not the pair |
| trust/still/moving AND a 6-row failure bar      | one state chart (still, moving, trust, fails)| reading "why not still" beside the verdict is the whole point |
| estimator bias AND calibration/sampling health  | two health charts, split by normalisation   | bias (deg/s) and calibration residuals (g/uT/Hz) cannot share an axis |

Layout (left column of the window, 3 rows x 2 columns):

    row 0:  Linear acceleration (3 axes)   |  Gyroscope (3 axes)
    row 1:  Attitude + gyro-only drift     |  State: still / moving / trust / failing checks
    row 2:  Velocity + speed               |  Estimator bias + calibration / sampling health

The right column holds the instrument cluster: robot 3D, heading dial, health
panel and the command console.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

# Axis colours, shared with the 3D box so a red line always means X.
X_C, Y_C, Z_C = "#d62728", "#2ca02c", "#1f77b4"
FAINT = dict(alpha=0.28, lw=0.8)


@dataclass
class Line:
    """One plotted series."""

    field: str                  # channel name from telemetry.CHANNELS
    label: str                  # legend text; "_" hides it
    color: str
    alpha: float = 1.0
    lw: float = 1.3
    ls: str = "-"
    step: bool = False          # draw as a step (0/1 signals)
    scale: float = 1.0
    offset: float = 0.0
    axis: str = "left"          # "right" plots against a second y-axis


@dataclass
class Panel:
    """One chart."""

    title: str
    ylabel: str
    pos: tuple                  # (row, col) in the left grid
    lines: list
    range_key: Optional[str] = None   # key into Config.range_presets
    ylim: Optional[tuple] = None      # fixed limits, ignores presets
    right_label: Optional[str] = None # second y-axis label, if any
    right_ylim: Optional[tuple] = None
    colspan: int = 1
    legend_cols: int = 4
    yticks: Optional[tuple] = None    # ((position, "label"), ...)


def _accel():
    return Panel(
        "Linear acceleration (gravity removed)",
        "m/s²",
        (0, 0),
        [
            Line("flax", "x (filtered)", X_C),
            Line("flay", "y (filtered)", Y_C),
            Line("flaz", "z (filtered)", Z_C),
            Line("lax", "_x raw", X_C, **FAINT),
            Line("lay", "_y raw", Y_C, **FAINT),
            Line("laz", "_z raw", Z_C, **FAINT),
        ],
        range_key="acc",
        legend_cols=3,
    )


def _gyro():
    return Panel(
        "Gyroscope",
        "deg/s",
        (0, 1),
        [
            Line("fgx", "x", X_C),
            Line("fgy", "y", Y_C),
            Line("fgz", "z", Z_C),
            Line("gx", "_x raw", X_C, **FAINT),
            Line("gy", "_y raw", Y_C, **FAINT),
            Line("gz", "_z raw", Z_C, **FAINT),
        ],
        range_key="gyro",
        legend_cols=3,
    )


def _attitude():
    # Yaw is deliberately absent: it spans +-180 while roll/pitch live in +-45,
    # so on a shared axis it would flatten them. Yaw and heading have their own
    # instrument (the compass dial) with an exact read-out.
    return Panel(
        "Attitude   (faint dashes = gyro-only shadow)",
        "deg",
        (1, 0),
        [
            Line("roll", "roll", X_C),
            Line("pitch", "pitch", Y_C),
            Line("rollG", "_roll shadow", X_C, ls="--", alpha=0.55, lw=1.0),
            Line("pitchG", "_pitch shadow", Y_C, ls="--", alpha=0.55, lw=1.0),
            Line("drift_roll", "roll drift", X_C, lw=1.0, ls=":", alpha=0.9),
            Line("drift_pitch", "pitch drift", Y_C, lw=1.0, ls=":", alpha=0.9),
        ],
        range_key="att",
        # matplotlib fills legends column-first, so two columns put each axis
        # next to its own drift line: "roll | roll drift", "pitch | pitch drift".
        legend_cols=2,
    )


def _state():
    return Panel(
        "State: still, moving, trust, and whether a check fails",
        "0 / 1",
        (1, 1),
        [
            Line("still_on", "STILL", "#2ca02c", lw=2.2, step=True),
            Line("moving", "moving", "#e08a00", lw=1.6, step=True),
            Line("trust", "trust", "#7b3fa0", lw=1.4),
            # Sits just above the 'yes' band; which check fails is named in the
            # live-state text so this stays one readable line.
            Line("fail_any", "a check failing", "#c0392b", lw=2.0, step=True,
                 scale=0.16, offset=1.12),
        ],
        ylim=(-0.05, 1.42),
        yticks=((0.0, "no"), (1.0, "yes")),
        legend_cols=4,
    )


def _velocity():
    return Panel(
        "Velocity   (dead reckoning)",
        "m/s",
        (2, 0),
        [
            Line("vx", "x", X_C),
            Line("vy", "y", Y_C),
            Line("vz", "z", Z_C),
            Line("speed", "|v|", "k", lw=1.0, ls="--"),
        ],
        range_key="vel",
    )


def _health():
    return Panel(
        "Estimator bias and calibration health",
        "deg/s",
        (2, 1),
        [
            Line("ebx", "bias x", X_C, alpha=0.8),
            Line("eby", "bias y", Y_C, alpha=0.8),
            Line("ebz", "bias z", Z_C, alpha=0.8),
            Line("bias_norm", "|bias|", "k", lw=1.0, ls="--"),
            Line("accel_rms_g", "accel rms (g)", "#e08a00", axis="right"),
            Line("mag_rms_ut", "mag rms (µT)", "#8c5bbf", axis="right"),
        ],
        range_key="bias",
        right_label="residual  (g, µT)",
        right_ylim=(0.0, 40.0),
    )


PANELS: list[Panel] = [
    _accel(),
    _gyro(),
    _attitude(),
    _state(),
    _velocity(),
    _health(),
]

# Still-check names for the status text, in bit order.
STILL_CHECK_NAMES = {0: "accel norm", 1: "gyro rate", 2: "accel spread"}


@dataclass
class RangePresets:
    """Y-range presets, cycled with 'm'.

    Every key must exist in every preset, or switching presets leaves a chart
    with a stale range.
    """

    overview: dict = field(
        default_factory=lambda: {
            "acc": 6.0,
            "gyro": 120.0,
            "vel": 1.0,
            "att": 45.0,
            "bias": 0.6,
        }
    )
    detail: dict = field(
        default_factory=lambda: {
            "acc": 1.0,
            "gyro": 30.0,
            "vel": 0.2,
            "att": 10.0,
            "bias": 0.2,
        }
    )
    auto: Optional[dict] = None

    def keys(self):
        return ("overview", "detail", "auto")

    def get(self, name: str) -> Optional[dict]:
        return getattr(self, name)

    def next_name(self, name: str) -> str:
        names = self.keys()
        return names[(names.index(name) + 1) % len(names)]
