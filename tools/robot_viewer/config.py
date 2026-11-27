"""Runtime configuration for the robot visualizer."""

from __future__ import annotations

from dataclasses import dataclass, field

from .panels import RangePresets


@dataclass
class Config:
    # ---- connection -------------------------------------------------------
    transport: str = "serial"       # "serial" | "tcp"
    port: str = "auto"              # serial device, or "auto"
    baud: int = 115200
    host: str = "192.168.4.1"       # TCP host
    tcp_port: int = 4242
    line_prefix: str = "IMU"
    tx_eol: str = "\n"

    # ---- time axis --------------------------------------------------------
    window_steps: tuple = (5, 10, 20, 30, 60, 120)
    window_index: int = 3

    # ---- drawing ----------------------------------------------------------
    fps: int = 30
    max_points: int = 900
    use_blit: bool = True
    buffer_s: float = 120.0
    max_rate_hz: int = 250
    figsize: tuple = (18.5, 10.0)

    # ---- ranges -----------------------------------------------------------
    presets: RangePresets = field(default_factory=RangePresets)
    preset: str = "overview"

    # ---- top-down ---------------------------------------------------------
    trail_s: float = 120.0
    topdown_min_half_span_m: float = 0.5
    auto_zero_on_connect: bool = True

    # ---- robot ------------------------------------------------------------
    box_size: tuple = (0.9, 1.4, 0.4)     # X, Y, Z (forward = +Y, longest)
    euler_signs: tuple = (1, 1, 1)
    heading_offset_deg: float = 270.0
    park_angles: tuple = (180, 180)

    # ---- console ----------------------------------------------------------
    send_hz: int = 20
    command_timeout_s: float = 2.5

    # ---- streaming --------------------------------------------------------
    # The robot does not stream to a new TCP client at all, and its serial port
    # runs whatever rate a previous session left behind (10 Hz at boot). Without
    # a nudge the viewer just sits on an empty port, so ask once data stops
    # arriving. 0 disables the request entirely.
    auto_stream: bool = True
    auto_stream_hz: int = 50
    # Give the robot a moment to announce itself before nagging it for a stream.
    stream_grace_s: float = 3.0

    # ---- logging ----------------------------------------------------------
    log: bool = False
    log_dir: str = "logs"
