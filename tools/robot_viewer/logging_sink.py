"""CSV recording, in the firmware's own column format.

A log written here can be replayed by this viewer *and* fed to the old
``imu_visualizer*.py`` tools, because the header lists the same column names the
firmware streams. Keeping our own copy (instead of the derived channels) is
deliberate: a replay should reproduce what the robot sent, not what one version
of the viewer computed from it.
"""

from __future__ import annotations

import time
from datetime import datetime
from pathlib import Path

import numpy as np

from .telemetry import CH, DEFAULT_FIELDS


class Logger:
    def __init__(self, folder: str, prefix: str = "") -> None:
        Path(folder).mkdir(parents=True, exist_ok=True)
        self.path = Path(folder) / f"robot_{datetime.now():%Y-%m-%d_%H-%M-%S}.csv"
        self._file = open(self.path, "w", encoding="utf-8")
        self._prefix = f"{prefix}," if prefix else ""
        self._file.write("#" + ",".join(DEFAULT_FIELDS) + "\n")
        self._names = DEFAULT_FIELDS[1:]
        self._indices = [CH[name] for name in self._names]
        self._last_flush = time.monotonic()
        self.rows = 0

    def write_row(self, row: np.ndarray) -> None:
        values = ",".join(
            "" if not np.isfinite(v) else f"{v:.4f}" for v in row[self._indices]
        )
        self._file.write(f"{self._prefix}{row[CH['t']] * 1000:.1f},{values}\n")
        self.rows += 1
        now = time.monotonic()
        if now - self._last_flush > 1.0:
            self._file.flush()
            self._last_flush = now

    def close(self) -> None:
        self._file.close()
