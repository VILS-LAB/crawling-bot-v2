"""Drawing primitives: the fast redraw helper and the robot scene geometry."""

from __future__ import annotations

import math
from typing import Optional

import numpy as np

from .telemetry import CH


class Blitter:
    """Redraw only the animated artists on top of a cached background.

    With ``enabled=False`` every update is a full redraw: slower, but immune to
    platform-specific blitting artifacts.
    """

    def __init__(self, canvas, enabled: bool = True) -> None:
        self.canvas = canvas
        self.enabled = enabled
        self.background = None
        self.artists = []
        canvas.mpl_connect("draw_event", self._on_draw)

    def add(self, *artists):
        for artist in artists:
            if self.enabled:
                artist.set_animated(True)
            self.artists.append(artist)

    def _draw(self):
        figure = self.canvas.figure
        for artist in self.artists:
            if not artist.get_visible():
                continue
            project = getattr(artist, "do_3d_projection", None)
            if project is not None:
                try:
                    project()
                except TypeError:      # older matplotlib wants a renderer
                    project(self.canvas.get_renderer())
            figure.draw_artist(artist)

    def _on_draw(self, _event):
        if self.enabled:
            self.background = self.canvas.copy_from_bbox(self.canvas.figure.bbox)
            self._draw()

    def update(self):
        if not self.enabled or self.background is None:
            self.canvas.draw_idle()
            return
        self.canvas.restore_region(self.background)
        self._draw()
        self.canvas.blit(self.canvas.figure.bbox)


# --- heading dial icon (body frame: x = right, y = front; dial radius = 1) ----
ICON_BODY = np.array([(-0.20, -0.34), (0.20, -0.34), (0.20, 0.34), (-0.20, 0.34)])
ICON_NOSE = np.array([(-0.14, 0.34), (0.14, 0.34), (0.0, 0.56)])
ICON_RIGHT = np.array([(0.20, -0.34), (0.20, 0.34)])
ICON_NEEDLE = np.array([(0.0, 0.56), (0.0, 0.86)])
ICON_HEAD = np.array([(-0.07, 0.84), (0.07, 0.84), (0.0, 1.0)])

# The firmware world frame is (north, west, up); every plot here uses a compass
# frame (east, north, up) so north is up and east is right.
WORLD_TO_DISPLAY = np.array(
    [
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ]
)


def rot2(points: np.ndarray, psi: float) -> np.ndarray:
    """Rotate body-frame points into dial coordinates by ``psi`` radians."""
    cos, sin = math.cos(psi), math.sin(psi)
    return np.column_stack(
        [
            points[:, 0] * cos - points[:, 1] * sin,
            points[:, 0] * sin + points[:, 1] * cos,
        ]
    )


def rot_matrix(roll: float, pitch: float, yaw: float, signs=(1, 1, 1)) -> np.ndarray:
    """Body (FLU) -> display (east, north, up) rotation."""
    r, p, y = np.radians([roll * signs[0], pitch * signs[1], yaw * signs[2]])
    cr, sr, cp, sp, cy, sy = (
        math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    )
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return WORLD_TO_DISPLAY @ (rz @ ry @ rx)


def compass_heading(yaw_deg: float, offset_deg: float = 270.0) -> float:
    """Compass heading (0 = north, clockwise) from the firmware yaw.

    The robot's nose is body +Y while the firmware reports yaw from body +X, so
    heading = (offset - yaw) mod 360. Verified against the firmware convention in
    docs/imu/ROTATION_MATRIX.md.
    """
    return (offset_deg - yaw_deg) % 360.0


def box_faces(rotation: np.ndarray, size) -> list:
    """Six faces of a box rotated by ``rotation``.

    Face order: x-, x+, y-, y+ (front), z-, z+ (top).
    """
    sx, sy, sz = np.array(size) / 2.0
    corners = np.array(
        [[x, y, z] for x in (-sx, sx) for y in (-sy, sy) for z in (-sz, sz)]
    ) @ rotation.T
    faces = [(0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4), (2, 3, 7, 6), (0, 2, 6, 4), (1, 3, 7, 5)]
    return [corners[list(face)] for face in faces]


def nice_ceil(value: float) -> Optional[float]:
    """Round up to a human-friendly number (1, 1.2, 1.5, 2, 2.5, 3, 4, 5, 6, 8, 10)."""
    if not np.isfinite(value) or value <= 0:
        return None
    exponent = 10 ** math.floor(math.log10(value))
    for mantissa in (1, 1.2, 1.5, 2, 2.5, 3, 4, 5, 6, 8, 10):
        if mantissa * exponent >= value - 1e-12:
            return round(mantissa * exponent, 6)
    return None


def running_mean(values: np.ndarray, window: int) -> np.ndarray:
    """Centred moving average that tolerates NaN (keeps the NaN gaps)."""
    if window <= 1 or len(values) < window:
        return values
    valid = np.isfinite(values)
    filled = np.where(valid, values, 0.0)
    kernel = np.ones(window)
    sums = np.convolve(filled, kernel, mode="same")
    counts = np.convolve(valid.astype(float), kernel, mode="same")
    with np.errstate(invalid="ignore", divide="ignore"):
        out = sums / counts
    out[counts < max(1, window // 3)] = np.nan
    return out


def yaw_unwrap_deg(yaw: np.ndarray) -> np.ndarray:
    """Unwrap a yaw series so a wrap from 179 to -179 does not draw a vertical line."""
    if len(yaw) == 0:
        return yaw
    return np.degrees(np.unwrap(np.radians(yaw)))
