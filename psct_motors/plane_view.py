"""
A picture of the focal plane, so you can see what the three motors are doing.

The gauge on the main window answers "how far from zero". This answers the
other question: "what shape is the plate in right now" -- which actuator is
extended, which way it is tipped, and whether a move did what you expected.

How it is drawn
---------------
An isometric view of the three ball-joint positions, connected into the
triangle they define. The dashed triangle is the zero plane; the solid one is
where the focal plane is now. Posts join the two, one per actuator, so the
extension of each is visible directly.

Vertical exaggeration
---------------------
Enormously exaggerated, and labelled as such. The plate is roughly a metre
across and moves tens of millimetres, so an honest 1:1 drawing would be three
dots on a line. The exaggeration is fixed from the configured focus limits
rather than from the live values, so the picture does not silently rescale
underneath you while you watch it -- a moving plate and a moving scale look
identical otherwise.
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import tkinter as tk

COLOR_NOMINAL = "#9aa5b1"
COLOR_PLANE = "#1b5fa8"
COLOR_PLANE_FILL = "#d6e6f7"
COLOR_POST = "#c77700"
COLOR_LABEL = "#222222"
COLOR_MUTED = "#777777"
COLOR_AXIS = "#b3231f"
COLOR_BAD = "#b3231f"

_COS30 = math.cos(math.radians(30.0))
_SIN30 = math.sin(math.radians(30.0))


class FocalPlaneView(tk.Canvas):
    """Isometric view of the focal plane on its three actuators."""

    def __init__(self, parent, points_xy_mm: Sequence[Tuple[float, float]],
                 names: Sequence[str], focus_span_mm: float = 24.0,
                 width: int = 460, height: int = 380, **kw):
        super().__init__(parent, width=width, height=height,
                         highlightthickness=0, bg="white", **kw)
        self.points = [(float(x), float(y)) for x, y in points_xy_mm]
        self.names = list(names)
        self.focus_span_mm = max(1e-6, float(focus_span_mm))
        self._z: Optional[List[float]] = None
        self._focus: Optional[float] = None
        self._tip = 0.0
        self._tilt = 0.0
        self._message = "not connected"
        self.bind("<Configure>", lambda _e: self._redraw())
        self._redraw()

    # ------------------------------------------------------------------ api

    def update_plane(self, actuator_mm: Optional[Sequence[float]],
                     focus_mm: Optional[float] = None,
                     tip_deg: float = 0.0, tilt_deg: float = 0.0,
                     message: str = "") -> None:
        self._z = list(actuator_mm) if actuator_mm is not None else None
        self._focus = focus_mm
        self._tip = tip_deg
        self._tilt = tilt_deg
        self._message = message
        self._redraw()

    # -------------------------------------------------------------- drawing

    def _project(self, x: float, y: float, z: float,
                 cx: float, cy: float, plan_scale: float,
                 z_scale: float) -> Tuple[float, float]:
        """(x, y, z) in millimetres to a point on the canvas."""
        sx = cx + (x - y) * _COS30 * plan_scale
        sy = cy + (x + y) * _SIN30 * plan_scale - z * z_scale
        return sx, sy

    def _redraw(self) -> None:
        self.delete("all")
        width = max(int(self.winfo_width()), 300)
        height = max(int(self.winfo_height()), 240)

        radius = max(1.0, max(math.hypot(x, y) for x, y in self.points))
        plan_scale = (min(width, height) * 0.30) / radius
        # The full focus range occupies a fixed fraction of the height, so the
        # exaggeration is a constant of the view rather than of the moment.
        z_scale = (height * 0.22) / self.focus_span_mm
        exaggeration = z_scale / plan_scale

        cx, cy = width * 0.5, height * 0.54

        # --- the zero plane -------------------------------------------------
        nominal = [self._project(x, y, 0.0, cx, cy, plan_scale, z_scale)
                   for x, y in self.points]
        self.create_polygon([c for point in nominal for c in point],
                            fill="", outline=COLOR_NOMINAL, width=1, dash=(4, 3))
        for (sx, sy), name in zip(nominal, self.names):
            self.create_oval(sx - 3, sy - 3, sx + 3, sy + 3,
                             fill=COLOR_NOMINAL, outline="")

        # --- the optical axis ------------------------------------------------
        axis_top = self._project(0, 0, self.focus_span_mm, cx, cy, plan_scale, z_scale)
        axis_bottom = self._project(0, 0, -self.focus_span_mm, cx, cy,
                                    plan_scale, z_scale)
        self.create_line(axis_bottom[0], axis_bottom[1], axis_top[0], axis_top[1],
                         fill=COLOR_AXIS, width=1, dash=(2, 3))
        self.create_text(axis_top[0], axis_top[1] - 10, text="towards M1",
                         fill=COLOR_AXIS, font=("TkDefaultFont", 8))
        self.create_text(axis_bottom[0], axis_bottom[1] + 10, text="towards M2",
                         fill=COLOR_AXIS, font=("TkDefaultFont", 8))

        if self._z is None or len(self._z) != len(self.points):
            self.create_text(width / 2, height - 16,
                             text=self._message or "no reading",
                             fill=COLOR_BAD, font=("TkDefaultFont", 10, "bold"))
            return

        # --- the plate itself -------------------------------------------------
        current = [self._project(x, y, z, cx, cy, plan_scale, z_scale)
                   for (x, y), z in zip(self.points, self._z)]
        self.create_polygon([c for point in current for c in point],
                            fill=COLOR_PLANE_FILL, outline=COLOR_PLANE, width=2)

        # --- posts, one per actuator ------------------------------------------
        for (nx, ny), (sx, sy), name, z in zip(nominal, current, self.names, self._z):
            self.create_line(nx, ny, sx, sy, fill=COLOR_POST, width=2)
            self.create_oval(sx - 4, sy - 4, sx + 4, sy + 4,
                             fill=COLOR_PLANE, outline="white", width=1)
            self.create_text(sx, sy - 14, text=f"{name}  {z:+.3f}",
                             fill=COLOR_LABEL, font=("TkDefaultFont", 8, "bold"))

        # --- centre of the plate ----------------------------------------------
        if self._focus is not None:
            fx, fy = self._project(0, 0, self._focus, cx, cy, plan_scale, z_scale)
            self.create_oval(fx - 3, fy - 3, fx + 3, fy + 3,
                             fill=COLOR_AXIS, outline="")

        # --- captions ----------------------------------------------------------
        self.create_text(8, 12, anchor="w",
                         text=f"vertical exaggerated x{exaggeration:,.0f}",
                         fill=COLOR_MUTED, font=("TkDefaultFont", 8))
        if self._focus is not None:
            self.create_text(width / 2, height - 26, anchor="c",
                             text=f"focus {self._focus:+.4f} mm",
                             fill=COLOR_PLANE, font=("TkDefaultFont", 11, "bold"))
            self.create_text(width / 2, height - 10, anchor="c",
                             text=f"tip {self._tip:+.5f}°   tilt {self._tilt:+.5f}°",
                             fill=COLOR_MUTED, font=("TkDefaultFont", 9))


__all__ = ["FocalPlaneView"]
