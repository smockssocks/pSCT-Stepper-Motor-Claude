"""
A vertical gauge showing how far the focal plane is from its zero.

Why a gauge and not just a number
---------------------------------
The number is already on screen. What a gauge adds is the answer to "are we
near the middle or near the end", read at a glance and from across the room --
which is the question being asked while someone is at the telescope watching
the camera rather than the screen.

Layout, top to bottom:

      + towards M1          the direction + counts move the camera
      ---- limit ----       the soft travel limit that way
           |
      ==== 0 ====           the zero set by `set-zero`: the focal position
           |
      ---- limit ----
      - towards M2

The filled marker is where the camera is now. The hollow one is where it has
been told to go, drawn only while those differ, so during a move you can see
the gap close.

Deliberately not to scale with the actuators: this shows *focus*, the position
of the focal plane along the optical axis, which is what an operator is
setting. Individual actuator positions are in the table.
"""

from __future__ import annotations

from typing import Optional

import tkinter as tk

COLOR_TRACK = "#d8d8d8"
COLOR_ZERO = "#333333"
COLOR_LIMIT = "#b3231f"
COLOR_NOW = "#1b5fa8"
COLOR_TARGET = "#c77700"
COLOR_TEXT = "#222222"
COLOR_MUTED = "#777777"
COLOR_BAND = "#cfe0f2"


class FocusGauge(tk.Canvas):
    """Vertical position indicator for the focal plane."""

    def __init__(self, parent, min_mm: float = -24.0, max_mm: float = 24.0,
                 width: int = 190, height: int = 340, **kw):
        super().__init__(parent, width=width, height=height,
                         highlightthickness=0, bg="white", **kw)
        self.min_mm = float(min_mm)
        self.max_mm = float(max_mm)
        self._position_mm: Optional[float] = None
        self._target_mm: Optional[float] = None
        self._valid = False
        self.bind("<Configure>", lambda _event: self._redraw())
        self._redraw()

    # ------------------------------------------------------------------ api

    def set_limits(self, min_mm: float, max_mm: float) -> None:
        self.min_mm, self.max_mm = float(min_mm), float(max_mm)
        self._redraw()

    def update_position(self, position_mm: Optional[float],
                        target_mm: Optional[float] = None,
                        valid: bool = True) -> None:
        self._position_mm = position_mm
        self._target_mm = target_mm
        self._valid = valid and position_mm is not None
        self._redraw()

    # -------------------------------------------------------------- drawing

    def _geometry(self):
        width = max(int(self.winfo_width()), 120)
        height = max(int(self.winfo_height()), 200)
        top = 34.0
        bottom = height - 46.0
        centre_x = width * 0.42
        return width, height, top, bottom, centre_x

    def _y_for(self, mm: float, top: float, bottom: float) -> float:
        """Millimetres to a y coordinate. Positive is up, as the label says."""
        span = self.max_mm - self.min_mm
        if span <= 0:
            return (top + bottom) / 2.0
        fraction = (mm - self.min_mm) / span
        fraction = min(1.0, max(0.0, fraction))
        return bottom - fraction * (bottom - top)

    def _redraw(self) -> None:
        self.delete("all")
        width, height, top, bottom, cx = self._geometry()
        track_half = 13.0

        # --- direction labels -------------------------------------------
        self.create_text(cx, 12, text="+  towards M1", fill=COLOR_MUTED,
                         font=("TkDefaultFont", 8))
        self.create_text(cx, height - 30, text="−  towards M2",
                         fill=COLOR_MUTED, font=("TkDefaultFont", 8))

        # --- track --------------------------------------------------------
        self.create_rectangle(cx - track_half, top, cx + track_half, bottom,
                              fill=COLOR_TRACK, outline="#b0b0b0")

        # --- shaded band from zero to the current position ----------------
        # Reading a signed offset off a bar is easier when the offset itself is
        # drawn, rather than left to be inferred from two marker positions.
        if self._valid and self._position_mm is not None:
            zero_y = self._y_for(0.0, top, bottom)
            now_y = self._y_for(self._position_mm, top, bottom)
            self.create_rectangle(cx - track_half + 1, min(zero_y, now_y),
                                  cx + track_half - 1, max(zero_y, now_y),
                                  fill=COLOR_BAND, outline="")

        # --- scale ticks ---------------------------------------------------
        for mm in self._tick_values():
            y = self._y_for(mm, top, bottom)
            major = abs(mm) < 1e-9
            self.create_line(cx - track_half - (7 if major else 4), y,
                             cx - track_half, y,
                             fill=COLOR_ZERO if major else "#9a9a9a",
                             width=2 if major else 1)
            self.create_text(cx - track_half - 10, y, anchor="e",
                             text=f"{mm:+.0f}" if not major else "0",
                             fill=COLOR_ZERO if major else COLOR_MUTED,
                             font=("TkDefaultFont", 8, "bold" if major else "normal"))

        # --- zero line ------------------------------------------------------
        zero_y = self._y_for(0.0, top, bottom)
        self.create_line(cx - track_half - 8, zero_y, cx + track_half + 8, zero_y,
                         fill=COLOR_ZERO, width=2)

        # --- limits ---------------------------------------------------------
        for limit in (self.min_mm, self.max_mm):
            y = self._y_for(limit, top, bottom)
            self.create_line(cx - track_half - 4, y, cx + track_half + 4, y,
                             fill=COLOR_LIMIT, width=2, dash=(3, 2))

        # --- target, drawn only when it differs from where we are -----------
        if (self._valid and self._target_mm is not None
                and self._position_mm is not None
                and abs(self._target_mm - self._position_mm) > 1e-4):
            self._marker(cx, self._y_for(self._target_mm, top, bottom),
                         track_half, COLOR_TARGET, filled=False)
            self.create_text(cx + track_half + 12,
                             self._y_for(self._target_mm, top, bottom),
                             anchor="w", text="target", fill=COLOR_TARGET,
                             font=("TkDefaultFont", 8))

        # --- where we are ----------------------------------------------------
        if self._valid and self._position_mm is not None:
            self._marker(cx, self._y_for(self._position_mm, top, bottom),
                         track_half, COLOR_NOW, filled=True)

        # --- readout ---------------------------------------------------------
        if self._valid and self._position_mm is not None:
            text = f"{self._position_mm:+.4f} mm"
            colour = COLOR_NOW
            if not (self.min_mm <= self._position_mm <= self.max_mm):
                colour = COLOR_LIMIT
                text += "  OUT OF RANGE"
        else:
            text, colour = "no reading", COLOR_LIMIT
        self.create_text(width / 2, height - 12, text=text, fill=colour,
                         font=("TkDefaultFont", 11, "bold"))

    def _marker(self, cx: float, y: float, track_half: float,
                colour: str, filled: bool) -> None:
        """A triangle pointing at the track from the right."""
        size = 8.0
        points = [cx + track_half + 2, y,
                  cx + track_half + 2 + size, y - size * 0.8,
                  cx + track_half + 2 + size, y + size * 0.8]
        self.create_polygon(points, fill=colour if filled else "white",
                            outline=colour, width=2)
        self.create_line(cx - track_half, y, cx + track_half, y,
                         fill=colour, width=2 if filled else 1,
                         dash=() if filled else (3, 2))

    def _tick_values(self):
        """Round tick positions across the range, including zero."""
        span = self.max_mm - self.min_mm
        if span <= 0:
            return [0.0]
        for step in (1, 2, 5, 10, 20, 25, 50, 100):
            if span / step <= 10:
                break
        values = []
        first = int(self.min_mm // step) * step
        value = first
        while value <= self.max_mm + 1e-9:
            if value >= self.min_mm - 1e-9:
                values.append(float(value))
            value += step
        if not any(abs(v) < 1e-9 for v in values) and self.min_mm <= 0 <= self.max_mm:
            values.append(0.0)
        return sorted(values)


__all__ = ["FocusGauge"]
