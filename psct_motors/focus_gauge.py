"""
A vertical gauge showing where the focal plane is along the optical axis.

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

What the numbers are measured from
----------------------------------
Three choices, because the people at the telescope think in more than one:

    "zero"   millimetres from the reference set by `set-zero`, signed, +
             towards M1. This is what every command is expressed in.
    "m1"     millimetres from the focal plane to M1, the primary mirror.
    "m2"     millimetres from the focal plane to M2, the secondary.

The last two need one extra number each: how far the zero reference is from
that mirror. Nobody has that figure yet, and it can change (the zero can be
re-set; the mirrors can be re-surveyed), so it is a setting rather than a
constant, and until it is entered the gauge says so instead of inventing one.
The geometry -- limits, stops, markers -- is always drawn in focus millimetres;
only the labels and the readout change with the reference.

Deliberately not to scale with the actuators: this shows *focus*, the position
of the focal plane along the optical axis, which is what an operator is
setting. Individual actuator positions are in the table.
"""

from __future__ import annotations

from typing import List, Optional

import tkinter as tk

COLOR_TRACK = "#d8d8d8"
COLOR_ZERO = "#333333"
COLOR_LIMIT = "#b3231f"
COLOR_NOW = "#1b5fa8"
COLOR_TARGET = "#c77700"
COLOR_TEXT = "#222222"
COLOR_MUTED = "#777777"
COLOR_BAND = "#cfe0f2"
#: The hard stops are drawn in the same red as the STOP controls, solid rather
#: than dashed: a soft limit is a setting and can be changed, an end stop is
#: the machine and cannot.
COLOR_HARD_STOP = "#b3231f"

#: The three things the gauge can measure from, and what to call each.
REFERENCES = ("zero", "m1", "m2")
REFERENCE_TITLES = {
    "zero": "Distance from zero",
    "m1": "Distance to M1",
    "m2": "Distance to M2",
}


class FocusGauge(tk.Canvas):
    """Vertical position indicator for the focal plane."""

    #: Room above and below the track. The top holds the direction label; the
    #: bottom holds the direction label, the readout and its caption, each on
    #: its own line so none of them is drawn over another.
    TOP_MARGIN = 44.0
    BOTTOM_MARGIN = 70.0

    def __init__(self, parent, min_mm: float = -24.0, max_mm: float = 24.0,
                 width: int = 230, height: int = 360, **kw):
        super().__init__(parent, width=width, height=height,
                         highlightthickness=0, bg="white", **kw)
        self.min_mm = float(min_mm)
        self.max_mm = float(max_mm)
        #: Where the mechanism actually stops, once `find-stop` has found it.
        #: Drawn solid, outside the dashed soft limits, because the distance
        #: between the two is the margin you have left -- and that is the thing
        #: worth seeing while nudging focus near the end of travel.
        self.hard_stop_low_mm: Optional[float] = None
        self.hard_stop_high_mm: Optional[float] = None
        #: What the labels are measured from -- see the module docstring.
        self.reference = "zero"
        self.zero_to_m1_mm: Optional[float] = None
        self.zero_to_m2_mm: Optional[float] = None
        self._position_mm: Optional[float] = None
        self._target_mm: Optional[float] = None
        self._valid = False
        self.bind("<Configure>", lambda _event: self._redraw())
        self._redraw()

    # ------------------------------------------------------------------ api

    def set_limits(self, min_mm: float, max_mm: float,
                   hard_stop_low_mm: Optional[float] = None,
                   hard_stop_high_mm: Optional[float] = None) -> None:
        self.min_mm, self.max_mm = float(min_mm), float(max_mm)
        self.hard_stop_low_mm = hard_stop_low_mm
        self.hard_stop_high_mm = hard_stop_high_mm
        self._redraw()

    def set_reference(self, reference: str,
                      zero_to_m1_mm: Optional[float] = None,
                      zero_to_m2_mm: Optional[float] = None) -> None:
        """Choose what the labels measure from, and supply the distances.

        The distances are taken every time, not only when the reference is
        M1 or M2, so that editing them while the gauge shows "from zero"
        still takes effect the moment the reference is switched.
        """
        if reference not in REFERENCES:
            raise ValueError(f"reference must be one of {REFERENCES}, got {reference!r}")
        self.reference = reference
        self.zero_to_m1_mm = zero_to_m1_mm
        self.zero_to_m2_mm = zero_to_m2_mm
        self._redraw()

    def update_position(self, position_mm: Optional[float],
                        target_mm: Optional[float] = None,
                        valid: bool = True) -> None:
        self._position_mm = position_mm
        self._target_mm = target_mm
        self._valid = valid and position_mm is not None
        self._redraw()

    # -------------------------------------------------------- the reference

    @property
    def reference_title(self) -> str:
        return REFERENCE_TITLES[self.reference]

    @property
    def reference_available(self) -> bool:
        """False when the chosen reference needs a distance nobody has entered."""
        if self.reference == "m1":
            return self.zero_to_m1_mm is not None
        if self.reference == "m2":
            return self.zero_to_m2_mm is not None
        return True

    def display_value(self, focus_mm: float) -> Optional[float]:
        """A focus position, expressed in the chosen reference.

        Focus is signed and + towards M1. So the distance left to M1 shrinks
        as focus grows, and the distance to M2 grows with it. None when the
        distance that reference depends on has not been entered.
        """
        if self.reference == "m1":
            if self.zero_to_m1_mm is None:
                return None
            return self.zero_to_m1_mm - focus_mm
        if self.reference == "m2":
            if self.zero_to_m2_mm is None:
                return None
            return self.zero_to_m2_mm + focus_mm
        return focus_mm

    def focus_for_display(self, value: float) -> Optional[float]:
        """The inverse of `display_value`: a labelled value back to focus mm."""
        if self.reference == "m1":
            if self.zero_to_m1_mm is None:
                return None
            return self.zero_to_m1_mm - value
        if self.reference == "m2":
            if self.zero_to_m2_mm is None:
                return None
            return value - self.zero_to_m2_mm
        return value

    def format_value(self, focus_mm: float, decimals: int = 4) -> str:
        """The readout text for a focus position in the chosen reference."""
        value = self.display_value(focus_mm)
        if value is None:
            return "distance not set"
        if self.reference == "zero":
            return f"{value:+.{decimals}f} mm"
        return f"{value:.{decimals}f} mm"

    # -------------------------------------------------------------- drawing

    def _geometry(self):
        width = max(int(self.winfo_width()), 140)
        height = max(int(self.winfo_height()), 220)
        top = self.TOP_MARGIN
        bottom = height - self.BOTTOM_MARGIN
        centre_x = width * 0.40
        return width, height, top, bottom, centre_x

    def drawn_range(self):
        """The millimetre range the track covers.

        The soft limits, widened to take in the hard stops when they are
        known. Without this a stop found beyond the soft limit -- which is
        where stops always are -- would be drawn clamped onto the end of the
        track, exactly on top of the limit it is supposed to sit outside.
        """
        low, high = self.min_mm, self.max_mm
        for stop in (self.hard_stop_low_mm, self.hard_stop_high_mm):
            if stop is None:
                continue
            low = min(low, stop)
            high = max(high, stop)
        if (low, high) != (self.min_mm, self.max_mm):
            margin = (high - low) * 0.04
            low, high = low - margin, high + margin
        return low, high

    def _y_for(self, mm: float, top: float, bottom: float) -> float:
        """Millimetres to a y coordinate. Positive is up, as the label says."""
        low, high = self.drawn_range()
        span = high - low
        if span <= 0:
            return (top + bottom) / 2.0
        fraction = (mm - low) / span
        fraction = min(1.0, max(0.0, fraction))
        return bottom - fraction * (bottom - top)

    def _redraw(self) -> None:
        self.delete("all")
        width, height, top, bottom, cx = self._geometry()
        track_half = 13.0
        right = cx + track_half        # x where things to the right of the track begin

        # --- direction labels -------------------------------------------
        # Each on its own line, clear of the track and of the end-of-travel
        # marks, which used to be drawn straight over them.
        self.create_text(cx, 14, text="+  towards M1", fill=COLOR_MUTED,
                         font=("TkDefaultFont", 8))
        self.create_text(cx, bottom + 16, text="−  towards M2",
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
        # Ticks are chosen as round numbers in the units being shown, then
        # placed at the focus position they correspond to. Otherwise "from
        # M1" would label the track 1231.0, 1221.0, 1211.0 ... which nobody
        # can read at a glance.
        zero_y = self._y_for(0.0, top, bottom)
        for value, mm in self._ticks():
            y = self._y_for(mm, top, bottom)
            major = abs(mm) < 1e-9
            self.create_line(cx - track_half - (7 if major else 4), y,
                             cx - track_half, y,
                             fill=COLOR_ZERO if major else "#9a9a9a",
                             width=2 if major else 1)
            self.create_text(cx - track_half - 10, y, anchor="e",
                             text=self._tick_label(value),
                             fill=COLOR_ZERO if major else COLOR_MUTED,
                             font=("TkDefaultFont", 8, "bold" if major else "normal"))

        # --- zero line ------------------------------------------------------
        # Always drawn: in the M1/M2 references it is still the point every
        # command is measured from, so it is labelled with what it reads there.
        self.create_line(cx - track_half - 8, zero_y, cx + track_half + 8, zero_y,
                         fill=COLOR_ZERO, width=2)
        if self.reference != "zero" and self.reference_available:
            self.create_text(right + 12, zero_y, anchor="w",
                             text=f"zero = {self.display_value(0.0):.1f}",
                             fill=COLOR_ZERO, font=("TkDefaultFont", 7))

        # --- limits ---------------------------------------------------------
        for limit in (self.min_mm, self.max_mm):
            y = self._y_for(limit, top, bottom)
            self.create_line(cx - track_half - 4, y, cx + track_half + 4, y,
                             fill=COLOR_LIMIT, width=2, dash=(3, 2))

        # --- hard stops, where the mechanism physically ends -----------------
        # The label sits to the right of the track, just outside the stop,
        # where nothing else is drawn. Centred over the track it landed on the
        # tick labels and the limit line.
        for stop in (self.hard_stop_low_mm, self.hard_stop_high_mm):
            if stop is None:
                continue
            y = self._y_for(stop, top, bottom)
            self.create_line(cx - track_half - 10, y, cx + track_half + 10, y,
                             fill=COLOR_HARD_STOP, width=3)
            self.create_text(right + 12, y + (8 if stop < 0 else -8), anchor="w",
                             text="END OF TRAVEL", fill=COLOR_HARD_STOP,
                             font=("TkDefaultFont", 7, "bold"))

        # --- target, drawn only when it differs from where we are -----------
        if (self._valid and self._target_mm is not None
                and self._position_mm is not None
                and abs(self._target_mm - self._position_mm) > 1e-4):
            target_y = self._y_for(self._target_mm, top, bottom)
            self._marker(cx, target_y, track_half, COLOR_TARGET, filled=False)
            self.create_text(right + 14, target_y, anchor="w", text="target",
                             fill=COLOR_TARGET, font=("TkDefaultFont", 8))

        # --- where we are ----------------------------------------------------
        if self._valid and self._position_mm is not None:
            self._marker(cx, self._y_for(self._position_mm, top, bottom),
                         track_half, COLOR_NOW, filled=True)

        # --- readout, on its own two lines below the direction label ---------
        caption = {"zero": "from zero", "m1": "to M1", "m2": "to M2"}[self.reference]
        if self._valid and self._position_mm is not None:
            if self.reference_available:
                text = self.format_value(self._position_mm)
                colour = COLOR_NOW
                if not (self.min_mm <= self._position_mm <= self.max_mm):
                    colour = COLOR_LIMIT
                    caption += "   OUT OF RANGE"
            else:
                text, colour = "distance not set", COLOR_LIMIT
                caption = f"enter the zero-to-{self.reference.upper()} distance"
        else:
            text, colour = "no reading", COLOR_LIMIT
        self.create_text(width / 2, height - 30, text=text, fill=colour,
                         font=("TkDefaultFont", 11, "bold"))
        self.create_text(width / 2, height - 12, text=caption, fill=colour,
                         font=("TkDefaultFont", 8))

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

    def _tick_label(self, value: float) -> str:
        if self.reference == "zero":
            return "0" if abs(value) < 1e-9 else f"{value:+.0f}"
        return f"{value:.0f}" if abs(value - round(value)) < 1e-9 else f"{value:.1f}"

    def _ticks(self) -> List[tuple]:
        """(labelled value, focus mm) for each tick.

        Round values in the units being shown, mapped back to focus so they
        land in the right place on a track that is always in focus mm.
        """
        low, high = self.drawn_range()
        if not self.reference_available:
            return [(0.0, 0.0)] if low <= 0 <= high else []
        d_low, d_high = sorted((self.display_value(low), self.display_value(high)))
        span = d_high - d_low
        if span <= 0:
            return [(self.display_value(0.0), 0.0)]
        for step in (1, 2, 5, 10, 20, 25, 50, 100, 200, 500, 1000):
            if span / step <= 10:
                break
        ticks: List[tuple] = []
        value = int(d_low // step) * step
        while value <= d_high + 1e-9:
            if value >= d_low - 1e-9:
                mm = self.focus_for_display(float(value))
                ticks.append((float(value), mm))
            value += step
        # Zero -- the point commands are measured from -- always gets a tick,
        # labelled with what this reference reads there.
        if low <= 0 <= high and not any(abs(mm) < 1e-9 for _v, mm in ticks):
            ticks.append((self.display_value(0.0), 0.0))
        return sorted(ticks, key=lambda t: t[1])

    def _tick_values(self):
        """Focus positions of the ticks, for callers that only want those."""
        return [mm for _value, mm in self._ticks()]


__all__ = ["FocusGauge", "REFERENCES", "REFERENCE_TITLES"]
