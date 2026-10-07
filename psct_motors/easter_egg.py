"""
A small square in the corner of the log. Click it.

An identity disk drawn as a dot matrix, and a few words about what is out
there. Nothing here touches the motors.
"""

from __future__ import annotations

import math
import random
import tkinter as tk
from typing import List, Optional, Tuple

BACKGROUND = "#000000"
DOT = "#f2fbff"
GLOW = "#2a5560"
QUOTE = "#bfefff"

QUOTE_LINES = (
    "Out there is a new world.",
    "Out there is our future.",
    "Out there... is our destiny.",
)


def disk_lit(x: float, y: float) -> bool:
    """Whether the disk is lit at (x, y), in units of its radius, y up.

    Concentric rings with a pair of brackets inside the middle ring, a short
    arc above the core, and four small ticks between them.
    """
    r = math.hypot(x, y)
    if r > 1.0:
        return False
    angle = math.degrees(math.atan2(y, x)) % 360.0

    if 0.86 <= r <= 1.0:                                   # outer ring
        return True
    if 0.62 <= r <= 0.70 and abs(x) <= 0.60:               # middle ring...
        return True
    if 0.60 <= abs(x) <= 0.66 and abs(y) <= 0.30:          # ...with straight sides
        return True
    if 0.44 <= abs(x) <= 0.50 and abs(y) <= 0.50:          # the brackets inside it
        return True
    if 0.30 <= r <= 0.36:                                  # the core
        return True
    if 0.41 <= r <= 0.45 and 55.0 <= angle <= 125.0 \
            and not 86.0 <= angle <= 94.0:                 # arc above the core
        return True
    for tick in (40.0, 140.0, 220.0, 320.0):               # the ticks
        if 0.39 <= r <= 0.45 and abs((angle - tick + 180.0) % 360.0 - 180.0) <= 3.5:
            return True
    return False


def disk_dots(radius_px: float, spacing_px: float) -> List[Tuple[float, float]]:
    """Centres of the lit dots, in pixels from the disk's centre."""
    dots = []
    steps = int(radius_px // spacing_px)
    for row in range(-steps, steps + 1):
        for column in range(-steps, steps + 1):
            px, py = column * spacing_px, row * spacing_px
            if disk_lit(px / radius_px, -py / radius_px):
                dots.append((px, py))
    return dots


class IdentityDiskButton(tk.Canvas):
    """The small square that opens it."""

    SIZE = 14

    def __init__(self, parent, root: tk.Misc):
        super().__init__(parent, width=self.SIZE, height=self.SIZE, bg="#111111",
                         highlightthickness=1, highlightbackground="#333333",
                         cursor="hand2")
        s = self.SIZE
        self.create_oval(2, 2, s - 2, s - 2, outline="#9fdfff", width=1)
        self.create_oval(5, 5, s - 5, s - 5, outline="#9fdfff", width=1)
        self._root = root
        self.window: Optional[IdentityDisk] = None
        self.bind("<Button-1>", lambda _event: self.open())

    def open(self) -> "IdentityDisk":
        if self.window is not None and self.window.winfo_exists():
            self.window.lift()
            return self.window
        self.window = IdentityDisk(self._root)
        return self.window


class IdentityDisk(tk.Toplevel):
    """The disk, lighting up from the centre out, and then the words."""

    WIDTH = 520
    DISK_RADIUS = 170
    SPACING = 5
    DOT_RADIUS = 1.7

    def __init__(self, root: tk.Misc):
        super().__init__(root, bg=BACKGROUND)
        self.title("")
        self.resizable(False, False)
        height = self.DISK_RADIUS * 2 + 200
        self.canvas = tk.Canvas(self, width=self.WIDTH, height=height,
                                bg=BACKGROUND, highlightthickness=0)
        self.canvas.pack()
        self._centre = (self.WIDTH / 2, self.DISK_RADIUS + 40)
        self._quote_top = self.DISK_RADIUS * 2 + 80
        self._pending: List[str] = []

        cx, cy = self._centre
        dots = disk_dots(self.DISK_RADIUS, self.SPACING)
        # Out from the centre, with a little scatter so it shimmers on.
        order = sorted(dots, key=lambda d: math.hypot(*d) + random.uniform(0, 14))
        self.dots = []
        d = self.DOT_RADIUS
        for px, py in order:
            self.dots.append(self.canvas.create_oval(
                cx + px - d, cy + py - d, cx + px + d, cy + py + d,
                fill=BACKGROUND, outline="", state="hidden"))
        self.quote_items: List[int] = []

        self.bind("<Escape>", lambda _event: self.destroy())
        self.canvas.bind("<Button-1>", lambda _event: self.destroy())
        self._after(200, self._light, 0)

    def _after(self, ms: int, fn, *args) -> None:
        self._pending.append(self.after(ms, fn, *args))

    def destroy(self) -> None:
        for job in self._pending:
            try:
                self.after_cancel(job)
            except tk.TclError:
                pass
        super().destroy()

    # -------------------------------------------------------------- the disk

    def _light(self, index: int) -> None:
        batch = 40
        for item in self.dots[index:index + batch]:
            self.canvas.itemconfigure(item, state="normal", fill=GLOW)
        for item in self.dots[max(0, index - 3 * batch):index - batch]:
            self.canvas.itemconfigure(item, fill=DOT)
        index += batch
        if index < len(self.dots) + 3 * batch:
            self._after(16, self._light, index)
        else:
            for item in self.dots:
                self.canvas.itemconfigure(item, fill=DOT)
            self._after(700, self._type, 0, 0)

    # ------------------------------------------------------------- the words

    def _type(self, line: int, chars: int) -> None:
        if line >= len(QUOTE_LINES):
            return
        text = QUOTE_LINES[line]
        y = self._quote_top + line * 30
        if chars == 0:
            self.quote_items.append(self.canvas.create_text(
                self.WIDTH / 2, y, text="", fill=QUOTE,
                font=("TkFixedFont", 13)))
        self.canvas.itemconfigure(self.quote_items[-1], text=text[:chars + 1])
        if chars + 1 < len(text):
            pause = 260 if text[chars] == "." and text[chars + 1:chars + 2] == "." else 55
            self._after(pause, self._type, line, chars + 1)
        else:
            self._after(900, self._type, line + 1, 0)


__all__ = ["IdentityDiskButton", "IdentityDisk", "disk_lit", "disk_dots", "QUOTE_LINES"]
