"""
Kinematics of a focal plane carried on three z-actuators.

This is the module that removes the hand calculation. You say "tilt the focal
plane 0.2 degrees about x and move it 1 mm closer to the secondary"; this
turns that into three actuator positions in millimetres, and turns three
measured actuator positions back into an orientation.

The mechanism
-------------
The pSCT camera's inner structure (which carries the focal plane) hangs off
the outer structure on three ball pin joints. Behind each joint is a motor
turning a drive screw that pushes or pulls a flange, and a rail system
constrains that flange to move only along z, the optical axis. So each
actuator contributes exactly one degree of freedom -- its own z -- and three
of them fully determine the plane the focal surface lies on.

Three actuators, three degrees of freedom, and the map between them is exact:

    focus  z   : translation along the optical axis (all three together)
    tip    thx : rotation about the +x axis
    tilt   thy : rotation about the +y axis

There is no approximation and no iteration here. A plane through three points
is a linear algebra problem with a closed-form answer, so `orientation_from_
actuators` and `actuators_from_orientation` are exact inverses of each other
(the round trip is tested to sub-nanometre agreement in tests/test_kinematics.py).

Sign conventions
----------------
Right-handed, with +z along the optical axis in the direction the actuators
push the focal plane, x and y in the focal plane, and azimuth measured
counter-clockwise from +x when looking back along -z.

    focus_mm  : z of the focal plane *on the optical axis* (x=0, y=0).
                Not the average of the three actuators -- those differ if the
                three actuators are not symmetric about the axis, and the
                distance to the secondary mirror on-axis is the number with
                physical meaning.
    tip_deg   : right-handed rotation about +x. Positive tip raises +y.
    tilt_deg  : right-handed rotation about +y. Positive tilt lowers +x.

The gradient form used internally is z(x, y) = c + a*x + b*y, with

    c = focus_mm,   a = dz/dx = -tan(tilt),   b = dz/dy = +tan(tip)

Angles are stored in degrees throughout the public API, because that is what
gets typed into a GUI. `total_tilt_arcmin` and `total_tilt_arcsec` are there
for when you want to talk about the result in observatory units.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple


# --------------------------------------------------------------------------
# Orientation
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class Orientation:
    """Where the focal plane is: one translation and two rotations."""

    focus_mm: float
    tip_deg: float = 0.0
    tilt_deg: float = 0.0

    # ---- derived views of the same two angles ----------------------------

    @property
    def total_tilt_deg(self) -> float:
        """Angle between the focal-plane normal and the optical axis.

        This is the single number to watch against a tilt limit: tip and tilt
        individually being small does not stop their combination from being
        large.
        """
        a = -math.tan(math.radians(self.tilt_deg))
        b = math.tan(math.radians(self.tip_deg))
        return math.degrees(math.atan(math.hypot(a, b)))

    @property
    def total_tilt_arcmin(self) -> float:
        return self.total_tilt_deg * 60.0

    @property
    def total_tilt_arcsec(self) -> float:
        return self.total_tilt_deg * 3600.0

    @property
    def tilt_azimuth_deg(self) -> float:
        """Azimuth of steepest ascent of the focal plane, degrees CCW from +x.

        Together with `total_tilt_deg` this is the polar form of the tilt:
        "tilted 0.3 degrees, uphill towards 45 degrees azimuth", which is
        often easier to reason about at the telescope than a tip/tilt pair.
        Returns 0.0 when the plane is flat, where azimuth is undefined.
        """
        a = -math.tan(math.radians(self.tilt_deg))
        b = math.tan(math.radians(self.tip_deg))
        if abs(a) < 1e-15 and abs(b) < 1e-15:
            return 0.0
        return math.degrees(math.atan2(b, a)) % 360.0

    # ---- constructors -----------------------------------------------------

    @classmethod
    def from_polar_tilt(
        cls, focus_mm: float, total_tilt_deg: float, azimuth_deg: float
    ) -> "Orientation":
        """Build an orientation from the polar form of the tilt."""
        slope = math.tan(math.radians(total_tilt_deg))
        a = slope * math.cos(math.radians(azimuth_deg))
        b = slope * math.sin(math.radians(azimuth_deg))
        return cls(
            focus_mm=focus_mm,
            tip_deg=math.degrees(math.atan(b)),
            tilt_deg=math.degrees(math.atan(-a)),
        )

    # ---- arithmetic -------------------------------------------------------

    def offset_by(self, d_focus_mm: float = 0.0, d_tip_deg: float = 0.0,
                  d_tilt_deg: float = 0.0) -> "Orientation":
        """A relative move. Angles add in degrees, which is exact only for
        small angles; over the sub-degree range these actuators can reach,
        the difference is far below the mechanism's resolution."""
        return Orientation(
            focus_mm=self.focus_mm + d_focus_mm,
            tip_deg=self.tip_deg + d_tip_deg,
            tilt_deg=self.tilt_deg + d_tilt_deg,
        )

    def describe(self) -> str:
        return (
            f"focus {self.focus_mm:+.4f} mm, "
            f"tip {self.tip_deg:+.5f} deg, tilt {self.tilt_deg:+.5f} deg "
            f"(total {self.total_tilt_arcmin:.3f} arcmin "
            f"towards {self.tilt_azimuth_deg:.1f} deg)"
        )

    def as_dict(self) -> dict:
        return {
            "focus_mm": self.focus_mm,
            "tip_deg": self.tip_deg,
            "tilt_deg": self.tilt_deg,
            "total_tilt_deg": self.total_tilt_deg,
            "total_tilt_arcmin": self.total_tilt_arcmin,
            "total_tilt_arcsec": self.total_tilt_arcsec,
            "tilt_azimuth_deg": self.tilt_azimuth_deg,
        }


# --------------------------------------------------------------------------
# The geometry
# --------------------------------------------------------------------------

class ThreePointPlatform:
    """The plate-on-three-posts geometry, and the transforms both ways.

    Construct with the (x, y) location of each actuator's ball joint in the
    focal plane, in millimetres, in the same order you will supply and
    receive actuator z values.
    """

    def __init__(self, points_xy_mm: Sequence[Tuple[float, float]]):
        pts = [(float(x), float(y)) for x, y in points_xy_mm]
        if len(pts) != 3:
            raise ValueError(f"Expected 3 actuator locations, got {len(pts)}")
        self.points: List[Tuple[float, float]] = pts

        # Determinant of [[1, x1, y1], [1, x2, y2], [1, x3, y3]]. Geometrically
        # this is twice the signed area of the actuator triangle: it is zero
        # exactly when the three actuators are collinear, in which case they
        # do not define a plane and neither transform is meaningful.
        (x1, y1), (x2, y2), (x3, y3) = pts
        self._det = (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)
        if abs(self._det) < 1e-9:
            raise ValueError(
                "The three actuators are collinear, so they cannot define a "
                "plane. Check the azimuth and radius of each actuator."
            )

    # ---- geometry helpers -------------------------------------------------

    @property
    def triangle_area_mm2(self) -> float:
        return abs(self._det) / 2.0

    def lever_arms_mm(self) -> List[float]:
        """Distance of each actuator from the optical axis."""
        return [math.hypot(x, y) for x, y in self.points]

    # ---- forward: actuators -> orientation --------------------------------

    def orientation_from_actuators(self, z_mm: Sequence[float]) -> Orientation:
        """Fit the plane through the three actuator positions.

        Exact, not a least-squares fit: three points determine a plane, so
        there is nothing to minimise.
        """
        z = [float(v) for v in z_mm]
        if len(z) != 3:
            raise ValueError(f"Expected 3 actuator positions, got {len(z)}")
        (x1, y1), (x2, y2), (x3, y3) = self.points
        z1, z2, z3 = z

        # Cramer's rule on [[1,x1,y1],[1,x2,y2],[1,x3,y3]] . [c,a,b] = [z1,z2,z3]
        det = self._det
        a = ((z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)) / det
        b = ((x2 - x1) * (z3 - z1) - (x3 - x1) * (z2 - z1)) / det
        c = z1 - a * x1 - b * y1

        return Orientation(
            focus_mm=c,
            tip_deg=math.degrees(math.atan(b)),
            tilt_deg=math.degrees(math.atan(-a)),
        )

    # ---- inverse: orientation -> actuators --------------------------------

    def actuators_from_orientation(self, orientation: Orientation) -> List[float]:
        """The three actuator z positions that realise this orientation."""
        a = -math.tan(math.radians(orientation.tilt_deg))
        b = math.tan(math.radians(orientation.tip_deg))
        c = orientation.focus_mm
        return [c + a * x + b * y for x, y in self.points]

    # ---- sensitivity ------------------------------------------------------

    def actuator_span_for_tilt(self, total_tilt_deg: float) -> float:
        """Peak-to-peak actuator travel needed for a given total tilt.

        Useful for a sanity check before committing to a tilt: with actuators
        on a 500 mm radius, one degree of tilt is about 17 mm of travel
        spread, a third of the whole range.
        """
        worst = 0.0
        for azimuth in range(0, 360, 1):
            o = Orientation.from_polar_tilt(0.0, total_tilt_deg, float(azimuth))
            zs = self.actuators_from_orientation(o)
            worst = max(worst, max(zs) - min(zs))
        return worst


# --------------------------------------------------------------------------
# Building the platform from configuration
# --------------------------------------------------------------------------

def platform_from_config(cfg) -> ThreePointPlatform:
    """Build a ThreePointPlatform from a PlatformConfig."""
    return ThreePointPlatform([a.position_xy_mm for a in cfg.actuators])


__all__ = ["Orientation", "ThreePointPlatform", "platform_from_config"]
