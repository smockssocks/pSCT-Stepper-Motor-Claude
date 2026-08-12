"""Tests for the three-point focal-plane kinematics.

The kinematics are the part of this package that nobody can eyeball for
correctness at the telescope, so they carry the heaviest tests. The key
property is that the two transforms are exact inverses: if they are not, a
commanded orientation and a reported orientation mean different things and
every closed-loop adjustment walks away from the truth.
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from psct_motors.kinematics import Orientation, ThreePointPlatform  # noqa: E402
from psct_motors.config import default_config  # noqa: E402


R = 500.0


def symmetric_platform(radius=R, start_deg=90.0):
    pts = []
    for i in range(3):
        a = math.radians(start_deg + 120.0 * i)
        pts.append((radius * math.cos(a), radius * math.sin(a)))
    return ThreePointPlatform(pts)


class TestRoundTrip(unittest.TestCase):
    """orientation -> actuators -> orientation must be the identity."""

    def test_round_trip_symmetric(self):
        platform = symmetric_platform()
        cases = [
            Orientation(25.0, 0.0, 0.0),
            Orientation(25.0, 0.1, 0.0),
            Orientation(25.0, 0.0, -0.1),
            Orientation(10.0, 0.25, 0.35),
            Orientation(48.0, -0.4, 0.2),
            Orientation(1.0, -0.75, -0.75),
        ]
        for want in cases:
            with self.subTest(orientation=want.describe()):
                zs = platform.actuators_from_orientation(want)
                got = platform.orientation_from_actuators(zs)
                self.assertAlmostEqual(want.focus_mm, got.focus_mm, places=9)
                self.assertAlmostEqual(want.tip_deg, got.tip_deg, places=9)
                self.assertAlmostEqual(want.tilt_deg, got.tilt_deg, places=9)

    def test_round_trip_asymmetric_triangle(self):
        """A lopsided triangle must work exactly as well as a symmetric one."""
        platform = ThreePointPlatform([(600.0, 0.0), (-200.0, 450.0), (-350.0, -510.0)])
        want = Orientation(22.0, 0.3, -0.2)
        zs = platform.actuators_from_orientation(want)
        got = platform.orientation_from_actuators(zs)
        self.assertAlmostEqual(want.focus_mm, got.focus_mm, places=9)
        self.assertAlmostEqual(want.tip_deg, got.tip_deg, places=9)
        self.assertAlmostEqual(want.tilt_deg, got.tilt_deg, places=9)

    def test_actuators_to_orientation_and_back(self):
        """Starting from arbitrary actuator heights, not orientations."""
        platform = symmetric_platform()
        zs = [24.0, 25.5, 23.25]
        o = platform.orientation_from_actuators(zs)
        again = platform.actuators_from_orientation(o)
        for want, got in zip(zs, again):
            self.assertAlmostEqual(want, got, places=9)


class TestPhysicalMeaning(unittest.TestCase):
    def test_pure_focus_moves_all_actuators_equally(self):
        platform = symmetric_platform()
        zs = platform.actuators_from_orientation(Orientation(30.0, 0.0, 0.0))
        for z in zs:
            self.assertAlmostEqual(z, 30.0, places=12)

    def test_focus_is_z_on_the_optical_axis(self):
        """Not the mean of the actuators -- they differ on a lopsided mount."""
        platform = ThreePointPlatform([(600.0, 0.0), (-200.0, 450.0), (-350.0, -510.0)])
        o = Orientation(20.0, 0.3, -0.2)
        zs = platform.actuators_from_orientation(o)
        self.assertAlmostEqual(platform.orientation_from_actuators(zs).focus_mm, 20.0,
                               places=9)
        # Confirm the distinction is real for this geometry, so the test would
        # actually catch a switch to a mean-based definition.
        self.assertNotAlmostEqual(sum(zs) / 3.0, 20.0, places=4)

    def test_positive_tip_raises_plus_y(self):
        """Right-handed rotation about +x lifts the +y side."""
        platform = ThreePointPlatform([(0.0, 500.0), (-433.0, -250.0), (433.0, -250.0)])
        zs = platform.actuators_from_orientation(Orientation(25.0, 0.2, 0.0))
        self.assertGreater(zs[0], 25.0)          # the +y actuator goes up
        self.assertLess(zs[1], 25.0)
        self.assertLess(zs[2], 25.0)

    def test_positive_tilt_lowers_plus_x(self):
        """Right-handed rotation about +y drops the +x side."""
        platform = ThreePointPlatform([(500.0, 0.0), (-250.0, 433.0), (-250.0, -433.0)])
        zs = platform.actuators_from_orientation(Orientation(25.0, 0.0, 0.2))
        self.assertLess(zs[0], 25.0)             # the +x actuator goes down
        self.assertGreater(zs[1], 25.0)
        self.assertGreater(zs[2], 25.0)

    def test_tilt_scales_with_lever_arm(self):
        """Twice the radius, twice the actuator travel for the same angle."""
        small = symmetric_platform(radius=250.0)
        large = symmetric_platform(radius=500.0)
        o = Orientation(25.0, 0.2, 0.0)
        span_small = max(small.actuators_from_orientation(o)) - min(small.actuators_from_orientation(o))
        span_large = max(large.actuators_from_orientation(o)) - min(large.actuators_from_orientation(o))
        self.assertAlmostEqual(span_large / span_small, 2.0, places=9)


class TestPolarForm(unittest.TestCase):
    def test_polar_round_trip(self):
        for tilt in (0.05, 0.3, 0.9):
            for azimuth in (0.0, 45.0, 137.0, 250.0, 359.0):
                with self.subTest(tilt=tilt, azimuth=azimuth):
                    o = Orientation.from_polar_tilt(25.0, tilt, azimuth)
                    self.assertAlmostEqual(o.total_tilt_deg, tilt, places=9)
                    self.assertAlmostEqual(o.tilt_azimuth_deg, azimuth % 360.0, places=6)

    def test_total_tilt_combines_tip_and_tilt(self):
        """Two small angles can add up to one that is not small."""
        o = Orientation(25.0, 0.3, 0.4)
        self.assertGreater(o.total_tilt_deg, 0.3)
        self.assertGreater(o.total_tilt_deg, 0.4)
        self.assertAlmostEqual(o.total_tilt_deg, math.hypot(0.3, 0.4), places=4)

    def test_flat_plane_has_defined_azimuth(self):
        self.assertEqual(Orientation(25.0, 0.0, 0.0).tilt_azimuth_deg, 0.0)

    def test_arcmin_and_arcsec_views(self):
        o = Orientation(25.0, 1.0 / 60.0, 0.0)
        self.assertAlmostEqual(o.total_tilt_arcmin, 1.0, places=6)
        self.assertAlmostEqual(o.total_tilt_arcsec, 60.0, places=4)


class TestGuards(unittest.TestCase):
    def test_collinear_actuators_rejected(self):
        with self.assertRaises(ValueError) as ctx:
            ThreePointPlatform([(0.0, 0.0), (100.0, 0.0), (200.0, 0.0)])
        self.assertIn("collinear", str(ctx.exception))

    def test_coincident_actuators_rejected(self):
        with self.assertRaises(ValueError):
            ThreePointPlatform([(10.0, 10.0), (10.0, 10.0), (10.0, 10.0)])

    def test_wrong_number_of_points(self):
        with self.assertRaises(ValueError):
            ThreePointPlatform([(0.0, 0.0), (100.0, 0.0)])

    def test_wrong_number_of_actuator_positions(self):
        with self.assertRaises(ValueError):
            symmetric_platform().orientation_from_actuators([1.0, 2.0])


class TestDefaultGeometry(unittest.TestCase):
    def test_default_config_geometry_is_usable(self):
        cfg = default_config()
        cfg.validate()
        from psct_motors.kinematics import platform_from_config
        platform = platform_from_config(cfg)
        self.assertGreater(platform.triangle_area_mm2, 0.0)
        for arm in platform.lever_arms_mm():
            self.assertAlmostEqual(arm, 500.0, places=6)

    def test_tilt_travel_budget_is_reported(self):
        """A degree of tilt at 500 mm radius costs most of the 50 mm range,
        which is exactly the kind of thing an operator should see before
        committing to a tilt."""
        platform = symmetric_platform()
        span = platform.actuator_span_for_tilt(1.0)
        self.assertGreater(span, 10.0)
        self.assertLess(span, 30.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
