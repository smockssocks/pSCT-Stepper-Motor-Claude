"""Keeping the three actuators together, the big-error stop, and settling.

A motor under more load than the others falls behind, and the plate tilts on
its way to the target. These check that the leaders are held for it, that a
motor which cannot keep up stops the move with the brakes applied, that a
motor far from where it was told to be does the same, and that a motor sitting
a little short of its target is nudged onto it.
"""

import os
import sys
import threading
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from psct_motors import safety  # noqa: E402
from psct_motors.jvl_motor import BrakeState  # noqa: E402
from psct_motors.kinematics import Orientation  # noqa: E402
from psct_motors.platform import FocalPlanePlatform, PlatformError  # noqa: E402


class _Base(unittest.TestCase):
    def _platform(self, **cfg_changes):
        cfg = safety.bench_config()
        cfg.simulated_speed_mm_per_s = 2.0
        for key, value in cfg_changes.items():
            setattr(cfg, key, value)
        cfg.validate()
        self.log = []
        platform = FocalPlanePlatform(cfg=cfg, simulate=True, logger=self.log.append)
        platform.connect()
        self.addCleanup(platform.disconnect)
        return platform

    def _brakes_engaged(self, platform) -> bool:
        return platform.external_brake.read_state("all", fresh=True) is BrakeState.ENGAGED

    def _move_watching_spread(self, platform, target):
        """Run a move and record the biggest gap between the actuators."""
        worst = [0.0]
        done = threading.Event()
        error = []

        def watch():
            while not done.is_set():
                try:
                    positions = platform.read_actuator_positions_mm()
                    worst[0] = max(worst[0], max(positions) - min(positions))
                except Exception:  # noqa: BLE001 -- a sample, not the test
                    pass
                time.sleep(0.02)

        watcher = threading.Thread(target=watch, daemon=True)
        watcher.start()
        try:
            platform.move_to_orientation(target)
        except PlatformError as exc:
            error.append(exc)
        finally:
            done.set()
            watcher.join(timeout=1.0)
        return worst[0], error


class TestStayingTogether(_Base):
    def test_a_slow_motor_is_waited_for_and_the_plate_stays_level(self):
        platform = self._platform()
        platform.motor("East")._transport.speed_factor = 0.5
        start = platform.read_orientation().focus_mm
        spread, error = self._move_watching_spread(
            platform, Orientation(start + 2.0, 0.0, 0.0))
        self.assertEqual(error, [])
        self.assertLess(spread, 0.25, "the plate tilted on the way")
        self.assertAlmostEqual(platform.read_orientation().focus_mm, start + 2.0,
                               places=2)
        self.assertTrue(any("holding it until the others catch up" in line
                            for line in self.log), self.log)

    def test_without_the_watch_the_same_motor_tilts_the_plate(self):
        """The control case: proof the test above is measuring something."""
        platform = self._platform(sync_pause_mm=5.0, sync_abort_mm=10.0)
        platform.motor("East")._transport.speed_factor = 0.5
        start = platform.read_orientation().focus_mm
        spread, _ = self._move_watching_spread(
            platform, Orientation(start + 2.0, 0.0, 0.0))
        self.assertGreater(spread, 0.8)

    def test_a_motor_that_cannot_keep_up_stops_the_move_with_the_brakes_on(self):
        platform = self._platform(sync_max_wait_s=0.5)
        platform.motor("East")._transport.speed_factor = 0.0      # blocked
        start = platform.read_orientation().focus_mm
        with self.assertRaises(PlatformError) as ctx:
            platform.move_to_orientation(Orientation(start + 2.0, 0.0, 0.0))
        message = str(ctx.exception)
        self.assertIn("East is not keeping up", message)
        self.assertIn("brakes were applied", message)
        self.assertTrue(self._brakes_engaged(platform))
        positions = platform.read_actuator_positions_mm()
        # Held at about the pause distance, plus what one check interval
        # lets through at 2 mm/s.
        self.assertLess(max(positions) - min(positions), 0.3)
        self.assertEqual(platform.history.last().kind, "move")
        self.assertFalse(platform.history.last().completed)


class TestBigErrorStop(_Base):
    def test_a_motor_far_behind_its_command_stops_everything_and_brakes(self):
        platform = self._platform()
        top = platform.motor("Top")
        # 0.2 mm behind at 1000 counts/mm, over the 0.1 mm limit.
        top._transport.follow_error_counts = 200
        start = platform.read_orientation().focus_mm
        with self.assertRaises(PlatformError) as ctx:
            platform.move_to_orientation(Orientation(start + 1.0, 0.0, 0.0))
        self.assertIn("Top fell 0.200 mm behind", str(ctx.exception))
        self.assertTrue(self._brakes_engaged(platform))

    def test_the_limit_is_a_setting(self):
        """And a steady lag does not leave the others waiting for ever: once
        the lagging motor's move has finished, settling takes over."""
        platform = self._platform(max_position_error_mm=0.5)
        top = platform.motor("Top")
        top._transport.follow_error_counts = 200
        start = platform.read_orientation().focus_mm
        platform.move_to_orientation(Orientation(start + 1.0, 0.0, 0.0))
        self.assertLessEqual(abs(top.cfg.mm_to_counts(start + 1.0)
                                 - top.get_position_counts()),
                             platform.cfg.settle_deadband_counts)


class TestAlreadyStraining(_Base):
    """A motor over its torque limit before anything moves is pushing on
    something. Moving would force it, so nothing moves."""

    def _straining(self, platform, name="East"):
        # 1600 of 2048 is 78%, over the 45% limit, while standing still.
        platform.motor(name)._transport.idle_torque = 1600

    def test_a_move_is_refused_and_nothing_is_touched(self):
        platform = self._platform()
        self._straining(platform)
        before = platform.read_actuator_positions_mm()
        brakes_before = platform.external_brake.read_state("all", fresh=True)
        start = platform.read_orientation().focus_mm
        with self.assertRaises(PlatformError) as ctx:
            platform.move_to_orientation(Orientation(start + 1.0, 0.0, 0.0))
        message = str(ctx.exception)
        self.assertIn("East at 78%", message)
        self.assertIn("already over the torque limit", message)
        self.assertEqual(platform.read_actuator_positions_mm(), before)
        self.assertIs(platform.external_brake.read_state("all", fresh=True),
                      brakes_before)
        self.assertEqual(len(platform.history), 0)

    def test_a_jog_is_refused_too(self):
        platform = self._platform()
        self._straining(platform, "Top")
        before = platform.read_actuator_positions_mm()
        with self.assertRaises(PlatformError) as ctx:
            platform.move_actuator_mm("West", 0.2, relative=True)
        self.assertIn("Top at 78%", str(ctx.exception))
        self.assertEqual(platform.read_actuator_positions_mm(), before)

    def test_under_the_limit_it_moves(self):
        platform = self._platform()
        platform.motor("East")._transport.idle_torque = 800     # 39%
        start = platform.read_orientation().focus_mm
        platform.move_to_orientation(Orientation(start + 0.5, 0.0, 0.0))
        self.assertAlmostEqual(platform.read_orientation().focus_mm, start + 0.5,
                               places=2)


class TestSettling(_Base):
    def test_a_motor_sitting_short_is_nudged_onto_its_target(self):
        platform = self._platform()
        top = platform.motor("Top")
        top._transport.follow_error_counts = 60     # short by 60 counts
        start = platform.read_orientation().focus_mm
        platform.move_to_orientation(Orientation(start + 1.0, 0.0, 0.0))
        target = top.cfg.mm_to_counts(start + 1.0)
        self.assertLessEqual(abs(target - top.get_position_counts()),
                             platform.cfg.settle_deadband_counts)
        self.assertTrue(any(line.startswith("Settled at the target")
                            and "Top +60 -> +0 counts" in line
                            for line in self.log), self.log)

    def test_settling_can_be_turned_off(self):
        platform = self._platform(settle_enabled=False)
        top = platform.motor("Top")
        top._transport.follow_error_counts = 60
        start = platform.read_orientation().focus_mm
        platform.move_to_orientation(Orientation(start + 1.0, 0.0, 0.0))
        self.assertEqual(top.cfg.mm_to_counts(start + 1.0) - top.get_position_counts(), 60)

    def test_small_differences_are_left_alone(self):
        platform = self._platform()
        platform.motor("Top")._transport.follow_error_counts = 20   # under 50
        start = platform.read_orientation().focus_mm
        platform.move_to_orientation(Orientation(start + 1.0, 0.0, 0.0))
        self.assertFalse(any(line.startswith("Settled") for line in self.log))

    def test_a_jog_settles_too(self):
        platform = self._platform()
        top = platform.motor("Top")
        top._transport.follow_error_counts = 60
        before = top.get_position_counts()
        platform.move_actuator_mm("Top", 0.5, relative=True)
        moved = abs(top.get_position_counts() - before)
        self.assertLessEqual(abs(moved - 0.5 * top.cfg.resolved_counts_per_mm),
                             platform.cfg.settle_deadband_counts)
        self.assertTrue(any(line.startswith("Settled") for line in self.log))


if __name__ == "__main__":
    unittest.main()
