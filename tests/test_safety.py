"""The safety drills are themselves tested, because they are the evidence.

A drill that silently stops provoking the situation it claims to provoke would
report PASS for ever, and the report would be worthless. So these check both
that every drill passes and that the drills are actually testing something:
each one is made to fail on purpose by removing the guard it covers.
"""

import os
import sys
import threading
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from psct_motors import safety  # noqa: E402

from psct_motors.external_brake import BrakeError, SimulatedBrakeController  # noqa: E402
from psct_motors.jvl_motor import BrakeState  # noqa: E402
from psct_motors.platform import FocalPlanePlatform, PlatformError  # noqa: E402


class TestSimulatedBrakeController(unittest.TestCase):
    def setUp(self):
        self.brake = SimulatedBrakeController(["Top", "East", "West"])

    def test_brakes_start_engaged(self):
        """Spring-applied: no power means on. The site's procedure describes
        finding them that way."""
        self.assertIs(self.brake.read_state(), BrakeState.ENGAGED)
        for name in ("Top", "East", "West"):
            self.assertTrue(self.brake.is_holding(name))

    def test_release_refused_unless_the_drives_are_holding(self):
        with self.assertRaises(BrakeError) as ctx:
            self.brake.release(drives_holding=False)
        self.assertIn("nothing is holding the focal plane", str(ctx.exception))
        self.assertIs(self.brake.read_state(), BrakeState.ENGAGED)

    def test_release_works_when_the_drives_are_holding(self):
        self.brake.release(drives_holding=True)
        self.assertIs(self.brake.read_state(), BrakeState.RELEASED)
        self.assertFalse(self.brake.is_holding("Top"))

    def test_an_unpowered_brake_cannot_be_released(self):
        self.brake.set_powered(False)
        with self.assertRaises(BrakeError) as ctx:
            self.brake.release(drives_holding=True)
        self.assertIn("no power to the brake supply", str(ctx.exception))
        self.assertTrue(self.brake.is_holding("Top"))

    def test_losing_power_clamps_brakes_that_were_off(self):
        """Fail-safe means the power cut makes them grip, not let go."""
        self.brake.release(drives_holding=True)
        self.assertFalse(self.brake.is_holding("East"))
        self.brake.set_powered(False)
        self.assertTrue(self.brake.is_holding("East"))

    def test_it_says_it_is_not_real(self):
        self.assertIn("simulated", self.brake.describe())


class TestBrakeInterlocks(unittest.TestCase):
    """The interlocks in the platform, not just in the brake device."""

    def _platform(self):
        platform = FocalPlanePlatform(cfg=safety.bench_config(), simulate=True)
        platform.connect()
        self.addCleanup(platform.disconnect)
        return platform

    def test_a_move_releases_the_brakes_and_reports_them_released(self):
        from psct_motors.kinematics import Orientation
        platform = self._platform()
        platform.move_to_orientation(Orientation(1.0, 0.0, 0.0))
        for status in platform.read_state().motors:
            self.assertIs(status.brake.state, BrakeState.RELEASED)
            self.assertFalse(status.brake.inferred,
                             "a readable brake must not be shown as inferred")

    def test_the_gui_sees_the_external_brake_not_the_motor_output(self):
        """Register 179 is 0 on these motors -- the brakes are elsewhere. An
        indicator wired to the motor's own output could never change."""
        platform = self._platform()
        platform.external_brake.engage()
        for status in platform.read_state().motors:
            self.assertIs(status.brake.state, BrakeState.ENGAGED)
        platform.external_brake._set("all", engaged=False)
        for status in platform.read_state().motors:
            self.assertIs(status.brake.state, BrakeState.RELEASED)

    def test_a_move_is_refused_when_the_brakes_cannot_release(self):
        from psct_motors.kinematics import Orientation
        platform = self._platform()
        platform.external_brake.set_powered(False)
        with self.assertRaises(PlatformError) as ctx:
            platform.move_to_orientation(Orientation(1.0, 0.0, 0.0))
        self.assertIn("did not release", str(ctx.exception))

    def test_a_move_is_refused_without_drive_power(self):
        from psct_motors.kinematics import Orientation
        platform = self._platform()
        for motor in platform.motors:
            motor._transport.set_powered(False)
        with self.assertRaises(PlatformError) as ctx:
            platform.move_to_orientation(Orientation(1.0, 0.0, 0.0))
        message = str(ctx.exception)
        # Naming the supply matters: the motors answer Modbus perfectly well
        # in this state, so it presents as the software being broken.
        self.assertIn("supply has failed", message)
        self.assertIn("breaker", message)

    def test_a_single_axis_move_enables_every_drive_first(self):
        """The brakes are one switch for all three, so releasing them with two
        drives passive would leave most of the plate held by nothing."""
        from psct_motors.registers import MotorMode
        platform = self._platform()
        platform.move_actuator_mm("East", 1.0)
        for motor in platform.motors:
            self.assertEqual(motor.get_mode(), int(MotorMode.POSITION),
                             f"{motor.name} was left passive")


class TestEmergencyDoesNotDropTheCamera(unittest.TestCase):
    """The failure this class exists for, reported from the telescope:

        "The emergency stop doesn't stop, instead it just moves the motors all
         the way down with no stopping and continues to keep going down till
         the end of time."

    EMERGENCY wrote MODE_REG = 0 straight away. The focal plane hangs on three
    screws; the site's brakes are on a separate device this software cannot
    command; so cutting drive power removed the only thing holding the camera
    and it sank, back-driving the screws, with the encoder running down for as
    long as there was travel left.
    """

    def _platform(self, brakes_controllable: bool):
        cfg = safety.bench_config()
        if brakes_controllable:
            from psct_motors.config import BrakeConfig
            for actuator in cfg.actuators:
                actuator.brake = BrakeConfig(mode="output", settle_s=0.0)
        platform = FocalPlanePlatform(cfg=cfg, simulate=True)
        if not brakes_controllable:
            # Exactly the site's situation: no brake this software can drive.
            from psct_motors.external_brake import BrakeController, ExternalBrakeConfig
            platform.external_brake = BrakeController(ExternalBrakeConfig(mode="none"))
        platform.connect()
        self.addCleanup(platform.disconnect)
        return platform

    def test_it_does_not_cut_power_when_nothing_else_is_holding(self):
        from psct_motors.kinematics import Orientation
        from psct_motors.registers import MotorMode

        platform = self._platform(brakes_controllable=False)
        platform.move_to_orientation(Orientation(1.0, 0.0, 0.0))

        result = platform.emergency_stop()

        self.assertTrue(result.stopped)
        self.assertFalse(result.drives_off, result.summary())
        self.assertTrue(result.holding)
        for motor in platform.motors:
            self.assertEqual(motor.get_mode(), int(MotorMode.POSITION),
                             f"{motor.name} was passivated with nothing holding it")

    def test_the_camera_does_not_move_after_an_emergency_stop(self):
        """The symptom itself: watch the axes afterwards and see them stay put.

        The simulated actuators are loaded and fall when nothing holds them,
        so this fails loudly against the old behaviour.
        """
        from psct_motors.kinematics import Orientation

        platform = self._platform(brakes_controllable=False)
        platform.move_to_orientation(Orientation(1.0, 0.0, 0.0))
        platform.emergency_stop()

        settled = platform.read_actuator_positions_mm()
        time.sleep(1.0)
        after = platform.read_actuator_positions_mm()
        drift = max(abs(a - b) for a, b in zip(settled, after))
        self.assertLess(drift, 0.05,
                        f"the focal plane moved {drift:.3f} mm after EMERGENCY")

    def test_it_says_in_plain_words_that_the_drives_are_still_on(self):
        from psct_motors.kinematics import Orientation
        platform = self._platform(brakes_controllable=False)
        platform.move_to_orientation(Orientation(1.0, 0.0, 0.0))
        summary = platform.emergency_stop().summary()
        self.assertIn("drives: ON", summary)
        self.assertIn("sink", summary.lower())
        self.assertIn("brake", summary.lower())

    def test_it_does_cut_power_once_the_brakes_are_confirmed(self):
        """The interlock must not be a blanket refusal -- with brakes that read
        back engaged, EMERGENCY still finishes the job."""
        from psct_motors.kinematics import Orientation
        from psct_motors.registers import MotorMode

        platform = self._platform(brakes_controllable=True)
        platform.move_to_orientation(Orientation(1.0, 0.0, 0.0))
        result = platform.emergency_stop()
        self.assertTrue(result.brakes_engaged, result.summary())
        self.assertTrue(result.drives_off, result.summary())
        for motor in platform.motors:
            self.assertEqual(motor.get_mode(), int(MotorMode.PASSIVE))

    def test_motion_is_halted_before_power_is_touched(self):
        """Passivating a moving shaft takes the power off something turning."""
        from psct_motors.kinematics import Orientation

        platform = self._platform(brakes_controllable=True)
        for actuator in platform.cfg.actuators:
            actuator.velocity_raw = 40
        platform.move_to_orientation(Orientation(0.0, 0.0, 0.0))

        done = threading.Event()

        def mover():
            try:
                platform.move_to_orientation(Orientation(20.0, 0.0, 0.0))
            except Exception:
                pass
            finally:
                done.set()

        threading.Thread(target=mover, daemon=True).start()
        time.sleep(0.7)
        result = platform.emergency_stop()
        done.wait(timeout=20)

        # It stopped where it was, nowhere near the commanded 20 mm.
        self.assertLess(platform.read_orientation().focus_mm, 19.0)
        for motor in platform.motors:
            self.assertLess(abs(motor.get_target_mm() - motor.get_position_mm()), 1.0,
                            f"{motor.name} was left commanded away from where it is")
        self.assertTrue(result.stopped)

    def test_passivate_all_refuses_unless_forced(self):
        from psct_motors.registers import MotorMode
        platform = self._platform(brakes_controllable=False)
        with self.assertRaises(PlatformError) as ctx:
            platform.passivate_all()
        self.assertIn("Refusing to turn the drives off", str(ctx.exception))
        # ...and the escape hatch still works, for a checked-by-hand shutdown.
        self.assertEqual(platform.passivate_all(force=True), [])
        for motor in platform.motors:
            self.assertEqual(motor.get_mode(), int(MotorMode.PASSIVE))


class TestBenchMode(unittest.TestCase):
    """One real motor and two stood in.

    What the site actually has is one motor on a bench. Without this the whole
    three-axis half of the application -- kinematics, coordinated moves, the
    hard-stop search, the emergency interlocks -- could not be exercised
    against real hardware at all until all three were wired.
    """

    def _cfg(self, bench="Top"):
        from psct_motors.cli import apply_bench
        cfg = safety.bench_config()
        apply_bench(cfg, bench)
        return cfg

    def test_only_the_named_motor_is_real(self):
        cfg = self._cfg("Top")
        self.assertFalse(cfg.actuator("Top").simulated)
        self.assertTrue(cfg.actuator("East").simulated)
        self.assertTrue(cfg.actuator("West").simulated)

    def test_the_name_is_matched_case_insensitively(self):
        cfg = self._cfg("top")
        self.assertFalse(cfg.actuator("Top").simulated)

    def test_an_unknown_name_is_refused_rather_than_ignored(self):
        """Silently simulating all three would be the worst outcome: the
        application would look like it was driving hardware."""
        from psct_motors.cli import apply_bench
        with self.assertRaises(ValueError) as ctx:
            apply_bench(safety.bench_config(), "Middle")
        self.assertIn("Middle", str(ctx.exception))
        self.assertIn("Top", str(ctx.exception))

    def test_the_platform_reports_which_axes_are_pretend(self):
        platform = FocalPlanePlatform(cfg=self._cfg("Top"), simulate=False)
        self.assertEqual(set(platform.simulated_names), {"East", "West"})
        self.assertTrue(platform.is_mixed)

    def test_a_fully_simulated_platform_is_not_called_mixed(self):
        platform = FocalPlanePlatform(cfg=safety.bench_config(), simulate=True)
        self.assertEqual(len(platform.simulated_names), 3)
        self.assertFalse(platform.is_mixed)

    def test_bench_mode_still_does_coordinated_moves(self):
        """The point of it: the three-axis code runs, against one real motor."""
        from psct_motors.kinematics import Orientation
        cfg = self._cfg("Top")
        cfg.actuator("Top").simulated = True     # no real motor in a test
        platform = FocalPlanePlatform(cfg=cfg, simulate=False)
        platform.connect()
        self.addCleanup(platform.disconnect)
        platform.move_to_orientation(Orientation(1.0, 0.0, 0.0))
        self.assertAlmostEqual(platform.read_orientation().focus_mm, 1.0, places=2)


class TestSimulatedBoundsAreCoherent(unittest.TestCase):
    """The simulated end stops have to agree with the configured limits.

    They used to be placed from the actuator travel limits, which on a
    configuration whose actuator limits are wider than its focus limits put the
    simulated end of travel far outside everything: the search ran past its
    budget without finding anything, and when it did find something the plate
    was parked well outside the limits and every ordinary move was refused.
    """

    def _platform(self):
        platform = FocalPlanePlatform(cfg=safety.bench_config(), simulate=True)
        platform.connect()
        self.addCleanup(platform.disconnect)
        return platform

    def test_the_stops_sit_just_outside_the_focus_limits(self):
        from psct_motors.platform import SIMULATED_STOP_MARGIN_MM
        platform = self._platform()
        limits = platform.cfg.limits
        for motor in platform.motors:
            high = motor.cfg.counts_to_mm(motor._transport.hard_stop_high)
            low = motor.cfg.counts_to_mm(motor._transport.hard_stop_low)
            self.assertAlmostEqual(high, limits.max_focus_mm + SIMULATED_STOP_MARGIN_MM,
                                   places=3)
            self.assertAlmostEqual(low, limits.min_focus_mm - SIMULATED_STOP_MARGIN_MM,
                                   places=3)
            # The soft limit is what stops an ordinary move first.
            self.assertGreater(high, limits.max_focus_mm)

    def test_the_default_budget_reaches_a_stop(self):
        """A rehearsal that never finds one only ever shows the failure path."""
        platform = self._platform()
        result = platform.seek_hard_stop_together(+1, budget_mm=30.0)
        self.assertTrue(result.stopped_by)

    def test_it_does_not_leave_the_plate_stranded(self):
        platform = self._platform()
        platform.seek_hard_stop_together(+1, budget_mm=30.0)
        focus = platform.read_orientation().focus_mm
        # Backed off from the stop, and within a millimetre of the soft limit
        # rather than stranded far outside it.
        self.assertLess(focus, platform.cfg.limits.max_focus_mm + 1.0)


class TestTheSoftwareCannotStrandItself(unittest.TestCase):
    """Reported: after find-stop, "I wouldn't move it back down from there."

    The search leaves the plate just outside the soft limit by construction.
    Every move back to the middle is then a step larger than the single-step
    limit, so the software refused all of them -- and the only way out was to
    edit the configuration file.
    """

    def _platform(self):
        platform = FocalPlanePlatform(cfg=safety.bench_config(), simulate=True)
        platform.connect()
        self.addCleanup(platform.disconnect)
        return platform

    def test_a_move_back_inside_the_limits_is_always_allowed(self):
        from psct_motors.kinematics import Orientation
        platform = self._platform()
        limits = platform.cfg.limits
        # Where a hard-stop search leaves you: outside, by less than a step.
        platform.seek_hard_stop_together(+1, budget_mm=30.0)
        self.assertGreater(platform.read_orientation().focus_mm,
                           limits.max_focus_mm - 1.0)

        # A full-travel move home is far more than max_step_mm...
        home = (limits.min_focus_mm + limits.max_focus_mm) / 2.0
        self.assertGreater(abs(platform.read_orientation().focus_mm - home),
                           limits.max_step_mm)
        # ...and is allowed anyway, because it ends somewhere legal.
        platform.move_to_orientation(Orientation(home, 0.0, 0.0))
        self.assertAlmostEqual(platform.read_orientation().focus_mm, home,
                               places=2)

    def test_an_oversized_move_inside_the_limits_is_still_refused(self):
        """The step limit still catches a typed mistake."""
        from psct_motors.kinematics import Orientation
        platform = self._platform()
        platform.move_to_orientation(Orientation(0.0, 0.0, 0.0))
        with self.assertRaises(PlatformError) as ctx:
            platform.move_to_orientation(
                Orientation(platform.cfg.limits.max_step_mm * 2, 0.0, 0.0))
        self.assertIn("single-step limit", str(ctx.exception))

    def test_a_move_further_outside_is_still_refused(self):
        """Recovery means coming back, not going further out."""
        from psct_motors.kinematics import Orientation
        platform = self._platform()
        platform.seek_hard_stop_together(+1, budget_mm=30.0)
        with self.assertRaises(PlatformError):
            platform.move_to_orientation(
                Orientation(platform.cfg.limits.max_focus_mm + 10.0, 0.0, 0.0))


class TestTheFoundStopBecomesTheLimit(unittest.TestCase):
    """The soft limits ship as a guess; a hard stop is a measurement."""

    def _platform(self):
        platform = FocalPlanePlatform(cfg=safety.bench_config(), simulate=True)
        platform.connect()
        self.addCleanup(platform.disconnect)
        return platform

    def test_the_upper_limit_follows_the_upper_stop(self):
        platform = self._platform()
        limits = platform.cfg.limits
        platform.adopt_hard_stop(+1, 25.4)
        self.assertAlmostEqual(limits.hard_stop_high_mm, 25.4)
        self.assertAlmostEqual(limits.max_focus_mm,
                               25.4 - limits.safety_margin_mm)

    def test_the_far_end_follows_from_the_known_travel(self):
        """Saves running the search downwards, which is the run that drives
        towards M2 with the camera's weight behind it."""
        platform = self._platform()
        limits = platform.cfg.limits
        limits.total_travel_mm = 50.8
        platform.adopt_hard_stop(+1, 25.4)
        self.assertAlmostEqual(limits.hard_stop_low_mm, 25.4 - 50.8)
        self.assertAlmostEqual(limits.min_focus_mm,
                               limits.hard_stop_low_mm + limits.safety_margin_mm)

    def test_a_measured_far_end_is_not_overwritten_by_the_derived_one(self):
        platform = self._platform()
        limits = platform.cfg.limits
        platform.adopt_hard_stop(-1, -20.0)          # measured
        platform.adopt_hard_stop(+1, 25.4)           # must not derive over it
        self.assertAlmostEqual(limits.hard_stop_low_mm, -20.0)

    def test_it_says_which_end_was_derived_rather_than_measured(self):
        platform = self._platform()
        notes = " ".join(platform.adopt_hard_stop(+1, 25.4))
        self.assertIn("DERIVED", notes)
        self.assertIn("not measured", notes)

    def test_limits_that_leave_no_room_are_refused(self):
        platform = self._platform()
        platform.cfg.limits.total_travel_mm = 0.1
        with self.assertRaises(PlatformError) as ctx:
            platform.adopt_hard_stop(+1, 25.4)
        self.assertIn("no room", str(ctx.exception))

    def test_after_a_search_the_plate_is_inside_the_new_limits(self):
        """The whole point: the run ends somewhere you can move away from."""
        platform = self._platform()
        result = platform.seek_hard_stop_together(+1, budget_mm=30.0)
        platform.adopt_hard_stop(+1, sum(result.stop_mm.values())
                                 / len(result.stop_mm))
        focus = platform.read_orientation().focus_mm
        limits = platform.cfg.limits
        self.assertLessEqual(focus, limits.max_focus_mm + 1e-6)
        self.assertGreaterEqual(focus, limits.min_focus_mm)

    def test_repeated_searches_do_not_walk_the_simulated_machine(self):
        """The stops used to be derived from the limits, which the search then
        moved -- so each rehearsal found the end further out than the last."""
        found = []
        for _ in range(3):
            platform = FocalPlanePlatform(cfg=safety.bench_config(), simulate=True)
            platform.connect()
            try:
                result = platform.seek_hard_stop_together(+1, budget_mm=30.0)
                stop = sum(result.stop_mm.values()) / len(result.stop_mm)
                platform.adopt_hard_stop(+1, stop)
                found.append(round(stop, 3))
            finally:
                platform.disconnect()
        self.assertEqual(len(set(found)), 1, f"the end of travel moved: {found}")


class TestSimulatedSpeed(unittest.TestCase):
    """How fast a stand-in axis moves is a choice, so it is a setting.

    Reported as "in the simulated, it moves really fast": the speed was a
    module constant picked to keep the test suite quick, which made a
    rehearsal finish before anybody could watch it.
    """

    def test_the_default_is_slow_enough_to_watch(self):
        from psct_motors.config import default_config
        cfg = default_config()
        # A 1 mm nudge should take a noticeable fraction of a second, not be
        # over before the gauge redraws.
        self.assertLessEqual(cfg.simulated_speed_mm_per_s, 3.0)
        self.assertGreater(cfg.simulated_speed_mm_per_s, 0.0)

    def test_the_setting_actually_changes_the_speed(self):
        from psct_motors.config import default_config
        speeds = {}
        for requested in (1.0, 8.0):
            cfg = default_config()
            cfg.simulated_speed_mm_per_s = requested
            platform = FocalPlanePlatform(cfg=cfg, simulate=True)
            actuator = cfg.actuators[0]
            transport = platform.motors[0]._transport
            speeds[requested] = (
                actuator.velocity_raw * transport.COUNTS_PER_SECOND_PER_VSOLL
                / actuator.resolved_counts_per_mm
            )
        for requested, actual in speeds.items():
            self.assertAlmostEqual(actual, requested, places=3)

    def test_a_non_positive_speed_is_refused(self):
        from psct_motors.config import default_config
        cfg = default_config()
        cfg.simulated_speed_mm_per_s = 0.0
        with self.assertRaises(ValueError):
            cfg.validate()

    def test_a_move_takes_about_as_long_as_the_speed_says(self):
        from psct_motors.kinematics import Orientation
        cfg = safety.bench_config()
        cfg.simulated_speed_mm_per_s = 10.0
        platform = FocalPlanePlatform(cfg=cfg, simulate=True)
        platform.connect()
        self.addCleanup(platform.disconnect)
        platform.move_to_orientation(Orientation(0.0, 0.0, 0.0))

        started = time.monotonic()
        platform.move_to_orientation(Orientation(5.0, 0.0, 0.0))
        elapsed = time.monotonic() - started
        # 5 mm at 10 mm/s is half a second, plus the settle poll.
        self.assertGreater(elapsed, 0.3)
        self.assertLess(elapsed, 3.0)


class TestSupplyVoltage(unittest.TestCase):
    """Register 97 is in the drive's own raw units and nobody knows the scale.

    The check used to compare it against register 139 ('Acceptance Voltage'),
    which is also raw but not known to be on the same scale. On the pSCT bench
    motor those read 1794 and 2054 -- so a motor running perfectly well at 48 V
    read as "below acceptance" and every move would have been refused.
    """

    def _platform(self, **actuator_kw):
        cfg = safety.bench_config()
        for actuator in cfg.actuators:
            for key, value in actuator_kw.items():
                setattr(actuator, key, value)
        platform = FocalPlanePlatform(cfg=cfg, simulate=True)
        platform.connect()
        self.addCleanup(platform.disconnect)
        return platform

    def test_a_move_is_not_blocked_just_because_97_is_below_139(self):
        """The exact situation on the bench motor: 97 reads 1794 with the
        supply on and healthy at 48 V, while 139 reads 2054."""
        from psct_motors.kinematics import Orientation
        platform = self._platform(supply_nominal_v=48.0,
                                  supply_raw_at_nominal=1794)
        for motor in platform.motors:
            motor._transport.registers[97] = 1794      # as dumped
            motor._transport.registers[139] = 2054     # as dumped
        platform.move_to_orientation(Orientation(1.0, 0.0, 0.0))
        self.assertAlmostEqual(platform.read_orientation().focus_mm, 1.0, places=2)

    def test_without_a_baseline_the_supply_cannot_be_judged(self):
        platform = self._platform(supply_nominal_v=None,
                                  supply_raw_at_nominal=None)
        verdict, explanation = platform.motors[0].supply_is_healthy()
        self.assertIsNone(verdict)
        self.assertIn("cli supply", explanation)

    def test_a_recorded_baseline_gives_volts(self):
        platform = self._platform(supply_nominal_v=48.0,
                                  supply_raw_at_nominal=4485)
        motor = platform.motors[0]
        self.assertAlmostEqual(motor.get_supply_volts(4485), 48.0, places=3)
        self.assertAlmostEqual(motor.get_supply_volts(2242), 24.0, places=1)
        self.assertAlmostEqual(platform.read_state().motors[0].supply_volts,
                               48.0, places=1)

    def test_a_real_supply_failure_is_caught_once_there_is_a_baseline(self):
        from psct_motors.kinematics import Orientation
        platform = self._platform(supply_nominal_v=48.0,
                                  supply_raw_at_nominal=4485)
        for motor in platform.motors:
            motor._transport.set_powered(False)        # drops 97 to 1794
        with self.assertRaises(PlatformError) as ctx:
            platform.move_to_orientation(Orientation(1.0, 0.0, 0.0))
        message = str(ctx.exception)
        self.assertIn("supply has failed", message.lower())
        self.assertIn("19.2 V", message)               # 1794 scaled to volts

    def test_a_reading_a_little_low_is_still_accepted(self):
        """48 V nominal, a few volts of sag, still fine."""
        platform = self._platform(supply_nominal_v=48.0,
                                  supply_raw_at_nominal=4485)
        motor = platform.motors[0]
        for volts in (48.0, 46.0, 45.0, 39.0):
            raw = int(4485 * volts / 48.0)
            verdict, explanation = motor.supply_is_healthy(raw)
            self.assertTrue(verdict, f"{volts} V rejected: {explanation}")
        # ...and 80% of nominal is where it stops being fine.
        verdict, _ = motor.supply_is_healthy(int(4485 * 0.7))
        self.assertFalse(verdict)

    def test_the_pair_must_be_set_together(self):
        from psct_motors.config import default_config
        cfg = default_config()
        cfg.actuators[0].supply_nominal_v = 48.0
        with self.assertRaises(ValueError) as ctx:
            cfg.validate()
        self.assertIn("together", str(ctx.exception))


class TestPollRate(unittest.TestCase):
    def test_a_fast_poll_skips_the_slow_registers(self):
        """Temperature and bus voltage move over minutes. Reading them at the
        position rate doubles the traffic for numbers that have not changed."""
        platform = FocalPlanePlatform(cfg=safety.bench_config(), simulate=True)
        platform.connect()
        self.addCleanup(platform.disconnect)

        # One read first, uncounted: the current limit is read once and cached,
        # so counting from cold would charge that one-off read to the slow set.
        platform.read_state(include_slow=True)

        motor = platform.motors[0]
        reads = []
        original = motor.read_register

        def counted(reg, *a, **k):
            reads.append(reg)
            return original(reg, *a, **k)

        motor.read_register = counted
        platform.read_state(include_slow=True)
        with_slow = len(reads)
        self.assertIn("BUS_VOLTAGE", reads)
        self.assertIn("TEMPERATURE_LOW_RES", reads)

        reads.clear()
        platform.read_state(include_slow=False)
        without_slow = len(reads)
        self.assertNotIn("BUS_VOLTAGE", reads)
        self.assertNotIn("TEMPERATURE_LOW_RES", reads)
        self.assertEqual(with_slow - without_slow, 2)

    def test_the_cached_values_are_still_reported(self):
        platform = FocalPlanePlatform(cfg=safety.bench_config(), simulate=True)
        platform.connect()
        self.addCleanup(platform.disconnect)
        first = platform.read_state(include_slow=True).motors[0]
        second = platform.read_state(include_slow=False).motors[0]
        self.assertIsNotNone(second.bus_voltage)
        self.assertEqual(first.bus_voltage, second.bus_voltage)
        self.assertEqual(first.temperature, second.temperature)

    def test_the_defaults_are_responsive_but_not_reckless(self):
        from psct_motors.config import default_config
        cfg = default_config()
        self.assertLessEqual(cfg.poll_interval_s, 0.2)
        self.assertGreaterEqual(cfg.idle_poll_interval_s, cfg.poll_interval_s)
        self.assertGreater(cfg.slow_poll_every, 1)


class TestSafetyDrills(unittest.TestCase):
    def test_every_drill_passes(self):
        report = safety.run_all()
        self.assertEqual(len(report.results), len(safety.DRILLS))
        failures = [f"{r.name}: {r.what_happened}" for r in report.failures]
        self.assertTrue(report.passed, "\n".join(failures))

    def test_a_drill_fails_when_its_guard_is_removed(self):
        """Otherwise a drill that stopped provoking anything would report PASS
        for ever, and the whole report would be worthless."""
        original = FocalPlanePlatform._check_drive_power
        FocalPlanePlatform._check_drive_power = lambda self: None
        try:
            result = safety.drill_move_refused_without_drive_power()
        finally:
            FocalPlanePlatform._check_drive_power = original
        self.assertFalse(result.passed)

    def test_the_report_serialises(self):
        report = safety.run_all(only=["focus_limit"])
        as_dict = report.as_dict()
        self.assertTrue(as_dict["passed"])
        self.assertEqual(len(as_dict["drills"]), 1)
        self.assertIn("verdict", as_dict["drills"][0])

    def test_the_drills_touch_no_configured_hardware(self):
        """They must be safe to run while connected to the telescope."""
        import psct_motors.transport as transport
        original = transport.PymodbusTransport.connect

        def refuse(self):
            raise AssertionError("a drill tried to open a real Modbus connection")

        transport.PymodbusTransport.connect = refuse
        try:
            safety.run_all(only=["brakes", "power", "focus_limit"])
        finally:
            transport.PymodbusTransport.connect = original


if __name__ == "__main__":
    unittest.main(verbosity=2)
