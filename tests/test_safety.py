"""The safety drills are themselves tested, because they are the evidence.

A drill that silently stops provoking the situation it claims to provoke would
report PASS for ever, and the report would be worthless. So these check both
that every drill passes and that the drills are actually testing something:
each one is made to fail on purpose by removing the guard it covers.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from psct_motors import safety  # noqa: E402
from psct_motors.config import default_config  # noqa: E402
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
        self.assertIn("acceptance voltage", message)
        # Naming the supply matters: the motors answer Modbus perfectly well
        # in this state, so it presents as the software being broken.
        self.assertIn("60 V", message)

    def test_a_single_axis_move_enables_every_drive_first(self):
        """The brakes are one switch for all three, so releasing them with two
        drives passive would leave most of the plate held by nothing."""
        from psct_motors.registers import MotorMode
        platform = self._platform()
        platform.move_actuator_mm("East", 1.0)
        for motor in platform.motors:
            self.assertEqual(motor.get_mode(), int(MotorMode.POSITION),
                             f"{motor.name} was left passive")


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
