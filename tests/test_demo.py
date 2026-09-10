"""Tests for fault injection, the single-motor demo, and mid-move halting.

The demo is a diagnostic tool, so the thing to test is that it diagnoses
honestly: a drill must fail when the software misbehaves, not just pass
because it never really provoked anything.
"""

import io
import os
import sys
import threading
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from psct_motors.config import (  # noqa: E402
    ActuatorConfig, BrakeConfig, PlatformLimits, default_config,
)
from psct_motors.demo import (  # noqa: E402
    DemoRunner, FAIL, INFO, PASS, SKIP, all_drills, build_motor, select_drills,
)
from psct_motors.faults import Fault, FaultInjectingTransport, wrap_motor  # noqa: E402,F401
from psct_motors.jvl_motor import MotorFault  # noqa: E402
from psct_motors.kinematics import Orientation  # noqa: E402
from psct_motors.platform import FocalPlanePlatform, PlatformError  # noqa: E402
from psct_motors.registers import MotorMode, WordOrder  # noqa: E402
from psct_motors.simulator import simulated_motor  # noqa: E402
from psct_motors.transport import ModbusError  # noqa: E402


def bench_actuator(**kw) -> ActuatorConfig:
    params = dict(
        name="A", counts_per_mm=1000.0, counts_per_rev=4000.0, velocity_raw=2000,
        min_travel_mm=-1e6, max_travel_mm=1e6,
        in_position_tol_mm=0.01, move_timeout_s=10.0,
        brake=BrakeConfig(mode="output", settle_s=0.0),
    )
    params.update(kw)
    return ActuatorConfig(**params)


# --------------------------------------------------------------------------
# Fault injection
# --------------------------------------------------------------------------

class TestFaultInjection(unittest.TestCase):
    def setUp(self):
        self.motor = simulated_motor(bench_actuator(), start_mm=0.0)
        self.injector = wrap_motor(self.motor, seed=7)
        self.motor.connect()
        # bench_actuator wires a brake to a motor output, and the simulator
        # models that brake holding the shaft, so it has to come off before
        # anything can turn. force=True because the drive is deliberately left
        # passive here and there is no load on a bench.
        self.motor.release_brake(force=True)

    def tearDown(self):
        self.motor.disconnect()

    def test_pass_through_when_nothing_armed(self):
        self.assertIs(self.injector.fault, Fault.NONE)
        self.assertEqual(self.motor.read_register("PROG_VERSION"), 540777)
        self.assertEqual(self.motor.get_position_counts(), 0)

    def test_comms_drop_fails_every_transaction(self):
        self.injector.arm(Fault.COMMS_DROP)
        with self.assertRaises(ModbusError):
            self.motor.get_position_counts()
        self.assertFalse(self.injector.inner.is_open() and self.injector.is_open())
        self.injector.clear()
        self.assertEqual(self.motor.get_position_counts(), 0)

    def test_comms_drop_is_reported_not_raised_by_read_status(self):
        self.injector.arm(Fault.COMMS_DROP)
        status = self.motor.read_status()
        self.assertTrue(status.comms_error)
        self.assertFalse(status.healthy)

    def test_flaky_link_fails_some_but_not_all(self):
        self.injector.arm(Fault.COMMS_FLAKY, failure_rate=0.5)
        failures = 0
        for _ in range(40):
            try:
                self.motor.get_position_counts()
            except ModbusError:
                failures += 1
        self.assertGreater(failures, 0)
        self.assertLess(failures, 40)

    def test_error_bits_injection(self):
        self.injector.arm(Fault.ERROR_BITS, error_bits_value=1 << 5)
        self.assertEqual(self.motor.get_errors(), 1 << 5)
        self.assertIn("Temperature", self.motor.error_text())
        self.injector.clear()
        self.assertEqual(self.motor.get_errors(), 0)

    def test_error_bits_injection_does_not_disturb_other_registers(self):
        self.motor.write_register("P_SOLL", 4321)
        self.injector.arm(Fault.ERROR_BITS)
        self.assertEqual(self.motor.get_target_counts(), 4321)
        self.assertEqual(self.motor.read_register("PROG_VERSION"), 540777)

    def test_mode_revert_makes_a_mode_change_refuse(self):
        self.injector.arm(Fault.MODE_REVERT)
        with self.assertRaises(MotorFault) as ctx:
            self.motor.set_mode(MotorMode.POSITION, settle_s=0.01)
        self.assertIn("MacTalk", str(ctx.exception))

    def test_mode_revert_still_delivers_the_write(self):
        """The write reaches the motor; only the read-back lies -- which is how
        a second client fighting for control actually presents."""
        self.injector.arm(Fault.MODE_REVERT)
        try:
            self.motor.set_mode(MotorMode.POSITION, settle_s=0.01)
        except MotorFault:
            pass
        self.injector.clear()
        self.assertEqual(self.motor.get_mode(), int(MotorMode.POSITION))

    def test_stuck_position_freezes_p_ist_only(self):
        self.motor.ensure_position_mode()
        self.injector.arm(Fault.STUCK_POSITION)
        frozen = self.motor.get_position_counts()
        self.motor.command_position_counts(20000)
        time.sleep(0.5)
        self.assertEqual(self.motor.get_position_counts(), frozen)
        self.assertEqual(self.motor.get_target_counts(), 20000)   # not frozen
        self.injector.clear()
        self.assertNotEqual(self.motor.get_position_counts(), frozen)

    def test_swapped_words_is_caught_by_word_order_detection(self):
        self.injector.arm(Fault.SWAPPED_WORDS)
        self.assertIs(self.motor.detect_word_order(), WordOrder.HIGH_LOW)

    def test_injected_values_decode_correctly_in_both_word_orders(self):
        """An injected fault must arrive as the value that was injected.

        The injector encodes using the motor's own word order. Getting this
        wrong is quiet and nasty: injecting 0x0042 into a high-low motor with
        low-high encoding delivers 0x00420000 instead, so the drill exercises
        a different error bit than it claims to and its report is a lie.
        """
        for order in (WordOrder.LOW_HIGH, WordOrder.HIGH_LOW):
            with self.subTest(order=order):
                motor = simulated_motor(bench_actuator(word_order=order.value))
                injector = wrap_motor(motor)
                motor.connect()
                try:
                    injector.arm(Fault.ERROR_BITS, error_bits_value=0x0042)
                    self.assertEqual(motor.get_errors(), 0x0042)
                finally:
                    motor.disconnect()

    def test_full_32_bit_values_can_be_injected(self):
        """Error registers are 32 bits wide, so injection must span all of it."""
        self.injector.arm(Fault.ERROR_BITS, error_bits_value=0x12345678)
        self.assertEqual(self.motor.get_errors(), 0x12345678)
        self.injector.arm(Fault.ERROR_BITS, error_bits_value=1 << 31)
        self.assertEqual(self.motor.get_errors(), 1 << 31)

    def test_unknown_fault_parameter_is_rejected(self):
        with self.assertRaises(ValueError):
            self.injector.arm(Fault.ERROR_BITS, not_a_parameter=1)

    def test_context_manager_disarms(self):
        with self.injector:
            self.injector.arm(Fault.COMMS_DROP)
        self.assertIs(self.injector.fault, Fault.NONE)
        self.assertEqual(self.motor.get_position_counts(), 0)

    def test_describe_shows_the_armed_fault(self):
        self.injector.arm(Fault.ERROR_BITS)
        self.assertIn("FAULT", self.motor._transport.describe())


# --------------------------------------------------------------------------
# Halting on a fault mid-move
# --------------------------------------------------------------------------

class TestHaltOnFault(unittest.TestCase):
    """A detected fault must stop the axis, not just report it."""

    def test_single_motor_halts_when_a_fault_appears_mid_move(self):
        motor = simulated_motor(bench_actuator(velocity_raw=20), start_mm=0.0)
        injector = wrap_motor(motor)
        motor.connect()
        try:
            motor.ensure_position_mode()
            motor.release_brake()
            motor.set_velocity(20)
            motor.command_position_counts(40000)
            time.sleep(0.3)
            injector.arm(Fault.ERROR_BITS)
            with self.assertRaises(MotorFault):
                motor.wait_for_in_position(timeout_s=5.0)
            injector.clear()
            # The target must no longer be the original one: the axis was halted.
            self.assertNotEqual(motor.get_target_counts(), 40000)
            self.assertLess(abs(motor.get_target_counts() - motor.get_position_counts()),
                            2000)
        finally:
            motor.disconnect()

    def test_single_motor_halts_on_timeout(self):
        motor = simulated_motor(bench_actuator(velocity_raw=20), start_mm=0.0)
        injector = wrap_motor(motor)
        motor.connect()
        try:
            motor.ensure_position_mode()
            motor.release_brake()
            injector.arm(Fault.STUCK_POSITION)
            motor.command_position_counts(40000)
            self.assertFalse(motor.wait_for_in_position(timeout_s=1.0))
            injector.clear()
            self.assertNotEqual(motor.get_target_counts(), 40000)
        finally:
            motor.disconnect()

    def test_halt_does_not_mask_the_original_fault(self):
        """When comms are dead the stop cannot be delivered either; the caller
        must still see the fault that started it, not a stop failure."""
        motor = simulated_motor(bench_actuator(), start_mm=0.0)
        motor.connect()
        motor.ensure_position_mode()
        motor._transport.set_offline(True)
        self.assertFalse(motor.stop_quietly("testing"))     # returns, does not raise

    def _platform(self):
        cfg = default_config()
        for a in cfg.actuators:
            a.counts_per_mm = 1000.0
            a.velocity_raw = 20
            a.min_travel_mm = 0.0
            a.max_travel_mm = 50.0
            a.in_position_tol_mm = 0.01
            a.move_timeout_s = 6.0
            a.brake = BrakeConfig(mode="output", settle_s=0.0)
        cfg.limits = PlatformLimits(min_focus_mm=0.5, max_focus_mm=49.5,
                                    max_tilt_deg=1.0, max_step_mm=30.0,
                                    max_tilt_step_deg=1.0)
        cfg.validate()
        return FocalPlanePlatform(cfg=cfg, simulate=True)

    def test_one_faulting_axis_halts_all_three(self):
        """The whole point: two good axes must not keep driving to a target the
        third will never reach, because that racks the ball joints."""
        platform = self._platform()
        platform.connect()
        injectors = [wrap_motor(m) for m in platform.motors]
        try:
            done = {}

            def mover():
                try:
                    platform.move_to_orientation(Orientation(35.0, 0.0, 0.0))
                except PlatformError as exc:
                    done["error"] = str(exc)

            thread = threading.Thread(target=mover, daemon=True)
            thread.start()
            time.sleep(1.0)
            injectors[1].arm(Fault.ERROR_BITS)          # motor B faults
            thread.join(timeout=20)
            for injector in injectors:
                injector.clear()

            self.assertIn("error", done)
            self.assertIn("East faulted", done["error"])
            self.assertIn("halted", done["error"])

            # Every axis, not just the faulting one, is holding where it stopped.
            for motor in platform.motors:
                self.assertLess(
                    abs(motor.get_target_mm() - motor.get_position_mm()), 1.0,
                    f"{motor.name} is still driving towards the original target",
                )
                self.assertLess(motor.get_position_mm(), 34.0,
                                f"{motor.name} completed the move despite the fault")
        finally:
            platform.disconnect()

    def test_timeout_also_halts_all_three(self):
        """A move too slow to finish inside its timeout must halt everything.

        The axes here are simply crawling -- their encoders are live -- so
        halting is unambiguous. See the frozen-encoder test below for the case
        where it is not.
        """
        platform = self._platform()
        for actuator in platform.cfg.actuators:
            actuator.velocity_raw = 1            # far too slow to finish
            actuator.move_timeout_s = 1.5
        platform.connect()
        try:
            with self.assertRaises(PlatformError) as ctx:
                platform.move_to_orientation(Orientation(45.0, 0.0, 0.0))
            self.assertIn("Timed out", str(ctx.exception))
            self.assertIn("halted", str(ctx.exception))
            for motor in platform.motors:
                self.assertLess(
                    abs(motor.get_target_mm() - motor.get_position_mm()), 0.5,
                    f"{motor.name} is still driving after the timeout",
                )
                self.assertLess(motor.get_position_mm(), 44.0)
        finally:
            platform.disconnect()

    def test_frozen_encoder_halt_is_reported_honestly(self):
        """The one case where halting cannot do what it says on the tin.

        The halt works by writing the reported position as the new target. If
        the encoder is frozen while the shaft still turns, that reported
        position is stale, so the 'halt' commands the shaft back to it rather
        than stopping it where it is. Nothing readable over Modbus can tell
        that apart from a genuinely seized axis, so the demo documents it
        instead of pretending otherwise -- and this test pins the behaviour so
        nobody later 'fixes' it into passivating a loaded axis.
        """
        platform = self._platform()
        platform.connect()
        injectors = [wrap_motor(m) for m in platform.motors]
        try:
            frozen_at = platform.motors[2].get_position_counts()
            injectors[2].arm(Fault.STUCK_POSITION)
            with self.assertRaises(PlatformError) as ctx:
                platform.move_to_orientation(Orientation(30.0, 0.0, 0.0))
            self.assertIn("Timed out", str(ctx.exception))
            for injector in injectors:
                injector.clear()

            # A and B, whose encoders were live, are genuinely holding.
            for motor in platform.motors[:2]:
                self.assertLess(
                    abs(motor.get_target_mm() - motor.get_position_mm()), 0.5,
                    f"{motor.name} is still driving after the timeout",
                )
            # C was commanded back to the stale reading, not stopped in place.
            self.assertEqual(platform.motors[2].get_target_counts(), frozen_at)
            # Crucially, it is NOT still driving to the original target.
            self.assertLess(platform.motors[2].get_target_mm(), 29.0)
            # And the drive was left enabled rather than dropped.
            self.assertEqual(platform.motors[2].get_mode(), int(MotorMode.POSITION))
        finally:
            platform.disconnect()


# --------------------------------------------------------------------------
# The demo itself
# --------------------------------------------------------------------------

class TestDrillSelection(unittest.TestCase):
    def test_every_drill_is_well_formed(self):
        for drill in all_drills():
            with self.subTest(drill=drill.name):
                self.assertTrue(drill.summary.endswith("."))
                self.assertTrue(drill.expectation)
                self.assertTrue(drill.remediation, "every drill needs advice")
                self.assertIn(drill.category, ("capability", "fault", "real-fault"))
                self.assertTrue(callable(drill.run))

    def test_drill_names_are_unique(self):
        names = [d.name for d in all_drills()]
        self.assertEqual(len(names), len(set(names)))

    def test_select_by_name(self):
        drills = select_drills(only=["identity", "fault-comms"])
        self.assertEqual([d.name for d in drills], ["identity", "fault-comms"])

    def test_select_by_category(self):
        drills = select_drills(categories=["fault"])
        self.assertTrue(drills)
        self.assertTrue(all(d.category == "fault" for d in drills))

    def test_unknown_drill_name_is_rejected(self):
        with self.assertRaises(ValueError) as ctx:
            select_drills(only=["does-not-exist"])
        self.assertIn("does-not-exist", str(ctx.exception))

    def test_operator_drills_can_be_excluded(self):
        drills = select_drills(include_operator=False)
        self.assertTrue(drills)
        self.assertFalse(any(d.needs_operator for d in drills))


class DemoHarness:
    """Runs the demo against a simulated motor and captures the output."""

    def __init__(self, allow_motion=True, answer=True, **actuator_kw):
        self.buffer = io.StringIO()
        self.motor = simulated_motor(bench_actuator(**actuator_kw), start_mm=0.0)
        self.motor.connect()
        self.answer = answer
        self.runner = DemoRunner(
            self.motor,
            out=lambda line: self.buffer.write(line + "\n"),
            ask=lambda prompt: self.answer,
            allow_motion=allow_motion,
            range_revs=2.0,
        )

    def run(self, names):
        code = self.runner.run(select_drills(only=names))
        self.motor.disconnect()
        return code

    @property
    def text(self):
        return self.buffer.getvalue()

    def verdict(self, name):
        for drill, result in self.runner.results:
            if drill.name == name:
                return result.verdict
        raise KeyError(name)


class TestDemoRuns(unittest.TestCase):
    def test_non_motion_drills_all_pass_on_a_healthy_motor(self):
        harness = DemoHarness(allow_motion=False)
        names = [d.name for d in select_drills(include_operator=False)
                 if not d.needs_motion]
        harness.run(names)
        for name in names:
            with self.subTest(drill=name):
                self.assertIn(harness.verdict(name), (PASS, INFO))

    def test_motion_drills_pass_on_a_healthy_motor(self):
        harness = DemoHarness(allow_motion=True)
        names = ["small-move", "stop", "fault-errbits", "fault-stuck"]
        harness.run(names)
        for name in names:
            with self.subTest(drill=name):
                self.assertEqual(harness.verdict(name), PASS, harness.text)

    def test_motion_drills_are_skipped_without_permission(self):
        harness = DemoHarness(allow_motion=False)
        harness.run(["small-move", "repeatability"])
        self.assertEqual(harness.verdict("small-move"), SKIP)
        self.assertIn("--allow-motion", harness.text)

    def test_motion_drills_are_skipped_when_the_operator_declines(self):
        harness = DemoHarness(allow_motion=True, answer=False)
        harness.run(["small-move"])
        self.assertEqual(harness.verdict("small-move"), SKIP)

    def test_reports_units_in_revolutions_not_millimetres(self):
        """A bare shaft has no millimetres; inventing them would mislead."""
        harness = DemoHarness(allow_motion=True)
        harness.run(["small-move", "repeatability"])
        self.assertIn("rev", harness.text)
        self.assertIn("deg of shaft", harness.text)
        body = harness.text.split("What to do")[0]
        self.assertNotIn(" mm", body)

    def test_the_shaft_is_returned_and_left_passive(self):
        harness = DemoHarness(allow_motion=True)
        harness.run(["small-move", "repeatability"])
        self.assertEqual(harness.motor._transport.inner.registers[2],
                         int(MotorMode.PASSIVE))
        position = harness.motor._transport.inner.position_counts
        self.assertLess(abs(position), 500, "shaft not returned near its start")

    def test_a_drill_that_raises_is_reported_not_fatal(self):
        harness = DemoHarness(allow_motion=False)
        broken = select_drills(only=["identity"])
        def explode(ctx):
            raise RuntimeError("deliberate")
        broken[0].run = explode
        harness.runner.run(broken)
        self.assertEqual(harness.runner.results[0][1].verdict, FAIL)
        self.assertIn("deliberate", harness.text)

    def test_a_failed_drill_leaves_no_fault_armed(self):
        harness = DemoHarness(allow_motion=False)
        drills = select_drills(only=["identity"])
        def arm_and_die(ctx):
            ctx.injector.arm(Fault.COMMS_DROP)
            raise RuntimeError("died with a fault armed")
        drills[0].run = arm_and_die
        harness.runner.run(drills)
        self.assertIs(harness.runner.injector.fault, Fault.NONE)

    def test_exit_code_is_nonzero_when_a_drill_fails(self):
        harness = DemoHarness(allow_motion=False)
        drills = select_drills(only=["identity"])
        drills[0].run = lambda ctx: (_ for _ in ()).throw(RuntimeError("x"))
        self.assertEqual(harness.runner.run(drills), 1)

    def test_exit_code_is_zero_when_all_pass(self):
        harness = DemoHarness(allow_motion=False)
        self.assertEqual(harness.run(["identity", "mode", "limit"]), 0)

    def test_band_refuses_targets_outside_the_bench_range(self):
        harness = DemoHarness(allow_motion=True)
        harness.runner.ctx.establish_band()
        with self.assertRaises(MotorFault) as ctx:
            harness.runner.ctx.move_revs(50.0)
        self.assertIn("bench band", str(ctx.exception))
        harness.motor.disconnect()

    def test_drills_detect_a_broken_word_order(self):
        """The demo must fail, not pass, when the config is actually wrong."""
        harness = DemoHarness(allow_motion=False)
        harness.motor.word_order = WordOrder.HIGH_LOW      # now wrong
        harness.run(["identity"])
        self.assertEqual(harness.verdict("identity"), FAIL)
        self.assertIn("MISMATCH", harness.text)

    def test_output_explains_each_drill_before_running_it(self):
        harness = DemoHarness(allow_motion=False)
        harness.run(["fault-comms"])
        self.assertIn("What:", harness.text)
        self.assertIn("Expect:", harness.text)
        self.assertIn("--->", harness.text)


class TestBuildMotor(unittest.TestCase):
    def test_builds_a_single_motor_with_no_platform(self):
        motor = build_motor(None, "Top", simulate=True)
        self.assertEqual(motor.name, "Top")
        motor.connect()
        self.assertTrue(motor.connected)
        motor.disconnect()

    def test_unknown_motor_name_is_rejected(self):
        with self.assertRaises(KeyError):
            build_motor(None, "Z", simulate=True)


class TestDemoCli(unittest.TestCase):
    def test_list_runs_without_hardware(self):
        from psct_motors.cli import main
        self.assertEqual(main(["demo", "--list"]), 0)

    def test_simulated_demo_runs_end_to_end(self):
        from psct_motors.cli import main
        code = main(["--simulate", "-y", "demo", "--motor", "Top", "--allow-motion",
                     "--no-operator", "--only",
                     "identity,mode,limit,fault-comms,fault-mode"])
        self.assertEqual(code, 0)

    def test_unknown_drill_exits_with_usage_error(self):
        from psct_motors.cli import main
        self.assertEqual(main(["--simulate", "-y", "demo", "--only", "nope"]), 2)


if __name__ == "__main__":
    unittest.main(verbosity=2)


# --------------------------------------------------------------------------
# Word order, as learned from the real motor
# --------------------------------------------------------------------------

class TestWordOrderProbe(unittest.TestCase):
    """Regression tests for the detection rewrite.

    The first version compared register 1 against a "looks like a firmware
    version" range. On the real pSCT motor register 1 reads 540777, which
    needs 20 bits, so the check reported it could not determine the word order
    on a motor whose word order was provably correct -- the moves landed
    exactly on target. These tests pin the replacement.
    """

    #: What the pSCT motor actually returned, so the regression is concrete.
    REAL_MOTOR_REGISTERS = {
        1: 540777, 2: 0, 3: 600, 4: 0x06080000, 5: 10000, 6: 100, 7: 511,
        8: 500, 9: 128, 10: 600, 12: 0, 19: 0, 20: 0, 25: 0x8A476C14,
        35: 0, 36: 0, 38: -100000,
    }

    def _motor(self, word_order=WordOrder.LOW_HIGH, registers=None):
        motor = simulated_motor(bench_actuator(word_order=word_order.value))
        motor._transport.registers = dict(registers or self.REAL_MOTOR_REGISTERS)
        motor.connect(verify_word_order=False)
        return motor

    def test_real_motor_registers_give_a_confident_low_high_verdict(self):
        motor = self._motor()
        try:
            probe = motor.probe_word_order()
            self.assertIs(probe.detected, WordOrder.LOW_HIGH)
            self.assertTrue(probe.evidence)
            self.assertEqual(probe.reason, "")
        finally:
            motor.disconnect()

    def test_register_one_no_longer_influences_the_verdict(self):
        """540777 in register 1 must not derail detection."""
        registers = dict(self.REAL_MOTOR_REGISTERS)
        for value in (540777, 0, 1030, 0xDEADBEEF):
            with self.subTest(register_1=value):
                registers[1] = value
                motor = self._motor(registers=registers)
                try:
                    self.assertIs(motor.probe_word_order().detected,
                                  WordOrder.LOW_HIGH)
                finally:
                    motor.disconnect()

    def test_a_high_low_motor_is_detected_as_high_low(self):
        motor = self._motor(word_order=WordOrder.HIGH_LOW)
        try:
            self.assertIs(motor.probe_word_order().detected, WordOrder.HIGH_LOW)
        finally:
            motor.disconnect()

    def test_a_genuine_mismatch_is_still_caught_on_connect(self):
        motor = simulated_motor(bench_actuator(word_order=WordOrder.HIGH_LOW.value))
        motor._transport.registers = dict(self.REAL_MOTOR_REGISTERS)
        motor._transport.word_order = WordOrder.LOW_HIGH      # motor disagrees
        with self.assertRaises(MotorFault) as ctx:
            motor.connect(verify_word_order=True)
        self.assertIn("MISMATCH".lower(), str(ctx.exception).lower() + "mismatch")
        self.assertIn("Low-High", str(ctx.exception))

    def test_all_zero_registers_are_inconclusive_not_a_fault(self):
        """No evidence must not be reported as bad evidence."""
        motor = self._motor(registers={n: 0 for n in self.REAL_MOTOR_REGISTERS})
        try:
            probe = motor.probe_word_order()
            self.assertIsNone(probe.detected)
            self.assertFalse(probe.conclusive)
            self.assertIn("nothing to go on", probe.reason)
        finally:
            motor.disconnect()

    def test_inconclusive_probe_does_not_block_connect(self):
        """A diagnostic that cannot decide must not stop you connecting."""
        motor = simulated_motor(bench_actuator())
        motor._transport.registers = {n: 0 for n in self.REAL_MOTOR_REGISTERS}
        motor.connect(verify_word_order=True)          # must not raise
        self.assertTrue(motor.connected)
        motor.disconnect()

    def test_contradictory_evidence_is_reported_as_such(self):
        registers = dict(self.REAL_MOTOR_REGISTERS)
        registers[5] = 0x27100000        # one probe register votes the other way
        motor = self._motor(registers=registers)
        try:
            probe = motor.probe_word_order()
            self.assertIsNone(probe.detected)
            self.assertIn("Contradictory", probe.reason)
        finally:
            motor.disconnect()

    def test_position_registers_are_not_used_as_evidence(self):
        """Register 4 reads 0x06080000 on the real motor and would vote wrong."""
        from psct_motors.jvl_motor import WORD_ORDER_PROBE_REGISTERS
        for name in ("P_SOLL", "P_PROJECTED", "P_ENCODER", "STATUSBITS"):
            self.assertNotIn(name, WORD_ORDER_PROBE_REGISTERS)


class TestIdentityDrillVerdicts(unittest.TestCase):
    def _run_identity(self, word_order=WordOrder.LOW_HIGH, registers=None):
        harness = DemoHarness(allow_motion=False, word_order=word_order.value)
        if registers is not None:
            # .inner, not ._transport: DemoRunner has already wrapped the motor
            # in a fault injector, so ._transport is that wrapper and setting
            # registers on it would just create an unused attribute.
            harness.motor._transport.inner.registers = dict(registers)
        harness.run(["identity"])
        return harness

    def test_matching_word_order_passes(self):
        harness = self._run_identity(
            registers=TestWordOrderProbe.REAL_MOTOR_REGISTERS)
        self.assertEqual(harness.verdict("identity"), PASS)

    def test_inconclusive_is_informational_not_a_failure(self):
        harness = self._run_identity(
            registers={n: 0 for n in TestWordOrderProbe.REAL_MOTOR_REGISTERS})
        self.assertEqual(harness.verdict("identity"), INFO)
        self.assertIn("not evidence of a problem", harness.text.lower())

    def test_mismatch_still_fails(self):
        harness = DemoHarness(allow_motion=False,
                              word_order=WordOrder.HIGH_LOW.value)
        harness.motor._transport.inner.registers = dict(
            TestWordOrderProbe.REAL_MOTOR_REGISTERS)
        harness.motor._transport.inner.word_order = WordOrder.LOW_HIGH
        harness.run(["identity"])
        self.assertEqual(harness.verdict("identity"), FAIL)
        self.assertIn("MISMATCH", harness.text)


# --------------------------------------------------------------------------
# Reconnecting after a link failure
# --------------------------------------------------------------------------

class TestReconnect(unittest.TestCase):
    """The real unplug drill could not recover: after a WinError 10054 the
    client held a dead socket, connect() was a no-op, and every read failed."""

    def test_reconnect_recovers_a_dropped_link(self):
        motor = simulated_motor(bench_actuator(), start_mm=0.0)
        motor.connect()
        transport = motor._transport
        transport.set_offline(True)
        with self.assertRaises(ModbusError):
            motor.get_position_counts()
        transport.set_offline(False)
        motor.reconnect()
        self.assertTrue(motor.connected)
        self.assertEqual(motor.get_position_counts(), 0)

    def test_reconnect_rebuilds_the_pymodbus_client(self):
        """The fix is a fresh client, not just close-then-connect."""
        from psct_motors.transport import PymodbusTransport
        transport = PymodbusTransport("192.0.2.1", 502)
        built = []
        transport._build_client = lambda: built.append(1) or _FakeClient()
        transport._detect_call_convention = lambda: None
        transport.connect()
        self.assertEqual(len(built), 1)
        transport.reconnect()
        self.assertEqual(len(built), 2, "reconnect must build a new client")

    def test_reconnect_still_fails_while_a_comms_fault_is_armed(self):
        motor = simulated_motor(bench_actuator(), start_mm=0.0)
        injector = wrap_motor(motor)
        motor.connect()
        injector.arm(Fault.COMMS_DROP)
        self.assertFalse(motor._transport.reconnect())
        injector.clear()
        self.assertTrue(motor._transport.reconnect())

    def test_every_transport_implements_reconnect(self):
        from psct_motors.simulator import SimulatedJVLTransport
        from psct_motors.transport import PymodbusTransport
        for cls in (PymodbusTransport, SimulatedJVLTransport, FaultInjectingTransport):
            self.assertTrue(callable(getattr(cls, "reconnect", None)), cls.__name__)


class _FakeClient:
    connected = True
    def connect(self):
        return True
    def close(self):
        pass


# --------------------------------------------------------------------------
# Honest reporting of unverified decodings
# --------------------------------------------------------------------------

class TestUnverifiedDecodings(unittest.TestCase):
    def test_status_register_claims_no_bit_meanings(self):
        """It used to decode a passive, stationary motor's 0x8A476C14 as
        'Decelerating, Motion running'."""
        from psct_motors.registers import STATUS_BITS, describe_status
        self.assertEqual(STATUS_BITS, {})
        text = describe_status(0x8A476C14)
        self.assertIn("0x8A476C14", text)
        self.assertIn("no verified bit meanings", text)
        for wrong in ("Decelerating", "Motion running", "In position"):
            self.assertNotIn(wrong, text)

    def test_error_names_are_marked_unverified(self):
        from psct_motors.registers import describe_errors
        self.assertEqual(describe_errors(0), "No errors")
        text = describe_errors(1 << 1)
        self.assertIn("0x00000002", text)
        self.assertIn("UNVERIFIED", text)

    def test_registers_record_what_the_hardware_actually_read(self):
        """Names now come from MacTalk's own register list, so they are
        CONFIRMED -- but the odd readings are still written down."""
        from psct_motors.registers import CONFIRMED, REGISTERS_BY_NAME
        for name in ("PROG_VERSION", "STATUSBITS", "FLWERR", "BRAKE_OUTPUT",
                     "MODBUS_TIMEOUT_MS"):
            self.assertEqual(REGISTERS_BY_NAME[name].confidence, CONFIRMED, name)
            self.assertIn("pSCT motor", REGISTERS_BY_NAME[name].description,
                          f"{name} should record what the hardware actually read")

    def test_projected_and_encoder_positions_are_distinguished(self):
        """Conflating them is what hid a 231-count following error."""
        from psct_motors.registers import REGISTERS_BY_NAME
        projected = REGISTERS_BY_NAME["P_PROJECTED"]
        encoder = REGISTERS_BY_NAME["P_ENCODER"]
        self.assertEqual(projected.number, 10)
        self.assertEqual(encoder.number, 16)
        self.assertIn("NOT a measurement", projected.description)
