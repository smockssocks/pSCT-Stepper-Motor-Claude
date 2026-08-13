"""Driver, brake and coordinated-move tests, run against the simulator.

These exercise the real driver code -- the substitution happens at the wire,
so register doubling, word-order packing, mode verification and the move
sequencing are all the production code paths.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from psct_motors.config import (  # noqa: E402
    ActuatorConfig, BrakeConfig, PlatformLimits, default_config,
)
from psct_motors.jvl_motor import BrakeState, JVLMotor, MotorFault  # noqa: E402
from psct_motors.kinematics import Orientation  # noqa: E402
from psct_motors.platform import FocalPlanePlatform, PlatformError  # noqa: E402
from psct_motors.registers import (  # noqa: E402
    MotorMode, WordOrder, decode_bits, describe_errors, int32_to_words,
    modbus_address, words_to_int32,
)
from psct_motors.simulator import SimulatedJVLTransport, simulated_motor  # noqa: E402
from psct_motors.transport import ModbusError  # noqa: E402


# --------------------------------------------------------------------------
# Register encoding
# --------------------------------------------------------------------------

class TestRegisterEncoding(unittest.TestCase):
    def test_modbus_address_is_double_the_register_number(self):
        self.assertEqual(modbus_address(2), 4)     # MODE_REG
        self.assertEqual(modbus_address(3), 6)     # P_SOLL
        self.assertEqual(modbus_address(10), 20)   # P_PROJECTED
        self.assertEqual(modbus_address(35), 70)   # ERR_BITS

    def test_word_order_round_trip(self):
        for order in (WordOrder.LOW_HIGH, WordOrder.HIGH_LOW):
            for value in (0, 1, -1, 12345, -12345, 2 ** 31 - 1, -(2 ** 31), 409600):
                with self.subTest(order=order, value=value):
                    words = int32_to_words(value, order)
                    self.assertEqual(len(words), 2)
                    self.assertTrue(all(0 <= w <= 0xFFFF for w in words))
                    self.assertEqual(words_to_int32(words, order), value)

    def test_word_orders_actually_differ(self):
        low = int32_to_words(0x00010002, WordOrder.LOW_HIGH)
        high = int32_to_words(0x00010002, WordOrder.HIGH_LOW)
        self.assertEqual(low, [0x0002, 0x0001])
        self.assertEqual(high, [0x0001, 0x0002])

    def test_negative_positions_survive(self):
        """Actuator counts go negative below the zero point."""
        words = int32_to_words(-409600, WordOrder.LOW_HIGH)
        self.assertEqual(words_to_int32(words, WordOrder.LOW_HIGH), -409600)

    def test_unsigned_read_of_bit_field(self):
        words = int32_to_words(0x80000001, WordOrder.LOW_HIGH)
        self.assertEqual(words_to_int32(words, WordOrder.LOW_HIGH, signed=False),
                         0x80000001)

    def test_error_decoding(self):
        self.assertEqual(describe_errors(0), "No errors")
        self.assertIn("Temperature too high", describe_errors(1 << 5))
        self.assertIn("unmapped", describe_errors(1 << 30))
        self.assertEqual(decode_bits(0b101, {0: "a", 2: "c"}), ["a", "c"])


# --------------------------------------------------------------------------
# Scaling
# --------------------------------------------------------------------------

class TestScaling(unittest.TestCase):
    def test_measured_scale_wins_over_drivetrain(self):
        cfg = ActuatorConfig(counts_per_mm=1000.0, counts_per_rev=409600.0,
                             gear_ratio=1.0, screw_lead_mm=2.54)
        self.assertEqual(cfg.resolved_counts_per_mm, 1000.0)
        self.assertTrue(cfg.scale_is_measured)

    def test_drivetrain_scale_used_when_unmeasured(self):
        cfg = ActuatorConfig(counts_per_mm=None, counts_per_rev=409600.0,
                             gear_ratio=1.0, screw_lead_mm=2.54)
        self.assertAlmostEqual(cfg.resolved_counts_per_mm, 409600.0 / 2.54, places=6)
        self.assertFalse(cfg.scale_is_measured)

    def test_gear_ratio_multiplies_counts_per_mm(self):
        direct = ActuatorConfig(gear_ratio=1.0).resolved_counts_per_mm
        geared = ActuatorConfig(gear_ratio=10.0).resolved_counts_per_mm
        self.assertAlmostEqual(geared / direct, 10.0, places=9)

    def test_counts_mm_round_trip(self):
        cfg = ActuatorConfig(counts_per_mm=1000.0, zero_counts=123456)
        for mm in (0.0, 1.0, 25.4, -3.0, 49.8):
            self.assertAlmostEqual(cfg.counts_to_mm(cfg.mm_to_counts(mm)), mm, places=6)

    def test_direction_inverts_travel_sense(self):
        fwd = ActuatorConfig(counts_per_mm=1000.0, direction=1)
        rev = ActuatorConfig(counts_per_mm=1000.0, direction=-1)
        self.assertEqual(fwd.mm_to_counts(5.0), -rev.mm_to_counts(5.0))
        self.assertAlmostEqual(fwd.counts_to_mm(5000), 5.0)
        self.assertAlmostEqual(rev.counts_to_mm(5000), -5.0)

    def test_zero_offset_shifts_origin(self):
        cfg = ActuatorConfig(counts_per_mm=1000.0, zero_counts=7000)
        self.assertAlmostEqual(cfg.counts_to_mm(7000), 0.0)
        self.assertAlmostEqual(cfg.counts_to_mm(8000), 1.0)

    def test_unusable_scale_is_rejected(self):
        cfg = ActuatorConfig(counts_per_mm=None, screw_lead_mm=0.0)
        with self.assertRaises(ValueError):
            cfg.resolved_counts_per_mm
        with self.assertRaises(ValueError):
            ActuatorConfig(counts_per_mm=-5.0).resolved_counts_per_mm


# --------------------------------------------------------------------------
# Single motor against the simulator
# --------------------------------------------------------------------------

def bench_actuator(**kw) -> ActuatorConfig:
    """A fast actuator with a simple scale, for tests."""
    params = dict(
        name="T", counts_per_mm=1000.0, velocity_raw=2000,
        min_travel_mm=0.0, max_travel_mm=50.0,
        in_position_tol_mm=0.01, move_timeout_s=10.0,
    )
    params.update(kw)
    return ActuatorConfig(**params)


class TestSingleMotor(unittest.TestCase):
    def setUp(self):
        self.cfg = bench_actuator()
        self.motor = simulated_motor(self.cfg, start_mm=25.0)
        self.motor.connect()

    def tearDown(self):
        self.motor.disconnect()

    def test_connect_and_read_position(self):
        self.assertTrue(self.motor.connected)
        self.assertAlmostEqual(self.motor.get_position_mm(), 25.0, places=3)

    def test_mode_set_and_verify(self):
        self.motor.set_mode(MotorMode.POSITION)
        self.assertEqual(self.motor.get_mode(), int(MotorMode.POSITION))

    def test_mode_verification_catches_a_fighting_client(self):
        """MacTalk holding the motor shows up as a mode that will not stick."""
        transport = self.motor._transport

        original = transport.write_holding

        def revert(address, values):
            original(address, values)
            if address == modbus_address(2):      # MODE_REG
                transport.registers[2] = int(MotorMode.PASSIVE)

        transport.write_holding = revert
        with self.assertRaises(MotorFault) as ctx:
            self.motor.set_mode(MotorMode.POSITION, settle_s=0.01)
        self.assertIn("MacTalk", str(ctx.exception))

    def test_move_completes(self):
        self.motor.ensure_position_mode()
        self.motor.command_position_mm(30.0)
        self.assertTrue(self.motor.wait_for_in_position(timeout_s=10.0))
        self.assertAlmostEqual(self.motor.get_position_mm(), 30.0, places=2)

    def test_travel_limits_reject_out_of_range(self):
        with self.assertRaises(MotorFault):
            self.motor.check_travel_limit(60.0)
        with self.assertRaises(MotorFault):
            self.motor.check_travel_limit(-1.0)
        self.motor.check_travel_limit(25.0)   # inside: no raise

    def test_stop_holds_current_position_rather_than_cutting_drive(self):
        self.motor.ensure_position_mode()
        self.motor.command_position_mm(45.0)
        self.motor.stop()
        # The drive stays enabled -- that is the whole point of a controlled stop.
        self.assertEqual(self.motor.get_mode(), int(MotorMode.POSITION))
        # And the target is now wherever it actually was, not 45 mm.
        self.assertLess(abs(self.motor.get_target_mm() - self.motor.get_position_mm()), 0.5)
        self.assertLess(self.motor.get_target_mm(), 45.0)

    def test_passivate_cuts_drive(self):
        self.motor.ensure_position_mode()
        self.motor.passivate()
        self.assertEqual(self.motor.get_mode(), int(MotorMode.PASSIVE))

    def test_word_order_detection(self):
        self.assertIs(self.motor.detect_word_order(), WordOrder.LOW_HIGH)

    def test_word_order_mismatch_is_caught_on_connect(self):
        """A motor whose words come back the other way must not be used."""
        cfg = bench_actuator(word_order=WordOrder.HIGH_LOW.value)
        transport = SimulatedJVLTransport(word_order=WordOrder.LOW_HIGH)
        motor = JVLMotor(cfg, transport=transport)
        with self.assertRaises(MotorFault) as ctx:
            motor.connect(verify_word_order=True)
        self.assertIn("word order", str(ctx.exception))

    def test_errors_reported_and_cleared(self):
        self.motor._transport.inject_error(1 << 5)
        self.assertNotEqual(self.motor.get_errors(), 0)
        self.assertIn("Temperature", self.motor.error_text())
        self.assertEqual(self.motor.clear_errors(), 0)

    def test_move_raises_when_the_motor_faults_mid_move(self):
        self.motor.ensure_position_mode()
        self.motor.command_position_mm(45.0)
        self.motor._transport.inject_error(1 << 1)
        with self.assertRaises(MotorFault):
            self.motor.wait_for_in_position(timeout_s=5.0)

    def test_status_snapshot_survives_a_dead_link(self):
        self.motor._transport.set_offline(True)
        status = self.motor.read_status()
        self.assertTrue(status.comms_error)
        self.assertFalse(status.healthy)

    def test_set_zero_here_reframes_positions(self):
        self.motor.set_zero_here()
        self.assertAlmostEqual(self.motor.get_position_mm(), 0.0, places=6)

    def test_read_only_registers_are_rejected(self):
        with self.assertRaises(ModbusError):
            self.motor.write_register("P_ENCODER", 0)


# --------------------------------------------------------------------------
# Brake
# --------------------------------------------------------------------------

class TestBrake(unittest.TestCase):
    def test_auto_brake_is_inferred_from_mode(self):
        motor = simulated_motor(bench_actuator(brake=BrakeConfig(mode="auto")),
                                start_mm=25.0)
        motor.connect()
        motor.set_mode(MotorMode.PASSIVE)
        status = motor.get_brake_status()
        self.assertIs(status.state, BrakeState.ENGAGED)
        self.assertTrue(status.inferred)

        motor.set_mode(MotorMode.POSITION)
        self.assertIs(motor.get_brake_status().state, BrakeState.RELEASED)

    def test_auto_brake_cannot_be_commanded(self):
        motor = simulated_motor(bench_actuator(brake=BrakeConfig(mode="auto")),
                                start_mm=25.0)
        motor.connect()
        with self.assertRaises(MotorFault):
            motor.release_brake()
        with self.assertRaises(MotorFault):
            motor.engage_brake()

    def test_output_brake_round_trip(self):
        cfg = bench_actuator(brake=BrakeConfig(mode="output", output_register=19,
                                               output_bit=0, settle_s=0.0))
        motor = simulated_motor(cfg, start_mm=25.0)
        motor.connect()
        motor.ensure_position_mode()

        self.assertIs(motor.get_brake_status().state, BrakeState.ENGAGED)
        motor.release_brake()
        status = motor.get_brake_status()
        self.assertIs(status.state, BrakeState.RELEASED)
        self.assertFalse(status.inferred)      # read back, not deduced
        motor.engage_brake()
        self.assertIs(motor.get_brake_status().state, BrakeState.ENGAGED)

    def test_output_brake_honours_inverted_polarity(self):
        cfg = bench_actuator(brake=BrakeConfig(mode="output", output_bit=0,
                                               energized_releases=False,
                                               settle_s=0.0))
        motor = simulated_motor(cfg, start_mm=25.0)
        motor.connect()
        motor.ensure_position_mode()
        # With inverted polarity a de-energised output means released.
        self.assertIs(motor.get_brake_status().state, BrakeState.RELEASED)
        motor.engage_brake()
        self.assertIs(motor.get_brake_status().state, BrakeState.ENGAGED)

    def test_refuses_to_release_brake_while_drive_is_passive(self):
        """The interlock that stops a loaded axis being dropped."""
        cfg = bench_actuator(brake=BrakeConfig(mode="output", settle_s=0.0))
        motor = simulated_motor(cfg, start_mm=25.0)
        motor.connect()
        motor.set_mode(MotorMode.PASSIVE)
        with self.assertRaises(MotorFault) as ctx:
            motor.release_brake()
        self.assertIn("passive", str(ctx.exception).lower())
        motor.release_brake(force=True)        # explicit override still works

    def test_passivate_engages_brake_first(self):
        cfg = bench_actuator(brake=BrakeConfig(mode="output", settle_s=0.0))
        motor = simulated_motor(cfg, start_mm=25.0)
        motor.connect()
        motor.ensure_position_mode()
        motor.release_brake()
        motor.passivate()
        self.assertEqual(motor.get_mode(), int(MotorMode.PASSIVE))
        self.assertIs(motor.get_brake_status().state, BrakeState.ENGAGED)

    def test_no_brake_configured_reports_unknown(self):
        motor = simulated_motor(bench_actuator(brake=BrakeConfig(mode="none")),
                                start_mm=25.0)
        motor.connect()
        self.assertIs(motor.get_brake_status().state, BrakeState.UNKNOWN)


# --------------------------------------------------------------------------
# Coordinated platform
# --------------------------------------------------------------------------

def bench_platform(**limit_kw) -> FocalPlanePlatform:
    cfg = default_config()
    for i, a in enumerate(cfg.actuators):
        a.counts_per_mm = 1000.0
        a.velocity_raw = 4000
        a.min_travel_mm = 0.0
        a.max_travel_mm = 50.0
        a.in_position_tol_mm = 0.01
        a.move_timeout_s = 20.0
        a.brake = BrakeConfig(mode="output", settle_s=0.0)
    limits = dict(min_focus_mm=0.5, max_focus_mm=49.5, max_tilt_deg=1.0,
                  max_step_mm=20.0, max_tilt_step_deg=1.0)
    limits.update(limit_kw)
    cfg.limits = PlatformLimits(**limits)
    cfg.validate()
    return FocalPlanePlatform(cfg=cfg, simulate=True)


class TestPlatform(unittest.TestCase):
    def setUp(self):
        self.platform = bench_platform()
        self.platform.connect()

    def tearDown(self):
        self.platform.disconnect()

    def test_starts_flat_at_mid_travel(self):
        o = self.platform.read_orientation()
        self.assertAlmostEqual(o.total_tilt_deg, 0.0, places=6)
        self.assertAlmostEqual(o.focus_mm, 25.0, places=2)

    def test_absolute_focus_move(self):
        self.platform.move_to_orientation(Orientation(30.0, 0.0, 0.0))
        o = self.platform.read_orientation()
        self.assertAlmostEqual(o.focus_mm, 30.0, places=2)
        self.assertAlmostEqual(o.total_tilt_deg, 0.0, places=4)

    def test_absolute_tilt_move(self):
        want = Orientation(25.0, 0.2, -0.1)
        self.platform.move_to_orientation(want)
        got = self.platform.read_orientation()
        self.assertAlmostEqual(got.focus_mm, want.focus_mm, places=2)
        self.assertAlmostEqual(got.tip_deg, want.tip_deg, places=4)
        self.assertAlmostEqual(got.tilt_deg, want.tilt_deg, places=4)

    def test_relative_move(self):
        start = self.platform.read_orientation()
        self.platform.move_relative(d_focus_mm=2.0, d_tip_deg=0.05)
        got = self.platform.read_orientation()
        self.assertAlmostEqual(got.focus_mm, start.focus_mm + 2.0, places=2)
        self.assertAlmostEqual(got.tip_deg, start.tip_deg + 0.05, places=4)

    def test_polar_tilt_move(self):
        self.platform.move_to_polar_tilt(25.0, 0.15, 60.0)
        got = self.platform.read_orientation()
        self.assertAlmostEqual(got.total_tilt_deg, 0.15, places=4)
        self.assertAlmostEqual(got.tilt_azimuth_deg, 60.0, places=2)

    def test_focus_limit_refuses_and_moves_nothing(self):
        before = self.platform.read_actuator_positions_mm()
        with self.assertRaises(PlatformError) as ctx:
            self.platform.move_to_orientation(Orientation(90.0, 0.0, 0.0))
        self.assertIn("focus", str(ctx.exception))
        after = self.platform.read_actuator_positions_mm()
        for b, a in zip(before, after):
            self.assertAlmostEqual(b, a, places=3)

    def test_tilt_limit_refuses(self):
        with self.assertRaises(PlatformError) as ctx:
            self.platform.move_to_orientation(Orientation(25.0, 5.0, 0.0))
        self.assertIn("tilt", str(ctx.exception))

    def test_actuator_travel_limit_refuses_before_moving(self):
        """A tilt that is within the angle limit but runs an actuator out of
        travel must still be refused, and refused before anything moves."""
        platform = bench_platform(max_tilt_deg=5.0, max_tilt_step_deg=5.0)
        platform.connect()
        try:
            before = platform.read_actuator_positions_mm()
            with self.assertRaises(PlatformError) as ctx:
                platform.move_to_orientation(Orientation(25.0, 4.0, 0.0))
            self.assertIn("travel limits", str(ctx.exception))
            for b, a in zip(before, platform.read_actuator_positions_mm()):
                self.assertAlmostEqual(b, a, places=3)
        finally:
            platform.disconnect()

    def test_step_limit_refuses_large_jumps(self):
        platform = bench_platform(max_step_mm=1.0)
        platform.connect()
        try:
            with self.assertRaises(PlatformError) as ctx:
                platform.move_to_orientation(Orientation(40.0, 0.0, 0.0))
            self.assertIn("single-step limit", str(ctx.exception))
        finally:
            platform.disconnect()

    def test_multiple_problems_reported_together(self):
        with self.assertRaises(PlatformError) as ctx:
            self.platform.move_to_orientation(Orientation(90.0, 5.0, 0.0))
        message = str(ctx.exception)
        self.assertIn("focus", message)
        self.assertIn("tilt", message)

    def test_move_refused_when_a_motor_is_disconnected(self):
        self.platform.motors[1].disconnect()
        with self.assertRaises(PlatformError) as ctx:
            self.platform.move_to_orientation(Orientation(30.0, 0.0, 0.0))
        self.assertIn("B", str(ctx.exception))

    def test_move_refused_when_a_motor_has_an_error(self):
        self.platform.motors[2]._transport.inject_error(1 << 3)
        with self.assertRaises(PlatformError) as ctx:
            self.platform.move_to_orientation(Orientation(30.0, 0.0, 0.0))
        self.assertIn("active error", str(ctx.exception))

    def test_velocity_synchronisation_scales_by_distance(self):
        """The axis with half the distance to cover gets half the speed."""
        self.platform.move_to_orientation(Orientation(25.0, 0.0, 0.0))
        targets = self.platform.geometry.actuators_from_orientation(
            Orientation(25.0, 0.1, 0.0)
        )
        self.platform._apply_synchronised_velocities(targets)
        current = self.platform.read_actuator_positions_mm()
        deltas = [abs(t - c) for t, c in zip(targets, current)]
        speeds = [m.read_register("V_SOLL") for m in self.platform.motors]
        longest = max(deltas)
        for delta, speed, motor in zip(deltas, speeds, self.platform.motors):
            expected = max(self.platform.cfg.min_velocity_raw,
                           round(motor.cfg.velocity_raw * delta / longest))
            self.assertEqual(speed, expected)

    def test_synchronised_speeds_never_fall_below_the_floor(self):
        targets = list(self.platform.read_actuator_positions_mm())
        targets[0] += 10.0            # one axis moves, two barely do
        self.platform._apply_synchronised_velocities(targets)
        for motor in self.platform.motors:
            self.assertGreaterEqual(motor.read_register("V_SOLL"),
                                    self.platform.cfg.min_velocity_raw)

    def test_brakes_released_automatically_for_a_move(self):
        for motor in self.platform.motors:
            self.assertIs(motor.get_brake_status().state, BrakeState.ENGAGED)
        self.platform.move_to_orientation(Orientation(27.0, 0.0, 0.0))
        for motor in self.platform.motors:
            self.assertIs(motor.get_brake_status().state, BrakeState.RELEASED)

    def test_stop_leaves_all_three_holding(self):
        self.platform.move_to_orientation(Orientation(45.0, 0.0, 0.0), wait=False)
        self.platform.stop()
        for motor in self.platform.motors:
            self.assertEqual(motor.get_mode(), int(MotorMode.POSITION))
            self.assertLess(
                abs(motor.get_target_mm() - motor.get_position_mm()), 0.5
            )

    def test_emergency_passivate_engages_brakes_and_kills_drive(self):
        self.platform.move_to_orientation(Orientation(27.0, 0.0, 0.0))
        self.platform.emergency_passivate()
        for motor in self.platform.motors:
            self.assertEqual(motor.get_mode(), int(MotorMode.PASSIVE))
            self.assertIs(motor.get_brake_status().state, BrakeState.ENGAGED)

    def test_state_snapshot_is_json_ready(self):
        import json
        state = self.platform.read_state()
        self.assertTrue(state.all_connected)
        self.assertTrue(state.orientation_valid)
        json.dumps(state.as_dict())      # must not raise

    def test_orientation_withheld_when_a_motor_is_unreadable(self):
        """Two live positions and one stale one must not become a tilt."""
        self.platform.motors[0]._transport.set_offline(True)
        state = self.platform.read_state()
        self.assertFalse(state.orientation_valid)
        self.assertIsNone(state.orientation)
        self.assertIn("A", state.message)

    def test_single_actuator_move_for_commissioning(self):
        status = self.platform.move_actuator_mm("B", 28.0)
        self.assertAlmostEqual(status.position_mm, 28.0, places=2)

    def test_single_actuator_move_still_honours_travel_limits(self):
        with self.assertRaises(MotorFault):
            self.platform.move_actuator_mm("B", 80.0)

    def test_set_zero_here_rebases_the_orientation(self):
        self.platform.move_to_orientation(Orientation(27.0, 0.05, 0.0))
        self.platform.set_zero_here(persist=False)
        o = self.platform.read_orientation()
        self.assertAlmostEqual(o.focus_mm, 0.0, places=2)
        self.assertAlmostEqual(o.total_tilt_deg, 0.0, places=3)


class TestConfigValidation(unittest.TestCase):
    def test_requires_exactly_three_actuators(self):
        cfg = default_config()
        cfg.actuators = cfg.actuators[:2]
        with self.assertRaises(ValueError) as ctx:
            cfg.validate()
        self.assertIn("three-point", str(ctx.exception))

    def test_rejects_duplicate_names(self):
        cfg = default_config()
        cfg.actuators[1].name = "A"
        with self.assertRaises(ValueError):
            cfg.validate()

    def test_rejects_collinear_actuators(self):
        cfg = default_config()
        for i, a in enumerate(cfg.actuators):
            a.azimuth_deg = 0.0 if i < 2 else 180.0
        with self.assertRaises(ValueError) as ctx:
            cfg.validate()
        self.assertIn("collinear", str(ctx.exception))

    def test_rejects_bad_direction(self):
        cfg = default_config()
        cfg.actuators[0].direction = 2
        with self.assertRaises(ValueError):
            cfg.validate()

    def test_config_round_trips_through_json(self):
        import json
        from psct_motors.config import config_from_dict, config_to_dict
        cfg = default_config()
        cfg.actuators[0].counts_per_mm = 1234.5
        cfg.actuators[0].brake = BrakeConfig(mode="output", output_bit=3)
        restored = config_from_dict(json.loads(json.dumps(config_to_dict(cfg))))
        restored.validate()
        self.assertEqual(restored.actuators[0].counts_per_mm, 1234.5)
        self.assertEqual(restored.actuators[0].brake.mode, "output")
        self.assertEqual(restored.actuators[0].brake.output_bit, 3)

    def test_unknown_config_key_is_reported_not_ignored(self):
        from psct_motors.config import config_from_dict
        with self.assertRaises(ValueError) as ctx:
            config_from_dict({"actuators": [], "nonsense_key": 1})
        self.assertIn("nonsense_key", str(ctx.exception))


if __name__ == "__main__":
    unittest.main(verbosity=2)
