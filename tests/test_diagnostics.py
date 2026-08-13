"""Tests for the event log and the "why is it not moving" diagnostics.

These target one reported symptom: a motor that works and then stops taking
position commands. The value of both tools is that they name the cause, so the
tests set up each cause in turn and check it is named.
"""

import json
import os
import sys
import tempfile
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from psct_motors.config import ActuatorConfig, BrakeConfig  # noqa: E402
from psct_motors.diagnostics import BLOCKING, diagnose  # noqa: E402
from psct_motors.eventlog import (  # noqa: E402
    DEBUG, ERROR, WARNING, EventLog, MotorWatcher, read_log,
)
from psct_motors.faults import Fault, wrap_motor  # noqa: E402
from psct_motors.registers import MotorMode  # noqa: E402
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


def healthy_motor(**kw):
    motor = simulated_motor(bench_actuator(**kw), start_mm=0.0)
    motor.connect()
    motor.set_mode(MotorMode.POSITION)
    motor.release_brake()          # bench_actuator sets brake.mode="output"
    return motor


# --------------------------------------------------------------------------
# Diagnostics
# --------------------------------------------------------------------------

class TestDiagnose(unittest.TestCase):
    def titles(self, diagnosis, verdict):
        return [f.title for f in diagnosis.findings if f.verdict == verdict]

    def test_healthy_motor_has_no_blockers(self):
        motor = healthy_motor()
        try:
            result = diagnose(motor)
            self.assertTrue(result.healthy, result.as_text())
            self.assertEqual(result.blockers, [])
        finally:
            motor.disconnect()

    def test_a_latched_supply_dip_is_a_suspect_not_a_blocker(self):
        """The simulator carries the real motor's Bus Voltage Min of 565
        against a bus of 1794. That is worth surfacing -- it is the only
        record of a brown-out once the error bits have been cleared -- but it
        does not stop the motor moving, so it must not be reported as if it
        did."""
        motor = healthy_motor()
        try:
            result = diagnose(motor)
            titles = [f.title for f in result.suspects]
            self.assertIn("Supply has been much lower than it is now", titles)
            self.assertTrue(result.healthy)
        finally:
            motor.disconnect()

    def test_supply_check_does_not_compare_across_unknown_scales(self):
        """Register 139 reads 2054 while the bus reads 1794 on a healthy
        motor, so they are not on a common scale and must not be compared."""
        motor = healthy_motor()
        try:
            motor._transport.registers[98] = motor._transport.registers[97]
            result = diagnose(motor)
            titles = [f.title for f in result.suspects]
            self.assertNotIn("Supply has been much lower than it is now", titles)
        finally:
            motor.disconnect()

    def test_a_perfectly_healthy_motor_reports_nothing_at_all(self):
        motor = healthy_motor()
        motor._transport.registers[98] = motor._transport.registers[97]
        try:
            result = diagnose(motor)
            self.assertEqual(result.suspects, [], result.as_text())
            self.assertIn("Nothing found", result.summary())
        finally:
            motor.disconnect()

    def test_drive_position_limits_block_when_outside(self):
        motor = healthy_motor()
        motor._transport.registers[28] = 500000
        motor._transport.registers[30] = 600000
        try:
            self.assertIn("Outside the drive's position limits",
                          self.titles(diagnose(motor), BLOCKING))
        finally:
            motor.disconnect()

    def test_armed_modbus_watchdog_is_flagged(self):
        """A drive that changes state by itself when polling stops looks
        exactly like a motor spontaneously refusing commands."""
        motor = healthy_motor()
        motor._transport.registers[199] = 500
        motor._transport.registers[200] = 1
        try:
            result = diagnose(motor)
            self.assertIn("Modbus watchdog is armed",
                          [f.title for f in result.suspects])
        finally:
            motor.disconnect()

    def test_large_follow_error_is_flagged_but_a_normal_one_is_not(self):
        """231 counts is normal on this motor; flagging any non-zero value
        would cry wolf on every healthy motor."""
        motor = healthy_motor()
        motor._transport.follow_error_counts = 231
        try:
            titles = [f.title for f in diagnose(motor).suspects]
            self.assertNotIn("Following error is large", titles)
            motor._transport.follow_error_counts = 9000
            titles = [f.title for f in diagnose(motor).suspects]
            self.assertIn("Following error is large", titles)
        finally:
            motor.disconnect()

    def test_passive_drive_is_named(self):
        """The single most common cause of 'stopped taking commands'."""
        motor = healthy_motor()
        motor.write_register("MODE_REG", int(MotorMode.PASSIVE))
        try:
            result = diagnose(motor)
            self.assertFalse(result.healthy)
            self.assertIn("Drive is passive", self.titles(result, BLOCKING))
            blocker = result.blockers[0]
            self.assertIn("succeed and are ignored", blocker.detail.lower())
            self.assertTrue(blocker.remedy)
        finally:
            motor.disconnect()

    def test_zero_velocity_is_named(self):
        """Accepts the target, approaches it at zero speed, reports nothing."""
        motor = healthy_motor()
        motor.write_register("V_SOLL", 0)
        try:
            result = diagnose(motor)
            self.assertIn("Velocity limit is zero", self.titles(result, BLOCKING))
        finally:
            motor.disconnect()

    def test_zero_acceleration_is_named(self):
        motor = healthy_motor()
        motor.write_register("A_SOLL", 0)
        try:
            self.assertIn("Acceleration is zero", self.titles(result := diagnose(motor),
                                                              BLOCKING), result.as_text())
        finally:
            motor.disconnect()

    def test_zero_run_current_is_named(self):
        motor = healthy_motor()
        motor.write_register("RUN_CURRENT", 0)
        try:
            self.assertIn("Run current is zero",
                          self.titles(diagnose(motor), BLOCKING))
        finally:
            motor.disconnect()

    def test_error_bits_are_named(self):
        motor = healthy_motor()
        injector = wrap_motor(motor)
        injector.arm(Fault.ERROR_BITS, error_bits_value=1 << 5)
        try:
            result = diagnose(motor)
            self.assertIn("Error bits set", self.titles(result, BLOCKING))
        finally:
            injector.clear()
            motor.disconnect()

    def test_engaged_brake_is_named(self):
        motor = healthy_motor()
        motor.engage_brake()
        try:
            self.assertIn("Brake is engaged", self.titles(diagnose(motor), BLOCKING))
        finally:
            motor.disconnect()

    def test_dead_link_short_circuits_with_one_clear_answer(self):
        motor = healthy_motor()
        motor._transport.set_offline(True)
        result = diagnose(motor)
        self.assertEqual(len(result.findings), 1)
        self.assertEqual(result.findings[0].verdict, BLOCKING)
        self.assertIn("not answering", result.findings[0].detail)
        self.assertIn("Reconnect", result.findings[0].remedy)

    def test_several_causes_are_all_reported(self):
        motor = healthy_motor()
        motor.write_register("MODE_REG", int(MotorMode.PASSIVE))
        motor.write_register("V_SOLL", 0)
        try:
            result = diagnose(motor)
            blockers = self.titles(result, BLOCKING)
            self.assertIn("Drive is passive", blockers)
            self.assertIn("Velocity limit is zero", blockers)
            self.assertIn("and 1 more", result.summary())
        finally:
            motor.disconnect()

    def test_write_probe_detects_a_target_being_overwritten(self):
        """A second client stamping on P_SOLL is otherwise invisible."""
        motor = healthy_motor()
        transport = motor._transport
        original_write = transport.write_holding

        def clobber(address, values):
            original_write(address, values)
            if address == 6:                    # P_SOLL
                transport.registers[3] = 999999

        transport.write_holding = clobber
        try:
            result = diagnose(motor, probe_writes=True)
            self.assertIn("Writes are not sticking", self.titles(result, BLOCKING))
            finding = next(f for f in result.blockers
                           if f.title == "Writes are not sticking")
            self.assertIn("second client", finding.remedy)
        finally:
            transport.write_holding = original_write
            motor.disconnect()

    def test_write_probe_commands_the_current_position_only(self):
        """The probe must never be able to cause motion."""
        motor = healthy_motor()
        try:
            before = motor.get_position_counts()
            diagnose(motor, probe_writes=True)
            self.assertEqual(motor.get_target_counts(), before)
            time.sleep(0.3)
            self.assertEqual(motor.get_position_counts(), before)
        finally:
            motor.disconnect()

    def test_write_probe_can_be_disabled(self):
        motor = healthy_motor()
        try:
            motor.command_position_counts(1234)
            diagnose(motor, probe_writes=False)
            self.assertEqual(motor.get_target_counts(), 1234)
        finally:
            motor.disconnect()

    def test_output_is_json_and_text_ready(self):
        motor = healthy_motor()
        try:
            result = diagnose(motor)
            json.dumps(result.as_dict())
            self.assertIn("[OK", result.as_text())
        finally:
            motor.disconnect()

    def test_every_blocking_finding_carries_a_remedy(self):
        for setup in (
            lambda m: m.write_register("MODE_REG", 0),
            lambda m: m.write_register("V_SOLL", 0),
            lambda m: m.write_register("A_SOLL", 0),
            lambda m: m.write_register("RUN_CURRENT", 0),
            lambda m: m.engage_brake(),
        ):
            motor = healthy_motor()
            setup(motor)
            try:
                for finding in diagnose(motor).blockers:
                    self.assertTrue(finding.remedy,
                                    f"{finding.title} has no remedy")
            finally:
                motor.disconnect()


# --------------------------------------------------------------------------
# Event log
# --------------------------------------------------------------------------

class TestEventLog(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.path = os.path.join(self.dir, "events.jsonl")

    def tearDown(self):
        for name in os.listdir(self.dir):
            os.remove(os.path.join(self.dir, name))
        os.rmdir(self.dir)

    def test_events_persist_to_disk_as_they_happen(self):
        """A recording has to survive the process being killed."""
        log = EventLog(path=self.path)
        log.info("test", "first")
        log.error("test", "second", detail=42)
        # Not closed: the point is that the lines are already on disk.
        replayed = read_log(self.path)
        self.assertEqual([e.message for e in replayed], ["first", "second"])
        self.assertEqual(replayed[1].severity, ERROR)
        self.assertEqual(replayed[1].data["detail"], 42)
        log.close()

    def test_severity_filtering(self):
        log = EventLog()
        log.debug("t", "d")
        log.info("t", "i")
        log.warning("t", "w")
        log.error("t", "e")
        self.assertEqual(len(log.events()), 4)
        self.assertEqual(len(log.events(min_severity=WARNING)), 2)
        self.assertEqual(len(log.events(min_severity=ERROR)), 1)

    def test_category_filtering(self):
        log = EventLog()
        log.info("comms", "a")
        log.info("motion", "b")
        self.assertEqual(len(log.events(categories=["comms"])), 1)

    def test_ring_is_bounded(self):
        log = EventLog(capacity=10)
        for i in range(50):
            log.info("t", str(i))
        events = log.events()
        self.assertEqual(len(events), 10)
        self.assertEqual(events[-1].message, "49")

    def test_export_text(self):
        log = EventLog()
        log.error("t", "something broke", remedy="fix it")
        out = os.path.join(self.dir, "out.txt")
        log.export_text(out)
        with open(out, encoding="utf-8") as fh:
            body = fh.read()
        self.assertIn("something broke", body)
        self.assertIn("remedy = fix it", body)

    def test_corrupt_lines_are_skipped_not_fatal(self):
        with open(self.path, "w", encoding="utf-8") as fh:
            fh.write('{"timestamp": 1.0, "severity": "INFO", "message": "good"}\n')
            fh.write("this is not json\n")
            fh.write('{"nope": true}\n')
        events = read_log(self.path)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0].message, "good")

    def test_on_event_callback(self):
        seen = []
        log = EventLog(on_event=seen.append)
        log.info("t", "hello")
        self.assertEqual(len(seen), 1)

    def test_a_broken_callback_cannot_break_logging(self):
        def explode(_):
            raise RuntimeError("bad subscriber")
        log = EventLog(on_event=explode)
        log.info("t", "still recorded")
        self.assertEqual(len(log.events()), 1)


# --------------------------------------------------------------------------
# The watcher: does it catch the thing that stopped the motor?
# --------------------------------------------------------------------------

class TestMotorWatcher(unittest.TestCase):
    def setUp(self):
        self.motor = healthy_motor()
        self.log = EventLog()
        self.watcher = MotorWatcher(self.motor, self.log, interval_s=0.05,
                                    heartbeat_s=0)

    def tearDown(self):
        self.watcher.stop()
        self.motor.disconnect()

    def messages(self, severity=DEBUG):
        return [e.message for e in self.log.events(min_severity=severity)]

    def test_only_changes_are_logged_not_every_poll(self):
        """A quiet motor must produce a quiet log, or nobody will read it."""
        for _ in range(20):
            self.watcher.poll_once()
        # First poll establishes the baseline; the other 19 change nothing.
        self.assertLessEqual(len(self.log.events()), 6,
                             "\n".join(self.messages()))

    def test_drive_going_passive_on_its_own_is_an_error(self):
        self.watcher.poll_once()
        self.motor.write_register("MODE_REG", int(MotorMode.PASSIVE))
        self.watcher.poll_once()
        errors = self.log.events(min_severity=ERROR)
        self.assertTrue(errors)
        self.assertIn("went passive on its own", errors[-1].message)
        self.assertIn("accepted and do nothing", errors[-1].message)

    def test_zero_velocity_is_flagged_as_an_error(self):
        self.watcher.poll_once()
        self.motor.write_register("V_SOLL", 0)
        self.watcher.poll_once()
        errors = [e for e in self.log.events(min_severity=ERROR)
                  if e.category == "config"]
        self.assertTrue(errors)
        self.assertIn("never move", errors[-1].message)

    def test_error_bits_appearing_and_clearing(self):
        injector = wrap_motor(self.motor)
        self.watcher.poll_once()
        injector.arm(Fault.ERROR_BITS, error_bits_value=1 << 3)
        self.watcher.poll_once()
        self.assertTrue(any("ERR_BITS set" in m for m in self.messages(ERROR)))
        injector.clear()
        self.watcher.poll_once()
        self.assertTrue(any("ERR_BITS cleared" in m for m in self.messages()))

    def test_comms_loss_and_recovery_are_logged(self):
        self.watcher.poll_once()
        self.motor._transport.set_offline(True)
        self.watcher.poll_once()
        self.assertTrue(any("stopped answering" in m for m in self.messages(ERROR)))
        self.motor._transport.set_offline(False)
        self.watcher.poll_once()
        self.assertTrue(any("is answering" in m for m in self.messages()))

    def test_target_changes_are_logged(self):
        self.watcher.poll_once()
        self.motor.command_position_counts(5000)
        self.watcher.poll_once()
        self.assertTrue(any("P_SOLL" in m for m in self.messages()))

    def test_slow_transactions_are_flagged(self):
        """A stall that then succeeds leaves no error -- but it is the hang."""
        self.watcher.slow_transaction_s = 0.05
        inner = self.motor._transport
        original = inner.read_holding

        def slow(address, count):
            time.sleep(0.02)
            return original(address, count)

        inner.read_holding = slow
        try:
            self.watcher.poll_once()
        finally:
            inner.read_holding = original
        timing = [e for e in self.log.events(min_severity=WARNING)
                  if e.category == "timing"]
        self.assertTrue(timing)
        self.assertIn("hang", timing[-1].message)
        self.assertGreater(timing[-1].data["seconds"], 0.05)

    def test_watcher_survives_an_exploding_poll(self):
        inner = self.motor._transport
        inner.read_holding = lambda *a, **k: (_ for _ in ()).throw(
            RuntimeError("unexpected"))
        self.watcher.start()
        time.sleep(0.3)
        self.assertTrue(self.watcher.running, "the watcher thread died")
        self.watcher.stop()

    def test_start_and_stop_are_recorded(self):
        self.watcher.start()
        time.sleep(0.2)
        self.watcher.stop()
        messages = self.messages()
        self.assertTrue(any("Started watching" in m for m in messages))
        self.assertTrue(any("Stopped watching" in m for m in messages))


# --------------------------------------------------------------------------
# Fast failure: the other half of the reported hang
# --------------------------------------------------------------------------

class TestFastFailure(unittest.TestCase):
    def test_retries_default_to_one_attempt(self):
        """pymodbus defaults to 3 retries, turning one failed read into a
        multi-second stall that looks like the application hanging."""
        from psct_motors.transport import PymodbusTransport
        transport = PymodbusTransport("192.0.2.1", 502, timeout_s=2.0)
        self.assertEqual(transport.retries, 1)
        self.assertEqual(transport.worst_case_transaction_s, 2.0)

    def test_retries_are_configurable_but_never_zero(self):
        from psct_motors.transport import PymodbusTransport
        self.assertEqual(PymodbusTransport("h", retries=4).retries, 4)
        self.assertEqual(PymodbusTransport("h", retries=0).retries, 1)

    def test_config_carries_the_retry_setting(self):
        from psct_motors.config import default_config
        self.assertEqual(default_config().modbus_retries, 1)

    def test_describe_reports_the_blocking_budget(self):
        from psct_motors.transport import PymodbusTransport
        text = PymodbusTransport("10.0.0.1", 502, timeout_s=1.5).describe()
        self.assertIn("timeout 1.5s", text)
        self.assertIn("1 attempt", text)


if __name__ == "__main__":
    unittest.main(verbosity=2)


# --------------------------------------------------------------------------
# Position semantics, as corrected by MacTalk's register list
# --------------------------------------------------------------------------

class TestPositionSemantics(unittest.TestCase):
    """Register 10 is 'Projected Position' -- the profile generator's output,
    which reaches the requested position by construction. Register 16 is
    'Actual Encoder Position'. Conflating them meant a settled motor 231
    counts short of its target reported a perfectly completed move."""

    def setUp(self):
        self.motor = healthy_motor()
        self.motor._transport.follow_error_counts = 231

    def tearDown(self):
        self.motor.disconnect()

    def test_position_comes_from_the_encoder_not_the_profile(self):
        self.motor.command_position_counts(20000)
        self.motor.wait_for_in_position(timeout_s=10.0)
        self.assertEqual(self.motor.get_projected_position_counts(), 20000)
        self.assertEqual(self.motor.get_position_counts(), 20000 - 231)
        self.assertEqual(self.motor.get_follow_error(), 231)

    def test_a_normal_standing_follow_error_still_counts_as_arrived(self):
        self.motor.command_position_counts(20000)
        self.assertTrue(self.motor.wait_for_in_position(timeout_s=10.0))

    def test_a_large_follow_error_is_not_arrival(self):
        """The condition the projected position cannot express: profile
        finished, shaft nowhere near."""
        self.motor._transport.follow_error_counts = 50000
        self.motor.command_position_counts(20000)
        self.assertFalse(self.motor.wait_for_in_position(timeout_s=2.0))

    def test_stop_freezes_the_profile_not_the_encoder(self):
        """Writing the encoder reading as the target would command a step
        equal to the standing follow error -- a stop that causes motion."""
        self.motor.command_position_counts(20000)
        self.motor.wait_for_in_position(timeout_s=10.0)
        self.motor.stop()
        self.assertEqual(self.motor.get_target_counts(),
                         self.motor.get_projected_position_counts())
        self.assertNotEqual(self.motor.get_target_counts(),
                            self.motor.get_position_counts())

    def test_status_reports_both_positions_and_the_follow_error(self):
        status = self.motor.read_status()
        self.assertEqual(status.projected_counts - status.position_counts, 231)
        self.assertEqual(status.follow_error, 231)
        self.assertIn("follow_error", status.as_dict())

    def test_falls_back_to_projected_when_the_encoder_is_unreadable(self):
        """An open-loop or differently-configured motor must still work."""
        from psct_motors.registers import modbus_address
        transport = self.motor._transport
        original = transport.read_holding
        encoder_address = modbus_address(16)

        def without_encoder(address, count):
            if address == encoder_address:
                raise ModbusError("register 16 does not exist on this motor")
            return original(address, count)

        transport.read_holding = without_encoder
        logged = []
        self.motor._log = logged.append
        try:
            self.assertEqual(self.motor.get_position_counts(),
                             self.motor.get_projected_position_counts())
            self.assertTrue(any("Projected Position" in m for m in logged), logged)
            # And it says so once, not on every read.
            self.motor.get_position_counts()
            self.assertEqual(len(logged), 1)
        finally:
            transport.read_holding = original


class TestBrakeConfiguration(unittest.TestCase):
    """Register 179 'Brake Output' selects WHICH output drives the brake. It
    reads 0 on the pSCT motor, so no output does."""

    def test_default_brake_mode_is_none(self):
        from psct_motors.config import BrakeConfig
        self.assertEqual(BrakeConfig().mode, "none")

    def test_unassigned_brake_output_contradicts_output_mode(self):
        from psct_motors.config import BrakeConfig
        motor = simulated_motor(bench_actuator(brake=BrakeConfig(mode="output")))
        motor.connect()
        try:
            self.assertEqual(motor.read_brake_output_assignment(), 0)
            message = motor.check_brake_configuration()
            self.assertIn("register 179", message)
            self.assertIn("no digital output is assigned", message)
        finally:
            motor.disconnect()

    def test_unassigned_brake_output_contradicts_auto_mode(self):
        from psct_motors.config import BrakeConfig
        motor = simulated_motor(bench_actuator(brake=BrakeConfig(mode="auto")))
        motor.connect()
        try:
            self.assertIn("guess with nothing behind it",
                          motor.check_brake_configuration())
        finally:
            motor.disconnect()

    def test_assigned_brake_output_contradicts_none_mode(self):
        from psct_motors.config import BrakeConfig
        motor = simulated_motor(bench_actuator(brake=BrakeConfig(mode="none")))
        motor.connect()
        motor._transport.registers[179] = 2
        try:
            self.assertIn("will not show or control it",
                          motor.check_brake_configuration())
        finally:
            motor.disconnect()

    def test_agreement_produces_no_message(self):
        from psct_motors.config import BrakeConfig
        motor = simulated_motor(bench_actuator(brake=BrakeConfig(mode="none")))
        motor.connect()
        try:
            self.assertEqual(motor.check_brake_configuration(), "")
        finally:
            motor.disconnect()


class TestMotorReport(unittest.TestCase):
    def test_report_runs_and_names_the_latched_history(self):
        import contextlib
        import io as _io
        from psct_motors.cli import main
        buffer = _io.StringIO()
        with contextlib.redirect_stdout(buffer):
            code = main(["--simulate", "motor-report", "--motor", "A"])
        body = buffer.getvalue()
        self.assertEqual(code, 0)
        self.assertIn("follow error max (reg 22)", body)
        self.assertIn("bus voltage min  (reg 98)", body)
        self.assertIn("keeps NO error history", body)
        self.assertIn("projected position (reg 10)", body)
        self.assertIn("encoder position   (reg 16)", body)
        self.assertIn("brake output           (179)", body)
        self.assertIn("modbus slave timeout   (199)", body)
