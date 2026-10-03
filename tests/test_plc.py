"""The brake PLC (a ControlByWeb X-432), end to end over real HTTP.

Run against a fake X-432 on localhost (tests/fake_plc.py). What is checked:

  * the driver reads relays and inputs, and switches the right relay the
    right way round;
  * it says "relay state" rather than "brake state" when nothing measures the
    brake, and EMERGENCY therefore leaves the drives holding;
  * with a feedback input wired, EMERGENCY does turn the drives off;
  * an unreachable or password-protected PLC gives a message that says so,
    and does not freeze the status poll;
  * `--simulate` never touches the configured PLC unless asked to;
  * in bench mode the stood-in motors obey the PLC's relay, so the whole
    brake sequence can be rehearsed with one real motor or none.
"""

import os
import sys
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from fake_plc import FakeX432  # noqa: E402

from psct_motors import safety  # noqa: E402
from psct_motors.external_brake import (  # noqa: E402
    BrakeController, BrakeError, ExternalBrakeConfig, parse_state)
from psct_motors.jvl_motor import BrakeState  # noqa: E402
from psct_motors.kinematics import Orientation  # noqa: E402
from psct_motors.platform import FocalPlanePlatform, PlatformError  # noqa: E402
from psct_motors.registers import MotorMode  # noqa: E402


def cbw(plc: FakeX432, **kw) -> ExternalBrakeConfig:
    params = dict(mode="controlbyweb", host="127.0.0.1", http_port=plc.port,
                  relays={"all": 1}, timeout_s=2.0)
    params.update(kw)
    return ExternalBrakeConfig(**params)


class TestParsing(unittest.TestCase):
    def test_x400_and_older_names_and_value_types(self):
        io = parse_state({"relay1": 1, "relay2": "0", "relay3state": "on",
                          "digitalInput4": "1", "input5state": 0,
                          "analogInput1": "2.5", "serialNumber": "x"})
        self.assertEqual(io["relays"], {1: True, 2: False, 3: True})
        self.assertEqual(io["inputs"], {4: True, 5: False})


class TestDriver(unittest.TestCase):
    def setUp(self):
        self.plc = FakeX432().start()
        self.addCleanup(self.plc.stop)

    def test_it_lists_every_relay_and_input(self):
        io = BrakeController(cbw(self.plc)).read_io()
        self.assertEqual(sorted(io["relays"]), list(range(1, 17)))
        self.assertEqual(sorted(io["inputs"]), list(range(1, 19)))
        self.assertIn("serialNumber", io["raw"])

    def test_release_energises_the_mapped_relay_and_engage_drops_it(self):
        controller = BrakeController(cbw(self.plc, relays={"all": 3}))
        self.assertIs(controller.read_state(fresh=True), BrakeState.ENGAGED)
        controller.release(drives_holding=True)
        self.assertEqual(self.plc.relays[3], 1)
        self.assertEqual([n for n, v in self.plc.relays.items() if v], [3])
        self.assertIs(controller.read_state(fresh=True), BrakeState.RELEASED)
        controller.engage()
        self.assertEqual(self.plc.relays[3], 0)
        self.assertIs(controller.read_state(fresh=True), BrakeState.ENGAGED)

    def test_the_polarity_can_be_the_other_way_round(self):
        """How the relay is wired is not known, so it is a setting."""
        controller = BrakeController(cbw(self.plc, energized_releases=False))
        self.assertIs(controller.read_state(fresh=True), BrakeState.RELEASED)
        controller.engage()
        self.assertEqual(self.plc.relays[1], 1)
        self.assertIs(controller.read_state(fresh=True), BrakeState.ENGAGED)

    def test_three_relays_switch_together_in_one_request(self):
        controller = BrakeController(
            cbw(self.plc, relays={"Top": 1, "East": 2, "West": 3}),
            names=["Top", "East", "West"])
        self.plc.requests.clear()
        controller.release("all", drives_holding=True)
        writes = [r for r in self.plc.requests if "relay" in r]
        self.assertEqual(len(writes), 1, writes)
        self.assertEqual([self.plc.relays[n] for n in (1, 2, 3)], [1, 1, 1])

    def test_one_relay_on_and_one_off_is_unknown_not_a_guess(self):
        controller = BrakeController(
            cbw(self.plc, relays={"Top": 1, "East": 2, "West": 3}),
            names=["Top", "East", "West"])
        self.plc.relays[2] = 1
        self.assertIs(controller.read_state(fresh=True), BrakeState.UNKNOWN)

    def test_release_still_refuses_unless_the_drives_hold(self):
        controller = BrakeController(cbw(self.plc))
        with self.assertRaises(BrakeError):
            controller.release(drives_holding=False)
        self.assertEqual(self.plc.relays[1], 0)

    def test_by_default_the_relay_reading_is_the_brake_state(self):
        """The site's brakes are fail-safe and the PLC reports its relays
        correctly, so the relay reading counts as the brake's state."""
        controller = BrakeController(cbw(self.plc), names=["Top", "East", "West"])
        self.assertTrue(controller.state_is_measured("all"))

    def test_without_trusting_the_relay_the_state_is_the_relays_not_the_brakes(self):
        controller = BrakeController(cbw(self.plc, trust_relay_state=False),
                                     names=["Top", "East", "West"])
        self.assertFalse(controller.state_is_measured("all"))

    def test_feedback_inputs_make_it_a_measurement(self):
        self.plc.wire(relay=1, input_number=5)          # input on = released
        controller = BrakeController(
            cbw(self.plc, feedback_inputs={"all": 5}), names=["Top", "East", "West"])
        self.assertTrue(controller.state_is_measured("all"))
        controller.release(drives_holding=True)
        self.assertIs(controller.read_state(fresh=True), BrakeState.RELEASED)

    def test_feedback_that_disagrees_with_the_relay_is_believed(self):
        """The whole point of a feedback input: the relay says released, the
        brake says it is still clamped (a blown fuse, say). The brake wins."""
        controller = BrakeController(
            cbw(self.plc, feedback_inputs={"all": 5}), names=["Top", "East", "West"])
        controller.release(drives_holding=True)          # input 5 unwired, stays 0
        self.assertEqual(self.plc.relays[1], 1)
        self.assertIs(controller.read_state(fresh=True), BrakeState.ENGAGED)

    def test_partial_feedback_confirms_nothing(self):
        controller = BrakeController(
            cbw(self.plc, relays={"all": 1}, feedback_inputs={"Top": 5},
                trust_relay_state=False),
            names=["Top", "East", "West"])
        self.assertFalse(controller.state_is_measured("all"))

    def test_a_relay_the_plc_will_not_set_is_an_error(self):
        self.plc.stuck[1] = 0
        controller = BrakeController(cbw(self.plc))
        with self.assertRaises(BrakeError) as ctx:
            controller.release(drives_holding=True)
        self.assertIn("reports relay 1 off", str(ctx.exception))

    def test_an_unmapped_relay_number_is_named(self):
        controller = BrakeController(cbw(self.plc, relays={"all": 40}))
        with self.assertRaises(BrakeError) as ctx:
            controller.read_state(fresh=True)
        self.assertIn("relay 40", str(ctx.exception))

    def test_state_xml_is_used_when_there_is_no_state_json(self):
        self.plc.serve_json = False
        controller = BrakeController(cbw(self.plc))
        controller.release(drives_holding=True)
        self.assertEqual(self.plc.relays[1], 1)
        self.assertTrue(any("state.xml" in r for r in self.plc.requests))

    def test_the_status_poll_is_cached(self):
        controller = BrakeController(cbw(self.plc))
        controller.read_state(fresh=True)
        self.plc.requests.clear()
        for _ in range(10):
            controller.read_state()
        self.assertEqual(self.plc.requests, [])

    def test_is_holding_never_touches_the_network(self):
        controller = BrakeController(cbw(self.plc))
        self.assertTrue(controller.is_holding("Top"))     # nothing read yet
        controller.release(drives_holding=True)
        self.plc.requests.clear()
        self.assertFalse(controller.is_holding("Top"))
        self.assertEqual(self.plc.requests, [])


class TestPlcProblems(unittest.TestCase):
    def test_a_password_protected_plc_says_so(self):
        with FakeX432(password="secret") as plc:
            with self.assertRaises(BrakeError) as ctx:
                BrakeController(cbw(plc)).read_state(fresh=True)
            self.assertIn("login", str(ctx.exception))
            controller = BrakeController(cbw(plc, password="secret"))
            self.assertIs(controller.read_state(fresh=True), BrakeState.ENGAGED)

    def test_an_unreachable_plc_fails_fast_and_then_backs_off(self):
        plc = FakeX432().start()
        port = plc.port
        plc.stop()                                     # nothing listening now
        controller = BrakeController(ExternalBrakeConfig(
            mode="controlbyweb", host="127.0.0.1", http_port=port,
            relays={"all": 1}, timeout_s=1.0))
        started = time.monotonic()
        with self.assertRaises(BrakeError) as ctx:
            controller.read_state()
        self.assertIn("Could not reach the PLC", str(ctx.exception))
        # The poll asks again straight away; it must not wait out another
        # timeout, or the motor readouts freeze behind a dead PLC.
        again = time.monotonic()
        with self.assertRaises(BrakeError):
            controller.read_state()
        self.assertLess(time.monotonic() - again, 0.05)
        self.assertLess(again - started, 2.0)

    def test_one_slow_reply_is_retried_rather_than_reported(self):
        """A small web PLC misses the odd reply. One miss used to show the
        brakes as unreadable for two seconds, which looked like the PLC
        dropping off the network and coming back."""
        with FakeX432() as plc:
            said = []
            controller = BrakeController(cbw(plc), logger=said.append)
            plc.slow_next = 1
            self.assertIs(controller.read_state(fresh=True), BrakeState.ENGAGED)
            self.assertEqual(controller.link_stats["retried"], 1)
            self.assertEqual(controller.link_stats["failed"], 0)
            self.assertEqual(said, [], "a hiccup is not worth a log line")

    def test_two_in_a_row_are_reported_with_the_reason(self):
        with FakeX432() as plc:
            controller = BrakeController(cbw(plc))
            plc.slow_next = 2
            with self.assertRaises(BrakeError) as ctx:
                controller.read_state(fresh=True)
            self.assertIn("timed out", str(ctx.exception))
            self.assertEqual(controller.link_stats["failed"], 1)
            self.assertIs(controller.read_state(fresh=True), BrakeState.ENGAGED)

    def test_the_display_rides_out_a_slow_plc(self):
        """The display's reading comes from a background reader: a PLC that
        stops answering for a moment neither freezes the poll nor flips the
        brakes to unreadable, until the last reading is 3 s old."""
        with FakeX432() as plc:
            said = []
            controller = BrakeController(cbw(plc), logger=said.append)
            self.addCleanup(controller.close)
            self.assertIs(controller.read_state(), BrakeState.ENGAGED)
            plc.slow_next = 4                      # about two failed reads
            started = time.monotonic()
            for _ in range(15):
                self.assertIs(controller.read_state(), BrakeState.ENGAGED)
                time.sleep(0.1)
            # Fifteen polls in about 1.5 s: none waited on the PLC.
            self.assertLess(time.monotonic() - started, 2.5)
            self.assertEqual(said, [])

    def test_a_plc_gone_for_good_is_reported_after_the_grace(self):
        plc = FakeX432().start()
        said = []
        controller = BrakeController(cbw(plc), logger=said.append)
        self.addCleanup(controller.close)
        self.assertIs(controller.read_state(), BrakeState.ENGAGED)
        plc.stop()
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline:
            try:
                controller.read_state()
            except BrakeError:
                break
            time.sleep(0.2)
        else:
            self.fail("the brakes were never reported unreadable")
        self.assertTrue(any("No reply from the PLC" in line for line in said), said)

    def test_watch_reports_a_clean_link(self):
        import contextlib
        import io
        from psct_motors.cli import _watch_plc
        with FakeX432() as plc:
            controller = BrakeController(cbw(plc))
            buffer = io.StringIO()
            with contextlib.redirect_stdout(buffer):
                code = _watch_plc(controller, 0.6, 0.1)
            self.assertEqual(code, 0)
            self.assertIn("0 failed (0.0%)", buffer.getvalue())

    def test_watch_shows_each_failure(self):
        import contextlib
        import io
        from psct_motors.cli import _watch_plc
        with FakeX432() as plc:
            controller = BrakeController(cbw(plc))
            plc.slow_next = 1
            buffer = io.StringIO()
            with contextlib.redirect_stdout(buffer):
                code = _watch_plc(controller, 2.0, 0.2)
            text = buffer.getvalue()
            self.assertEqual(code, 1)
            self.assertIn("FAILED after", text)
            self.assertIn("Timeouts:", text)

    def test_an_incomplete_configuration_is_refused(self):
        with self.assertRaises(ValueError):
            ExternalBrakeConfig(mode="controlbyweb").validate()
        with self.assertRaises(ValueError):
            ExternalBrakeConfig(mode="controlbyweb", host="h").validate()
        with self.assertRaises(ValueError):
            ExternalBrakeConfig(mode="controlbyweb", host="h",
                                relays={"all": 0}).validate()

    def test_a_relay_mapped_to_a_name_that_is_not_an_actuator_is_refused(self):
        cfg = safety.bench_config()
        cfg.external_brake.mode = "controlbyweb"
        cfg.external_brake.host = "h"
        cfg.external_brake.relays = {"Tpo": 1}
        with self.assertRaises(ValueError) as ctx:
            cfg.validate()
        self.assertIn("Tpo", str(ctx.exception))


class TestPlatformWithThePlc(unittest.TestCase):
    """Three simulated motors and the (fake) real PLC: `--simulate
    --real-brakes`. The same code path as `--bench`, where the stood-in
    motors obey the PLC's relay too."""

    def setUp(self):
        self.plc = FakeX432().start()
        self.addCleanup(self.plc.stop)

    def _platform(self, use_real_brakes=True, **brake):
        cfg = safety.bench_config()
        settings = cfg.external_brake
        settings.mode = "controlbyweb"
        settings.host = "127.0.0.1"
        settings.http_port = self.plc.port
        settings.relays = {"all": 1}
        for key, value in brake.items():
            setattr(settings, key, value)
        cfg.validate()
        platform = FocalPlanePlatform(cfg=cfg, simulate=True,
                                      use_real_brakes=use_real_brakes)
        platform.connect()
        self.addCleanup(platform.disconnect)
        return platform

    def test_simulate_alone_never_touches_the_plc(self):
        platform = self._platform(use_real_brakes=False)
        platform.move_to_orientation(Orientation(1.0, 0.0, 0.0))
        platform.read_state()
        self.assertEqual(self.plc.requests, [])
        self.assertIn("simulated", platform.external_brake.describe())

    def test_a_move_releases_the_plc_brakes_first(self):
        platform = self._platform()
        platform.move_to_orientation(Orientation(1.0, 0.0, 0.0))
        self.assertEqual(self.plc.relays[1], 1)
        self.assertAlmostEqual(platform.read_orientation().focus_mm, 1.0, places=2)

    def test_the_stood_in_axes_obey_the_plc(self):
        """A relay the PLC holds off keeps the brakes clamped, so the move is
        refused -- exactly as it would be with the brakes fitted."""
        self.plc.stuck[1] = 0
        platform = self._platform()
        with self.assertRaises(PlatformError) as ctx:
            platform.move_to_orientation(Orientation(1.0, 0.0, 0.0))
        self.assertIn("did not release", str(ctx.exception))
        self.assertAlmostEqual(platform.read_orientation().focus_mm,
                               (platform.cfg.limits.min_focus_mm
                                + platform.cfg.limits.max_focus_mm) / 2, places=2)

    def test_emergency_trusting_the_relay_turns_the_drives_off(self):
        platform = self._platform()
        platform.move_to_orientation(Orientation(1.0, 0.0, 0.0))
        result = platform.emergency_stop()
        self.assertEqual(self.plc.relays[1], 0)            # brakes on
        self.assertTrue(result.drives_off)
        self.assertTrue(all(m.get_mode() == int(MotorMode.PASSIVE)
                            for m in platform.motors))

    def test_emergency_without_feedback_leaves_the_drives_holding(self):
        """With trust_relay_state off: the relay reads off, but nothing says
        the brake clamped, so the drives stay on."""
        platform = self._platform(trust_relay_state=False)
        platform.move_to_orientation(Orientation(1.0, 0.0, 0.0))
        result = platform.emergency_stop()
        self.assertEqual(self.plc.relays[1], 0)            # brakes commanded on
        self.assertFalse(result.drives_off)
        self.assertIn("nothing measures", result.brake_message)
        self.assertTrue(all(m.get_mode() == int(MotorMode.POSITION)
                            for m in platform.motors))

    def test_emergency_with_feedback_turns_the_drives_off(self):
        self.plc.wire(relay=1, input_number=5)
        platform = self._platform(feedback_inputs={"all": 5})
        platform.move_to_orientation(Orientation(1.0, 0.0, 0.0))
        result = platform.emergency_stop()
        self.assertTrue(result.brakes_engaged)
        self.assertTrue(result.drives_off)

    def test_the_status_line_says_what_it_is_reading(self):
        platform = self._platform()
        state = platform.read_state()
        self.assertIn("ControlByWeb", state.brake_summary)
        self.assertNotIn("relay state", state.brake_summary)
        self.assertFalse(any(m.brake.inferred for m in state.motors))
        platform = self._platform(trust_relay_state=False)
        state = platform.read_state()
        self.assertIn("relay state", state.brake_summary)
        self.assertTrue(all(m.brake.inferred for m in state.motors))

    def test_a_dead_plc_is_reported_and_does_not_stall_the_poll(self):
        platform = self._platform()
        self.plc.stop()
        started = time.monotonic()
        for _ in range(5):
            state = platform.read_state()
        self.assertLess(time.monotonic() - started, 3.5)
        self.assertIn("NOT READABLE", state.brake_summary)
        self.assertTrue(all(m.brake.state is BrakeState.UNKNOWN for m in state.motors))

    def test_enabling_the_drives_holds_them_where_they_are(self):
        """The first half of 'release the brakes': drives on, nothing moves,
        even when a stale target is sitting in P_SOLL."""
        platform = self._platform()
        before = platform.read_actuator_positions_mm()
        for motor in platform.motors:
            motor.write_register("P_SOLL", motor.get_position_counts() + 5000)
        enabled = platform.enable_drives()
        self.assertEqual(sorted(enabled), ["East", "Top", "West"])
        time.sleep(0.4)
        after = platform.read_actuator_positions_mm()
        self.assertTrue(all(abs(a - b) < 1e-3 for a, b in zip(before, after)))
        self.assertTrue(platform.drives_holding)

    def test_release_from_a_row_switches_all_three_when_they_share_a_relay(self):
        platform = self._platform()
        platform.enable_drives()
        result = platform.set_brake("Top", engaged=False)
        self.assertIn("all three", result["all"])
        self.assertEqual(self.plc.relays[1], 1)

    def test_bench_mode_uses_the_plc_and_a_clamped_brake_stops_a_stood_in_axis(self):
        """`--bench`: not a simulation, so the configured PLC is used without
        asking, and an axis stood in for a missing motor behaves as if its
        brake were fitted -- it will not turn while the relay says clamped."""
        cfg = safety.bench_config()
        for actuator in cfg.actuators:
            actuator.simulated = True       # stand-ins, as --bench makes them
        cfg.external_brake.mode = "controlbyweb"
        cfg.external_brake.host = "127.0.0.1"
        cfg.external_brake.http_port = self.plc.port
        cfg.external_brake.relays = {"all": 1}
        platform = FocalPlanePlatform(cfg=cfg, simulate=False)
        platform.connect()
        self.addCleanup(platform.disconnect)
        self.assertIn("ControlByWeb", platform.external_brake.describe())

        platform.read_state()                           # a reading: clamped
        motor = platform.motors[0]
        motor.ensure_position_mode()
        start = motor.get_position_mm()
        motor.command_position_mm(start + 2.0)          # straight at the drive
        time.sleep(0.4)
        self.assertAlmostEqual(motor.get_position_mm(), start, places=3)

        platform.enable_drives()
        platform.set_all_brakes(engaged=False)          # the PLC lets go
        time.sleep(0.4)
        self.assertGreater(motor.get_position_mm(), start + 0.1)

    # ---- separate brakes ------------------------------------------------------

    def _separate(self):
        return self._platform(relays={"Top": 1, "East": 2, "West": 3},
                              all_or_nothing=False)

    def test_separate_brakes_switch_one_at_a_time_from_their_rows(self):
        platform = self._separate()
        platform.enable_drives()
        result = platform.set_brake("East", engaged=False)
        self.assertEqual(result, {"East": "ok"})
        self.assertEqual([self.plc.relays[n] for n in (1, 2, 3)], [0, 1, 0])
        state = platform.read_state()
        brakes = {m.name: m.brake.state for m in state.motors}
        self.assertEqual(brakes, {"Top": BrakeState.ENGAGED,
                                  "East": BrakeState.RELEASED,
                                  "West": BrakeState.ENGAGED})

    def test_one_separate_brake_needs_only_its_own_drive_holding(self):
        platform = self._separate()
        platform.motor("West").ensure_position_mode()      # only West is on
        refused = platform.set_brake("East", engaged=False)
        self.assertIn("Refusing", refused["East"])
        self.assertEqual(self.plc.relays[2], 0)
        self.assertEqual(platform.set_brake("West", engaged=False), {"West": "ok"})
        self.assertEqual(self.plc.relays[3], 1)

    def test_a_jog_releases_only_its_own_brake_when_they_are_separate(self):
        platform = self._separate()
        platform.move_actuator_mm("Top", 0.5, relative=True)
        self.assertEqual([self.plc.relays[n] for n in (1, 2, 3)], [1, 0, 0])

    def test_a_focus_move_releases_all_three_separate_brakes(self):
        platform = self._separate()
        platform.move_to_orientation(Orientation(1.0, 0.0, 0.0))
        self.assertEqual([self.plc.relays[n] for n in (1, 2, 3)], [1, 1, 1])

    def test_a_jog_releases_every_brake_when_one_relay_holds_them_all(self):
        platform = self._platform()
        platform.move_actuator_mm("Top", 0.5, relative=True)
        self.assertEqual(self.plc.relays[1], 1)

    # ---- the checks before a jog releases anything ------------------------------

    def test_a_jog_is_refused_over_a_drive_error_and_touches_no_brake(self):
        platform = self._platform()
        platform.motors[1]._transport.inject_error(1 << 6)
        with self.assertRaises(PlatformError) as ctx:
            platform.move_actuator_mm("Top", 0.5, relative=True)
        self.assertIn("East has an active error", str(ctx.exception))
        self.assertEqual(self.plc.relays[1], 0)
        self.assertTrue(all("relay" not in r for r in self.plc.requests))

    def test_a_jog_is_refused_with_a_failed_supply_and_touches_no_brake(self):
        """A drive with no main supply answers Modbus and can read back as
        enabled while holding nothing. Releasing a brake over it drops that
        corner of the plate."""
        platform = self._platform()
        for actuator in platform.cfg.actuators:
            actuator.supply_nominal_v, actuator.supply_raw_at_nominal = 48.0, 4485
        platform.motor("Top")._transport.set_powered(False)
        with self.assertRaises(PlatformError) as ctx:
            platform.move_actuator_mm("Top", 0.5, relative=True)
        self.assertIn("supply has failed", str(ctx.exception))
        self.assertEqual(self.plc.relays[1], 0)

    def test_release_from_a_row_is_refused_with_the_drives_off(self):
        platform = self._platform()
        result = platform.set_brake("Top", engaged=False)
        self.assertIn("Refusing", result["all"])
        self.assertEqual(self.plc.relays[1], 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
