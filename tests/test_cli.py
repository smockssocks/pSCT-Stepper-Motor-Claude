"""Tests for the command line.

The CLI is the commissioning interface: everything that has to happen before
the GUI means anything is done from here, so a broken argument or a command
that no longer exists is worth catching.
"""

import argparse
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


class TestCli(unittest.TestCase):
    def test_help_builds(self):
        from psct_motors.cli import build_parser
        parser = build_parser()
        self.assertIn("commissioning order", parser.format_help())

    def test_simulated_status_runs(self):
        from psct_motors.cli import main
        self.assertEqual(main(["--simulate", "status", "--json"]), 0)

    def test_simulated_move_runs(self):
        from psct_motors.cli import main
        self.assertEqual(main(["--simulate", "-y", "move", "--focus", "2.0"]), 0)

    def test_refused_move_exits_nonzero(self):
        from psct_motors.cli import main
        self.assertEqual(main(["--simulate", "-y", "move", "--focus", "900"]), 1)

    def test_preview_needs_no_hardware(self):
        from psct_motors.cli import main
        self.assertEqual(main(["preview", "--focus", "2.0", "--tip", "0.1"]), 0)

    def test_register_table_prints_offline(self):
        from psct_motors.cli import main
        self.assertEqual(main(["verify-registers", "--offline"]), 0)

    def test_global_flags_work_on_either_side_of_the_subcommand(self):
        """`cli gui --simulate` reads as naturally as `cli --simulate gui`, and
        the README used to print it that way, so both have to parse."""
        from psct_motors.cli import build_parser
        parser = build_parser()
        for argv in (["--simulate", "status"], ["status", "--simulate"]):
            args = parser.parse_args(argv)
            self.assertTrue(args.simulate, argv)
            self.assertFalse(args.yes, argv)
            self.assertIsNone(args.config, argv)
        for argv in (["-y", "--config", "c.json", "move", "--focus", "1"],
                     ["move", "--focus", "1", "-y", "--config", "c.json"]):
            args = parser.parse_args(argv)
            self.assertTrue(args.yes, argv)
            self.assertEqual(args.config, "c.json", argv)
        # ...and a command given neither still gets the top-level defaults.
        args = parser.parse_args(["status"])
        self.assertFalse(args.simulate)
        self.assertFalse(args.yes)
        self.assertIsNone(args.config)

    def test_every_subcommand_accepts_the_global_flags(self):
        """One missed `parents=` and a command silently loses --simulate."""
        from psct_motors.cli import build_parser
        parser = build_parser()
        actions = [a for a in parser._actions
                   if isinstance(a, argparse._SubParsersAction)]
        self.assertEqual(len(actions), 1)
        names = sorted(actions[0].choices)
        self.assertIn("gui", names)
        self.assertGreater(len(names), 20)
        for name in names:
            options = set()
            for action in actions[0].choices[name]._actions:
                options.update(action.option_strings)
            self.assertTrue({"--simulate", "--config", "-y"} <= options,
                            f"{name} does not accept the global flags")

    def test_simulated_gui_flag_order_reaches_the_command(self):
        """The exact invocation from the README, minus actually opening a
        window."""
        from psct_motors import cli
        seen = {}

        def fake_gui(args):
            seen["simulate"] = args.simulate
            return 0

        original = cli.cmd_gui
        cli.cmd_gui = fake_gui
        try:
            self.assertEqual(cli.main(["gui", "--simulate"]), 0)
        finally:
            cli.cmd_gui = original
        self.assertTrue(seen["simulate"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
