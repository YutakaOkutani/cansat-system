import os
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import patch

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mission.config import MissionConfigError, load_mission_config, load_run_context


VALID_CONFIG = """
[target]
latitude = 38.260728
longitude = 140.854073

[radio]
control = "off"
pre_off_delay_sec = 3
restore_timeout_sec = 180
use_sudo = true
dry_run = false
"""


class MissionConfigTest(unittest.TestCase):
    def _write_config(self, contents):
        temp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(temp_dir.cleanup)
        path = Path(temp_dir.name) / "mission.toml"
        path.write_text(contents, encoding="utf-8")
        return path

    def test_loads_valid_config(self):
        config = load_mission_config(self._write_config(VALID_CONFIG))
        self.assertEqual(config.target.latitude, 38.260728)
        self.assertEqual(config.target.longitude, 140.854073)
        self.assertEqual(config.radio.control, "off")
        self.assertTrue(config.radio.use_sudo)

    def test_approach_defaults_and_overrides(self):
        config = load_mission_config(self._write_config(VALID_CONFIG))
        self.assertEqual(config.approach.far_forward_sec, .4)
        config = load_mission_config(self._write_config(VALID_CONFIG +
            '\n[approach]\nfar_forward_sec = 0.5\nfar_align_pwm = 70\n'))
        self.assertEqual(config.approach.far_forward_sec, .5)
        self.assertEqual(config.approach.far_align_pwm, 70)
        self.assertEqual(config.approach.far_forward_boost_sec, .6)

    def test_invalid_approach_values_are_rejected(self):
        for setting in ('far_forward_sec = 0', 'far_forward_sec = true',
                        'far_forward_sec = nan', 'far_forward_sec = 0.7',
                        'far_align_pwm = 101', 'unknown = 1',
                        'far_forward_sec = 0.5\nfar_forward_boost_sec = 0.4'):
            with self.subTest(setting=setting), self.assertRaises(MissionConfigError):
                load_mission_config(self._write_config(VALID_CONFIG+'\n[approach]\n'+setting+'\n'))

    def test_missing_file_is_rejected(self):
        with self.assertRaisesRegex(MissionConfigError, "Mission config not found"):
            load_mission_config("/does/not/exist/mission.toml")

    def test_repository_example_cannot_start_without_editing_target(self):
        with self.assertRaisesRegex(MissionConfigError, "target.latitude"):
            load_mission_config(PROJECT_ROOT / "mission.toml.example")

    def test_partial_target_is_rejected(self):
        contents = VALID_CONFIG.replace("longitude = 140.854073\n", "")
        with self.assertRaisesRegex(MissionConfigError, "target.longitude is required"):
            load_mission_config(self._write_config(contents))

    def test_out_of_range_coordinate_is_rejected(self):
        contents = VALID_CONFIG.replace("latitude = 38.260728", "latitude = 91")
        with self.assertRaisesRegex(MissionConfigError, "between -90 and 90"):
            load_mission_config(self._write_config(contents))

    def test_coordinate_boundaries_are_accepted(self):
        contents = VALID_CONFIG.replace("latitude = 38.260728", "latitude = -90")
        contents = contents.replace("longitude = 140.854073", "longitude = 180")
        config = load_mission_config(self._write_config(contents))
        self.assertEqual(config.target.latitude, -90.0)
        self.assertEqual(config.target.longitude, 180.0)

    def test_out_of_range_longitude_is_rejected(self):
        contents = VALID_CONFIG.replace("longitude = 140.854073", "longitude = -180.0001")
        with self.assertRaisesRegex(MissionConfigError, "between -180 and 180"):
            load_mission_config(self._write_config(contents))

    def test_malformed_toml_is_rejected(self):
        with self.assertRaisesRegex(MissionConfigError, "Failed to read"):
            load_mission_config(self._write_config("[target\nlatitude = 35"))

    def test_invalid_radio_mode_is_rejected(self):
        contents = VALID_CONFIG.replace('control = "off"', 'control = "auto"')
        with self.assertRaisesRegex(MissionConfigError, "radio.control"):
            load_mission_config(self._write_config(contents))

    def test_unknown_key_is_rejected(self):
        contents = VALID_CONFIG.replace("longitude = 140.854073", "longitude = 140.854073\nlongitdue = 140")
        with self.assertRaisesRegex(MissionConfigError, "longitdue"):
            load_mission_config(self._write_config(contents))

    def test_legacy_target_environment_variables_are_ignored(self):
        with patch.dict(
            os.environ,
            {"CANSAT_TARGET_LAT": "1.25", "CANSAT_TARGET_LNG": "2.5"},
            clear=False,
        ):
            config = load_mission_config(self._write_config(VALID_CONFIG))
        self.assertEqual(config.target.latitude, 38.260728)
        self.assertEqual(config.target.longitude, 140.854073)

    def test_reloading_edited_local_file_changes_target(self):
        path = self._write_config(VALID_CONFIG)
        first = load_mission_config(path)
        path.write_text(
            VALID_CONFIG.replace("38.260728", "35.123456").replace("140.854073", "139.654321"),
            encoding="utf-8",
        )
        second = load_mission_config(path)
        self.assertEqual((first.target.latitude, first.target.longitude), (38.260728, 140.854073))
        self.assertEqual((second.target.latitude, second.target.longitude), (35.123456, 139.654321))

    def test_non_finite_coordinate_is_rejected(self):
        contents = VALID_CONFIG.replace("latitude = 38.260728", "latitude = inf")
        with self.assertRaisesRegex(MissionConfigError, "must be finite"):
            load_mission_config(self._write_config(contents))

    def test_boolean_is_not_accepted_as_number(self):
        contents = VALID_CONFIG.replace("pre_off_delay_sec = 3", "pre_off_delay_sec = true")
        with self.assertRaisesRegex(MissionConfigError, "must be a number"):
            load_mission_config(self._write_config(contents))

    def test_negative_radio_timeout_is_rejected(self):
        contents = VALID_CONFIG.replace("restore_timeout_sec = 180", "restore_timeout_sec = -1")
        with self.assertRaisesRegex(MissionConfigError, "must be non-negative"):
            load_mission_config(self._write_config(contents))

    def test_unsupported_python_is_reported_as_config_error(self):
        import mission.config as mission_config

        with patch.object(mission_config, "tomllib", None):
            with self.assertRaisesRegex(MissionConfigError, "Python 3.11 or newer"):
                mission_config.load_mission_config(self._write_config(VALID_CONFIG))

    def test_invalid_config_stops_before_controller_import(self):
        import mission.run as mission_run

        with patch.object(mission_run, "load_mission_config", side_effect=MissionConfigError("invalid")):
            with self.assertRaisesRegex(MissionConfigError, "invalid"):
                mission_run._build_controller()

    def test_runtime_passes_file_config_to_controller(self):
        import mission.run as mission_run

        config = load_mission_config(self._write_config(VALID_CONFIG))
        captured = {}

        class FakeController:
            def __init__(self, received_config, run_context, log_dir=None):
                captured["config"] = received_config
                captured["run_context"] = run_context
                captured["log_dir"] = log_dir
                self.run_dir = "/tmp/cansat-test/run"

        fake_ctrl_module = types.ModuleType("mission.ctrl")
        fake_ctrl_module.CanSatController = FakeController
        with patch.object(mission_run, "load_mission_config", return_value=config):
            with patch.dict(sys.modules, {"mission.ctrl": fake_ctrl_module}):
                controller = mission_run._build_controller(log_dir="/tmp/cansat-test")

        self.assertIsInstance(controller, FakeController)
        self.assertIs(captured["config"], config)
        self.assertEqual(captured["run_context"].event_id, "unclassified")
        self.assertEqual(captured["log_dir"], "/tmp/cansat-test")

    def test_missing_run_context_uses_unclassified_metadata(self):
        context = load_run_context("/does/not/exist/run-context.toml")
        self.assertEqual(context.event_id, "unclassified")
        self.assertEqual(context.run_kind, "mission")
        self.assertIsNone(context.source)

    def test_loads_structured_run_context(self):
        path = self._write_config(
            """
[event]
id = "nse2026"
name = "NSE 2026"

[run]
kind = "competition"
label = "attempt-1"
location = "field-a"
operator = "team"
tags = ["dry", "final"]
notes = "windy"
"""
        )
        context = load_run_context(path)
        self.assertEqual(context.event_id, "nse2026")
        self.assertEqual(context.run_kind, "competition")
        self.assertEqual(context.tags, ("dry", "final"))
        self.assertEqual(context.notes, "windy")

    def test_repository_run_context_example_is_valid(self):
        context = load_run_context(PROJECT_ROOT / "run-context.toml.example")
        self.assertEqual(context.event_id, "nse2026")
        self.assertEqual(context.run_kind, "competition")

    def test_run_context_rejects_unsafe_event_id(self):
        path = self._write_config(
            """
[event]
id = "NSE 2026/../../"
name = "unsafe"

[run]
kind = "competition"
"""
        )
        with self.assertRaisesRegex(MissionConfigError, "event.id"):
            load_run_context(path)

    def test_target_constants_were_removed(self):
        from mission import const

        self.assertFalse(hasattr(const, "TARGET_LAT"))
        self.assertFalse(hasattr(const, "TARGET_LNG"))

    def test_main_rejects_legacy_arguments_before_starting(self):
        import main

        with patch.object(sys, "argv", ["main.py", "--target-lat", "35.0"]):
            with patch.object(main, "run_full_mission") as run_full_mission:
                with self.assertRaisesRegex(SystemExit, "does not accept configuration arguments"):
                    main.main()
        run_full_mission.assert_not_called()

    def test_main_reports_config_error_without_traceback_path(self):
        import main

        with patch.object(sys, "argv", ["main.py"]):
            with patch.object(main, "run_full_mission", side_effect=MissionConfigError("bad target")):
                with self.assertRaisesRegex(SystemExit, "Mission configuration error: bad target"):
                    main.main()


if __name__ == "__main__":
    unittest.main()
