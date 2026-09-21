import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mission.const import (
    DEVICE_MOTOR_1_PWM,
    DEVICE_MOTOR_1_DIR,
    DEVICE_MOTOR_2_PWM,
    DEVICE_MOTOR_2_DIR,
    PHASE4_ALIGN_PIVOT_SPEED,
    PHASE4_SEARCH_OUTER_SPEED,
    PHASE5_BASE_SPEED,
    PHASE5_TURN_CLAMP,
)

manager_spec = importlib.util.spec_from_file_location(
    "motor_output_manager_under_test", PROJECT_ROOT / "mission" / "mgr" / "mtr_mgr.py"
)
manager_module = importlib.util.module_from_spec(manager_spec)
manager_spec.loader.exec_module(manager_module)
MotorManager = manager_module.MotorManager

MOTOR_DIAG_PATH = PROJECT_ROOT / "runs" / "diag" / "motor.py"
spec = importlib.util.spec_from_file_location("motor_diag_under_test", MOTOR_DIAG_PATH)
motor_diag = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = motor_diag
spec.loader.exec_module(motor_diag)


class MotorDiagnosticSafetyTest(unittest.TestCase):
    def test_real_output_paths_apply_trim_and_drive_floor(self):
        class Device:
            value = 0.0

        cases = [
            # logical L/R request, expected physical L/R PWM after trim
            ((100, True, 100, True), (100, 90)),
            ((100, False, 100, False), (100, 90)),
            ((45, True, 60, True), (65, 78)),
            ((60, True, 45, True), (60 * 65 / 40.5, 65)),
            ((45, True, 85, True), (65, 100)),
            ((85, True, 45, True), (100, 65)),
            ((60, True, 100, True), (65, 97.5)),
            ((100, True, 60, True), (100, 65)),
            ((45, True, 45, True), (65 / 0.9, 65)),
            ((45, False, 45, False), (65 / 0.9, 65)),
            ((65, True, 65, True), (65 / 0.9, 65)),
            ((70, True, 70, True), (65 / 0.9, 65)),
            ((70, False, 70, False), (65 / 0.9, 65)),
            ((80, True, 80, True), (80, 72)),
            ((80, False, 80, False), (80, 72)),
            ((0, False, 0, False), (0, 0)),
            ((0, True, 0, True), (0, 0)),
            ((0, True, 45, True), (0, 40.5)),
            ((100, True, 0, True), (100, 0)),
            ((45, False, 60, True), (45, 54)),
        ]
        for requested, expected in cases:
            with self.subTest(requested=requested):
                pwm1, dir1, pwm2, dir2 = [Device() for _ in range(4)]
                manager = MotorManager()
                manager.devices = {
                    DEVICE_MOTOR_1_PWM: pwm1, DEVICE_MOTOR_1_DIR: dir1,
                    DEVICE_MOTOR_2_PWM: pwm2, DEVICE_MOTOR_2_DIR: dir2,
                }
                manager.motor_state = {}
                manager.set_motors(*requested, ramp_time=0)
                production_output = (pwm1.value, pwm2.value, dir1.value, dir2.value)
                with patch.multiple(
                    motor_diag,
                    motor_1_pwm=pwm1, motor_1_dir=dir1,
                    motor_2_pwm=pwm2, motor_2_dir=dir2,
                    motor_state={
                        'A': {'speed': 0.0, 'direction': True},
                        'B': {'speed': 0.0, 'direction': True},
                    },
                ):
                    motor_diag.set_motors(*requested, ramp_time=0)
                self.assertEqual(
                    (pwm1.value, pwm2.value, dir1.value, dir2.value), production_output
                )
                self.assertAlmostEqual(pwm1.value * 100, expected[0])
                self.assertAlmostEqual(pwm2.value * 100, expected[1])
                self.assertEqual(
                    motor_diag._format_output(*requested),
                    f"L={expected[0]:.1f}%{'F' if requested[1] else 'R'} "
                    f"R={expected[1]:.1f}%{'F' if requested[3] else 'R'}",
                )
                self.assertEqual(dir1.value, int(not requested[1]))
                self.assertEqual(dir2.value, int(requested[3]))

    def test_every_phase_exposes_all_wasd_keys(self):
        self.assertEqual(set(motor_diag.PHASE_DRIVE_PROFILES), set("1234567"))
        for phase, profiles in motor_diag.PHASE_DRIVE_PROFILES.items():
            with self.subTest(phase=phase):
                self.assertGreaterEqual(len(profiles), 1)
                for profile in profiles:
                    self.assertEqual(set(profile["commands"]), set("wasd"))

    def test_production_motor_direction_matches_current_airframe(self):
        self.assertEqual(motor_diag.motor_forward_to_dir_value(1, True), 0)
        self.assertEqual(motor_diag.motor_forward_to_dir_value(2, True), 1)

    def test_production_motor_trim_matches_current_airframe(self):
        self.assertEqual(motor_diag.MOTOR_SPEED_SCALE_1, 1.00)
        self.assertEqual(motor_diag.MOTOR_SPEED_SCALE_2, 0.90)

    def test_logical_left_right_routes_to_measured_physical_channels(self):
        mapped = motor_diag.map_logical_wheels_to_physical(
            30.0,
            True,
            70.0,
            False,
        )

        self.assertEqual(mapped, (30.0, True, 70.0, False))

    def test_manual_left_turn_drives_physical_right_wheel_faster(self):
        with patch.object(motor_diag, "set_motors") as set_motors:
            self.assertTrue(motor_diag._apply_manual_drive_pattern("a", 100.0))

        set_motors.assert_called_once_with(60.0, 1, 100.0, 1)
        logical_call = set_motors.call_args.args
        mapped = motor_diag.map_logical_wheels_to_physical(*logical_call)
        self.assertEqual(mapped[0], 60.0)
        self.assertEqual(mapped[2], 100.0)

    def test_manual_right_turn_drives_physical_left_wheel_faster(self):
        with patch.object(motor_diag, "set_motors") as set_motors:
            self.assertTrue(motor_diag._apply_manual_drive_pattern("d", 100.0))

        set_motors.assert_called_once_with(100.0, 1, 60.0, 1)
        logical_call = set_motors.call_args.args
        mapped = motor_diag.map_logical_wheels_to_physical(*logical_call)
        self.assertEqual(mapped[0], 100.0)
        self.assertEqual(mapped[2], 60.0)

    def test_phase3_navigation_profile_matches_production_outputs(self):
        forward = motor_diag.get_phase_drive_pattern("3", 0, "w")
        left = motor_diag.get_phase_drive_pattern("3", 0, "a")
        right = motor_diag.get_phase_drive_pattern("3", 0, "d")

        self.assertEqual((forward["speed_left"], forward["speed_right"]), (70.0, 70.0))
        self.assertEqual((left["speed_left"], left["speed_right"]), (50.0, 75.0))
        self.assertEqual((right["speed_left"], right["speed_right"]), (75.0, 50.0))

    def test_phase3_large_arc_profile_keeps_both_wheels_powered(self):
        left = motor_diag.get_phase_drive_pattern("P3", 1, "a")
        right = motor_diag.get_phase_drive_pattern("P3", 1, "d")

        self.assertEqual((left["speed_left"], left["speed_right"]), (45.0, 85.0))
        self.assertEqual((right["speed_left"], right["speed_right"]), (85.0, 45.0))
        self.assertGreater(min(left["speed_left"], left["speed_right"]), 0.0)
        self.assertGreater(min(right["speed_left"], right["speed_right"]), 0.0)

    def test_phase2_profiles_cover_calibration_offset_and_reorient_outputs(self):
        profiles = motor_diag.PHASE_DRIVE_PROFILES["2"]

        calibration_left = profiles[1]["commands"]["a"]
        offset_left = profiles[2]["commands"]["a"]
        reorient_left = profiles[3]["commands"]["a"]

        self.assertEqual(
            (calibration_left["speed_left"], calibration_left["speed_right"]),
            (45.0, 70.0),
        )
        self.assertEqual(
            (offset_left["speed_left"], offset_left["speed_right"]),
            (70.0, 100.0),
        )
        self.assertEqual(
            (reorient_left["speed_left"], reorient_left["speed_right"]),
            (45.0, 70.0),
        )
        self.assertGreaterEqual(
            min(calibration_left["speed_left"], calibration_left["speed_right"]),
            45.0,
        )
        self.assertGreaterEqual(
            min(reorient_left["speed_left"], reorient_left["speed_right"]),
            45.0,
        )

    def test_phase4_and_phase5_profiles_expose_configured_extreme_turns(self):
        phase4_search_left = motor_diag.get_phase_drive_pattern("4", 0, "a")
        phase4_align_left = motor_diag.get_phase_drive_pattern("4", 1, "a")
        phase5_left = motor_diag.get_phase_drive_pattern("5", 0, "a")

        self.assertEqual(
            (phase4_search_left["speed_left"], phase4_search_left["speed_right"]),
            (45.0, float(PHASE4_SEARCH_OUTER_SPEED)),
        )
        self.assertEqual(
            (phase4_align_left["speed_left"], phase4_align_left["speed_right"]),
            (45.0, float(PHASE4_ALIGN_PIVOT_SPEED)),
        )
        self.assertGreaterEqual(
            min(phase4_align_left["speed_left"], phase4_align_left["speed_right"]),
            45.0,
        )
        self.assertEqual(
            (phase5_left["speed_left"], phase5_left["speed_right"]),
            (
                65.0,
                min(100.0, float(PHASE5_BASE_SPEED + PHASE5_TURN_CLAMP)),
            ),
        )

    def test_phase6_profile_uses_grass_safe_minimum_speed(self):
        expected_speeds = {
            "w": (45.0, 45.0),
            "a": (45.0, 75.0),
            "s": (45.0, 45.0),
            "d": (75.0, 45.0),
        }
        for cmd, expected in expected_speeds.items():
            with self.subTest(cmd=cmd):
                pattern = motor_diag.get_phase_drive_pattern("6", 0, cmd)
                actual = (pattern["speed_left"], pattern["speed_right"])
                self.assertEqual(actual, expected)
                self.assertGreaterEqual(min(actual), 45.0)

    def test_phase7_wasd_always_stops(self):
        for cmd in "wasd":
            with self.subTest(cmd=cmd):
                pattern = motor_diag.get_phase_drive_pattern("7", 0, cmd)
                self.assertEqual((pattern["speed_left"], pattern["speed_right"]), (0.0, 0.0))

        with patch.object(motor_diag, "stop") as stop:
            self.assertTrue(motor_diag._apply_phase_drive_pattern("7", 0, "w"))
        stop.assert_called_once_with()

    def test_phase_profile_application_uses_profile_ramp(self):
        with patch.object(motor_diag, "set_motors") as set_motors:
            self.assertTrue(motor_diag._apply_phase_drive_pattern("3", 1, "a"))

        set_motors.assert_called_once_with(
            45.0,
            1,
            85.0,
            1,
            ramp_time=0.1,
            step_interval=0.05,
        )

    def test_runtime_can_select_phase_and_cycle_profile(self):
        args = types.SimpleNamespace(
            default_speed=50.0,
            phase="manual",
            profile=None,
            list_profiles=False,
        )
        commands = iter(("3", "m", "a", "q"))
        with patch.object(motor_diag, "parse_args", return_value=args):
            with patch.object(motor_diag, "setup"):
                with patch.object(motor_diag, "_get_command", side_effect=lambda: next(commands)):
                    with patch.object(motor_diag.time, "sleep"):
                        with patch.object(motor_diag, "set_motors") as set_motors:
                            with patch.object(motor_diag, "stop"):
                                motor_diag.main()

        set_motors.assert_called_once_with(
            45.0,
            1,
            85.0,
            1,
            ramp_time=0.1,
            step_interval=0.05,
        )

    def test_quit_always_stops_motors(self):
        args = types.SimpleNamespace(default_speed=50.0)
        with patch.object(motor_diag, "parse_args", return_value=args):
            with patch.object(motor_diag, "setup"):
                with patch.object(motor_diag, "_get_command", return_value="q"):
                    with patch.object(motor_diag, "stop") as stop:
                        motor_diag.main()
        stop.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
