import sys
import unittest
import importlib.util
import time
from pathlib import Path
from unittest.mock import patch

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

MTR_MGR_PATH = PROJECT_ROOT / "mission" / "mgr" / "mtr_mgr.py"
spec = importlib.util.spec_from_file_location("mtr_mgr_under_test", MTR_MGR_PATH)
mtr_mgr_under_test = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mtr_mgr_under_test)
MotorManager = mtr_mgr_under_test.MotorManager


from mission.const import (
    PHASE4_ALIGN_PIVOT_SPEED,
    PHASE4_SEARCH_OUTER_SPEED,
    PHASE5_BASE_SPEED,
    PHASE5_MID_SPEED,
    PHASE5_NEAR_SPEED,
)
from mission.motor_map import map_logical_wheels_to_physical

class _HeadingOnlyController(MotorManager):
    def __init__(self):
        self.bno_heading_offset_valid = False
        self.bno_heading_offset_verified = False
        self.bno_heading_offset_deg = 0.0
        self.mag_heading_offset_valid = False
        self.mag_heading_offset_deg = 0.0
        self.bno_stale_sec = 0.0
        self.phase2_offset_reference_bno_deg = None
        self.phase2_offset_heading_error_deg = 0.0
        self.phase2_offset_turn_target_deg = None
        self.phase3_no_heading_start = None
        self.motor_commands = []

    def set_motors(self, *args, **kwargs):
        self.motor_commands.append((args, kwargs))

    def stop_motors(self):
        self.motor_commands.append(((0.0, True, 0.0, True), {"cmd_type": "stop"}))

    def _normalize_heading_deg(self, value):
        try:
            deg = float(value)
        except (TypeError, ValueError):
            return None
        return deg % 360.0

    def _angle_diff_deg(self, target, current):
        diff = (float(target) - float(current) + 180.0) % 360.0 - 180.0
        return diff

    def _heading_offset_learning_ready(self, snapshot):
        return False

    def _update_bno_heading_offset_from_gps(self, snapshot):
        return None

    def _update_mag_heading_offset_from_gps(self, snapshot, gps_heading, mag_heading):
        return None

    def _bno_heading_aligned_to_gps(self, snapshot):
        heading = self._normalize_heading_deg(snapshot.get("angle"))
        if heading is None:
            return None
        return self._normalize_heading_deg(heading + self.bno_heading_offset_deg)


class Phase3HeadingTest(unittest.TestCase):
    @staticmethod
    def _fresh_camera_snapshot(**overrides):
        snapshot = {
            "cone_probability": 0.0,
            "cone_is_reached": False,
            "cone_direction": 0.5,
            "cone_valid": True,
            "cone_sequence": 1,
            "cone_updated_at": time.time(),
            "cone_debug": {},
        }
        snapshot.update(overrides)
        return snapshot

    def test_phase2_calibration_uses_forward_only_alternating_arcs(self):
        ctrl = _HeadingOnlyController()

        first = ctrl._phase2_calibration_pattern(0.0)
        second = ctrl._phase2_calibration_pattern(3.1)

        self.assertNotEqual(first[0], first[2])
        self.assertNotEqual(second[0], second[2])
        self.assertTrue(first[1])
        self.assertTrue(first[3])
        self.assertTrue(second[1])
        self.assertTrue(second[3])
        self.assertEqual(first[0], second[2])
        self.assertEqual(first[2], second[0])
        self.assertGreaterEqual(min(first[0], first[2]), 45.0)

    def test_phase2_reorient_uses_forward_only_arc(self):
        ctrl = _HeadingOnlyController()
        ctrl.phase2_offset_stage_retry_count = 1

        ctrl._drive_phase2_forward_reorient()

        args, kwargs = ctrl.motor_commands[-1]
        self.assertTrue(args[1])
        self.assertTrue(args[3])
        self.assertNotEqual(args[0], args[2])
        self.assertGreaterEqual(min(args[0], args[2]), 45.0)
        self.assertIn("forward_reorient", kwargs["cmd_type"])

    def test_phase4_search_calls_configured_forward_arc(self):
        ctrl = _HeadingOnlyController()

        ctrl._drive_phase4_camera(self._fresh_camera_snapshot())

        args, kwargs = ctrl.motor_commands[-1]
        self.assertEqual((args[0], args[2]), (45, int(PHASE4_SEARCH_OUTER_SPEED)))
        self.assertGreaterEqual(min(args[0], args[2]), 45.0)
        self.assertTrue(args[1])
        self.assertTrue(args[3])
        self.assertEqual(kwargs["cmd_type"], "phase4_search_arc")

    def test_phase4_single_candidate_applies_stronger_centering_arc(self):
        ctrl = _HeadingOnlyController()

        ctrl._drive_phase4_camera(
            self._fresh_camera_snapshot(
                cone_probability=0.5,
                cone_direction=0.0,
                cone_debug={"strict_red_ok": 1},
            )
        )

        args, kwargs = ctrl.motor_commands[-1]
        self.assertEqual((args[0], args[2]), (45, 85))
        self.assertGreaterEqual(min(args[0], args[2]), 45.0)
        self.assertTrue(args[1])
        self.assertTrue(args[3])
        self.assertEqual(kwargs["cmd_type"], "phase4_candidate_observe_arc")

    def test_phase4_close_reached_stops_immediately(self):
        ctrl = _HeadingOnlyController()

        ctrl._drive_phase4_camera(
            self._fresh_camera_snapshot(
                cone_probability=1.0,
                cone_is_reached=True,
                cone_debug={},
            )
        )

        args, kwargs = ctrl.motor_commands[-1]
        self.assertEqual((args[0], args[2]), (0.0, 0.0))
        self.assertEqual(kwargs["cmd_type"], "stop")

    def test_phase4_tiny_single_frame_does_not_change_search_motor_command(self):
        ctrl = _HeadingOnlyController()

        ctrl._drive_phase4_camera(
            self._fresh_camera_snapshot(
                cone_probability=0.17,
                cone_direction=0.9,
                cone_debug={
                    "strict_red_ok": 0,
                    "occupancy": 0.001,
                    "candidate_probability": 0.80,
                    "cone_shape_score": 0.80,
                    "hue_redness_score": 0.80,
                    "sv_score": 0.70,
                    "roi_support_ratio": 0.60,
                    "roi_absolute_support": 0.20,
                },
            )
        )

        args, kwargs = ctrl.motor_commands[-1]
        self.assertEqual((args[0], args[2]), (45, int(PHASE4_SEARCH_OUTER_SPEED)))
        self.assertEqual(kwargs["cmd_type"], "phase4_search_arc")

    def test_phase4_tiny_candidate_requires_three_consistent_frames_to_steer(self):
        ctrl = _HeadingOnlyController()
        debug = {
            "strict_red_ok": 0,
            "occupancy": 0.001,
            "candidate_probability": 0.80,
            "cone_shape_score": 0.80,
            "hue_redness_score": 0.80,
            "sv_score": 0.70,
            "roi_support_ratio": 0.60,
            "roi_absolute_support": 0.20,
        }
        for sequence, now in ((1, 100.0), (2, 100.4), (3, 100.8)):
            with patch.object(mtr_mgr_under_test.time, "time", return_value=now):
                ctrl._drive_phase4_camera(
                    self._fresh_camera_snapshot(
                        cone_sequence=sequence,
                        cone_updated_at=now,
                        cone_probability=0.17,
                        cone_direction=0.8,
                        cone_debug=debug,
                    )
                )
            if sequence < 3:
                self.assertEqual(
                    ctrl.motor_commands[-1][1]["cmd_type"],
                    "phase4_search_arc",
                )

        args, kwargs = ctrl.motor_commands[-1]
        self.assertEqual((args[0], args[2]), (81, 45))
        self.assertEqual(kwargs["cmd_type"], "phase4_candidate_capture_arc")

    def test_phase4_centered_single_candidate_moves_straight_without_rotation(self):
        ctrl = _HeadingOnlyController()

        ctrl._drive_phase4_camera(
            self._fresh_camera_snapshot(
                cone_probability=0.5,
                cone_direction=0.5,
                cone_debug={"strict_red_ok": 1},
            )
        )

        args, kwargs = ctrl.motor_commands[-1]
        self.assertEqual((args[0], args[2]), (45, 45))
        self.assertTrue(args[1])
        self.assertTrue(args[3])
        self.assertEqual(kwargs["cmd_type"], "phase4_candidate_observe_forward")

    def test_phase4_near_center_candidate_uses_proportional_right_steering(self):
        ctrl = _HeadingOnlyController()

        ctrl._drive_phase4_camera(
            self._fresh_camera_snapshot(
                cone_probability=0.5,
                cone_direction=0.58,
                cone_debug={"strict_red_ok": 1},
            )
        )

        args, kwargs = ctrl.motor_commands[-1]
        self.assertGreater(args[0], args[2])
        self.assertLess(args[0], 60)
        self.assertEqual(args[2], 45)
        self.assertEqual(kwargs["cmd_type"], "phase4_candidate_observe_arc")

        physical = map_logical_wheels_to_physical(
            args[0],
            args[1],
            args[2],
            args[3],
        )
        # Physical MTR1 is the left wheel and must be faster for a right turn.
        self.assertGreater(physical[0], physical[2])

    def test_phase4_image_left_steers_physical_right_wheel_faster(self):
        ctrl = _HeadingOnlyController()

        ctrl._drive_phase4_camera(
            self._fresh_camera_snapshot(
                cone_probability=0.5,
                cone_direction=0.2,
                cone_debug={"strict_red_ok": 1},
            )
        )

        args, _kwargs = ctrl.motor_commands[-1]
        physical = map_logical_wheels_to_physical(
            args[0],
            args[1],
            args[2],
            args[3],
        )
        # Physical MTR2 is the right wheel and must be faster for a left turn.
        self.assertGreater(physical[2], physical[0])

    def test_phase4_roi_supported_weak_candidate_starts_center_hold(self):
        ctrl = _HeadingOnlyController()

        ctrl._drive_phase4_camera(
            self._fresh_camera_snapshot(
                cone_probability=0.17,
                cone_direction=0.5,
                cone_debug={
                    "strict_red_ok": 0,
                    "occupancy": 0.20,
                    "candidate_probability": 0.53,
                    "cone_shape_score": 0.48,
                    "hue_redness_score": 0.52,
                    "sv_score": 0.97,
                    "roi_support_ratio": 0.20,
                    "roi_absolute_support": 0.025,
                    "bbox_height": 350,
                    "bbox_height_frac": 350 / 480,
                    "bbox_bottom_frac": 0.73,
                },
            )
        )

        args, kwargs = ctrl.motor_commands[-1]
        self.assertEqual((args[0], args[2]), (45, 45))
        self.assertEqual(kwargs["cmd_type"], "phase4_candidate_observe_forward")

    def test_phase4_low_probability_candidate_without_roi_keeps_searching(self):
        ctrl = _HeadingOnlyController()

        ctrl._drive_phase4_camera(
            self._fresh_camera_snapshot(
                cone_probability=0.194,
                cone_direction=0.48,
                cone_debug={
                    "strict_red_ok": 1,
                    "occupancy": 0.004,
                    "candidate_probability": 0.35,
                    "cone_shape_score": 0.73,
                    "hue_redness_score": 0.70,
                    "sv_score": 0.53,
                    "roi_support_ratio": 0.0,
                    "roi_absolute_support": 0.0,
                    "bbox_height": 105,
                    "bbox_height_frac": 105 / 480,
                    "bbox_bottom_frac": 0.54,
                },
            )
        )

        args, kwargs = ctrl.motor_commands[-1]
        self.assertEqual((args[0], args[2]), (45, int(PHASE4_SEARCH_OUTER_SPEED)))
        self.assertEqual(kwargs["cmd_type"], "phase4_search_arc")

    def test_phase4_loss_rechecks_last_direction_then_resumes_search(self):
        ctrl = _HeadingOnlyController()
        with patch.object(mtr_mgr_under_test.time, "time", return_value=100.0):
            ctrl._drive_phase4_camera(
                self._fresh_camera_snapshot(
                    cone_sequence=1,
                    cone_updated_at=100.0,
                    cone_probability=0.5,
                    cone_direction=0.2,
                    cone_debug={"strict_red_ok": 1},
                )
            )

        with patch.object(mtr_mgr_under_test.time, "time", return_value=100.6):
            ctrl._drive_phase4_camera(
                self._fresh_camera_snapshot(
                    cone_sequence=2,
                    cone_updated_at=100.6,
                )
            )
        self.assertEqual(
            ctrl.motor_commands[-1][1]["cmd_type"],
            "phase4_candidate_observe_arc",
        )

        with patch.object(mtr_mgr_under_test.time, "time", return_value=101.2):
            ctrl._drive_phase4_camera(
                self._fresh_camera_snapshot(
                    cone_sequence=3,
                    cone_updated_at=101.2,
                )
            )
        self.assertEqual(
            ctrl.motor_commands[-1][1]["cmd_type"],
            "phase4_candidate_observe_arc",
        )

    def test_phase4_three_fresh_frames_confirm_left_image_steering(self):
        ctrl = _HeadingOnlyController()
        for sequence, now in ((1, 100.0), (2, 100.4), (3, 100.8)):
            with patch.object(mtr_mgr_under_test.time, "time", return_value=now):
                ctrl._drive_phase4_camera(
                    self._fresh_camera_snapshot(
                        cone_sequence=sequence,
                        cone_updated_at=now,
                        cone_probability=0.5,
                        cone_direction=0.2,
                        cone_debug={"strict_red_ok": 1, "occupancy": 0.01},
                    )
                )

        args, kwargs = ctrl.motor_commands[-1]
        self.assertEqual((args[0], args[2]), (45, 81))
        self.assertEqual(kwargs["cmd_type"], "phase4_candidate_capture_arc")

    def test_phase4_three_fresh_frames_confirm_right_image_steering(self):
        ctrl = _HeadingOnlyController()
        for sequence, now in ((1, 100.0), (2, 100.4), (3, 100.8)):
            with patch.object(mtr_mgr_under_test.time, "time", return_value=now):
                ctrl._drive_phase4_camera(
                    self._fresh_camera_snapshot(
                        cone_sequence=sequence,
                        cone_updated_at=now,
                        cone_probability=0.5,
                        cone_direction=0.8,
                        cone_debug={"strict_red_ok": 1, "occupancy": 0.01},
                    )
                )

        args, kwargs = ctrl.motor_commands[-1]
        self.assertEqual((args[0], args[2]), (81, 45))
        self.assertEqual(kwargs["cmd_type"], "phase4_candidate_capture_arc")

    def test_phase5_approach_calls_configured_maximum_steering_arc(self):
        ctrl = _HeadingOnlyController()

        ctrl._drive_phase5_camera(
            self._fresh_camera_snapshot(
                cone_probability=0.5,
                cone_direction=0.0,
            )
        )

        args, kwargs = ctrl.motor_commands[-1]
        self.assertEqual((args[0], args[2]), (45.0, 100.0))
        self.assertTrue(args[1])
        self.assertTrue(args[3])
        self.assertEqual(kwargs["cmd_type"], "phase5_approach_steer_left")

    def test_phase5_image_right_steers_right(self):
        ctrl = _HeadingOnlyController()

        ctrl._drive_phase5_camera(
            self._fresh_camera_snapshot(
                cone_probability=0.5,
                cone_direction=1.0,
            )
        )

        args, kwargs = ctrl.motor_commands[-1]
        self.assertEqual((args[0], args[2]), (100.0, 45.0))
        self.assertEqual(kwargs["cmd_type"], "phase5_approach_steer_right")

    def test_phase5_direction_filter_updates_once_per_camera_sequence(self):
        ctrl = _HeadingOnlyController()
        ctrl._reset_phase45_camera_track()
        ctrl.phase45_filtered_cone_dir = 0.20
        snapshot = self._fresh_camera_snapshot(
            cone_sequence=10,
            cone_updated_at=100.0,
            cone_probability=0.5,
            cone_direction=0.90,
        )

        with patch.object(mtr_mgr_under_test.time, "time", return_value=100.0):
            ctrl._drive_phase5_camera(snapshot)
        first_filtered = ctrl.phase45_filtered_cone_dir
        first_command = ctrl.motor_commands[-1]

        with patch.object(mtr_mgr_under_test.time, "time", return_value=100.2):
            ctrl._drive_phase5_camera(snapshot)

        self.assertAlmostEqual(first_filtered, 0.704)
        self.assertAlmostEqual(ctrl.phase45_filtered_cone_dir, first_filtered)
        self.assertEqual(ctrl.motor_commands[-1], first_command)

        next_snapshot = dict(snapshot, cone_sequence=11, cone_updated_at=100.4)
        with patch.object(mtr_mgr_under_test.time, "time", return_value=100.4):
            ctrl._drive_phase5_camera(next_snapshot)

        self.assertGreater(ctrl.phase45_filtered_cone_dir, first_filtered)

    def test_phase5_approach_speed_decreases_with_red_occupancy(self):
        cases = (
            (0.0, PHASE5_BASE_SPEED),
            (0.04, PHASE5_MID_SPEED),
            (0.10, PHASE5_NEAR_SPEED),
        )
        for occupancy, expected_speed in cases:
            with self.subTest(occupancy=occupancy):
                ctrl = _HeadingOnlyController()
                ctrl._drive_phase5_camera(
                    self._fresh_camera_snapshot(
                        cone_probability=0.5,
                        cone_direction=0.5,
                        cone_debug={
                            "strict_red_ok": 1,
                            "occupancy": occupancy,
                            "frame_red_occupancy": occupancy,
                        },
                    )
                )

                args, kwargs = ctrl.motor_commands[-1]
                self.assertEqual(
                    (args[0], args[2]),
                    (float(expected_speed), float(expected_speed)),
                )
                self.assertEqual(kwargs["cmd_type"], "phase5_approach_forward")

    def test_phase5_close_reached_stops_immediately(self):
        ctrl = _HeadingOnlyController()

        ctrl._drive_phase5_camera(
            self._fresh_camera_snapshot(
                cone_probability=1.0,
                cone_is_reached=True,
                cone_debug={},
            )
        )

        args, kwargs = ctrl.motor_commands[-1]
        self.assertEqual((args[0], args[2]), (0.0, 0.0))
        self.assertEqual(kwargs["cmd_type"], "stop")

    def test_phase5_close_off_center_cone_steers_using_latest_image(self):
        for direction, expected_side in ((0.2, "left"), (0.8, "right")):
            with self.subTest(direction=direction):
                ctrl = _HeadingOnlyController()
                ctrl.phase45_filtered_cone_dir = 1.0 - direction
                ctrl.phase45_filtered_cone_seq = 1
                ctrl._drive_phase5_camera(self._fresh_camera_snapshot(
                    cone_sequence=1, cone_probability=0.08,
                    cone_is_reached=True, cone_direction=direction,
                    cone_debug={"occupancy": 0.10},
                ))
                args, kwargs = ctrl.motor_commands[-1]
                self.assertEqual(kwargs["cmd_type"], f"phase5_approach_steer_{expected_side}")
                self.assertEqual(args[0] > args[2], expected_side == "right")

    def test_phase5_small_offset_has_stronger_steering_without_saturation(self):
        ctrl = _HeadingOnlyController()
        ctrl._drive_phase5_camera(self._fresh_camera_snapshot(
            cone_probability=0.5, cone_direction=0.60,
        ))
        args, _ = ctrl.motor_commands[-1]
        self.assertAlmostEqual(args[0], 95.0)
        self.assertAlmostEqual(args[2], 75.0)

    def test_phase5_roi_supported_weak_candidate_remains_trackable(self):
        ctrl = _HeadingOnlyController()

        ctrl._drive_phase5_camera(
            self._fresh_camera_snapshot(
                cone_probability=0.17,
                cone_direction=0.5,
                cone_debug={
                    "strict_red_ok": 0,
                    "occupancy": 0.20,
                    "candidate_probability": 0.53,
                    "cone_shape_score": 0.48,
                    "hue_redness_score": 0.52,
                    "sv_score": 0.97,
                    "roi_support_ratio": 0.20,
                    "roi_absolute_support": 0.025,
                    "bbox_height": 350,
                    "bbox_height_frac": 350 / 480,
                    "bbox_bottom_frac": 0.73,
                },
            )
        )

        args, kwargs = ctrl.motor_commands[-1]
        self.assertEqual((args[0], args[2]), (75, 75))
        self.assertEqual(kwargs["cmd_type"], "phase5_approach_forward")

    def test_phase5_brief_loss_keeps_turning_toward_last_image_direction(self):
        ctrl = _HeadingOnlyController()
        with patch.object(mtr_mgr_under_test.time, "time", return_value=100.0):
            ctrl._drive_phase5_camera(
                self._fresh_camera_snapshot(
                    cone_sequence=1,
                    cone_updated_at=100.0,
                    cone_probability=0.5,
                    cone_direction=0.9,
                )
            )

        with patch.object(mtr_mgr_under_test.time, "time", return_value=100.2):
            ctrl._drive_phase5_camera(
                self._fresh_camera_snapshot(
                    cone_sequence=2,
                    cone_updated_at=100.2,
                )
            )

        args, kwargs = ctrl.motor_commands[-1]
        self.assertGreater(args[0], args[2])
        self.assertEqual(kwargs["cmd_type"], "phase5_reacquire_right")

        with patch.object(mtr_mgr_under_test.time, "time", return_value=101.0):
            ctrl._drive_phase5_camera(
                self._fresh_camera_snapshot(
                    cone_sequence=3,
                    cone_updated_at=101.0,
                )
            )

        args, kwargs = ctrl.motor_commands[-1]
        self.assertEqual((args[0], args[2]), (0.0, 0.0))
        self.assertEqual(kwargs["cmd_type"], "stop")

    def test_stale_camera_observation_stops_phase4_motion(self):
        ctrl = _HeadingOnlyController()

        ctrl._drive_phase4_camera(
            self._fresh_camera_snapshot(cone_updated_at=time.time() - 2.0)
        )

        args, kwargs = ctrl.motor_commands[-1]
        self.assertEqual((args[0], args[2]), (0.0, 0.0))
        self.assertEqual(kwargs["cmd_type"], "stop")

    def test_normal_roi_frame_interval_does_not_stop_phase4_motion(self):
        ctrl = _HeadingOnlyController()
        now = time.time()

        ctrl._drive_phase4_camera(
            self._fresh_camera_snapshot(cone_updated_at=now - 0.85)
        )

        args, kwargs = ctrl.motor_commands[-1]
        self.assertEqual((args[0], args[2]), (45, int(PHASE4_SEARCH_OUTER_SPEED)))
        self.assertEqual(kwargs["cmd_type"], "phase4_search_arc")

    def test_phase4_search_remains_continuous_across_distinct_frames(self):
        ctrl = _HeadingOnlyController()
        ctrl._reset_phase4_search_cycle(100.0)

        for sequence, now in ((1, 100.0), (2, 100.6), (3, 101.2), (4, 101.8)):
            with patch.object(mtr_mgr_under_test.time, "time", return_value=now):
                ctrl._drive_phase4_camera(
                    self._fresh_camera_snapshot(
                        cone_sequence=sequence,
                        cone_updated_at=now,
                    )
                )
            self.assertEqual(ctrl.phase4_search_state, "drive")
            self.assertEqual(
                ctrl.motor_commands[-1][1]["cmd_type"],
                "phase4_search_arc",
            )

    def test_phase4_reentry_turns_toward_last_phase5_direction(self):
        ctrl = _HeadingOnlyController()
        ctrl.phase45_filtered_cone_dir = 0.20
        ctrl._reset_phase4_search_cycle(100.0)

        with patch.object(mtr_mgr_under_test.time, "time", return_value=100.0):
            ctrl._drive_phase4_camera(
                self._fresh_camera_snapshot(cone_sequence=1, cone_updated_at=100.0)
            )

        args, kwargs = ctrl.motor_commands[-1]
        self.assertEqual((args[0], args[2]), (45, PHASE4_ALIGN_PIVOT_SPEED))
        self.assertEqual(kwargs["cmd_type"], "phase4_reacquire_last_candidate")

    def test_phase2_offset_hold_steers_back_to_relative_bno_heading(self):
        ctrl = _HeadingOnlyController()
        ctrl.phase2_offset_reference_bno_deg = 100.0

        speed_l, speed_r, error = ctrl._phase2_offset_hold_speeds(
            {"angle_valid": True, "angle": 90.0}
        )

        self.assertGreater(speed_l, speed_r)
        self.assertAlmostEqual(error, 10.0)

    def test_phase2_offset_hold_handles_heading_wraparound(self):
        ctrl = _HeadingOnlyController()
        ctrl.phase2_offset_reference_bno_deg = 2.0

        speed_l, speed_r, error = ctrl._phase2_offset_hold_speeds(
            {"angle_valid": True, "angle": 358.0}
        )

        self.assertGreater(speed_l, speed_r)
        self.assertAlmostEqual(error, 4.0)

    def test_uses_bno_after_gps_alignment_is_valid(self):
        ctrl = _HeadingOnlyController()
        ctrl.bno_heading_offset_valid = True
        ctrl.bno_heading_offset_verified = True
        ctrl.bno_heading_offset_deg = 10.0

        heading, source = ctrl._phase3_heading(
            {
                "gps_heading_valid": True,
                "gps_heading": 90.0,
                "angle_valid": True,
                "angle": 30.0,
                "mag": [0.0, 0.0, 0.0],
            }
        )

        self.assertEqual(source, "BNO_ALIGNED")
        self.assertAlmostEqual(heading, 40.0)

    def test_falls_back_to_gps_before_bno_alignment_is_valid(self):
        ctrl = _HeadingOnlyController()

        heading, source = ctrl._phase3_heading(
            {
                "gps_heading_valid": True,
                "gps_heading": 90.0,
                "angle_valid": True,
                "angle": 30.0,
                "mag": [0.0, 0.0, 0.0],
            }
        )

        self.assertEqual(source, "GPS_PRIMARY")
        self.assertAlmostEqual(heading, 90.0)

    def test_does_not_fall_back_to_mag_when_bno_and_gps_are_untrusted(self):
        ctrl = _HeadingOnlyController()
        ctrl.mag_heading_offset_valid = True
        ctrl.mag_heading_offset_deg = 25.0

        heading, source = ctrl._phase3_heading(
            {
                "gps_heading_valid": False,
                "gps_heading": 0.0,
                "angle_valid": False,
                "angle": 30.0,
                "mag": [100.0, 25.0, 0.0],
            }
        )

        self.assertIsNone(heading)
        self.assertNotEqual(source, "MAG_ALIGNED")

    def test_unverified_bno_is_not_used_when_gps_course_is_missing(self):
        ctrl = _HeadingOnlyController()
        ctrl.bno_heading_offset_valid = True
        ctrl.bno_heading_offset_verified = False
        ctrl.bno_heading_offset_deg = 120.0

        heading, source = ctrl._phase3_heading(
            {
                "gps_heading_valid": False,
                "angle_valid": True,
                "angle": 30.0,
            }
        )

        self.assertIsNone(heading)
        self.assertEqual(source, "NO_HEADING_SOURCE")

    def test_phase3_large_gps_error_uses_forward_arc_not_pivot(self):
        ctrl = _HeadingOnlyController()
        ctrl.phase3_heading_entry_ready = True

        snapshot = {
            "gps_heading_valid": True,
            "gps_heading": 280.0,
            "gps_fix_seq": 10,
            "angle_valid": False,
            "angle": 0.0,
        }

        _, source, diff = ctrl._drive_phase3_navigation(snapshot, 90.0, now=10.0)

        args, kwargs = ctrl.motor_commands[-1]
        self.assertEqual(source, "GPS_PRIMARY")
        self.assertAlmostEqual(diff, 170.0)
        self.assertEqual(kwargs["cmd_type"], "phase3_gps_arc")
        self.assertTrue(args[1])
        self.assertTrue(args[3])
        self.assertGreater(args[0], 0.0)
        self.assertGreater(args[2], 0.0)
        self.assertGreater(args[0], args[2])

    def test_phase3_missing_heading_probes_straight_then_stops(self):
        ctrl = _HeadingOnlyController()
        snapshot = {
            "gps_heading_valid": False,
            "angle_valid": True,
            "angle": 30.0,
        }

        ctrl._drive_phase3_navigation(snapshot, 90.0, now=10.0)
        first_args, first_kwargs = ctrl.motor_commands[-1]
        ctrl._drive_phase3_navigation(snapshot, 90.0, now=14.0)
        _, second_kwargs = ctrl.motor_commands[-1]

        self.assertEqual(first_kwargs["cmd_type"], "phase3_gps_probe")
        self.assertEqual(first_args[0], first_args[2])
        self.assertEqual(second_kwargs["cmd_type"], "stop")

    def test_phase3_gps_loss_stops_even_with_a_heading_source(self):
        ctrl = _HeadingOnlyController()
        ctrl.phase3_heading_entry_ready = True
        ctrl.bno_heading_offset_valid = True
        ctrl.bno_heading_offset_verified = True
        ctrl.bno_heading_offset_deg = 0.0
        snapshot = {
            "gps_detect": 0,
            "gps_heading_valid": False,
            "angle_valid": True,
            "angle": 30.0,
        }

        heading, source, diff = ctrl._drive_phase3_navigation(snapshot, 90.0, now=10.0)

        _, kwargs = ctrl.motor_commands[-1]
        self.assertIsNone(heading)
        self.assertEqual(source, "GPS_LOST_STOP")
        self.assertIsNone(diff)
        self.assertEqual(kwargs["cmd_type"], "stop")

    def test_phase3_active_gps_keeps_probing_until_course_is_available(self):
        ctrl = _HeadingOnlyController()
        snapshot = {
            "gps_detect": 1,
            "gps_heading_valid": False,
            "angle_valid": False,
            "angle": 0.0,
        }

        ctrl._drive_phase3_navigation(snapshot, 90.0, now=10.0)
        ctrl._drive_phase3_navigation(snapshot, 90.0, now=20.0)

        args, kwargs = ctrl.motor_commands[-1]
        self.assertEqual(kwargs["cmd_type"], "phase3_gps_probe")
        self.assertEqual(args[0], 70)
        self.assertEqual(args[2], 70)

    def test_phase3_large_verified_bno_error_uses_powered_forward_arc(self):
        ctrl = _HeadingOnlyController()
        ctrl.phase3_heading_entry_ready = True
        ctrl.bno_heading_offset_valid = True
        ctrl.bno_heading_offset_verified = True
        ctrl.bno_heading_offset_deg = 0.0

        snapshot = {
            "gps_heading_valid": False,
            "angle_valid": True,
            "angle": 280.0,
        }

        _, source, diff = ctrl._drive_phase3_navigation(snapshot, 90.0, now=10.0)

        args, kwargs = ctrl.motor_commands[-1]
        self.assertEqual(source, "BNO_ALIGNED")
        self.assertAlmostEqual(diff, 170.0)
        self.assertEqual(kwargs["cmd_type"], "phase3_bno_large_arc")
        # Positive compass-heading error requires a right turn: logical left
        # wheel is faster, but both remain powered to avoid dragging on grass.
        self.assertEqual(args[0], 85)
        self.assertEqual(args[2], 45)
        self.assertTrue(args[1])
        self.assertTrue(args[3])


if __name__ == "__main__":
    unittest.main()
