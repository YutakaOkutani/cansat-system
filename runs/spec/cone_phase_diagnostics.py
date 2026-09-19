import sys
import unittest
from pathlib import Path
from unittest.mock import patch


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mission.const import Phase, CONE_PHASE5_REACH_CONFIRM_FRAMES
from mission.phases.p4 import Phase4Handler
from mission.phases.p5 import Phase5Handler
from mission.st import CanSatState


class _VisionController:
    def __init__(self, phase):
        self.st = CanSatState()
        self.st.update_navigation(phase=int(phase))
        self.devices = {}
        self.searching_flag = True
        self.time_start_searching_cone = 99.0
        self.camera_phase4_attempts = 1
        self.camera_phase4_start = 99.0
        self.camera_phase5_attempts = 1
        self.camera_phase5_start = 99.0
        self.camera_dead_since = None
        self.phase4_detect_confirm_count = 0
        self.phase4_detect_confirm_marker = None
        self.phase5_reach_confirm_count = 0
        self.phase5_entry_marker = 1.0
        self.phase_entry_time = 1.0
        self.phase4_camera_recovery_marker = 1.0
        self.phase5_timeout_limit_sec = 45.0
        self.phase5_entry_reason = "phase4_detected"
        self.time_camera_start = 99.0
        self.count_cone_lost = 0
        self.led_blink_timer = 0
        self.camera_recovery_exhausted = False
        self.camera_reinit_attempt_count = 0
        self.mission_end_reason = "RUNNING"
        self.stop_motor_calls = 0
        self.recovery_reset_calls = 0

    def update_cone_frame(
        self,
        *,
        direction=0.5,
        probability=0.0,
        reached=False,
        observation_time=100.0,
        debug=None,
        include_close_debug=True,
    ):
        if debug is None:
            debug = {"strict_red_ok": 1}
        debug = dict(debug)
        if reached and include_close_debug:
            debug["close_reached_ok"] = 1
        self.st.update_cone(
            cone_direction=direction,
            cone_probability=probability,
            cone_is_reached=reached,
            cone_valid=True,
            cone_status="ok",
            cone_debug=debug,
            observation_time=observation_time,
            observation_accepted=True,
        )

    def transition_to_give_up(self, reason):
        self.mission_end_reason = reason
        self.st.update_navigation(phase=int(Phase.PHASE7))
        self.stop_motor_calls += 1

    def reset_camera_recovery_window(self):
        self.camera_recovery_exhausted = False
        self.camera_reinit_attempt_count = 0
        self.camera_recovery_started_at = None
        self.recovery_reset_calls += 1


class ConePhaseDiagnosticsTest(unittest.TestCase):
    def test_new_phase4_entry_rearms_stale_camera_recovery_before_give_up(self):
        ctrl = _VisionController(Phase.PHASE4)
        ctrl.phase4_camera_recovery_marker = 0.0
        ctrl.camera_recovery_started_at = 0.5
        ctrl.camera_recovery_exhausted = True
        ctrl.camera_reinit_attempt_count = 3

        with patch("mission.phases.p4.time.time", return_value=100.0):
            Phase4Handler().execute(ctrl, ctrl.st.snapshot())

        self.assertEqual(ctrl.st.snapshot()["phase"], int(Phase.PHASE4))
        self.assertEqual(ctrl.recovery_reset_calls, 1)
        self.assertEqual(ctrl.mission_end_reason, "RUNNING")

    def test_phase4_timeout_stops_and_skips_directly_to_phase7(self):
        ctrl = _VisionController(Phase.PHASE4)

        with patch("mission.phases.p4.time.time", return_value=160.0):
            Phase4Handler().execute(ctrl, ctrl.st.snapshot())

        self.assertEqual(ctrl.st.snapshot()["phase"], int(Phase.PHASE7))
        self.assertEqual(ctrl.mission_end_reason, "PHASE4_TIMEOUT_GIVE_UP")
        self.assertEqual(ctrl.cone_phase_decision, "p4_timeout_to_p7_give_up")
        self.assertEqual(ctrl.stop_motor_calls, 1)

    def test_phase4_camera_recovery_exhaustion_gives_up(self):
        ctrl = _VisionController(Phase.PHASE4)
        ctrl.camera_recovery_exhausted = True
        ctrl.camera_reinit_attempt_count = 3

        with patch("mission.phases.p4.time.time", return_value=100.0):
            Phase4Handler().execute(ctrl, ctrl.st.snapshot())

        self.assertEqual(ctrl.st.snapshot()["phase"], int(Phase.PHASE7))
        self.assertEqual(ctrl.mission_end_reason, "PHASE4_CAMERA_DEAD_GIVE_UP")
        self.assertEqual(ctrl.cone_phase_decision, "p4_camera_dead_to_p7_give_up")
        self.assertEqual(ctrl.stop_motor_calls, 1)

    def test_phase4_records_direction_consistency_reset(self):
        ctrl = _VisionController(Phase.PHASE4)
        ctrl.phase4_detect_confirm_count = 1
        ctrl.phase4_detect_confirm_marker = 0.65
        ctrl.phase4_detect_track_signature = {
            "direction": 0.65,
            "bearing": None,
            "bbox_valid": False,
        }
        ctrl.update_cone_frame(direction=0.90, probability=0.30)

        with patch("mission.phases.p4.time.time", return_value=100.0):
            Phase4Handler().execute(ctrl, ctrl.st.snapshot())

        self.assertEqual(ctrl.cone_phase_decision, "p4_track_reset_direction")
        self.assertTrue(ctrl.cone_phase_detected)
        self.assertTrue(ctrl.cone_phase_centered)
        self.assertFalse(ctrl.cone_phase_direction_consistent)
        self.assertEqual(ctrl.cone_phase_confirm_count, 1)
        self.assertEqual(ctrl.cone_phase_direction_tolerance, 0.22)

    def test_phase4_heading_compensation_accepts_camera_position_shift(self):
        ctrl = _VisionController(Phase.PHASE4)
        first_debug = {
            "strict_red_ok": 1,
            "bbox_x": 145,
            "bbox_y": 260,
            "bbox_width": 30,
            "bbox_height": 60,
            "bbox_width_frac": 30 / 640,
            "bbox_height_frac": 0.125,
        }
        ctrl.st.update_imu(angle=100.0, angle_valid=True)
        ctrl.update_cone_frame(
            direction=0.25,
            probability=0.40,
            debug=first_debug,
        )
        with patch("mission.phases.p4.time.time", return_value=100.0):
            Phase4Handler().execute(ctrl, ctrl.st.snapshot())

        ctrl.st.update_imu(angle=81.4, angle_valid=True)
        ctrl.update_cone_frame(
            direction=0.55,
            probability=0.40,
            observation_time=100.6,
            debug={**first_debug, "bbox_x": 337},
        )
        with patch("mission.phases.p4.time.time", return_value=100.6):
            Phase4Handler().execute(ctrl, ctrl.st.snapshot())

        ctrl.st.update_imu(angle=80.7, angle_valid=True)
        ctrl.update_cone_frame(
            direction=0.56,
            probability=0.41,
            observation_time=100.8,
            debug={**first_debug, "bbox_x": 343},
        )
        with patch("mission.phases.p4.time.time", return_value=100.8):
            Phase4Handler().execute(ctrl, ctrl.st.snapshot())

        self.assertEqual(ctrl.st.snapshot()["phase"], int(Phase.PHASE5))

    def test_phase4_rejects_bbox_hop_even_when_direction_matches(self):
        ctrl = _VisionController(Phase.PHASE4)
        ctrl.update_cone_frame(
            direction=0.50,
            probability=0.40,
            debug={
                "strict_red_ok": 1,
                "bbox_x": 300,
                "bbox_y": 353,
                "bbox_width": 25,
                "bbox_height": 59,
                "bbox_height_frac": 59 / 480,
            },
        )
        with patch("mission.phases.p4.time.time", return_value=100.0):
            Phase4Handler().execute(ctrl, ctrl.st.snapshot())

        ctrl.update_cone_frame(
            direction=0.50,
            probability=0.40,
            observation_time=100.6,
            debug={
                "strict_red_ok": 1,
                "bbox_x": 307,
                "bbox_y": 458,
                "bbox_width": 30,
                "bbox_height": 10,
                "bbox_height_frac": 10 / 480,
            },
        )
        with patch("mission.phases.p4.time.time", return_value=100.6):
            Phase4Handler().execute(ctrl, ctrl.st.snapshot())

        self.assertEqual(ctrl.st.snapshot()["phase"], int(Phase.PHASE4))
        self.assertEqual(ctrl.phase4_detect_confirm_count, 1)
        self.assertIn(
            ctrl.cone_phase_decision,
            ("p4_track_reset_bbox_y", "p4_track_reset_bbox_size"),
        )

    def test_phase4_relaxed_score_cannot_be_strong_without_strict_red(self):
        ctrl = _VisionController(Phase.PHASE4)
        ctrl.update_cone_frame(
            direction=0.50,
            probability=0.24,
            debug={"strict_red_ok": 0},
        )

        with patch("mission.phases.p4.time.time", return_value=100.0):
            Phase4Handler().execute(ctrl, ctrl.st.snapshot())

        self.assertEqual(ctrl.phase4_detect_confirm_count, 0)
        self.assertEqual(ctrl.cone_phase_decision, "p4_low_confidence_hold")

    def test_phase4_tiny_strict_score_is_not_a_single_frame_detection(self):
        ctrl = _VisionController(Phase.PHASE4)
        ctrl.update_cone_frame(
            direction=0.50,
            probability=0.80,
            debug={"strict_red_ok": 1, "occupancy": 0.001},
        )

        with patch("mission.phases.p4.time.time", return_value=100.0):
            Phase4Handler().execute(ctrl, ctrl.st.snapshot())

        self.assertEqual(ctrl.phase4_detect_confirm_count, 0)
        self.assertFalse(ctrl.cone_phase_detected)

    def test_phase4_tiny_candidate_does_not_replace_large_live_track(self):
        ctrl = _VisionController(Phase.PHASE4)
        ctrl.update_cone_frame(
            direction=0.45,
            probability=0.35,
            debug={"strict_red_ok": 1, "occupancy": 0.02},
        )
        with patch("mission.phases.p4.time.time", return_value=100.0):
            Phase4Handler().execute(ctrl, ctrl.st.snapshot())

        original_signature = dict(ctrl.phase4_detect_track_signature)
        ctrl.update_cone_frame(
            direction=0.80,
            probability=0.17,
            observation_time=100.2,
            debug={
                "strict_red_ok": 0,
                "occupancy": 0.001,
                "candidate_probability": 0.75,
                "cone_shape_score": 0.75,
                "hue_redness_score": 0.80,
                "sv_score": 0.70,
                "roi_support_ratio": 0.50,
                "roi_absolute_support": 0.20,
            },
        )
        with patch("mission.phases.p4.time.time", return_value=100.2):
            Phase4Handler().execute(ctrl, ctrl.st.snapshot())

        self.assertEqual(ctrl.phase4_detect_track_signature, original_signature)
        self.assertEqual(ctrl.phase4_detect_confirm_count, 1)
        self.assertTrue(ctrl.cone_phase_decision.startswith("p4_track_hold_"))

    def test_phase4_close_reached_bypasses_probability_and_enters_phase5(self):
        ctrl = _VisionController(Phase.PHASE4)
        ctrl.update_cone_frame(
            direction=0.50,
            probability=0.08,
            reached=True,
            debug={},
            include_close_debug=False,
        )

        with patch("mission.phases.p4.time.time", return_value=100.0):
            Phase4Handler().execute(ctrl, ctrl.st.snapshot())

        self.assertEqual(ctrl.cone_phase_decision, "p4_close_reached_to_p5")
        self.assertTrue(ctrl.cone_phase_reached_effective)
        self.assertEqual(ctrl.cone_phase_reached_probability_threshold, 0.0)
        self.assertEqual(ctrl.st.snapshot()["phase"], int(Phase.PHASE5))

    def test_phase5_close_reached_bypasses_probability_and_confirms_twice(self):
        ctrl = _VisionController(Phase.PHASE5)
        ctrl.update_cone_frame(direction=0.50, probability=0.08, reached=True)

        with patch("mission.phases.p5.time.time", return_value=100.0):
            Phase5Handler().execute(ctrl, ctrl.st.snapshot())

        self.assertEqual(ctrl.cone_phase_decision, "p5_reached_confirm")
        self.assertTrue(ctrl.cone_phase_reached_effective)
        self.assertEqual(ctrl.cone_phase_reached_probability_threshold, 0.0)
        self.assertEqual(ctrl.cone_phase_required_confirm_frames, int(CONE_PHASE5_REACH_CONFIRM_FRAMES))

        ctrl.update_cone_frame(
            direction=0.50,
            probability=0.08,
            reached=True,
            observation_time=100.2,
        )
        with patch("mission.phases.p5.time.time", return_value=100.2):
            Phase5Handler().execute(ctrl, ctrl.st.snapshot())

        self.assertEqual(ctrl.cone_phase_decision, "p5_reached_to_p6")
        self.assertEqual(ctrl.st.snapshot()["phase"], int(Phase.PHASE6))

    def test_phase5_off_center_close_cone_requires_new_centered_confirmations(self):
        ctrl = _VisionController(Phase.PHASE5)
        handler = Phase5Handler()
        for now, direction, count in (
            (100.0, 0.50, 1), (100.2, 0.80, 0),
            (100.4, 0.50, 1), (100.6, 0.50, 2),
        ):
            ctrl.update_cone_frame(
                direction=direction, probability=0.08, reached=True,
                observation_time=now,
            )
            with patch("mission.phases.p5.time.time", return_value=now):
                handler.execute(ctrl, ctrl.st.snapshot())
            self.assertEqual(ctrl.phase5_reach_confirm_count, count)
            self.assertEqual(ctrl.count_cone_lost, 0)
            self.assertEqual(
                ctrl.st.snapshot()["phase"],
                int(Phase.PHASE6 if count == 2 else Phase.PHASE5),
            )

    def test_phase4_confirmation_counts_each_camera_sequence_once(self):
        ctrl = _VisionController(Phase.PHASE4)
        ctrl.update_cone_frame(direction=0.50, probability=0.30)

        with patch("mission.phases.p4.time.time", return_value=100.0):
            Phase4Handler().execute(ctrl, ctrl.st.snapshot())
            Phase4Handler().execute(ctrl, ctrl.st.snapshot())

        self.assertEqual(ctrl.st.snapshot()["phase"], int(Phase.PHASE4))
        self.assertEqual(ctrl.phase4_detect_confirm_count, 1)
        self.assertEqual(ctrl.cone_phase_decision, "p4_wait_new_frame")

        ctrl.update_cone_frame(direction=0.51, probability=0.31, observation_time=100.1)
        with patch("mission.phases.p4.time.time", return_value=100.1):
            Phase4Handler().execute(ctrl, ctrl.st.snapshot())

        self.assertEqual(ctrl.st.snapshot()["phase"], int(Phase.PHASE4))
        ctrl.update_cone_frame(direction=0.50, probability=0.32, observation_time=100.2)
        with patch("mission.phases.p4.time.time", return_value=100.2):
            Phase4Handler().execute(ctrl, ctrl.st.snapshot())

        self.assertEqual(ctrl.st.snapshot()["phase"], int(Phase.PHASE5))

    def test_phase5_loss_counts_each_camera_sequence_once(self):
        ctrl = _VisionController(Phase.PHASE5)
        ctrl.update_cone_frame(probability=0.0)

        with patch("mission.phases.p5.time.time", return_value=100.0):
            Phase5Handler().execute(ctrl, ctrl.st.snapshot())
            Phase5Handler().execute(ctrl, ctrl.st.snapshot())

        self.assertEqual(ctrl.count_cone_lost, 1)
        self.assertEqual(ctrl.cone_phase_decision, "p5_wait_new_frame")

    def test_phase4_stale_frame_is_not_counted(self):
        ctrl = _VisionController(Phase.PHASE4)
        ctrl.update_cone_frame(direction=0.50, probability=0.50, observation_time=90.0)

        with patch("mission.phases.p4.time.time", return_value=100.0):
            Phase4Handler().execute(ctrl, ctrl.st.snapshot())

        self.assertEqual(ctrl.phase4_detect_confirm_count, 0)
        self.assertEqual(ctrl.cone_phase_decision, "p4_camera_stale_wait")

    def test_phase4_relaxed_candidate_requires_three_distinct_frames(self):
        ctrl = _VisionController(Phase.PHASE4)
        debug = {
            "candidate_probability": 0.32,
            "cone_shape_score": 0.55,
            "hue_redness_score": 0.50,
            "sv_score": 0.40,
            "roi_support_ratio": 0.30,
        }

        for idx in range(3):
            now = 100.0 + idx * 0.1
            ctrl.update_cone_frame(
                direction=0.50 + idx * 0.01,
                probability=0.12,
                observation_time=now,
                debug=debug,
            )
            with patch("mission.phases.p4.time.time", return_value=now):
                Phase4Handler().execute(ctrl, ctrl.st.snapshot())
            if idx < 2:
                self.assertEqual(ctrl.st.snapshot()["phase"], int(Phase.PHASE4))

        self.assertEqual(ctrl.st.snapshot()["phase"], int(Phase.PHASE5))
        self.assertEqual(ctrl.cone_phase_decision, "p4_weak_confirmed_to_p5")

    def test_phase4_strict_then_roi_supported_weak_confirms_logged_cone_pass(self):
        ctrl = _VisionController(Phase.PHASE4)
        ctrl.st.update_imu(angle=52.38, angle_valid=True)
        ctrl.update_cone_frame(
            direction=0.770312,
            probability=0.356926,
            debug={
                "strict_red_ok": 1,
                "candidate_probability": 0.356926,
                "cone_shape_score": 0.469891,
                "hue_redness_score": 0.586179,
                "sv_score": 0.970826,
                "roi_support_ratio": 0.447097,
                "bbox_x": 95,
                "bbox_y": 0,
                "bbox_width": 99,
                "bbox_height": 171,
                "bbox_width_frac": 99 / 640,
                "bbox_height_frac": 171 / 480,
                "bbox_bottom_frac": 171 / 480,
            },
        )
        with patch("mission.phases.p4.time.time", return_value=100.0):
            Phase4Handler().execute(ctrl, ctrl.st.snapshot())

        ctrl.st.update_imu(angle=38.38, angle_valid=True)
        ctrl.update_cone_frame(
            direction=0.357812,
            probability=0.17,
            observation_time=100.21,
            debug={
                "strict_red_ok": 0,
                "candidate_probability": 0.521763,
                "cone_shape_score": 0.851679,
                "hue_redness_score": 0.566641,
                "sv_score": 0.909951,
                "roi_support_ratio": 0.325208,
                "bbox_x": 358,
                "bbox_y": 0,
                "bbox_width": 105,
                "bbox_height": 233,
                "bbox_width_frac": 105 / 640,
                "bbox_height_frac": 233 / 480,
                "bbox_bottom_frac": 233 / 480,
            },
        )
        with patch("mission.phases.p4.time.time", return_value=100.21):
            Phase4Handler().execute(ctrl, ctrl.st.snapshot())

        ctrl.update_cone_frame(
            direction=0.36,
            probability=0.17,
            observation_time=100.42,
            debug={
                "strict_red_ok": 0,
                "candidate_probability": 0.50,
                "cone_shape_score": 0.84,
                "hue_redness_score": 0.56,
                "sv_score": 0.90,
                "roi_support_ratio": 0.31,
                "bbox_x": 356,
                "bbox_y": 0,
                "bbox_width": 106,
                "bbox_height": 232,
                "bbox_width_frac": 106 / 640,
                "bbox_height_frac": 232 / 480,
                "bbox_bottom_frac": 232 / 480,
            },
        )
        with patch("mission.phases.p4.time.time", return_value=100.42):
            Phase4Handler().execute(ctrl, ctrl.st.snapshot())

        self.assertEqual(ctrl.st.snapshot()["phase"], int(Phase.PHASE5))
        self.assertEqual(ctrl.cone_phase_decision, "p4_confirmed_to_p5")

    def test_phase4_handheld_centered_cone_transitions_after_three_frames(self):
        """Regression for the stationary cone visible in log frames 146-152."""
        ctrl = _VisionController(Phase.PHASE4)
        debug = {
            "strict_red_ok": 0,
            "candidate_probability": 0.53,
            "cone_shape_score": 0.48,
            "hue_redness_score": 0.52,
            "sv_score": 0.97,
            "roi_support_ratio": 0.20,
            "roi_absolute_support": 0.025,
            "occupancy": 0.20,
            "bbox_x": 185,
            "bbox_y": 0,
            "bbox_width": 270,
            "bbox_height": 350,
            "bbox_width_frac": 270 / 640,
            "bbox_height_frac": 350 / 480,
            "bbox_bottom_frac": 0.73,
        }

        for idx in range(3):
            now = 100.0 + idx * 0.2
            ctrl.update_cone_frame(
                direction=0.5,
                probability=0.17,
                observation_time=now,
                debug=debug,
            )
            with patch("mission.phases.p4.time.time", return_value=now):
                Phase4Handler().execute(ctrl, ctrl.st.snapshot())

        self.assertEqual(ctrl.st.snapshot()["phase"], int(Phase.PHASE5))
        self.assertEqual(ctrl.cone_phase_decision, "p4_weak_confirmed_to_p5")

    def test_phase5_keeps_roi_supported_relaxed_cone_in_tracking(self):
        ctrl = _VisionController(Phase.PHASE5)
        ctrl.update_cone_frame(
            direction=0.5,
            probability=0.17,
            debug={
                "strict_red_ok": 0,
                "candidate_probability": 0.53,
                "cone_shape_score": 0.48,
                "hue_redness_score": 0.52,
                "sv_score": 0.97,
                "roi_support_ratio": 0.20,
                "roi_absolute_support": 0.025,
                "occupancy": 0.20,
                "bbox_height": 350,
                "bbox_height_frac": 350 / 480,
                "bbox_bottom_frac": 0.73,
            },
        )

        with patch("mission.phases.p5.time.time", return_value=100.0):
            Phase5Handler().execute(ctrl, ctrl.st.snapshot())

        self.assertEqual(ctrl.st.snapshot()["phase"], int(Phase.PHASE5))
        self.assertEqual(ctrl.count_cone_lost, 0)
        self.assertTrue(ctrl.cone_phase_detected)
        self.assertEqual(ctrl.cone_phase_decision, "p5_tracking_weak")

    def test_phase4_rejects_night_light_track_without_roi_support(self):
        ctrl = _VisionController(Phase.PHASE4)
        debug = {
            "strict_red_ok": 1,
            "candidate_probability": 0.35,
            "cone_shape_score": 0.73,
            "hue_redness_score": 0.70,
            "sv_score": 0.53,
            "roi_support_ratio": 0.0,
            "roi_absolute_support": 0.0,
            "occupancy": 0.004,
            "bbox_x": 290,
            "bbox_y": 155,
            "bbox_width": 28,
            "bbox_height": 105,
            "bbox_width_frac": 28 / 640,
            "bbox_height_frac": 105 / 480,
            "bbox_bottom_frac": 0.54,
        }

        for idx in range(3):
            now = 100.0 + idx * 0.2
            ctrl.update_cone_frame(
                direction=0.48,
                probability=0.194,
                observation_time=now,
                debug=debug,
            )
            with patch("mission.phases.p4.time.time", return_value=now):
                Phase4Handler().execute(ctrl, ctrl.st.snapshot())

        self.assertEqual(ctrl.st.snapshot()["phase"], int(Phase.PHASE4))
        self.assertEqual(ctrl.phase4_detect_confirm_count, 0)
        self.assertFalse(ctrl.cone_phase_detected)


if __name__ == "__main__":
    unittest.main()
