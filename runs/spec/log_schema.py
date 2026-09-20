import sys
import tempfile
import types
import unittest
import importlib.util
from pathlib import Path
from unittest.mock import patch

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mission.const import LOG_HEADER
from mission.st import CanSatState

sys.modules.setdefault("serial", types.SimpleNamespace())
sys.modules.setdefault("lib.bno055", types.SimpleNamespace())
sys.modules.setdefault("lib.cone_detect", types.SimpleNamespace())
sys.modules.setdefault(
    "mission.gps_util",
    types.SimpleNamespace(
        coerce_gga_metrics=lambda *_args, **_kwargs: None,
        gga_quality_ok=lambda *_args, **_kwargs: False,
        open_gps_serial=lambda *_args, **_kwargs: None,
        parse_gga_sentence=lambda *_args, **_kwargs: None,
    ),
)

SNS_MGR_PATH = PROJECT_ROOT / "mission" / "mgr" / "sns_mgr.py"
spec = importlib.util.spec_from_file_location("sns_mgr_under_test", SNS_MGR_PATH)
sns_mgr_under_test = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sns_mgr_under_test)
SensorManager = sns_mgr_under_test.SensorManager


class _LogOnlyController(SensorManager):
    def __init__(self):
        self.st = CanSatState()
        self.target_lat = 0.0
        self.target_lng = 0.0
        self.run_id = "test-run"
        self.mission_start_time = 100.0
        self.last_motor_command = {}
        self.phase7_arrival_reason = "RUNNING"
        self.mission_end_reason = "RUNNING"
        self.mission_total_timeout_triggered = False
        self.radio_disabled = True
        self.radio_control_mode = "mission"
        self.radio_last_event = "disabled:phase0_start"
        self.radio_config_source = "process_env"
        self.radio_restore_deadline = 130.0


class LogSchemaTest(unittest.TestCase):
    def test_log_worker_does_not_write_after_shutdown(self):
        ctrl = _LogOnlyController()
        ctrl._shutdown_requested = True
        with tempfile.TemporaryDirectory() as tmp_dir:
            ctrl.log_path = str(Path(tmp_dir) / "after_shutdown.csv")

            ctrl.log_thread()

            self.assertFalse(Path(ctrl.log_path).exists())

    def test_log_header_and_row_lengths_match(self):
        ctrl = _LogOnlyController()

        row = ctrl._build_log_row()

        self.assertEqual(len(row), len(LOG_HEADER))
        self.assertEqual(len(LOG_HEADER), len(set(LOG_HEADER)))
        by_name = dict(zip(LOG_HEADER, row))
        self.assertEqual(by_name["LogSchemaVersion"], 2)
        self.assertEqual(by_name["RunId"], "test-run")

    def test_cone_diagnostics_are_written_with_freshness_and_gate_context(self):
        ctrl = _LogOnlyController()
        ctrl.cone_phase_decision = "p4_direction_reset"
        ctrl.cone_phase_threshold = 0.20
        ctrl.cone_phase_reached_probability_threshold = 0.28
        ctrl.cone_phase_center_tolerance = 0.42
        ctrl.cone_phase_direction_tolerance = 0.14
        ctrl.cone_phase_required_confirm_frames = 2
        ctrl.cone_phase_detected = True
        ctrl.cone_phase_reached_effective = False
        ctrl.cone_phase_centered = True
        ctrl.cone_phase_direction_consistent = False
        ctrl.cone_phase_confirm_count = 1
        ctrl.phase4_detect_confirm_count = 1
        ctrl.phase4_detect_confirm_marker = 0.73
        ctrl.count_cone_lost = 0
        ctrl.phase5_reach_confirm_count = 0
        ctrl.phase5_entry_reason = "phase4_detected"
        ctrl.camera_fail_count = 5
        ctrl.camera_reinit_attempt_count = 2
        ctrl.camera_recovery_started_at = 100.0
        ctrl.camera_recovery_exhausted = False
        ctrl.st.update_cone(
            cone_direction=0.73,
            cone_probability=0.08,
            cone_is_reached=True,
            cone_method="as_is:close_red_region",
            cone_valid=True,
            cone_status="ok",
            cone_debug={
                "is_detected": 1,
                "raw_reached": 1,
                "close_reached_ok": 1,
                "raw_probability": 0.08,
                "candidate_probability": 0.62,
                "pre_filter_probability": 1.0,
                "roi_hist_available": 0,
                "strict_red_ok": 0,
                "penalty_flags": "strict_red_x0.08",
            },
            observation_time=101.5,
            observation_accepted=True,
        )

        with patch.object(sns_mgr_under_test.time, "time", return_value=102.0):
            row = ctrl._build_log_row()
        by_name = dict(zip(LOG_HEADER, row))

        self.assertEqual(by_name["ConeDiagSchemaVersion"], 4)
        self.assertIn("ConeROIAbsoluteSupport", by_name)
        self.assertIn("ConeROINegativeSupport", by_name)
        self.assertEqual(by_name["ConeValid"], 1)
        self.assertEqual(by_name["ConeSeq"], 1)
        self.assertEqual(by_name["ConeIsReached"], 1)
        self.assertEqual(by_name["ConeCloseReachedOK"], 1)
        self.assertEqual(by_name["ConePreFilterProb"], "1.000000")
        self.assertEqual(by_name["ConeProb"], "0.080000")
        self.assertEqual(by_name["ConePenaltyFlags"], "strict_red_x0.08")
        self.assertEqual(by_name["ConeStaleSec"], "0.50")
        self.assertEqual(by_name["ConeUpdatedElapsedSec"], "1.50")
        self.assertEqual(by_name["ConePhaseDecision"], "p4_direction_reset")
        self.assertEqual(by_name["ConePhaseReachedProbabilityThreshold"], "0.280")
        self.assertEqual(by_name["ConePhaseDirectionTolerance"], "0.140")
        self.assertEqual(by_name["ConePhaseRequiredConfirmFrames"], 2)
        self.assertEqual(by_name["ConePhaseConfirmCount"], 1)
        self.assertEqual(by_name["CameraFailCount"], 5)
        self.assertEqual(by_name["CameraReinitAttemptCount"], 2)
        self.assertEqual(by_name["CameraRecoveryElapsedSec"], "2.00")
        self.assertEqual(by_name["CameraRecoveryExhausted"], 0)

    def test_radio_columns_are_written_to_log_row(self):
        ctrl = _LogOnlyController()

        row = ctrl._build_log_row()
        by_name = dict(zip(LOG_HEADER, row))

        self.assertEqual(by_name["RadioDisabled"], 1)
        self.assertEqual(by_name["RadioControlMode"], "mission")
        self.assertEqual(by_name["RadioLastEvent"], "disabled:phase0_start")
        self.assertEqual(by_name["RadioConfigSource"], "process_env")
        self.assertEqual(by_name["RadioRestoreDeadlineElapsedSec"], "30.00")

    def test_sensor_update_elapsed_columns_are_written_to_log_row(self):
        ctrl = _LogOnlyController()
        ctrl.bno_last_acc_time = 101.25
        ctrl.bmp_last_valid_time = 102.5

        row = ctrl._build_log_row()
        by_name = dict(zip(LOG_HEADER, row))

        self.assertEqual(by_name["BNOAccUpdatedElapsedSec"], "1.25")
        self.assertEqual(by_name["BMPUpdatedElapsedSec"], "2.50")

    def test_sonar_freshness_columns_are_written_to_log_row(self):
        ctrl = _LogOnlyController()
        ctrl.st.update_sonar(
            sonar_distance_cm=24.5,
            sonar_valid=True,
            sonar_stale_sec=0.4,
            sonar_sequence=17, sonar_observed_at=1234.5,
        )

        row = ctrl._build_log_row()
        by_name = dict(zip(LOG_HEADER, row))

        self.assertEqual(by_name["SonarDistanceCm"], "24.50")
        self.assertEqual(by_name["SonarValid"], 1)
        self.assertEqual(by_name["SonarStaleSec"], "0.40")
        self.assertEqual(by_name["SonarSeq"], 17)
        self.assertEqual(by_name["SonarObservedAt"], "1234.500")
        self.assertEqual(by_name["GoalDecision"], "inactive")

    def test_phase0_exit_columns_are_written_to_log_row(self):
        ctrl = _LogOnlyController()
        ctrl.phase0_exit_reason = "impact"
        ctrl.phase0_exit_detail = "delta=8.20"

        row = ctrl._build_log_row()
        by_name = dict(zip(LOG_HEADER, row))

        self.assertEqual(by_name["Phase0ExitReason"], "impact")
        self.assertEqual(by_name["Phase0ExitDetail"], "delta=8.20")

    def test_navigation_diagnostic_columns_are_written_to_log_row(self):
        ctrl = _LogOnlyController()
        ctrl.st.update_gps(
            gps_heading=123.4,
            gps_heading_valid=True,
            gps_heading_baseline_m=2.5,
            gps_fix_accepted=True,
        )
        ctrl.phase2_stage = "offset"
        ctrl.bno_calib = {"valid": True, "value": (3, 2, 1, 2)}
        ctrl.bno_heading_offset_candidate_deg = 11.8
        ctrl.bno_heading_offset_candidate_count = 4
        ctrl.phase2_offset_reference_bno_deg = 118.0
        ctrl.phase2_offset_heading_error_deg = -2.5
        ctrl.phase2_offset_distance_m = 5.4
        ctrl.phase2_offset_path_efficiency = 0.91
        ctrl.phase2_offset_course_deg = 123.0
        ctrl.phase2_offset_bno_mean_deg = 111.0
        ctrl.phase2_offset_bno_spread_deg = 3.5
        ctrl.phase2_offset_subsegment_diff_deg = 4.0
        ctrl.phase2_offset_attempt_count = 1
        ctrl.phase2_offset_mode = "turnaround"
        ctrl.phase2_offset_turn_target_deg = 298.0
        ctrl.phase2_offset_stage_retry_count = 1
        ctrl.phase2_offset_reject_reason = ""
        ctrl.bno_heading_recovery_active = True
        ctrl.bno_heading_recovery_count = 2
        ctrl.bno_heading_recovery_seq = 1
        ctrl.st.update_navigation(
            nav_heading=120.0,
            nav_heading_source="BNO_ALIGNED",
            heading_diff=3.4,
            heading_trust=1.0,
            bno_trusted=True,
            bno_offset_deg=12.3,
            bno_offset_valid=True,
            arrival_inside=True,
            arrival_confirm_count=3,
            phase3_arrived_latched=True,
        )

        row = ctrl._build_log_row()
        by_name = dict(zip(LOG_HEADER, row))

        self.assertEqual(by_name["GpsHeading"], "123.40")
        self.assertEqual(by_name["GpsHeadingValid"], 1)
        self.assertEqual(by_name["GPSFixSeq"], 1)
        self.assertEqual(by_name["GPSHeadingBaselineM"], "2.50")
        self.assertEqual(by_name["NavHeading"], "120.00")
        self.assertEqual(by_name["NavHeadingSource"], "BNO_ALIGNED")
        self.assertEqual(by_name["HeadingDiff"], "3.40")
        self.assertEqual(by_name["HeadingTrust"], "1.00")
        self.assertEqual(by_name["BNOTrusted"], 1)
        self.assertEqual(by_name["BNOOffsetDeg"], "12.30")
        self.assertEqual(by_name["BNOOffsetValid"], 1)
        self.assertEqual(by_name["BNOOffsetCandidateDeg"], "11.80")
        self.assertEqual(by_name["BNOOffsetCandidateCount"], 4)
        self.assertEqual(by_name["Phase2Stage"], "offset")
        self.assertEqual(by_name["BNOCalibSys"], 3)
        self.assertEqual(by_name["BNOCalibGyro"], 2)
        self.assertEqual(by_name["BNOCalibAcc"], 1)
        self.assertEqual(by_name["BNOCalibMag"], 2)
        self.assertEqual(by_name["Phase2OffsetRefDeg"], "118.00")
        self.assertEqual(by_name["Phase2OffsetHeadingErrorDeg"], "-2.50")
        self.assertEqual(by_name["Phase2OffsetDistanceM"], "5.40")
        self.assertEqual(by_name["Phase2OffsetPathEfficiency"], "0.910")
        self.assertEqual(by_name["Phase2OffsetCourseDeg"], "123.00")
        self.assertEqual(by_name["Phase2OffsetBNOMeanDeg"], "111.00")
        self.assertEqual(by_name["Phase2OffsetBNOSpreadDeg"], "3.50")
        self.assertEqual(by_name["Phase2OffsetSubsegmentDiffDeg"], "4.00")
        self.assertEqual(by_name["Phase2OffsetAttemptCount"], 1)
        self.assertEqual(by_name["Phase2OffsetMode"], "turnaround")
        self.assertEqual(by_name["Phase2OffsetTurnTargetDeg"], "298.00")
        self.assertEqual(by_name["Phase2OffsetStageRetryCount"], 1)
        self.assertEqual(by_name["Phase2OffsetRejectReason"], "")
        self.assertEqual(by_name["BNORecoveryActive"], 1)
        self.assertEqual(by_name["BNORecoveryCount"], 2)
        self.assertEqual(by_name["BNORecoverySeq"], 1)
        self.assertEqual(by_name["ArrivalInside"], 1)
        self.assertEqual(by_name["ArrivalConfirmCount"], 3)
        self.assertEqual(by_name["Phase3ArrivedLatched"], 1)

    def test_bno_heading_rebootstraps_after_stable_dropout_samples(self):
        ctrl = _LogOnlyController()
        ctrl.bno_last_valid_time = 100.0
        ctrl.bno_last_valid = {"angle": 10.0}
        ctrl.bno_heading_recovery_seq = 0

        with patch.object(sns_mgr_under_test.time, "time", return_value=100.1):
            self.assertFalse(ctrl._angle_jump_ok(100.0))
        with patch.object(sns_mgr_under_test.time, "time", return_value=101.0):
            self.assertFalse(ctrl._angle_jump_ok(100.0))
        with patch.object(sns_mgr_under_test.time, "time", return_value=101.2):
            self.assertFalse(ctrl._angle_jump_ok(102.0))
        with patch.object(sns_mgr_under_test.time, "time", return_value=101.4):
            self.assertTrue(ctrl._angle_jump_ok(104.0))

        self.assertEqual(ctrl.bno_heading_recovery_seq, 1)


if __name__ == "__main__":
    unittest.main()
