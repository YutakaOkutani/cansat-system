import math
import time
import csv
import threading

from mission.const import (
    CAMERA_CONTROL_INVERT_X,
    CAMERA_FRAME_STALE_STOP_SEC,
    DEFAULT_BNO_CALIB,
    DEFAULT_VECTOR3,
    DEVICE_KEYS,
    HEADING_OFFSET_LEARN_BOOTSTRAP_MAX_RESIDUAL_DEG,
    HEADING_OFFSET_LEARN_BOOTSTRAP_SAMPLES,
    HEADING_OFFSET_LEARN_MAX_ABS_OFFSET_DEG,
    HEADING_OFFSET_LEARN_MAX_RESIDUAL_DEG,
    HEADING_OFFSET_LEARN_MIN_SPEED_MPS,
    HEADING_MAG_CALIB_MAX,
    HEADING_SOURCE_BNO,
    HEADING_SOURCE_GPS,
    HEADING_SOURCE_INVALID,
    HEADING_SOURCE_JOINER,
    HEADING_WEIGHT_BNO_BASE,
    HEADING_WEIGHT_BNO_MAX,
    HEADING_WEIGHT_BNO_MIN,
    HEADING_WEIGHT_BNO_STEP,
    HEADING_WEIGHT_GPS,
    GPS_HEADING_BASELINE_MIN_DIST,
    LED_SIGNAL_COUNT,
    MAIN_LOOP_INTERVAL,
    MISSION_PHASE_TIME_BUDGETS,
    MISSION_PHASE_TIMEOUT_TRANSITIONS,
    MISSION_TIMEOUT_TOTAL,
    PHASE3_BNO_GPS_OFFSET_ALPHA,
    PHASE3_BNO_GPS_OFFSET_MAX_STEP_DEG,
    PHASE2_OFFSET_LEG_MAX_TIME,
    PHASE2_STAGE_ESCAPE,
    PHASE2_OFFSET_MODE_COLLECT,
    ROI_PATH_1,
    ROI_PATH_2,
    Phase,
)
from mission.cone_candidate import camera_has_visible_cone
from mission.mgr import HardwareManager, LedManager, MotorManager, RadioManager, SensorManager
from mission.phases import (
    Phase0Handler,
    Phase1Handler,
    Phase2Handler,
    Phase3Handler,
    Phase4Handler,
    Phase5Handler,
    Phase6Handler,
    Phase7Handler,
)
from mission.st import CanSatState
from mission.run_bundle import create_run_bundle


class CanSatController(HardwareManager, SensorManager, MotorManager, LedManager, RadioManager):
    def __init__(self, mission_config, run_context, log_dir=None):
        self.st = CanSatState()
        self.mission_config = mission_config
        self.run_context = run_context
        self.target_lat = mission_config.target.latitude
        self.target_lng = mission_config.target.longitude
        self.radio_config = mission_config.radio
        self.camera_control_invert_x = bool(CAMERA_CONTROL_INVERT_X)
        self.roi_path_1 = ROI_PATH_1
        self.roi_path_2 = ROI_PATH_2

        self.run_bundle = create_run_bundle(mission_config, run_context, log_root=log_dir)
        self.run_id = self.run_bundle.run_id
        self.run_stem = self.run_id
        self.run_dir = str(self.run_bundle.run_dir)
        self.log_path = str(self.run_bundle.log_path)
        self.capture_reached_path = str(self.run_bundle.reached_image_path)
        self.camera_evidence_dir = str(self.run_bundle.camera_dir)

        self.devices = {key: None for key in DEVICE_KEYS}
        self.i2c_lock = threading.RLock()
        self._log_lock = threading.Lock()
        self.led_blink_timer = 0
        self.searching_flag = False
        self.count_cone_lost = 0
        self.time_phase1_start = None
        self.phase1_offset_samples = []
        self.phase1_offset_last_gps_fix_seq = 0
        self.phase1_offset_candidate_valid = False
        self.phase1_offset_candidate_deg = 0.0
        self.phase1_offset_distance_m = 0.0
        self.phase1_offset_path_efficiency = 0.0
        self.phase1_offset_bno_spread_deg = 0.0
        self.phase1_offset_subsegment_diff_deg = 0.0
        self.phase1_offset_reject_reason = ""
        self.phase0_entry_marker = None
        self.phase0_initial_alt = None
        self.phase0_drop_detect_time = None
        self.phase0_drop_detect_reason = None
        self.phase0_exit_reason = ""
        self.phase0_exit_detail = ""
        self.phase0_acc_baseline = None
        self.phase0_impact_confirm_count = 0
        self.time_phase3_start = 0.0
        self.time_phase4_start = 0.0
        self.time_phase5_start = 0.0
        self.time_start_searching_cone = 0.0
        self.time_camera_start = 0.0
        self.motor_state = {}
        self.last_motor_command = {
            "type": "init",
            "updated_ms": 0,
            "motor1_speed": 0.0,
            "motor1_forward": 1,
            "motor2_speed": 0.0,
            "motor2_forward": 1,
        }
        self.bno_fail_count = 0
        self.bno_last_reinit_time = 0.0
        self.bno_last_valid = {
            "acc": list(DEFAULT_VECTOR3),
            "gyro": list(DEFAULT_VECTOR3),
            "mag": list(DEFAULT_VECTOR3),
            "angle": 0.0,
        }
        self.bno_last_acc_time = 0.0
        self.bno_last_valid_time = 0.0
        self.bno_stale_sec = 0.0
        self.bno_acc_stale_sec = 0.0
        self.bno_heading_recovery_active = False
        self.bno_heading_recovery_count = 0
        self.bno_heading_recovery_seq = 0
        self._bno_heading_recovery_candidate = None
        self.bno_calib = dict(DEFAULT_BNO_CALIB)
        self.bmp_last_valid_time = 0.0
        self.bmp_stale_sec = 0.0
        self.bmp_fail_count = 0
        self.bmp_last_reinit_time = 0.0
        self.phase2_start_time = None
        self.phase2_stage = PHASE2_STAGE_ESCAPE
        self.phase2_stage_start = None
        self.phase2_calib_ready_since = None
        self.phase2_offset_reference_bno_deg = None
        self.phase2_offset_heading_error_deg = 0.0
        self.phase2_offset_samples = []
        self.phase2_offset_last_gps_fix_seq = 0
        self.phase2_offset_distance_m = 0.0
        self.phase2_offset_path_efficiency = 0.0
        self.phase2_offset_course_deg = 0.0
        self.phase2_offset_bno_mean_deg = 0.0
        self.phase2_offset_bno_spread_deg = 0.0
        self.phase2_offset_subsegment_diff_deg = 0.0
        self.phase2_offset_attempt_count = 0
        self.phase2_offset_mode = PHASE2_OFFSET_MODE_COLLECT
        self.phase2_offset_turn_target_deg = None
        self.phase2_offset_turn_confirm_count = 0
        self.phase2_offset_stage_retry_count = 0
        self.phase2_offset_settle_until = 0.0
        self.phase2_offset_leg_start_time = 0.0
        self.phase2_offset_progress_anchor_time = 0.0
        self.phase2_offset_progress_anchor_distance_m = 0.0
        self.phase2_offset_near_goal_active = False
        self.phase2_offset_leg_time_limit_sec = PHASE2_OFFSET_LEG_MAX_TIME
        self.phase2_offset_observed_bno_recovery_seq = 0
        self.phase2_offset_reject_reason = ""
        self.phase2_offset_mode_start_time = 0.0
        self.phase2_entry_ready_count = 0
        self.phase2_heading_quality_valid = False
        self.phase2_last_bno_recovery_time = 0.0
        self.phase3_heading_entry_ready = False
        self.roi_img = None
        self.camera_fail_count = 0
        self.camera_last_reinit = 0.0
        self.camera_dead_since = None
        self.camera_recovery_started_at = None
        self.camera_reinit_attempt_count = 0
        self.camera_recovery_exhausted = False
        self.phase4_camera_recovery_marker = None
        self.camera_phase4_attempts = 0
        self.camera_phase5_attempts = 0
        self.camera_phase4_start = None
        self.camera_phase5_start = None
        self.camera_candidate_last_dir = None
        self.camera_candidate_last_time = 0.0
        self.camera_candidate_streak = 0
        self.camera_stable_dir = None
        self.camera_stable_prob = 0.0
        self.camera_stable_seen_time = 0.0
        self.phase5_entry_marker = None
        self.phase5_entry_reason = "unknown"
        self.phase5_timeout_limit_sec = 0.0
        self.phase0_wait_log_counter = 0
        self.phase3_no_heading_start = None
        self.phase3_arrival_confirm_count = 0
        self.phase3_arrival_inside_since = None
        self.phase3_arrived_latched = False
        self.phase4_detect_confirm_count = 0
        self.phase4_detect_confirm_marker = None
        self.phase4_detect_track_signature = None
        self.phase4_track_has_strict = False
        self.phase4_candidate_missed_frames = 0
        self.phase4_candidate_active_until = 0.0
        self.phase4_last_candidate_dir = None
        self.phase4_search_state = "drive"
        self.phase4_last_processed_cone_seq = 0
        self.phase4_weak_confirm_count = 0
        self.phase4_weak_confirm_marker = None
        self.phase4_weak_missed_frames = 0
        self.phase4_weak_track_signature = None
        self.phase5_last_processed_cone_seq = 0
        self.cone_phase_decision = "not_evaluated"
        self.cone_phase_threshold = 0.0
        self.cone_phase_reached_probability_threshold = 0.0
        self.cone_phase_center_tolerance = 0.0
        self.cone_phase_direction_tolerance = 0.0
        self.cone_phase_required_confirm_frames = 0
        self.cone_phase_detected = False
        self.cone_phase_reached_effective = False
        self.cone_phase_centered = False
        self.cone_phase_direction_consistent = False
        self.cone_phase_confirm_count = 0
        self.phase5_reach_confirm_count = 0
        self.bno_heading_offset_deg = 0.0
        self.bno_heading_offset_valid = False
        self.bno_heading_offset_verified = False
        self.bno_heading_offset_candidate_deg = None
        self.bno_heading_offset_candidate_count = 0
        self._heading_offset_last_gps_fix_seq = 0
        self.mag_heading_offset_deg = 0.0
        self.mag_heading_offset_valid = False
        self.mag_heading_offset_candidate_deg = None
        self.mag_heading_offset_candidate_count = 0
        self.mission_start_time = None
        self.phase_entry_time = None
        self.last_phase_observed = None
        # timeout 判定は累積滞在時間で見るため、フェーズ遷移時にここへ積む。
        self.phase_elapsed_totals = {phase: 0.0 for phase in Phase}
        self.phase7_arrival_reason = "RUNNING"
        self.mission_end_reason = "RUNNING"
        self.mission_total_timeout_triggered = False
        self._shutdown_requested = False
        self.radio_control_mode = ""
        self.radio_disabled = False
        self.radio_simulated_disabled = False
        self.radio_disable_time = None
        self.radio_restore_deadline = None
        self.radio_last_event = "not_configured"
        self.radio_config_source = str(mission_config.source)

        self.phase_handlers = {
            Phase.PHASE0: Phase0Handler(),
            Phase.PHASE1: Phase1Handler(),
            Phase.PHASE2: Phase2Handler(),
            Phase.PHASE3: Phase3Handler(),
            Phase.PHASE4: Phase4Handler(),
            Phase.PHASE5: Phase5Handler(),
            Phase.PHASE6: Phase6Handler(),
            Phase.PHASE7: Phase7Handler(),
        }

    def request_shutdown(self, reason="shutdown"):
        if self._shutdown_requested:
            return
        self._shutdown_requested = True
        if self.mission_end_reason == "RUNNING":
            self.mission_end_reason = reason
        if self.phase7_arrival_reason == "RUNNING" and self.st.snapshot().get("phase") == int(Phase.PHASE7):
            self.phase7_arrival_reason = self._resolve_phase7_arrival_reason()
        try:
            self.restore_mission_radio(f"shutdown_{reason}")
        except Exception as exc:
            print(f"Radio restore on shutdown failed: {exc}")
        try:
            self.stop_motors()
        except Exception as exc:
            print(f"Emergency stop failed: {exc}")
        try:
            self.close_hardware()
        except Exception as exc:
            print(f"Hardware shutdown failed: {exc}")
        self._write_final_log_row()
        run_bundle = getattr(self, "run_bundle", None)
        if run_bundle is not None:
            try:
                run_bundle.finalize(self.mission_end_reason)
            except Exception as exc:
                print(f"Run manifest finalize failed: {exc}")

    def transition_to_give_up(self, reason):
        """Enter the safe terminal phase without passing through approach/final ram."""
        self.mission_end_reason = str(reason)
        self.initialize_phase(Phase.PHASE7)
        self.stop_motors()

    def _resolve_phase7_arrival_reason(self):
        reason = str(getattr(self, "mission_end_reason", "RUNNING"))
        if reason in {"GOAL_PROXIMITY_CONFIRMED", "GOAL_APPROACH_TIMEOUT", "GOAL_OBSERVATION_LOST", "GOAL_RANGE_UNCONFIRMED"}:
            return reason
        if reason == "GOAL_REACHED":
            return "GOAL_REACHED"
        if reason == "MISSION_TOTAL_TIMEOUT":
            return "MISSION_TOTAL_TIMEOUT"
        if reason == "PHASE5_TIMEOUT_FORCED_GOAL":
            return "PHASE5_TIMEOUT_FORCED_GOAL"
        if reason == "PHASE4_TIMEOUT_GIVE_UP":
            return "PHASE4_TIMEOUT_GIVE_UP"
        if reason == "PHASE4_CAMERA_DEAD_GIVE_UP":
            return "PHASE4_CAMERA_DEAD_GIVE_UP"
        if reason == "PHASE5_CAMERA_STALE_GIVE_UP":
            return "PHASE5_CAMERA_STALE_GIVE_UP"
        if reason == "PHASE5_VISUAL_LOST_TIMEOUT":
            return "PHASE5_VISUAL_LOST_TIMEOUT"
        return "OTHER_ABNORMAL_EXIT"

    def _write_final_log_row(self):
        try:
            log_lock = getattr(self, "_log_lock", None)
            if log_lock is None:
                log_lock = threading.Lock()
                self._log_lock = log_lock
            with log_lock:
                with open(self.log_path, "a", newline="") as file_obj:
                    writer = csv.writer(file_obj)
                    self._append_log_row(writer, file_obj)
        except Exception as exc:
            print(f"Final Log Error: {exc}")

    def initialize_phase(self, phase):
        phase_enum = Phase(phase)
        now = time.time()
        if phase_enum != Phase.PHASE0:
            self.restore_mission_radio(f"enter_{phase_enum.name.lower()}")
        if self.mission_start_time is None:
            self.mission_start_time = now
        self.last_phase_observed = phase_enum
        self.phase_entry_time = now
        self.st.update_navigation(phase=int(phase_enum))
        if phase_enum == Phase.PHASE1:
            self.time_phase1_start = now
        elif phase_enum == Phase.PHASE2:
            self.phase2_start_time = now
            self.phase2_stage = PHASE2_STAGE_ESCAPE
            self.phase2_stage_start = now
            self.phase2_calib_ready_since = None
        elif phase_enum == Phase.PHASE3:
            self.time_phase3_start = now
        elif phase_enum == Phase.PHASE4:
            self.time_phase4_start = now
            self.searching_flag = False
            self.phase4_detect_confirm_count = 0
            self.phase4_detect_confirm_marker = None
            self.phase4_detect_track_signature = None
            self.phase4_track_has_strict = False
            self.phase4_candidate_missed_frames = 0
            self.phase4_candidate_active_until = 0.0
            self.phase4_last_candidate_dir = None
            self.phase4_search_state = "drive"
            self.phase4_last_processed_cone_seq = 0
            self.phase4_weak_confirm_count = 0
            self.phase4_weak_confirm_marker = None
            self.phase4_weak_missed_frames = 0
            self.phase4_weak_track_signature = None
        elif phase_enum == Phase.PHASE5:
            self.time_phase5_start = now
            self.phase5_last_processed_cone_seq = 0

    def _sync_phase_time_tracking(self, current_phase):
        now = time.time()
        if self.mission_start_time is None:
            self.mission_start_time = now
        if self.last_phase_observed is None:
            self.last_phase_observed = current_phase
            self.phase_entry_time = now
            return
        if current_phase != self.last_phase_observed:
            if self.phase_entry_time is not None:
                self.phase_elapsed_totals[self.last_phase_observed] += max(0.0, now - self.phase_entry_time)
            self.last_phase_observed = current_phase
            self.phase_entry_time = now

    def _current_phase_elapsed(self, phase, now):
        elapsed = self.phase_elapsed_totals.get(phase, 0.0)
        if self.last_phase_observed == phase and self.phase_entry_time is not None:
            elapsed += max(0.0, now - self.phase_entry_time)
        return elapsed

    def _commit_active_phase_elapsed(self, phase, now):
        if self.last_phase_observed != phase or self.phase_entry_time is None:
            return
        self.phase_elapsed_totals[phase] += max(0.0, now - self.phase_entry_time)

    def _handle_timeout_transitions(self, current_phase):
        now = time.time()
        if self.mission_start_time is None:
            self.mission_start_time = now
        mission_elapsed = now - self.mission_start_time

        # 総ミッション時間超過は安全側に倒して Phase7 へ送る。
        if mission_elapsed >= MISSION_TIMEOUT_TOTAL:
            if not self.mission_total_timeout_triggered:
                print(f"MISSION TIMEOUT ({mission_elapsed:.1f}s): forcing Phase7 give-up")
                self._commit_active_phase_elapsed(current_phase, now)
                self.mission_end_reason = "MISSION_TOTAL_TIMEOUT"
                self.mission_total_timeout_triggered = True
                self.initialize_phase(Phase.PHASE7)
                return True
            return current_phase != Phase.PHASE7

        phase_budget = MISSION_PHASE_TIME_BUDGETS.get(current_phase)
        if phase_budget is None:
            return False
        phase_elapsed = self._current_phase_elapsed(current_phase, now)
        if phase_elapsed < phase_budget:
            return False

        if current_phase in (Phase.PHASE4, Phase.PHASE5):
            camera_snapshot = self.st.snapshot()
            try:
                camera_fresh = (
                    camera_snapshot.get("cone_valid", False)
                    and int(camera_snapshot.get("cone_sequence", 0)) > 0
                    and 0 <= now - float(camera_snapshot.get("cone_updated_at", 0))
                    <= CAMERA_FRAME_STALE_STOP_SEC
                )
            except (TypeError, ValueError, OverflowError):
                camera_fresh = False
            # A failed camera gets the remaining mission time to recover.
            # MotorManager independently stops motion on stale observations.
            if not camera_fresh:
                return False
            if camera_has_visible_cone(camera_snapshot, now):
                # A live target remains authoritative until the global deadline.
                return False

        # 個別フェーズの累積超過は、定数で定義された遷移先へ強制遷移する。
        next_phase = MISSION_PHASE_TIMEOUT_TRANSITIONS.get(current_phase, Phase.PHASE6)
        if current_phase == Phase.PHASE2:
            if getattr(self, "phase3_arrived_latched", False):
                next_phase = Phase.PHASE4
            else:
                next_phase = Phase.PHASE3
                if not getattr(self, "phase3_heading_entry_ready", False):
                    fallback_offset = float(getattr(self, "bno_heading_offset_candidate_deg", 0.0) or 0.0)
                    self.bno_heading_offset_deg = fallback_offset
                    self.bno_heading_offset_valid = False
                    self.bno_heading_offset_verified = False
                    self.phase2_heading_quality_valid = False
                    self.phase3_heading_entry_ready = True
                    print(
                        "Phase2 timeout: forcing Phase3 in GPS-only mode "
                        f"(unverified candidate={fallback_offset:.1f} deg)"
                    )
        phase5_camera_stale = False
        if current_phase == Phase.PHASE5:
            camera_snapshot = self.st.snapshot()
            try:
                updated_at = float(camera_snapshot.get("cone_updated_at", 0.0))
            except (TypeError, ValueError):
                updated_at = 0.0
            try:
                camera_sequence = int(camera_snapshot.get("cone_sequence", 0))
            except (TypeError, ValueError):
                camera_sequence = 0
            camera_age = float("inf") if updated_at <= 0.0 else max(0.0, now - updated_at)
            phase5_camera_stale = not bool(
                camera_snapshot.get("cone_valid", False)
                and camera_sequence > 0
                and camera_age <= float(CAMERA_FRAME_STALE_STOP_SEC)
            )
        timeout_target = Phase.PHASE7 if phase5_camera_stale else next_phase
        print(
            f"{current_phase.name} cumulative timeout ({phase_elapsed:.1f}s / {phase_budget:.1f}s): "
            f"forcing {timeout_target.name}"
        )
        self._commit_active_phase_elapsed(current_phase, now)
        if current_phase == Phase.PHASE4:
            self.cone_phase_decision = "p4_cumulative_timeout_to_p7_give_up"
            self.transition_to_give_up("PHASE4_TIMEOUT_GIVE_UP")
            return True
        if phase5_camera_stale:
            self.cone_phase_decision = "p5_camera_stale_to_p7_give_up"
            self.transition_to_give_up("PHASE5_CAMERA_STALE_GIVE_UP")
            return True
        if current_phase == Phase.PHASE5:
            self.cone_phase_decision = "p5_visual_lost_timeout_to_p7_give_up"
            self.transition_to_give_up("PHASE5_VISUAL_LOST_TIMEOUT")
            return True
        if next_phase == Phase.PHASE6 and self.mission_end_reason == "RUNNING":
            self.mission_end_reason = f"{current_phase.name}_CUM_TIMEOUT_TO_PHASE6"
        self.initialize_phase(next_phase)
        return True

    def run(self, start_phase=Phase.PHASE0, allowed_phases=None):
        try:
            self.setup_hardware()
            self.signal_led(LED_SIGNAL_COUNT)
            self.prepare_mission_radio_control(start_phase)
            self.initialize_phase(start_phase)
            allowed_set = None
            if allowed_phases is not None:
                allowed_set = {Phase(value) for value in allowed_phases}
            while not self._shutdown_requested:
                if allowed_set is not None:
                    # デバッグ実行では許可フェーズを抜けた時点で正常終了扱いにする。
                    current_phase = Phase(self.st.snapshot()["phase"])
                    if current_phase not in allowed_set:
                        print(f"Phase subset completed at phase {int(current_phase)}")
                        self.request_shutdown("PHASE_SUBSET_COMPLETED")
                        return
                self.loop_once()
                time.sleep(MAIN_LOOP_INTERVAL)
        except KeyboardInterrupt:
            print("\nKeyboardInterrupt")
            self.request_shutdown("KEYBOARD_INTERRUPT")
            print("Emergency stop requested. Motors are stopping.")
        finally:
            self.request_shutdown("RUN_EXIT")

    def loop_once(self):
        self.check_radio_failsafe()
        snapshot = self.st.snapshot()
        phase = Phase(snapshot["phase"])
        self._sync_phase_time_tracking(phase)
        if self._handle_timeout_transitions(phase):
            self.check_radio_failsafe()
            return
        self.led_blink_timer += 1
        handler = self.phase_handlers.get(phase)
        if handler is not None:
            handler.execute(self, snapshot)
            post_phase = Phase(self.st.snapshot()["phase"])
            if phase == Phase.PHASE0 and post_phase != Phase.PHASE0:
                self.restore_mission_radio(f"phase0_to_{post_phase.name.lower()}")
            self._sync_phase_time_tracking(post_phase)

    def _angle_diff_deg(self, target_deg, current_deg):
        diff = target_deg - current_deg
        if diff > 180.0:
            diff -= 360.0
        if diff < -180.0:
            diff += 360.0
        return diff

    def _normalize_heading_deg(self, value):
        try:
            deg = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(deg):
            return None
        deg %= 360.0
        if deg < 0.0:
            deg += 360.0
        return deg

    def _update_bno_heading_offset_from_gps(self, snapshot):
        if not self._heading_offset_learning_ready(snapshot):
            return
        if not snapshot.get("angle_valid", False):
            return
        gps_heading = self._normalize_heading_deg(snapshot.get("gps_heading"))
        bno_heading = self._normalize_heading_deg(snapshot.get("angle"))
        if gps_heading is None or bno_heading is None:
            return

        try:
            gps_fix_seq = int(snapshot.get("gps_fix_seq", 0))
        except (TypeError, ValueError):
            return
        if gps_fix_seq <= self._heading_offset_last_gps_fix_seq:
            return
        self._heading_offset_last_gps_fix_seq = gps_fix_seq

        observed_offset = self._angle_diff_deg(gps_heading, bno_heading)
        self._update_heading_offset_estimate("bno", observed_offset)

    def _update_mag_heading_offset_from_gps(self, snapshot, gps_heading, mag_heading):
        if not self._heading_offset_learning_ready(snapshot):
            return
        gps_heading = self._normalize_heading_deg(gps_heading)
        mag_heading = self._normalize_heading_deg(mag_heading)
        if gps_heading is None or mag_heading is None:
            return

        observed_offset = self._angle_diff_deg(gps_heading, mag_heading)
        self._update_heading_offset_estimate("mag", observed_offset)

    def _bno_heading_aligned_to_gps(self, snapshot):
        bno_heading = self._normalize_heading_deg(snapshot.get("angle"))
        if bno_heading is None:
            return None
        if not self.bno_heading_offset_valid:
            return bno_heading
        return self._normalize_heading_deg(bno_heading + self.bno_heading_offset_deg)

    def _mag_heading_aligned_to_gps(self, mag_heading):
        mag_heading = self._normalize_heading_deg(mag_heading)
        if mag_heading is None:
            return None
        if not self.mag_heading_offset_valid:
            return mag_heading
        return self._normalize_heading_deg(mag_heading + self.mag_heading_offset_deg)

    def _heading_offset_learning_ready(self, snapshot):
        if not snapshot.get("gps_heading_valid", False):
            return False
        speed = self._coerce_speed(snapshot.get("gps_speed_mps", 0.0))
        if speed < float(HEADING_OFFSET_LEARN_MIN_SPEED_MPS):
            return False
        baseline = self._coerce_speed(snapshot.get("gps_heading_baseline_m", 0.0))
        if baseline < float(GPS_HEADING_BASELINE_MIN_DIST):
            return False
        last_cmd = str(getattr(self, "last_motor_command", {}).get("type", ""))
        if last_cmd not in (
            "phase3_gps_forward",
            "phase3_gps_probe",
            "phase3_bno_forward",
            "phase2_escape_forward",
            "phase2_offset_heading_hold_straight",
            "phase2_offset_forward",
        ):
            try:
                heading_diff = abs(float(snapshot.get("heading_diff", 180.0)))
            except (TypeError, ValueError):
                heading_diff = 180.0
            if heading_diff > 15.0:
                return False
        return True

    def _coerce_speed(self, value):
        try:
            speed = float(value)
        except (TypeError, ValueError):
            return 0.0
        if not math.isfinite(speed):
            return 0.0
        return max(0.0, speed)

    def _update_bno_heading_offset_from_gps(self, snapshot):
        if not self._heading_offset_learning_ready(snapshot):
            return
        if not snapshot.get("angle_valid", False):
            return
        gps_heading = self._normalize_heading_deg(snapshot.get("gps_heading"))
        bno_heading = self._normalize_heading_deg(snapshot.get("angle"))
        if gps_heading is None or bno_heading is None:
            return

        try:
            gps_fix_seq = int(snapshot.get("gps_fix_seq", 0))
        except (TypeError, ValueError):
            return
        if gps_fix_seq <= self._heading_offset_last_gps_fix_seq:
            return
        self._heading_offset_last_gps_fix_seq = gps_fix_seq

        observed_offset = self._angle_diff_deg(gps_heading, bno_heading)
        self._update_heading_offset_estimate("bno", observed_offset)

    def _update_mag_heading_offset_from_gps(self, snapshot, gps_heading, mag_heading):
        if not self._heading_offset_learning_ready(snapshot):
            return
        gps_heading = self._normalize_heading_deg(gps_heading)
        mag_heading = self._normalize_heading_deg(mag_heading)
        if gps_heading is None or mag_heading is None:
            return

        try:
            gps_fix_seq = int(snapshot.get("gps_fix_seq", 0))
        except (TypeError, ValueError):
            return
        if gps_fix_seq <= getattr(self, "_mag_heading_offset_last_gps_fix_seq", 0):
            return
        self._mag_heading_offset_last_gps_fix_seq = gps_fix_seq

        observed_offset = self._angle_diff_deg(gps_heading, mag_heading)
        self._update_heading_offset_estimate("mag", observed_offset)

    def _update_heading_offset_estimate(self, prefix, observed_offset):
        observed_offset_norm = self._normalize_heading_deg(observed_offset)
        if observed_offset_norm is None:
            return
        observed_offset = observed_offset_norm

        offset_attr = f"{prefix}_heading_offset_deg"
        valid_attr = f"{prefix}_heading_offset_valid"
        candidate_attr = f"{prefix}_heading_offset_candidate_deg"
        count_attr = f"{prefix}_heading_offset_candidate_count"
        update_residual_limit = max(1.0, float(HEADING_OFFSET_LEARN_MAX_RESIDUAL_DEG))
        bootstrap_residual_limit = max(
            1.0,
            float(HEADING_OFFSET_LEARN_BOOTSTRAP_MAX_RESIDUAL_DEG),
        )

        if getattr(self, valid_attr, False):
            is_verified = bool(getattr(self, f"{prefix}_heading_offset_verified", False))
            if is_verified:
                current_offset = self._normalize_heading_deg(getattr(self, offset_attr, 0.0))
                if current_offset is None:
                    return
                residual = abs(self._angle_diff_deg(observed_offset, current_offset))
                if residual > update_residual_limit:
                    return
                offset_delta = self._angle_diff_deg(observed_offset, current_offset)
                max_step = max(1.0, float(PHASE3_BNO_GPS_OFFSET_MAX_STEP_DEG))
                offset_delta = max(-max_step, min(max_step, offset_delta))
                alpha = max(0.0, min(1.0, float(PHASE3_BNO_GPS_OFFSET_ALPHA)))
                setattr(
                    self,
                    offset_attr,
                    self._normalize_heading_deg(current_offset + offset_delta * alpha),
                )
                return

        candidate = self._normalize_heading_deg(getattr(self, candidate_attr, None))
        candidate_count = int(getattr(self, count_attr, 0))
        if candidate is None:
            setattr(self, candidate_attr, observed_offset)
            setattr(self, count_attr, 1)
            return

        residual = abs(self._angle_diff_deg(observed_offset, candidate))
        if residual > bootstrap_residual_limit:
            setattr(self, candidate_attr, observed_offset)
            setattr(self, count_attr, 1)
            return

        candidate_count += 1
        bootstrap_alpha = 1.0 / float(candidate_count)
        candidate = self._normalize_heading_deg(
            candidate + self._angle_diff_deg(observed_offset, candidate) * bootstrap_alpha
        )
        setattr(self, candidate_attr, candidate)
        setattr(self, count_attr, candidate_count)

        if candidate_count >= int(HEADING_OFFSET_LEARN_BOOTSTRAP_SAMPLES):
            setattr(self, offset_attr, candidate)
            setattr(self, valid_attr, True)
            setattr(self, f"{prefix}_heading_offset_verified", True)

    def _weighted_heading(self, snapshot):
        weights = []
        headings = []
        source_parts = []

        if snapshot.get("gps_heading_valid", False):
            weights.append(HEADING_WEIGHT_GPS)
            headings.append(snapshot.get("gps_heading", 0.0))
            source_parts.append(HEADING_SOURCE_GPS)

        if snapshot.get("angle_valid", False):
            bno_weight = HEADING_WEIGHT_BNO_BASE
            calib = self.bno_calib
            if calib.get("valid") and len(calib.get("value", ())) >= 4:
                try:
                    mag_cal = int(calib["value"][3])
                    bno_weight += HEADING_WEIGHT_BNO_STEP * max(0, min(HEADING_MAG_CALIB_MAX, mag_cal))
                except Exception:
                    pass
            bno_weight = max(HEADING_WEIGHT_BNO_MIN, min(HEADING_WEIGHT_BNO_MAX, bno_weight))
            aligned_bno = self._bno_heading_aligned_to_gps(snapshot)
            if aligned_bno is None:
                aligned_bno = self._normalize_heading_deg(snapshot.get("angle", 0.0))
            weights.append(bno_weight)
            headings.append(aligned_bno if aligned_bno is not None else 0.0)
            source_parts.append(HEADING_SOURCE_BNO)

        if not headings:
            return None, HEADING_SOURCE_INVALID, 0.0

        sx = 0.0
        sy = 0.0
        total_w = 0.0
        for heading, weight in zip(headings, weights):
            rad = math.radians(heading)
            sx += math.cos(rad) * weight
            sy += math.sin(rad) * weight
            total_w += weight
        if total_w <= 0.0:
            return None, HEADING_SOURCE_INVALID, 0.0
        fused = math.degrees(math.atan2(sy, sx))
        if fused < 0.0:
            fused += 360.0
        source = HEADING_SOURCE_JOINER.join(source_parts)
        return fused, source, total_w
