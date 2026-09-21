import csv
import math
import os
import threading
import time
import traceback

import serial

from lib import bno055
from lib import cone_detect as dc
from lib.cone_diagnostics import (
    detector_diagnostics,
    mission_log_values,
    normalize_cone_diagnostics,
)

from mission.const import (
    BMP_ALTITUDE_MAX_VALID,
    BMP_ALTITUDE_MIN_VALID,
    BMP_FAIL_LIMIT,
    BMP_PRESSURE_MAX_VALID,
    BMP_PRESSURE_MIN_VALID,
    BMP_REINIT_COOLDOWN,
    BMP_SAMPLING_RATE,
    BMP_SEA_LEVEL_PRESSURE_PA,
    BNO_ACC_MAX,
    BNO_ANGLE_JUMP_MAX,
    BNO_CALIB_MAG_MIN,
    BNO_FAIL_LIMIT,
    BNO_FREEZE_EPS,
    BNO_FUSION_OK_STATES,
    BNO_GYRO_MAX,
    BNO_HEADING_RECOVERY_MAX_STEP_DEG,
    BNO_HEADING_RECOVERY_SAMPLES,
    BNO_HEADING_RECOVERY_STALE_SEC,
    BNO_MAG_MAX,
    BNO_REINIT_COOLDOWN,
    BNO_STALE_TIMEOUT,
    CAMERA_ACTIVE_SLEEP,
    CAMERA_DEAD_TIMEOUT,
    CAMERA_FAIL_LIMIT,
    CAMERA_IDLE_SLEEP,
    CAMERA_RECOVERY_GRACE_SEC,
    CAMERA_REINIT_INTERVAL,
    CAMERA_REINIT_MAX_ATTEMPTS,
    CONE_CENTER_POSITION,
    DATA_SAMPLING_RATE,
    DEFAULT_BNO_CALIB,
    DEFAULT_SONAR_DIST_CM,
    DEVICE_BMP,
    DEVICE_BNO,
    DEVICE_DETECTOR,
    DEVICE_SONAR,
    GPS_ACTIVE_DETECT,
    GPS_BUFFER_CLEAR_INTERVAL,
    GPS_BUFFER_CLEAR_THRESHOLD,
    GPS_DIAGNOSTIC_LOG_INTERVAL,
    GPS_FIX_LOSS_TIMEOUT,
    GPS_HEADING_BASELINE_MIN_DIST,
    GPS_HEADING_HOLD_SEC,
    GPS_HEADING_MIN_DIST,
    GPS_HEADING_WINDOW_SEC,
    GPS_INACTIVE_DETECT,
    GPS_MAX_HDOP,
    GPS_MAX_SPEED_MPS,
    GPS_MIN_FIX_QUAL,
    GPS_MIN_SATELLITES,
    GPS_NO_DATA_REOPEN_TIMEOUT,
    GPS_NON_GGA_REOPEN_TIMEOUT,
    GPS_RECONNECT_SLEEP,
    GPS_STABLE_FIX_COUNT,
    MISSION_LOG_SCHEMA_VERSION,
    PHASES_CAMERA_ACTIVE,
    Phase,
    SONAR_MAX_DISTANCE,
    SONAR_MIN_DISTANCE_CM,
    SONAR_STALE_TIMEOUT_SEC,
)
from mission.gps_util import coerce_gga_metrics, gga_quality_ok, open_gps_serial, parse_gga_sentence
from mission.nav import calc_distance_and_azimuth


class SensorManager:
    def _get_i2c_lock(self):
        lock = getattr(self, "i2c_lock", None)
        if lock is None:
            lock = threading.RLock()
            self.i2c_lock = lock
        return lock

    def _transform_cone_direction_for_control(self, cone_direction):
        """Map detector X position into the rover control frame."""
        try:
            cdir = float(cone_direction)
        except (TypeError, ValueError):
            return CONE_CENTER_POSITION
        if not math.isfinite(cdir):
            return CONE_CENTER_POSITION

        if bool(getattr(self, "camera_control_invert_x", False)):
            cdir = 1.0 - cdir

        return max(0.0, min(1.0, cdir))

    def _coerce_float(self, value, default=0.0):
        try:
            casted = float(value)
            if not math.isfinite(casted):
                return float(default)
            return casted
        except (TypeError, ValueError):
            return float(default)

    def _coerce_int(self, value, default=0):
        try:
            return int(value)
        except (TypeError, ValueError):
            return int(default)

    def _snapshot_vec3(self, snapshot, key):
        value = snapshot.get(key, (0.0, 0.0, 0.0))
        if not isinstance(value, (list, tuple)) or len(value) < 3:
            return [0.0, 0.0, 0.0]
        return [
            self._coerce_float(value[0]),
            self._coerce_float(value[1]),
            self._coerce_float(value[2]),
        ]

    def _build_log_row(self):
        current_data = self.st.snapshot()
        motor_cmd = getattr(self, "last_motor_command", {})
        now = time.time()
        mission_start = getattr(self, "mission_start_time", None)
        mission_elapsed_sec = 0.0
        if mission_start:
            mission_elapsed_sec = max(0.0, now - mission_start)
        radio_restore_deadline = getattr(self, "radio_restore_deadline", None)
        radio_restore_deadline_elapsed_sec = 0.0
        if mission_start and radio_restore_deadline is not None:
            radio_restore_deadline_elapsed_sec = max(0.0, radio_restore_deadline - mission_start)

        acc = self._snapshot_vec3(current_data, "acc")
        gyro = self._snapshot_vec3(current_data, "gyro")
        mag = self._snapshot_vec3(current_data, "mag")

        motor_cmd_updated_ms = self._coerce_int(motor_cmd.get("updated_ms", 0))
        motor_cmd_updated_elapsed_sec = 0.0
        if mission_start and motor_cmd_updated_ms > 0:
            motor_cmd_updated_elapsed_sec = max(0.0, (motor_cmd_updated_ms / 1000.0) - mission_start)
        bno_acc_updated_elapsed_sec = 0.0
        bno_last_acc_time = self._coerce_float(getattr(self, "bno_last_acc_time", 0.0))
        if mission_start and bno_last_acc_time > 0.0:
            bno_acc_updated_elapsed_sec = max(0.0, bno_last_acc_time - mission_start)
        bno_acc_stale_sec = BNO_STALE_TIMEOUT + 1.0
        if bno_last_acc_time > 0.0:
            bno_acc_stale_sec = max(0.0, now - bno_last_acc_time)
        bno_last_valid_time = self._coerce_float(getattr(self, "bno_last_valid_time", 0.0))
        bno_stale_sec = BNO_STALE_TIMEOUT + 1.0
        if bno_last_valid_time > 0.0:
            bno_stale_sec = max(0.0, now - bno_last_valid_time)
        bmp_updated_elapsed_sec = 0.0
        bmp_last_valid_time = self._coerce_float(getattr(self, "bmp_last_valid_time", 0.0))
        if mission_start and bmp_last_valid_time > 0.0:
            bmp_updated_elapsed_sec = max(0.0, bmp_last_valid_time - mission_start)
        bmp_stale_sec = BNO_STALE_TIMEOUT + 1.0
        if bmp_last_valid_time > 0.0:
            bmp_stale_sec = max(0.0, now - bmp_last_valid_time)
        calib_values = getattr(self, "bno_calib", {}).get("value", (0, 0, 0, 0))
        if not isinstance(calib_values, (list, tuple)) or len(calib_values) < 4:
            calib_values = (0, 0, 0, 0)
        cone_debug = dict(current_data.get("cone_debug", {}) or {})
        cone_last_valid_at = self._coerce_float(current_data.get("cone_last_valid_at", 0.0))
        cone_updated_at = self._coerce_float(current_data.get("cone_updated_at", 0.0))
        cone_stale_sec = CAMERA_DEAD_TIMEOUT + 1.0
        if cone_last_valid_at > 0.0:
            cone_stale_sec = max(0.0, now - cone_last_valid_at)
        cone_updated_elapsed_sec = 0.0
        if mission_start and cone_updated_at > 0.0:
            cone_updated_elapsed_sec = max(0.0, cone_updated_at - mission_start)
        camera_recovery_started_at = getattr(self, "camera_recovery_started_at", None)
        camera_recovery_elapsed_sec = 0.0
        if camera_recovery_started_at is not None:
            camera_recovery_elapsed_sec = max(0.0, now - float(camera_recovery_started_at))

        return [
            MISSION_LOG_SCHEMA_VERSION,
            str(getattr(self, "run_id", "legacy")),
            f"{mission_elapsed_sec:.2f}",
            self._coerce_int(current_data.get("phase", 0)),
            str(getattr(self, "phase0_exit_reason", "")),
            str(getattr(self, "phase0_exit_detail", "")),
            f"{acc[0]:.2f}",
            f"{acc[1]:.2f}",
            f"{acc[2]:.2f}",
            f"{gyro[0]:.2f}",
            f"{gyro[1]:.2f}",
            f"{gyro[2]:.2f}",
            f"{mag[0]:.2f}",
            f"{mag[1]:.2f}",
            f"{mag[2]:.2f}",
            f"{self._coerce_float(current_data.get('lat', 0.0)):.6f}",
            f"{self._coerce_float(current_data.get('lng', 0.0)):.6f}",
            f"{self._coerce_float(current_data.get('gps_speed_mps', 0.0)):.2f}",
            self._coerce_int(current_data.get("gps_fix_qual", 0)),
            self._coerce_int(current_data.get("gps_sats", 0)),
            f"{self._coerce_float(current_data.get('gps_hdop', 0.0)):.2f}",
            f"{self._coerce_float(current_data.get('gps_heading', 0.0)):.2f}",
            self._coerce_int(bool(current_data.get("gps_heading_valid", False))),
            self._coerce_int(current_data.get("gps_fix_seq", 0)),
            f"{self._coerce_float(current_data.get('nav_heading', 0.0)):.2f}",
            str(current_data.get("nav_heading_source", "")),
            f"{self._coerce_float(current_data.get('heading_diff', 0.0)):.2f}",
            f"{self._coerce_float(current_data.get('heading_trust', 0.0)):.2f}",
            self._coerce_int(bool(current_data.get("bno_trusted", False))),
            f"{self._coerce_float(current_data.get('bno_offset_deg', 0.0)):.2f}",
            self._coerce_int(bool(current_data.get("bno_offset_valid", False))),
            f"{self._coerce_float(getattr(self, 'bno_heading_offset_candidate_deg', 0.0)):.2f}",
            self._coerce_int(getattr(self, "bno_heading_offset_candidate_count", 0)),
            f"{self._coerce_float(current_data.get('gps_heading_baseline_m', 0.0)):.2f}",
            str(getattr(self, "phase2_stage", "")),
            self._coerce_int(calib_values[0]),
            self._coerce_int(calib_values[1]),
            self._coerce_int(calib_values[2]),
            self._coerce_int(calib_values[3]),
            f"{self._coerce_float(getattr(self, 'phase2_offset_reference_bno_deg', 0.0)):.2f}",
            f"{self._coerce_float(getattr(self, 'phase2_offset_heading_error_deg', 0.0)):.2f}",
            f"{self._coerce_float(getattr(self, 'phase2_offset_distance_m', 0.0)):.2f}",
            f"{self._coerce_float(getattr(self, 'phase2_offset_path_efficiency', 0.0)):.3f}",
            f"{self._coerce_float(getattr(self, 'phase2_offset_course_deg', 0.0)):.2f}",
            f"{self._coerce_float(getattr(self, 'phase2_offset_bno_mean_deg', 0.0)):.2f}",
            f"{self._coerce_float(getattr(self, 'phase2_offset_bno_spread_deg', 0.0)):.2f}",
            f"{self._coerce_float(getattr(self, 'phase2_offset_subsegment_diff_deg', 0.0)):.2f}",
            self._coerce_int(getattr(self, "phase2_offset_attempt_count", 0)),
            str(getattr(self, "phase2_offset_mode", "")),
            f"{self._coerce_float(getattr(self, 'phase2_offset_turn_target_deg', 0.0)):.2f}",
            self._coerce_int(getattr(self, "phase2_offset_stage_retry_count", 0)),
            int(bool(getattr(self, "phase2_offset_near_goal_active", False))),
            f"{self._coerce_float(getattr(self, 'phase2_offset_leg_time_limit_sec', 0.0)):.2f}",
            str(getattr(self, "phase2_offset_reject_reason", "")),
            self._coerce_int(bool(getattr(self, "phase1_offset_candidate_valid", False))),
            f"{self._coerce_float(getattr(self, 'phase1_offset_candidate_deg', 0.0)):.2f}",
            f"{self._coerce_float(getattr(self, 'phase1_offset_distance_m', 0.0)):.2f}",
            f"{self._coerce_float(getattr(self, 'phase1_offset_path_efficiency', 0.0)):.3f}",
            f"{self._coerce_float(getattr(self, 'phase1_offset_bno_spread_deg', 0.0)):.2f}",
            f"{self._coerce_float(getattr(self, 'phase1_offset_subsegment_diff_deg', 0.0)):.2f}",
            str(getattr(self, "phase1_offset_reject_reason", "")),
            self._coerce_int(bool(current_data.get("arrival_inside", False))),
            self._coerce_int(current_data.get("arrival_confirm_count", 0)),
            self._coerce_int(bool(current_data.get("phase3_arrived_latched", False))),
            f"{self._coerce_float(current_data.get('alt', 0.0)):.2f}",
            f"{self._coerce_float(current_data.get('pres', 0.0)):.2f}",
            f"{self._coerce_float(current_data.get('distance', 0.0)):.2f}",
            f"{self._coerce_float(current_data.get('azimuth', 0.0)):.2f}",
            f"{self._coerce_float(getattr(self, 'target_lat', 0.0)):.6f}",
            f"{self._coerce_float(getattr(self, 'target_lng', 0.0)):.6f}",
            f"{self._coerce_float(current_data.get('angle', 0.0)):.2f}",
            f"{self._coerce_float(current_data.get('direction', 0.0)):.2f}",
            f"{self._coerce_float(current_data.get('fall', 0.0)):.2f}",
            *mission_log_values(cone_debug),
            f"{cone_stale_sec:.2f}",
            f"{cone_updated_elapsed_sec:.2f}",
            str(getattr(self, "cone_phase_decision", "not_evaluated")),
            f"{self._coerce_float(getattr(self, 'cone_phase_threshold', 0.0)):.3f}",
            f"{self._coerce_float(getattr(self, 'cone_phase_reached_probability_threshold', 0.0)):.3f}",
            f"{self._coerce_float(getattr(self, 'cone_phase_center_tolerance', 0.0)):.3f}",
            f"{self._coerce_float(getattr(self, 'cone_phase_direction_tolerance', 0.0)):.3f}",
            self._coerce_int(getattr(self, "cone_phase_required_confirm_frames", 0)),
            self._coerce_int(bool(getattr(self, "cone_phase_detected", False))),
            self._coerce_int(bool(getattr(self, "cone_phase_reached_effective", False))),
            self._coerce_int(bool(getattr(self, "cone_phase_centered", False))),
            self._coerce_int(bool(getattr(self, "cone_phase_direction_consistent", False))),
            self._coerce_int(getattr(self, "cone_phase_confirm_count", 0)),
            self._coerce_int(getattr(self, "phase4_detect_confirm_count", 0)),
            f"{self._coerce_float(getattr(self, 'phase4_detect_confirm_marker', 0.0)):.6f}",
            self._coerce_int(getattr(self, "count_cone_lost", 0)),
            self._coerce_int(getattr(self, "phase5_reach_confirm_count", 0)),
            str(getattr(self, "phase5_entry_reason", "unknown")),
            self._coerce_int(getattr(self, "camera_fail_count", 0)),
            self._coerce_int(getattr(self, "camera_reinit_attempt_count", 0)),
            f"{camera_recovery_elapsed_sec:.2f}",
            self._coerce_int(bool(getattr(self, "camera_recovery_exhausted", False))),
            f"{self._coerce_float(current_data.get('sonar_distance_cm', 0.0)):.2f}",
            self._coerce_int(bool(current_data.get("sonar_valid", False))),
            f"{self._coerce_float(current_data.get('sonar_stale_sec', 0.0)):.2f}",
            self._coerce_int(current_data.get("sonar_sequence", 0)),
            f"{self._coerce_float(current_data.get('sonar_observed_at', 0.0)):.3f}",
            str(getattr(self, "goal_decision", "inactive")),
            self._coerce_int(getattr(self, "goal_confirm_count", 0)),
            self._coerce_int(bool(current_data.get("angle_valid", False))),
            f"{bno_stale_sec:.2f}",
            self._coerce_int(bool(getattr(self, "bno_heading_recovery_active", False))),
            self._coerce_int(getattr(self, "bno_heading_recovery_count", 0)),
            self._coerce_int(getattr(self, "bno_heading_recovery_seq", 0)),
            self._coerce_int(
                bool(
                    bno_last_acc_time > 0.0
                    and bno_acc_stale_sec <= BNO_STALE_TIMEOUT
                )
            ),
            f"{bno_acc_stale_sec:.2f}",
            f"{bno_acc_updated_elapsed_sec:.2f}",
            self._coerce_int(
                bool(
                    bmp_last_valid_time > 0.0
                    and bmp_stale_sec <= BNO_STALE_TIMEOUT
                )
            ),
            f"{bmp_stale_sec:.2f}",
            f"{bmp_updated_elapsed_sec:.2f}",
            str(motor_cmd.get("type", "")),
            f"{motor_cmd_updated_elapsed_sec:.2f}",
            f"{self._coerce_float(motor_cmd.get('motor1_speed', 0.0)):.2f}",
            self._coerce_int(bool(motor_cmd.get("motor1_forward", 1))),
            f"{self._coerce_float(motor_cmd.get('motor2_speed', 0.0)):.2f}",
            self._coerce_int(bool(motor_cmd.get("motor2_forward", 1))),
            str(getattr(self, "phase7_arrival_reason", "RUNNING")),
            str(getattr(self, "mission_end_reason", "RUNNING")),
            self._coerce_int(bool(getattr(self, "mission_total_timeout_triggered", False))),
            f"{self._coerce_float(mission_elapsed_sec):.2f}",
            self._coerce_int(bool(getattr(self, "radio_disabled", False))),
            str(getattr(self, "radio_control_mode", "")),
            str(getattr(self, "radio_last_event", "")),
            str(getattr(self, "radio_config_source", "")),
            f"{self._coerce_float(radio_restore_deadline_elapsed_sec):.2f}",
        ]

    def _append_log_row(self, writer, file_obj):
        writer.writerow(self._build_log_row())
        file_obj.flush()
        # On Windows (especially cloud-synced folders like OneDrive),
        # fsync per row can block for a long time and stall logging.
        if os.name != "nt":
            os.fsync(file_obj.fileno())

    def log_thread(self):
        log_error_last_print = 0.0
        while not bool(getattr(self, "_shutdown_requested", False)):
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
                now = time.time()
                if now - log_error_last_print >= 1.0:
                    print(f"Log Error: {exc}")
                    traceback.print_exc()
                    log_error_last_print = now
            time.sleep(DATA_SAMPLING_RATE)

    def _vector_within(self, vec, max_abs):
        try:
            for value in vec:
                if not math.isfinite(value) or abs(value) > max_abs:
                    return False
        except Exception:
            return False
        return True

    def _vector_near_zero(self, vec, eps):
        try:
            for value in vec:
                if not math.isfinite(value) or abs(value) > eps:
                    return False
        except Exception:
            return False
        return True

    def _vector_finite(self, vec):
        try:
            values = [float(value) for value in vec]
        except Exception:
            return False
        return len(values) >= 3 and all(math.isfinite(value) for value in values)

    def _vector_norm(self, vec):
        try:
            values = [float(value) for value in vec]
        except Exception:
            return None
        if len(values) < 3 or not all(math.isfinite(value) for value in values[:3]):
            return None
        return math.sqrt(values[0] ** 2 + values[1] ** 2 + values[2] ** 2)

    def _scalar_within(self, value, min_value, max_value):
        try:
            scalar = float(value)
        except (TypeError, ValueError):
            return False
        if not math.isfinite(scalar):
            return False
        return min_value <= scalar <= max_value

    def _mark_bno_acc_stale(self):
        now = time.time()
        if getattr(self, "bno_last_acc_time", 0.0) > 0.0:
            self.bno_acc_stale_sec = max(0.0, now - self.bno_last_acc_time)
        else:
            self.bno_acc_stale_sec = BNO_STALE_TIMEOUT + 1.0

    def _mark_bmp_stale(self):
        now = time.time()
        if getattr(self, "bmp_last_valid_time", 0.0) > 0.0:
            self.bmp_stale_sec = max(0.0, now - self.bmp_last_valid_time)
        else:
            self.bmp_stale_sec = BNO_STALE_TIMEOUT + 1.0

    def _angle_jump_ok(self, angle):
        if self.bno_last_valid_time <= 0:
            self.bno_heading_recovery_active = False
            self.bno_heading_recovery_count = 0
            self._bno_heading_recovery_candidate = None
            return True
        last = self.bno_last_valid.get("angle", 0.0)
        diff = abs(((angle - last + 180.0) % 360.0) - 180.0)
        if diff <= BNO_ANGLE_JUMP_MAX:
            self.bno_heading_recovery_active = False
            self.bno_heading_recovery_count = 0
            self._bno_heading_recovery_candidate = None
            return True

        stale_sec = max(0.0, time.time() - self.bno_last_valid_time)
        if stale_sec < float(BNO_HEADING_RECOVERY_STALE_SEC):
            self.bno_heading_recovery_active = False
            self.bno_heading_recovery_count = 0
            self._bno_heading_recovery_candidate = None
            return False

        previous = getattr(self, "_bno_heading_recovery_candidate", None)
        if previous is None:
            recovery_count = 1
        else:
            recovery_step = abs(((angle - previous + 180.0) % 360.0) - 180.0)
            recovery_count = (
                int(getattr(self, "bno_heading_recovery_count", 0)) + 1
                if recovery_step <= float(BNO_HEADING_RECOVERY_MAX_STEP_DEG)
                else 1
            )
        self._bno_heading_recovery_candidate = angle
        self.bno_heading_recovery_active = True
        self.bno_heading_recovery_count = recovery_count
        if recovery_count < int(BNO_HEADING_RECOVERY_SAMPLES):
            return False

        self.bno_heading_recovery_active = False
        self.bno_heading_recovery_count = 0
        self._bno_heading_recovery_candidate = None
        self.bno_heading_recovery_seq = int(getattr(self, "bno_heading_recovery_seq", 0)) + 1
        print(f"BNO heading recovered at {angle:.1f} deg after {stale_sec:.2f}s dropout")
        return True

    def _try_reinit_bno(self):
        now = time.time()
        if now - self.bno_last_reinit_time < BNO_REINIT_COOLDOWN:
            return
        self.bno_last_reinit_time = now
        try:
            with self._get_i2c_lock():
                bno = bno055.BNO055()
                if bno.setUp():
                    self.devices[DEVICE_BNO] = bno
                    self.bno_fail_count = 0
                    print("BNO055: Reinitialized after failures.")
                else:
                    print("BNO055: Reinit failed.")
        except Exception as exc:
            print(f"BNO055: Reinit error {exc}.")

    def _try_reinit_bmp(self, reason="failure"):
        now = time.time()
        last_reinit = getattr(self, "bmp_last_reinit_time", 0.0)
        if now - last_reinit < BMP_REINIT_COOLDOWN:
            return
        self.bmp_last_reinit_time = now
        try:
            from lib import bmp180

            with self._get_i2c_lock():
                bmp = bmp180.BMP180(oss=3)
                if bmp.setUp():
                    self.devices[DEVICE_BMP] = bmp
                    self.bmp_fail_count = 0
                    self._bmp_last_pressure_sample = None
                    self._bmp_same_pressure_since = now
                    print(f"BMP180: Reinitialized after {reason}.")
                else:
                    print(f"BMP180: Reinit failed after {reason}.")
        except Exception as exc:
            print(f"BMP180: Reinit error after {reason}: {exc}.")

    def _record_bmp_failure(self, reason):
        self.bmp_fail_count = int(getattr(self, "bmp_fail_count", 0)) + 1
        if self.bmp_fail_count >= BMP_FAIL_LIMIT:
            self._try_reinit_bmp(reason)

    def _try_reinit_camera(self, force=False, reason=None):
        now = time.time()
        attempts = int(getattr(self, "camera_reinit_attempt_count", 0))
        retry_interval = (CAMERA_RECOVERY_GRACE_SEC
                          if attempts >= CAMERA_REINIT_MAX_ATTEMPTS
                          else CAMERA_REINIT_INTERVAL)
        if (not force) and now - self.camera_last_reinit < retry_interval:
            return False
        self.camera_last_reinit = now
        self.camera_reinit_attempt_count = attempts + 1
        if reason:
            print(f"Camera: Reinit requested ({reason}).")
        print(
            "Camera: Reinit attempt "
            f"{self.camera_reinit_attempt_count} (retry interval {retry_interval:.0f}s)."
        )
        try:
            if hasattr(self, "_release_camera_detector"):
                self._release_camera_detector()
            detector = dc.detector()
            roi_reference = getattr(self, "roi_references", None)
            if not roi_reference:
                roi_reference = getattr(self, "roi_img", None)
            detector.set_roi_img(roi_reference)
            self.devices[DEVICE_DETECTOR] = detector
            # Give the recreated detector a fresh capture window, but keep the
            # recovery campaign active until an actual frame is accepted.
            self.camera_fail_count = 0
            print("Camera: Detector recreated; waiting for a valid frame.")
            return True
        except Exception as exc:
            print(f"Camera: Reinit error {exc}.")
            self._update_camera_recovery_exhausted(time.time())
            return False

    def _begin_camera_recovery(self, now=None):
        now = time.time() if now is None else float(now)
        if getattr(self, "camera_recovery_started_at", None) is None:
            self.camera_recovery_started_at = now
        if getattr(self, "camera_dead_since", None) is None:
            self.camera_dead_since = now
        self.camera_recovery_exhausted = False

    def reset_camera_recovery_window(self):
        """Grant a fresh bounded recovery campaign when navigation reaches P4."""
        self.camera_fail_count = 0
        self.camera_last_reinit = 0.0
        self.camera_dead_since = None
        self.camera_recovery_started_at = None
        self.camera_reinit_attempt_count = 0
        self.camera_recovery_exhausted = False
        print("Camera: Fresh Phase4 recovery window armed.")

    def _update_camera_recovery_exhausted(self, now=None):
        now = time.time() if now is None else float(now)
        started = getattr(self, "camera_recovery_started_at", None)
        attempts = int(getattr(self, "camera_reinit_attempt_count", 0))
        exhausted = (
            started is not None
            and attempts >= CAMERA_REINIT_MAX_ATTEMPTS
            and now - float(started) >= CAMERA_RECOVERY_GRACE_SEC
        )
        self.camera_recovery_exhausted = bool(exhausted)
        return bool(exhausted)

    def _record_camera_recovered(self):
        was_recovering = getattr(self, "camera_recovery_started_at", None) is not None
        self.camera_fail_count = 0
        self.camera_dead_since = None
        self.camera_recovery_started_at = None
        self.camera_reinit_attempt_count = 0
        self.camera_recovery_exhausted = False
        if was_recovering:
            print("Camera: Recovery confirmed by a valid frame.")

    def get_bno_data(self):
        bno_instance = self.devices.get(DEVICE_BNO)
        if bno_instance is None:
            self._mark_bno_acc_stale()
            return None
        try:
            with self._get_i2c_lock():
                acc = bno_instance.getAcc()
                gyro = bno_instance.getGyro()
                mag = bno_instance.getMag()
                euler = bno_instance.getEuler()
                calib = bno_instance.getCalibrationStatus()
                sys_status = bno_instance.getSystemStatus()
                sys_error = bno_instance.getSystemError()

            i2c_ok = acc["valid"] and gyro["valid"] and mag["valid"] and euler["valid"]
            if not i2c_ok:
                self.bno_fail_count += 1
                if self.bno_fail_count >= BNO_FAIL_LIMIT:
                    self._try_reinit_bno()
            else:
                self.bno_fail_count = 0

            freeze = False
            if i2c_ok:
                signature = (
                    tuple(acc["value"][:3]),
                    tuple(gyro["value"][:3]),
                    tuple(mag["value"][:3]),
                    tuple(euler["value"][:3]),
                )
                now_sig = time.time()
                if signature == getattr(self, "_bno_last_sample_signature", None):
                    same_since = getattr(self, "_bno_same_sample_since", now_sig)
                    self._bno_same_sample_since = same_since
                    if now_sig - same_since > BNO_STALE_TIMEOUT:
                        freeze = True
                else:
                    self._bno_last_sample_signature = signature
                    self._bno_same_sample_since = now_sig
                euler_zero = False
                if euler["valid"] and len(euler["value"]) >= 1:
                    try:
                        euler_zero = abs(float(euler["value"][0])) <= BNO_FREEZE_EPS
                    except Exception:
                        euler_zero = False
                # A real BNO055 at rest still reports gravity on accel and a non-zero
                # magnetic field vector. Treat all-zero raw vectors as a dead sensor
                # even when a stale heading register contains a tiny non-zero value.
                raw_vectors_dead = (
                    self._vector_near_zero(acc["value"], BNO_FREEZE_EPS)
                    and self._vector_near_zero(mag["value"], BNO_FREEZE_EPS)
                    and self._vector_near_zero(gyro["value"], BNO_FREEZE_EPS)
                )
                freeze = freeze or (
                    raw_vectors_dead
                    and euler_zero
                )
                if raw_vectors_dead and not freeze:
                    freeze = True
                if freeze:
                    self.bno_fail_count += 1
                    if self.bno_fail_count % 25 == 0:
                        print(
                            "BNO frozen-output detected: "
                            f"acc={acc['value']} gyro={gyro['value']} mag={mag['value']} "
                            f"sys={sys_status.get('value')} err={sys_error.get('value')}"
                        )
                    if self.bno_fail_count >= BNO_FAIL_LIMIT:
                        self._try_reinit_bno()

            acc_ok = (not freeze) and acc["valid"] and self._vector_within(acc["value"], BNO_ACC_MAX)
            gyro_ok = (not freeze) and gyro["valid"] and self._vector_within(gyro["value"], BNO_GYRO_MAX)
            mag_ok = (not freeze) and mag["valid"] and self._vector_within(mag["value"], BNO_MAG_MAX)

            angle_val = 0.0
            if euler["valid"] and len(euler["value"]) >= 1:
                angle_val = float(euler["value"][0])
            angle_sample_valid = (
                (not freeze)
                and euler["valid"]
                and math.isfinite(angle_val)
                and 0.0 <= angle_val < 360.0
            )
            if angle_sample_valid:
                angle_jump_ok = self._angle_jump_ok(angle_val)
            else:
                angle_jump_ok = False
                self.bno_heading_recovery_active = False
                self.bno_heading_recovery_count = 0
                self._bno_heading_recovery_candidate = None
            angle_ok = angle_sample_valid and angle_jump_ok

            sys_ok = sys_status["valid"] and sys_error["valid"]
            sys_error_ok = sys_ok and sys_error["value"] == 0
            fusion_ok = sys_ok and sys_status["value"] in BNO_FUSION_OK_STATES

            if acc_ok:
                self.bno_last_valid["acc"] = list(acc["value"])
                self.bno_last_acc_time = time.time()
            if gyro_ok:
                self.bno_last_valid["gyro"] = list(gyro["value"])
            if mag_ok:
                self.bno_last_valid["mag"] = list(mag["value"])
            if angle_ok:
                self.bno_last_valid["angle"] = angle_val
                self.bno_last_valid_time = time.time()

            now = time.time()
            self._mark_bno_acc_stale()
            acc_val = list(self.bno_last_valid["acc"]) if self.bno_last_acc_time > 0.0 else None
            gyro_val = list(self.bno_last_valid["gyro"])
            raw_mag_available = (
                mag.get("valid", False)
                and self._vector_finite(mag.get("value", ()))
                and (not self._vector_near_zero(mag.get("value", ()), BNO_FREEZE_EPS))
            )
            # Phase3 legacy steering derives heading directly from mag XY.
            mag_val = list(mag["value"]) if raw_mag_available else list(self.bno_last_valid["mag"])
            angle_val = float(self.bno_last_valid["angle"])
            fall = None
            if acc_val is not None:
                fall = math.sqrt(acc_val[0] ** 2 + acc_val[1] ** 2 + acc_val[2] ** 2)

            calib_ok = calib["valid"] and calib["value"][3] >= BNO_CALIB_MAG_MIN
            # Allow heading use when Euler/system status is healthy even if magnetometer
            # calibration is still incomplete (common during short ground E2E runs).
            # Field tests can keep fusion status/calibration low for a while; use a
            # sane Euler heading if it passes local consistency checks.
            angle_valid = angle_ok
            self.bno_calib = calib if calib else DEFAULT_BNO_CALIB
            if self.bno_last_valid_time > 0:
                self.bno_stale_sec = now - self.bno_last_valid_time
            else:
                # No accepted heading sample has ever been observed in this run.
                # Mark as stale so logs clearly show "BNO unavailable" instead of 0.0.
                self.bno_stale_sec = BNO_STALE_TIMEOUT + 1.0
            if self.bno_stale_sec > BNO_STALE_TIMEOUT:
                angle_valid = False
            if not angle_valid:
                invalid_count = int(getattr(self, "_bno_heading_invalid_count", 0)) + 1
                self._bno_heading_invalid_count = invalid_count
                if invalid_count % 50 == 0:
                    print(
                        "BNO heading invalid: "
                        f"i2c_ok={int(i2c_ok)} freeze={int(freeze)} euler_valid={int(euler['valid'])} "
                        f"angle={angle_val:.2f} jump_ok={int(angle_jump_ok)} stale={self.bno_stale_sec:.2f}s"
                    )
            else:
                self._bno_heading_invalid_count = 0

            return {
                "acc": acc_val,
                "gyro": gyro_val,
                "mag": mag_val,
                "fall": fall,
                "acc_valid": self.bno_last_acc_time > 0.0 and self.bno_acc_stale_sec <= BNO_STALE_TIMEOUT,
                "angle": angle_val,
                "valid": acc_ok and gyro_ok and mag_ok,
                "angle_valid": angle_valid,
                "calib": calib,
                "sys_status": sys_status,
                "sys_error": sys_error,
                "stale_sec": self.bno_stale_sec,
            }
        except Exception:
            self.bno_fail_count += 1
            if self.bno_fail_count >= BNO_FAIL_LIMIT:
                self._try_reinit_bno()
            self._mark_bno_acc_stale()
            return None

    def get_bmp_data(self):
        bmp_instance = self.devices.get(DEVICE_BMP)
        if bmp_instance is None:
            self._mark_bmp_stale()
            self._record_bmp_failure("missing_device")
            return None
        try:
            # BMP180 pressure compensation depends on the latest temperature read.
            with self._get_i2c_lock():
                temp = float(bmp_instance.getTemperature())
                pres = float(bmp_instance.getPressure())
            if not self._scalar_within(pres, BMP_PRESSURE_MIN_VALID, BMP_PRESSURE_MAX_VALID):
                self._mark_bmp_stale()
                self._record_bmp_failure("pressure_out_of_range")
                return None
            now = time.time()
            frozen_pressure = False
            if pres == getattr(self, "_bmp_last_pressure_sample", None):
                same_since = getattr(self, "_bmp_same_pressure_since", now)
                self._bmp_same_pressure_since = same_since
                if now - same_since > (BNO_STALE_TIMEOUT * 5.0):
                    frozen_pressure = True
            else:
                self._bmp_last_pressure_sample = pres
                self._bmp_same_pressure_since = now

            alt = 44330.0 * (1.0 - math.pow(pres / BMP_SEA_LEVEL_PRESSURE_PA, 1.0 / 5.255))
            if not self._scalar_within(alt, BMP_ALTITUDE_MIN_VALID, BMP_ALTITUDE_MAX_VALID):
                self._mark_bmp_stale()
                self._record_bmp_failure("altitude_out_of_range")
                return None

            if frozen_pressure:
                self._record_bmp_failure("frozen_pressure")
            else:
                self.bmp_fail_count = 0
            self.bmp_last_valid_time = now
            self.bmp_stale_sec = 0.0
            return {"alt": alt, "pres": pres, "temp": temp, "valid": True, "stale_sec": 0.0}
        except Exception:
            self._mark_bmp_stale()
            self._record_bmp_failure("read_exception")
            return None

    def get_sonar_data(self):
        sensor = self.devices.get(DEVICE_SONAR)
        return sensor.read_sample() if sensor is not None else None

    def _update_sonar_state(self, sample, now=None):
        now = time.monotonic() if now is None else float(now)
        distance = sample.distance_cm if sample is not None else None
        age = max(0.0, now - sample.observed_monotonic) if sample is not None else SONAR_STALE_TIMEOUT_SEC + 1.0
        valid = bool(
            sample is not None and sample.sequence > 0
            and sample.observed_monotonic <= now
            and distance is not None and math.isfinite(distance)
            and SONAR_MIN_DISTANCE_CM <= distance < SONAR_MAX_DISTANCE * 100.0
            and age < SONAR_STALE_TIMEOUT_SEC
        )
        self.st.update_sonar(
            sonar_distance_cm=distance if valid else DEFAULT_SONAR_DIST_CM,
            sonar_valid=valid, sonar_stale_sec=age,
            sonar_sequence=sample.sequence if sample is not None else 0,
            sonar_observed_at=sample.observed_at if sample is not None else 0.0,
            sonar_observed_monotonic=sample.observed_monotonic if sample is not None else 0.0,
        )
        return valid

    def cone_detect(self):
        detector = self.devices.get(DEVICE_DETECTOR)
        if detector is None:
            now = time.time()
            self._begin_camera_recovery(now)
            self._try_reinit_camera()
            self._update_camera_recovery_exhausted(time.time())
            self.st.update_cone(
                cone_direction=CONE_CENTER_POSITION,
                cone_probability=0.0,
                cone_is_reached=False,
                cone_method="detector_unavailable",
                cone_valid=False,
                cone_status="detector_unavailable",
                cone_debug=normalize_cone_diagnostics({"status": "detector_unavailable"}),
                observation_time=time.time(),
            )
            return
        try:
            captured = detector.detect_cone()
            if not captured:
                raise RuntimeError("camera capture failed")
            prob = detector.probability if detector.probability else 0.0
            cdir = CONE_CENTER_POSITION
            image_dir = CONE_CENTER_POSITION
            if detector.cone_direction is not None:
                # detector.cone_direction is in camera image coordinates.
                # Keep that physical image position explicitly and apply the
                # optional mount compensation exactly once for motor control.
                image_dir = max(0.0, min(1.0, float(detector.cone_direction)))
                cdir = self._transform_cone_direction_for_control(image_dir)
            cone_method = str(getattr(detector, "debug_method", "unknown"))
            cone_debug = detector_diagnostics(detector, valid=True, status="ok")
            self.st.update_cone(
                cone_direction=cdir,
                cone_image_direction=image_dir,
                cone_probability=prob,
                cone_is_reached=detector.is_reached,
                cone_method=cone_method,
                cone_valid=True,
                cone_status="ok",
                cone_debug=cone_debug,
                observation_time=time.time(),
                observation_accepted=True,
            )
            last_method = getattr(self, "_last_logged_cone_method", None)
            if cone_method != last_method:
                print(f"Cone detector method: {cone_method}")
                self._last_logged_cone_method = cone_method
            self._record_camera_recovered()
        except Exception:
            self.camera_fail_count += 1
            if self.camera_fail_count >= CAMERA_FAIL_LIMIT:
                self.devices[DEVICE_DETECTOR] = None
                self._begin_camera_recovery(time.time())
            self.st.update_cone(
                cone_direction=CONE_CENTER_POSITION,
                cone_probability=0.0,
                cone_is_reached=False,
                cone_method="camera_error",
                cone_valid=False,
                cone_status="camera_error",
                cone_debug=normalize_cone_diagnostics({"status": "camera_error"}),
                observation_time=time.time(),
            )

    def gps_thread(self):
        serial_obj, selected_port, selected_baud = open_gps_serial()
        last_buffer_clear = time.time()
        serial_opened_at = last_buffer_clear
        last_serial_data_time = last_buffer_clear
        last_gga_time = 0.0
        last_fix_time = 0.0
        last_valid_fix_time = 0.0
        last_valid_latlng = None
        recent_valid_fixes = []
        last_heading = None
        last_heading_time = 0.0
        stable_count = 0
        diag_last_log = 0.0
        diag = {
            "status": "OPENING",
            "raw_lines": 0,
            "gga_lines": 0,
            "qual_fail": 0,
            "speed_fail": 0,
            "empty_reads": 0,
            "last_line": "",
            "last_fix_qual": 0,
            "last_sats": 0,
            "last_hdop": 0.0,
            "last_reopen_reason": "INIT",
            "reopen_count": 0,
        }

        def _set_gps_inactive_state():
            self.st.update_gps(
                gps_detect=GPS_INACTIVE_DETECT,
                gps_heading_valid=False,
                gps_speed_mps=0.0,
            )

        def _force_reopen(reason, now_ts):
            nonlocal serial_obj, stable_count, recent_valid_fixes, last_valid_latlng
            nonlocal last_fix_time, last_valid_fix_time, last_heading, last_heading_time
            nonlocal serial_opened_at, last_serial_data_time, last_gga_time
            try:
                if serial_obj is not None:
                    serial_obj.close()
            except Exception:
                pass
            serial_obj = None
            stable_count = 0
            recent_valid_fixes = []
            last_valid_latlng = None
            last_fix_time = 0.0
            last_valid_fix_time = 0.0
            last_heading = None
            last_heading_time = 0.0
            serial_opened_at = now_ts
            last_serial_data_time = now_ts
            last_gga_time = 0.0
            diag["status"] = "REOPENING"
            diag["last_reopen_reason"] = reason
            diag["reopen_count"] += 1
            _set_gps_inactive_state()

        while not bool(getattr(self, "_shutdown_requested", False)):
            try:
                if serial_obj is None or not serial_obj.is_open:
                    diag["status"] = "REOPENING"
                    serial_obj, selected_port, selected_baud = open_gps_serial()
                    if serial_obj is not None and serial_obj.is_open:
                        serial_opened_at = time.time()
                        last_serial_data_time = serial_opened_at
                        last_gga_time = 0.0
                    time.sleep(GPS_RECONNECT_SLEEP)
                    continue
                now = time.time()
                if now - diag_last_log >= GPS_DIAGNOSTIC_LOG_INTERVAL:
                    no_data_for = max(0.0, now - last_serial_data_time)
                    no_gga_for = max(0.0, now - max(last_gga_time, serial_opened_at))
                    print(
                        "GPS diag: "
                        f"port={selected_port or 'none'} baud={selected_baud or 'none'} "
                        f"status={diag['status']} raw={diag['raw_lines']} gga={diag['gga_lines']} "
                        f"qual_fail={diag['qual_fail']} speed_fail={diag['speed_fail']} "
                        f"empty={diag['empty_reads']} stable={stable_count}/{GPS_STABLE_FIX_COUNT} "
                        f"fix={diag['last_fix_qual']} sats={diag['last_sats']} hdop={diag['last_hdop']:.2f} "
                        f"no_data={no_data_for:.1f}s no_gga={no_gga_for:.1f}s "
                        f"reopen={diag['reopen_count']}({diag['last_reopen_reason']}) "
                        f"last={diag['last_line'] or 'none'}"
                    )
                    diag_last_log = now

                no_data_elapsed = now - last_serial_data_time
                if no_data_elapsed >= GPS_NO_DATA_REOPEN_TIMEOUT:
                    print(
                        "GPS watchdog: no serial bytes "
                        f"for {no_data_elapsed:.1f}s on {selected_port or 'unknown'} @ "
                        f"{selected_baud or 'unknown'}; forcing reopen."
                    )
                    _force_reopen("NO_BYTES_TIMEOUT", now)
                    time.sleep(GPS_RECONNECT_SLEEP)
                    continue

                no_gga_elapsed = now - max(last_gga_time, serial_opened_at)
                if diag["raw_lines"] > 0 and no_gga_elapsed >= GPS_NON_GGA_REOPEN_TIMEOUT:
                    print(
                        "GPS watchdog: serial bytes are present but no valid GGA "
                        f"for {no_gga_elapsed:.1f}s on {selected_port or 'unknown'} @ "
                        f"{selected_baud or 'unknown'}; forcing reopen."
                    )
                    _force_reopen("NO_GGA_TIMEOUT", now)
                    time.sleep(GPS_RECONNECT_SLEEP)
                    continue

                if last_valid_fix_time > 0 and now - last_valid_fix_time > GPS_FIX_LOSS_TIMEOUT:
                    stable_count = 0
                    recent_valid_fixes = []
                    diag["status"] = "FIX_LOST"
                    _set_gps_inactive_state()
                if serial_obj.in_waiting > GPS_BUFFER_CLEAR_THRESHOLD and now - last_buffer_clear >= GPS_BUFFER_CLEAR_INTERVAL:
                    try:
                        serial_obj.reset_input_buffer()
                        print("GPS buffer cleared to drop stale data.")
                    except Exception:
                        pass
                    last_buffer_clear = now
                    stable_count = 0
                line_bytes = serial_obj.readline()
                if not line_bytes:
                    diag["status"] = "NO_BYTES"
                    diag["empty_reads"] += 1
                    continue
                last_serial_data_time = now
                line = line_bytes.decode("utf-8", errors="ignore").strip()
                diag["raw_lines"] += 1
                diag["last_line"] = line[:120]
                parsed = parse_gga_sentence(line)
                if parsed is None:
                    diag["status"] = "NON_GGA"
                    continue
                last_gga_time = now
                diag["gga_lines"] += 1
                lat = parsed["lat"]
                lng = parsed["lng"]
                gps_qual = parsed["gps_qual"]
                num_sats = parsed["num_sats"]
                hdop = parsed["hdop"]
                # Record the latest parsed coordinates for logging/diagnostics even
                # before they satisfy the "stable fix" gating used for navigation.
                if lat != 0.0 or lng != 0.0:
                    self.st.update_gps(lat=lat, lng=lng)
                qual_ok, sats_ok, hdop_ok = gga_quality_ok(gps_qual, num_sats, hdop)
                gps_fix_qual_val, gps_sats_val, gps_hdop_val = coerce_gga_metrics(gps_qual, num_sats, hdop)
                diag["last_fix_qual"] = gps_fix_qual_val
                diag["last_sats"] = gps_sats_val
                diag["last_hdop"] = gps_hdop_val
                self.st.update_gps(
                    gps_fix_qual=gps_fix_qual_val,
                    gps_sats=gps_sats_val,
                    gps_hdop=gps_hdop_val,
                )

                if not (qual_ok and sats_ok and hdop_ok and (lat != 0.0 or lng != 0.0)):
                    stable_count = 0
                    diag["status"] = "GGA_REJECTED"
                    diag["qual_fail"] += 1
                    self.st.update_gps(gps_speed_mps=0.0)
                    continue

                speed_ok = True
                speed = 0.0
                if last_valid_latlng is not None:
                    dist, _ = calc_distance_and_azimuth(last_valid_latlng[0], last_valid_latlng[1], lat, lng)
                    dt = now - last_fix_time if last_fix_time > 0 else 0
                    if dt > 0:
                        speed = dist / dt
                        if speed > GPS_MAX_SPEED_MPS:
                            speed_ok = False
                if not speed_ok:
                    stable_count = 0
                    diag["status"] = "SPEED_REJECTED"
                    diag["speed_fail"] += 1
                    self.st.update_gps(gps_speed_mps=0.0)
                    continue

                stable_count += 1
                diag["status"] = "STABILIZING"
                last_fix_time = now
                if stable_count >= GPS_STABLE_FIX_COUNT:
                    gps_heading = None
                    gps_heading_valid = False
                    gps_heading_baseline_m = 0.0
                    gps_speed_mps = 0.0
                    recent_valid_fixes.append((now, lat, lng))
                    cutoff = now - max(1.0, float(GPS_HEADING_WINDOW_SEC))
                    recent_valid_fixes = [fix for fix in recent_valid_fixes if fix[0] >= cutoff]
                    if last_valid_latlng is not None:
                        dist, course = calc_distance_and_azimuth(last_valid_latlng[0], last_valid_latlng[1], lat, lng)
                        dt_valid = now - last_valid_fix_time if last_valid_fix_time > 0 else 0.0
                        if dt_valid > 0:
                            gps_speed_mps = dist / dt_valid
                        if dist >= GPS_HEADING_MIN_DIST:
                            gps_heading = course
                            gps_heading_valid = True
                            gps_heading_baseline_m = dist
                    if not gps_heading_valid and recent_valid_fixes:
                        best_dist = 0.0
                        best_course = None
                        for _, old_lat, old_lng in recent_valid_fixes:
                            span_dist, span_course = calc_distance_and_azimuth(old_lat, old_lng, lat, lng)
                            if span_dist >= GPS_HEADING_BASELINE_MIN_DIST and span_dist > best_dist:
                                best_dist = span_dist
                                best_course = span_course
                        if best_course is not None:
                            gps_heading = best_course
                            gps_heading_valid = True
                            gps_heading_baseline_m = best_dist
                    if gps_heading_valid and gps_heading is not None:
                        last_heading = gps_heading
                        last_heading_time = now
                    elif last_heading is not None and (now - last_heading_time) <= GPS_HEADING_HOLD_SEC:
                        gps_heading = last_heading
                        gps_heading_valid = True
                        gps_heading_baseline_m = 0.0
                    self.st.update_gps(
                        lat=lat,
                        lng=lng,
                        gps_detect=GPS_ACTIVE_DETECT,
                        gps_heading=gps_heading,
                        gps_heading_valid=gps_heading_valid,
                        gps_heading_baseline_m=gps_heading_baseline_m,
                        gps_speed_mps=gps_speed_mps,
                        gps_fix_qual=gps_fix_qual_val,
                        gps_sats=gps_sats_val,
                        gps_hdop=gps_hdop_val,
                        gps_fix_accepted=True,
                    )
                    diag["status"] = "ACTIVE"
                    last_valid_fix_time = now
                    last_valid_latlng = (lat, lng)
                else:
                    _set_gps_inactive_state()
            except Exception as exc:
                print(f"GPS serial error ({type(exc).__name__}: {exc}); attempting reconnect.")
                _force_reopen(f"SERIAL_ERROR:{type(exc).__name__}", time.time())
                time.sleep(GPS_RECONNECT_SLEEP)

    def camera_thread(self):
        while not bool(getattr(self, "_shutdown_requested", False)):
            try:
                current_phase = self.st.snapshot()["phase"]
                if self._sync_camera_runtime_for_phase(current_phase):
                    t_start = time.time()
                    self.cone_detect()
                    elapsed = time.time() - t_start
                    remain = float(CAMERA_ACTIVE_SLEEP) - elapsed
                    if remain > 0.0:
                        time.sleep(remain)
                else:
                    time.sleep(CAMERA_IDLE_SLEEP)
            except Exception as exc:
                # Camera failures are optional-subsystem failures.  Keep them
                # inside this worker so they can never request a motor stop.
                print(f"Camera Thread Slice Error: {exc}")
                time.sleep(CAMERA_IDLE_SLEEP)

    def _sync_camera_runtime_for_phase(self, phase):
        """Keep camera hardware completely inactive outside P4/P5/P6."""
        try:
            phase_enum = Phase(phase)
        except (TypeError, ValueError):
            phase_enum = None
        camera_needed = phase_enum in PHASES_CAMERA_ACTIVE

        if not camera_needed:
            detector = self.devices.get(DEVICE_DETECTOR)
            if bool(getattr(self, "_camera_runtime_active", False)) or detector is not None:
                if hasattr(self, "_release_camera_detector"):
                    self._release_camera_detector()
            self._camera_runtime_active = False
            return False

        if not bool(getattr(self, "_camera_runtime_active", False)):
            self._camera_runtime_active = True
            if self.devices.get(DEVICE_DETECTOR) is None:
                setup = getattr(self, "_setup_camera_detector", None)
                if callable(setup):
                    setup()
        return True

    def bno_thread(self):
        suspicious_bno_counter = 0
        next_bmp_read = 0.0
        while not bool(getattr(self, "_shutdown_requested", False)):
            bno_data = None
            try:
                bno_data = self.get_bno_data()
            except Exception as exc:
                print(f"BNO Thread Slice Error: {exc}")
                traceback.print_exc()
            if bno_data:
                self.st.update_imu(
                    acc=bno_data["acc"],
                    gyro=bno_data["gyro"],
                    mag=bno_data["mag"],
                    fall=bno_data["fall"],
                    angle=bno_data["angle"],
                    angle_valid=bno_data["angle_valid"],
                )
                if bno_data.get("sys_status", {}).get("valid") and bno_data.get("sys_error", {}).get("valid"):
                    if (
                        bno_data["sys_error"]["value"] != 0
                        or bno_data["sys_status"]["value"] not in BNO_FUSION_OK_STATES
                    ):
                        print(
                            f"BNO status warn: sys={bno_data['sys_status']['value']} "
                            f"err={bno_data['sys_error']['value']}"
                        )
                acc_norm = self._vector_norm(bno_data.get("acc"))
                mag_norm = self._vector_norm(bno_data.get("mag"))
                if bno_data.get("acc_valid") and acc_norm is not None and mag_norm is not None:
                    if acc_norm < 1.0 or mag_norm < 1.0:
                        suspicious_bno_counter += 1
                        if suspicious_bno_counter % 25 == 0:
                            print(
                                "BNO suspicious sample: "
                                f"acc={bno_data.get('acc')} |acc|={acc_norm:.3f} "
                                f"mag={bno_data.get('mag')} |mag|={mag_norm:.3f} "
                                f"angle={float(bno_data.get('angle', 0.0)):.2f}"
                            )
                    else:
                        suspicious_bno_counter = 0
            else:
                self.st.update_imu(angle_valid=False)
                self._mark_bno_acc_stale()

            now = time.time()
            if now >= next_bmp_read:
                next_bmp_read = now + max(float(BMP_SAMPLING_RATE), float(DATA_SAMPLING_RATE))
                try:
                    bmp_data = self.get_bmp_data()
                except Exception as exc:
                    print(f"BMP Thread Slice Error: {exc}")
                    traceback.print_exc()
                    bmp_data = None
                if bmp_data:
                    self.st.update_barometer(
                        alt=bmp_data["alt"],
                        pres=bmp_data["pres"],
                    )

            time.sleep(DATA_SAMPLING_RATE)

    def sonar_thread(self):
        while not bool(getattr(self, "_shutdown_requested", False)):
            try:
                sonar_dist = self.get_sonar_data()
            except Exception as exc:
                print(f"Sonar Thread Slice Error: {exc}")
                traceback.print_exc()
                sonar_dist = None
            self._update_sonar_state(sonar_dist)
            time.sleep(DATA_SAMPLING_RATE)

    def data_thread(self):
        """Compatibility worker for tests or old launch scripts."""
        threading.Thread(target=self.sonar_thread, daemon=True).start()
        self.bno_thread()
