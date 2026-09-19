import math
import time

from mission.const import (
    APPROACH_TURN_GAIN,
    BASE_SPEED,
    CAMERA_FRAME_STALE_STOP_SEC,
    CAMERA_EDGE_TURN_ERROR,
    CAMERA_EDGE_TURN_RELEASE_ERROR,
    CAMERA_EDGE_TURN_SPEED,
    CONE_CENTER_POSITION,
    CAMERA_TINY_MIN_CONSISTENT_FRAMES,
    CAMERA_TINY_OCCUPANCY_THRESHOLD,
    CONE_PROBABILITY_THRESHOLD_PHASE5,
    DEVICE_MOTOR_1_DIR,
    DEVICE_MOTOR_1_PWM,
    DEVICE_MOTOR_2_DIR,
    DEVICE_MOTOR_2_PWM,
    GPS_ACTIVE_DETECT,
    GRASS_MIN_MOTOR_SPEED,
    MOTOR_IDLE_SLEEP,
    MOTOR_LOOP_INTERVAL,
    MOTOR_RAMP_STEP,
    MOTOR_RAMP_TIME,
    MOTOR_SPEED_OFFSET_1,
    MOTOR_SPEED_OFFSET_2,
    MOTOR_SPEED_SCALE_1,
    MOTOR_SPEED_SCALE_2,
    OBSTACLE_AVOID_DIST,
    OBSTACLE_BACKUP_TIME,
    OBSTACLE_CONFIRM_COUNT,
    OBSTACLE_PAUSE_TIME,
    OBSTACLE_SPEED,
    OBSTACLE_TURN_TIME,
    PARACHUTE_DIRECTION,
    PARACHUTE_MOTOR_PULSE,
    PARACHUTE_SEPARATION_SPEED,
    PHASE1_SOFTSTART_RAMP_TIME,
    PHASE1_SOFTSTART_STEP,
    PHASE4_ALIGN_INNER_SPEED,
    PHASE4_ALIGN_PIVOT_SPEED,
    PHASE4_ALIGN_STOP_DEADBAND,
    PHASE4_CANDIDATE_INNER_SPEED,
    PHASE4_CANDIDATE_CAPTURE_SEC,
    PHASE4_CANDIDATE_CAPTURE_MAX_FRAMES,
    PHASE4_CANDIDATE_CAPTURE_MIN_FRAMES,
    PHASE4_CANDIDATE_OUTER_SPEED,
    PHASE4_CANDIDATE_TRACK_CONFIRM_FRAMES,
    PHASE4_MOTOR_RAMP_TIME,
    PHASE4_REACQUIRE_TURN_SEC,
    PHASE4_TRACK_MIN_ROTATE_SPEED,
    PHASE4_TRACK_ROTATE_CLAMP,
    PHASE4_TRACK_ROTATE_GAIN,
    PHASE45_CONE_DIR_FILTER_ALPHA,
    PHASE45_MOTOR_LOOP_INTERVAL,
    PHASE5_BASE_SPEED,
    PHASE5_MID_OCCUPANCY_THRESHOLD,
    PHASE5_MID_SPEED,
    PHASE5_MOTOR_RAMP_TIME,
    PHASE5_NEAR_OCCUPANCY_THRESHOLD,
    PHASE5_NEAR_SPEED,
    PHASE5_REACQUIRE_GRACE_SEC,
    PHASE5_REACQUIRE_INNER_SPEED,
    PHASE5_REACQUIRE_OUTER_SPEED,
    PHASE5_STEER_DEADBAND,
    PHASE5_TURN_CLAMP,
    PHASE2_SPEED,
    PHASE2_OFFSET_HOLD_DEADBAND_DEG,
    PHASE2_OFFSET_HOLD_KP,
    PHASE2_OFFSET_HOLD_MAX_DELTA,
    PHASE2_OFFSET_MODE_REORIENT,
    PHASE2_OFFSET_MODE_BNO_WAIT,
    PHASE2_OFFSET_MODE_READINESS,
    PHASE2_OFFSET_SPEED,
    PHASE2_FORWARD_REORIENT_INNER_SPEED,
    PHASE2_FORWARD_REORIENT_OUTER_SPEED,
    PHASE2_CALIB_ARC_INNER_SPEED,
    PHASE2_CALIB_ARC_INTERVAL_SEC,
    PHASE2_CALIB_ARC_OUTER_SPEED,
    PHASE2_OFFSET_CORRECTION_BASE_SPEED,
    PHASE2_STAGE_CALIBRATION,
    PHASE2_STAGE_ESCAPE,
    PHASE2_STAGE_OFFSET,
    PHASE2_TURN_INTERVAL,
    PHASE2_RAMP_TIME,
    PHASE3_FORWARD_RAMP_TIME,
    PHASE3_FORWARD_SPEED,
    PHASE3_GPS_ARC_BASE_SPEED,
    PHASE3_GPS_ARC_KP,
    PHASE3_GPS_ARC_MAX_DELTA,
    PHASE3_GPS_ARC_MAX_HOLD_SEC,
    PHASE3_GPS_ARC_MIN_DELTA,
    PHASE3_GPS_FALLBACK_DEADBAND_DEG,
    PHASE3_HEADING_DEADBAND_DEG,
    PHASE3_BNO_TRUST_MAX_JUMP_DEG,
    PHASE3_BNO_TRUST_MAX_OFFSET_DEG,
    PHASE3_BNO_TRUST_MAX_STALE_SEC,
    PHASE3_LARGE_ERROR_DEG,
    PHASE3_LARGE_ERROR_INNER_SPEED,
    PHASE3_LARGE_ERROR_OUTER_SPEED,
    PHASE3_MAG_STUCK_MIN_DELTA_DEG,
    PHASE3_MAG_STUCK_TIMEOUT_SEC,
    PHASE3_MOTOR_LOOP_INTERVAL,
    PHASE3_NO_HEADING_SPEED,
    PHASE3_NO_HEADING_TIMEOUT_SEC,
    PHASE3_TURN_RAMP_TIME,
    PHASE3_TURN_INNER_SPEED,
    PHASE3_TURN_OUTER_SPEED,
    PHASES_SKIP_OBSTACLE,
    PHASES_STOP_MOTORS,
    PWM_DUTY_MAX,
    PWM_DUTY_MIN,
    PWM_PERCENT_MAX,
    PWM_PERCENT_MIN,
    RAMP_HALF_DIVISOR,
    PHASE4_SEARCH_OUTER_SPEED,
    PHASE4_SEARCH_INNER_SPEED,
    TURN_GAIN_SCALE_MAX,
    TURN_GAIN_SCALE_MIN,
    Phase,
)
from mission.cone_candidate import cone_centered_for_final_ram, evaluate_cone_candidate
from mission.motor_map import (
    apply_turn_speed_floor,
    forward_to_dir_value,
    get_manual_drive_pattern,
    map_logical_wheels_to_physical,
    motor_forward_to_dir_value,
)


class MotorManager:
    def _shutdown_active(self):
        return bool(getattr(self, "_shutdown_requested", False))

    def _set_forward_diff_turn(self, fast_side, speed_fast, speed_slow, cmd_type, ramp_time=MOTOR_RAMP_TIME):
        """Steer with differential forward speeds only (no reverse)."""
        speed_fast = self._clamp_percent(speed_fast)
        speed_slow = self._clamp_percent(speed_slow)
        if fast_side == "left":
            self.set_motors(speed_fast, True, speed_slow, True, ramp_time=ramp_time, cmd_type=cmd_type)
        else:
            self.set_motors(speed_slow, True, speed_fast, True, ramp_time=ramp_time, cmd_type=cmd_type)

    def _phase2_offset_hold_speeds(self, snapshot):
        reference = getattr(self, "phase2_offset_reference_bno_deg", None)
        if reference is None or not snapshot.get("angle_valid", False):
            return None
        try:
            reference = float(reference) % 360.0
            angle = float(snapshot.get("angle")) % 360.0
        except (TypeError, ValueError):
            return None
        if not math.isfinite(reference) or not math.isfinite(angle):
            return None

        error = self._angle_diff_deg(reference, angle)
        self.phase2_offset_heading_error_deg = error
        straight = self._clamp_percent(PHASE2_OFFSET_SPEED)
        if abs(error) <= float(PHASE2_OFFSET_HOLD_DEADBAND_DEG):
            return straight, straight, error

        base = self._clamp_percent(PHASE2_OFFSET_CORRECTION_BASE_SPEED)
        delta = min(
            float(PHASE2_OFFSET_HOLD_MAX_DELTA),
            abs(error) * float(PHASE2_OFFSET_HOLD_KP),
        )
        fast = self._clamp_percent(base + delta)
        slow = self._clamp_percent(base - delta)
        if error > 0.0:
            return fast, slow, error
        return slow, fast, error

    def _phase2_calibration_pattern(self, elapsed):
        outer = self._clamp_percent(PHASE2_CALIB_ARC_OUTER_SPEED)
        inner = self._clamp_percent(PHASE2_CALIB_ARC_INNER_SPEED)
        left_turn = int(float(elapsed) // PHASE2_CALIB_ARC_INTERVAL_SEC) % 2 == 0
        if left_turn:
            return inner, True, outer, True
        return outer, True, inner, True

    def _drive_phase2_forward_reorient(self):
        outer = self._clamp_percent(PHASE2_FORWARD_REORIENT_OUTER_SPEED)
        inner = self._clamp_percent(PHASE2_FORWARD_REORIENT_INNER_SPEED)
        retry = int(getattr(self, "phase2_offset_stage_retry_count", 0))
        if retry % 2:
            self.set_motors(
                inner, True, outer, True,
                ramp_time=PHASE2_RAMP_TIME,
                cmd_type="phase2_offset_forward_reorient_left",
            )
        else:
            self.set_motors(
                outer, True, inner, True,
                ramp_time=PHASE2_RAMP_TIME,
                cmd_type="phase2_offset_forward_reorient_right",
            )

    def _set_forward_pivot_turn(
        self,
        turn_side,
        speed_outer,
        cmd_type,
        speed_inner=0.0,
        ramp_time=MOTOR_RAMP_TIME,
    ):
        """Turn with one wheel driving forward and the inner wheel stopped."""
        speed_outer = self._clamp_percent(speed_outer)
        speed_inner = self._clamp_percent(speed_inner)
        if turn_side == "left":
            self.set_motors(speed_inner, True, speed_outer, True, ramp_time=ramp_time, cmd_type=cmd_type)
        else:
            self.set_motors(speed_outer, True, speed_inner, True, ramp_time=ramp_time, cmd_type=cmd_type)

    def _mag_heading_from_snapshot(self, snapshot):
        mag = snapshot.get("mag")
        if not isinstance(mag, (list, tuple)) or len(mag) < 2:
            return None
        try:
            mx = float(mag[0])
            my = float(mag[1])
        except (TypeError, ValueError):
            return None
        if not math.isfinite(mx) or not math.isfinite(my):
            return None
        if abs(mx) < 1e-6 and abs(my) < 1e-6:
            return None
        # Legacy rover logic (main(7).py): magnetometer XY -> heading.
        azimuth = 90.0 - math.degrees(math.atan2(my, mx))
        azimuth *= -1.0
        azimuth %= 360.0
        return azimuth

    def _mag_heading_is_stuck(self, mag_heading):
        now = time.time()
        last_heading = getattr(self, "_phase3_last_mag_heading", None)
        last_change = float(getattr(self, "_phase3_last_mag_heading_change", 0.0))
        if last_heading is None:
            self._phase3_last_mag_heading = mag_heading
            self._phase3_last_mag_heading_change = now
            self._phase3_mag_stuck_reported = False
            return False

        delta = abs(self._angle_diff_deg(mag_heading, last_heading))
        if delta >= float(PHASE3_MAG_STUCK_MIN_DELTA_DEG):
            self._phase3_last_mag_heading = mag_heading
            self._phase3_last_mag_heading_change = now
            self._phase3_mag_stuck_reported = False
            return False

        last_cmd = str(getattr(self, "last_motor_command", {}).get("type", ""))
        actively_turning = last_cmd == "phase3_gps_turn"
        stuck = actively_turning and (now - last_change) >= float(PHASE3_MAG_STUCK_TIMEOUT_SEC)
        if stuck and not getattr(self, "_phase3_mag_stuck_reported", False):
            print(
                "Phase3 magnetometer heading appears stuck; "
                f"mag_heading={mag_heading:.1f} last_cmd={last_cmd} "
                f"stuck_for={now - last_change:.2f}s"
            )
            self._phase3_mag_stuck_reported = True
        return stuck

    def _phase3_heading(self, snapshot):
        # Phase3 policy:
        # - A verified BNO/GPS alignment enables high-rate BNO steering.
        # - Otherwise use GPS course only; never add an unverified BNO offset.
        # GPS-only motor control uses forward arcs below so that it keeps
        # producing displacement from which the next GPS course can be measured.
        gps_heading = None
        if snapshot.get("gps_heading_valid", False):
            gps_heading = self._normalize_heading_deg(snapshot.get("gps_heading"))

        try:
            angle = float(snapshot.get("angle", 0.0))
        except (TypeError, ValueError):
            angle = 0.0

        if gps_heading is not None:
            if snapshot.get("angle_valid", False) and math.isfinite(angle) and hasattr(self, "_update_bno_heading_offset_from_gps"):
                self._update_bno_heading_offset_from_gps(snapshot)

        bno_trusted = self._phase3_bno_trusted(snapshot, angle)
        self._phase3_last_bno_trusted = bno_trusted
        if bno_trusted:
            if getattr(self, "bno_heading_offset_valid", False) and hasattr(self, "_bno_heading_aligned_to_gps"):
                aligned = self._bno_heading_aligned_to_gps(snapshot)
                if aligned is not None:
                    self._phase3_last_heading_trust = 1.0
                    return aligned, "BNO_ALIGNED"

        if gps_heading is not None:
            self._phase3_last_heading_trust = 0.45
            return gps_heading, "GPS_PRIMARY"

        if snapshot.get("gps_heading_valid", False):
            self._phase3_last_heading_trust = 0.0
            return None, "GPS_PARSE_FAIL"
        self._phase3_last_heading_trust = 0.0
        return None, "NO_HEADING_SOURCE"

    def _phase3_bno_trusted(self, snapshot, angle=None):
        if not snapshot.get("angle_valid", False):
            return False
        if angle is None:
            try:
                angle = float(snapshot.get("angle", 0.0))
            except (TypeError, ValueError):
                return False
        if not math.isfinite(angle):
            return False
        stale = float(getattr(self, "bno_stale_sec", 999.0))
        if stale > float(PHASE3_BNO_TRUST_MAX_STALE_SEC):
            return False
        if not getattr(self, "bno_heading_offset_valid", False):
            return False
        if not getattr(self, "bno_heading_offset_verified", False):
            return False
        offset = self._normalize_heading_deg(getattr(self, "bno_heading_offset_deg", 0.0))
        if offset is None:
            return False
        if abs(self._angle_diff_deg(offset, 0.0)) > float(PHASE3_BNO_TRUST_MAX_OFFSET_DEG):
            return False
        last = getattr(self, "_phase3_last_trusted_bno_angle", None)
        if last is not None:
            jump = abs(self._angle_diff_deg(angle, last))
            if jump > float(PHASE3_BNO_TRUST_MAX_JUMP_DEG):
                return False
        self._phase3_last_trusted_bno_angle = angle
        return True

    def _clamp_percent(self, value):
        return max(PWM_PERCENT_MIN, min(PWM_PERCENT_MAX, value))

    def _get_motor_speed_scale(self, motor_index):
        default = MOTOR_SPEED_SCALE_1 if motor_index == 1 else MOTOR_SPEED_SCALE_2
        attr_name = f"motor_speed_scale_{motor_index}"
        try:
            scale = float(getattr(self, attr_name, default))
        except (TypeError, ValueError):
            scale = default
        return max(0.0, scale)

    def _get_motor_speed_offset(self, motor_index):
        default = MOTOR_SPEED_OFFSET_1 if motor_index == 1 else MOTOR_SPEED_OFFSET_2
        attr_name = f"motor_speed_offset_{motor_index}"
        try:
            offset = float(getattr(self, attr_name, default))
        except (TypeError, ValueError):
            offset = default
        return offset

    def _apply_motor_speed_scale(self, speed, motor_index):
        scaled = float(speed) * self._get_motor_speed_scale(motor_index)
        adjusted = scaled + self._get_motor_speed_offset(motor_index)
        return self._clamp_percent(adjusted)

    def _phase45_bno_heading(self, snapshot):
        heading, _ = self._phase3_heading(snapshot)
        return heading

    def _phase3_legacy_drive_speeds(self, turn_scale=1.0):
        # Match the proven main(8).py strategy:
        # - straight: 40/40
        # - turn: 25/15
        # For GPS-only fallback, blend back toward straight to reduce wandering.
        straight = self._clamp_percent(PHASE3_FORWARD_SPEED)
        outer_legacy = self._clamp_percent(PHASE3_TURN_OUTER_SPEED)
        inner_legacy = self._clamp_percent(PHASE3_TURN_INNER_SPEED)
        scale = max(0.0, min(1.0, float(turn_scale)))
        outer = self._clamp_percent(straight - (straight - outer_legacy) * scale)
        inner = self._clamp_percent(straight - (straight - inner_legacy) * scale)
        return straight, outer, inner

    def _phase3_gps_arc_speeds(self, heading_error_deg):
        """Return a gentle forward-only arc for sparse GPS course feedback."""
        magnitude = abs(float(heading_error_deg))
        delta = min(
            float(PHASE3_GPS_ARC_MAX_DELTA),
            max(float(PHASE3_GPS_ARC_MIN_DELTA), magnitude * float(PHASE3_GPS_ARC_KP)),
        )
        base = float(PHASE3_GPS_ARC_BASE_SPEED)
        fast = self._clamp_percent(base + delta / 2.0)
        slow = self._clamp_percent(base - delta / 2.0)
        if heading_error_deg > 0.0:
            return fast, slow
        return slow, fast

    def _drive_phase3_navigation(self, snapshot, target_heading, now=None):
        """Apply one Phase3 motor decision and return heading diagnostics."""
        if now is None:
            now = time.time()
        if snapshot.get("gps_detect", GPS_ACTIVE_DETECT) != GPS_ACTIVE_DETECT:
            # A stale target heading is not enough to navigate safely after the
            # position fix is lost.  The GPS thread keeps reconnecting while the
            # rover waits here, and normal navigation resumes automatically on
            # the first active fix.
            self.phase3_no_heading_start = None
            self.stop_motors()
            return None, "GPS_LOST_STOP", None

        nav_heading, heading_source = self._phase3_heading(snapshot)
        if nav_heading is None:
            if self.phase3_no_heading_start is None:
                self.phase3_no_heading_start = now
            elapsed = now - self.phase3_no_heading_start
            gps_fix_active = snapshot.get("gps_detect") == GPS_ACTIVE_DETECT
            if gps_fix_active or elapsed <= float(PHASE3_NO_HEADING_TIMEOUT_SEC):
                probe_speed = self._clamp_percent(PHASE3_NO_HEADING_SPEED)
                self.set_motors(
                    probe_speed,
                    True,
                    probe_speed,
                    True,
                    ramp_time=PHASE3_FORWARD_RAMP_TIME,
                    cmd_type="phase3_gps_probe",
                )
            else:
                self.stop_motors()
            return nav_heading, heading_source, None

        self.phase3_no_heading_start = None
        diff = self._angle_diff_deg(target_heading, nav_heading)
        if heading_source.startswith("GPS"):
            try:
                gps_fix_seq = int(snapshot.get("gps_fix_seq", 0))
            except (TypeError, ValueError):
                gps_fix_seq = 0
            if gps_fix_seq != getattr(self, "_phase3_gps_arc_fix_seq", None):
                self._phase3_gps_arc_fix_seq = gps_fix_seq
                self._phase3_gps_arc_started_at = now

            if abs(diff) <= float(PHASE3_GPS_FALLBACK_DEADBAND_DEG):
                straight = self._clamp_percent(PHASE3_FORWARD_SPEED)
                self.set_motors(
                    straight,
                    True,
                    straight,
                    True,
                    ramp_time=PHASE3_FORWARD_RAMP_TIME,
                    cmd_type="phase3_gps_forward",
                )
            elif (
                now - float(getattr(self, "_phase3_gps_arc_started_at", now))
                > float(PHASE3_GPS_ARC_MAX_HOLD_SEC)
            ):
                probe_speed = self._clamp_percent(PHASE3_NO_HEADING_SPEED)
                self.set_motors(
                    probe_speed,
                    True,
                    probe_speed,
                    True,
                    ramp_time=PHASE3_FORWARD_RAMP_TIME,
                    cmd_type="phase3_gps_probe",
                )
            else:
                left_speed, right_speed = self._phase3_gps_arc_speeds(diff)
                self.set_motors(
                    left_speed,
                    True,
                    right_speed,
                    True,
                    ramp_time=PHASE3_TURN_RAMP_TIME,
                    cmd_type="phase3_gps_arc",
                )
            return nav_heading, heading_source, diff

        straight_speed, turn_outer, turn_inner = self._phase3_legacy_drive_speeds()
        if abs(diff) <= float(PHASE3_HEADING_DEADBAND_DEG):
            self.set_motors(
                straight_speed,
                True,
                straight_speed,
                True,
                ramp_time=PHASE3_FORWARD_RAMP_TIME,
                cmd_type="phase3_bno_forward",
            )
        elif abs(diff) >= float(PHASE3_LARGE_ERROR_DEG):
            # A stopped inner wheel dragged badly on grass. Keep both wheels
            # powered and use a high-torque forward arc for large errors.
            fast_side = "left" if diff > 0 else "right"
            self._set_forward_diff_turn(
                fast_side,
                speed_fast=float(PHASE3_LARGE_ERROR_OUTER_SPEED),
                speed_slow=float(PHASE3_LARGE_ERROR_INNER_SPEED),
                cmd_type="phase3_bno_large_arc",
                ramp_time=PHASE3_TURN_RAMP_TIME,
            )
        elif diff > 0:
            self.set_motors(
                turn_outer,
                True,
                turn_inner,
                True,
                ramp_time=PHASE3_TURN_RAMP_TIME,
                cmd_type="phase3_bno_turn",
            )
        else:
            self.set_motors(
                turn_inner,
                True,
                turn_outer,
                True,
                ramp_time=PHASE3_TURN_RAMP_TIME,
                cmd_type="phase3_bno_turn",
            )
        return nav_heading, heading_source, diff

    def _reset_phase45_camera_track(self):
        self.phase45_edge_turn_side = None
        self.phase45_filtered_cone_dir = None
        self.phase45_filtered_cone_seq = None
        self.phase45_last_seen_time = None

    def _reset_phase4_search_cycle(self, now=None):
        now = time.time() if now is None else float(now)
        last_candidate = getattr(self, "phase45_filtered_cone_dir", None)
        self.phase4_search_state = "reacquire" if last_candidate is not None else "drive"
        self.phase4_reacquire_until = (
            now + float(PHASE4_REACQUIRE_TURN_SEC)
            if last_candidate is not None
            else None
        )
        self.phase4_last_candidate_dir = last_candidate
        self.phase4_candidate_active_until = 0.0
        self.phase4_motor_candidate_seq = 0
        self.phase4_observe_frame_count = 0
        self.phase4_motor_track_count = 0
        self.phase4_motor_track_signature = None
        self.phase4_motor_missed_frames = 0
        self.phase4_motor_track_updated_at = 0.0

    @staticmethod
    def _snapshot_float(snapshot, key, default=0.0):
        try:
            return float(snapshot.get(key, default))
        except (TypeError, ValueError):
            return float(default)

    def _camera_observation_fresh(self, snapshot, now=None):
        now = time.time() if now is None else float(now)
        sequence = int(self._snapshot_float(snapshot, "cone_sequence", 0.0))
        updated_at = self._snapshot_float(snapshot, "cone_updated_at", 0.0)
        if updated_at <= 0.0:
            return False
        age = max(0.0, now - updated_at)
        return bool(
            snapshot.get("cone_valid", False)
            and sequence > 0
            and age <= float(CAMERA_FRAME_STALE_STOP_SEC)
        )

    def _drive_phase4_toward_last_candidate(self):
        last_dir = getattr(self, "phase4_last_candidate_dir", None)
        try:
            error = float(last_dir) - CONE_CENTER_POSITION
        except (TypeError, ValueError):
            return False
        if abs(error) <= float(PHASE4_ALIGN_STOP_DEADBAND):
            return False
        turn_side = "right" if error > 0.0 else "left"
        self._set_forward_pivot_turn(
            turn_side,
            PHASE4_ALIGN_PIVOT_SPEED,
            cmd_type="phase4_reacquire_last_candidate",
            speed_inner=PHASE4_ALIGN_INNER_SPEED,
            ramp_time=PHASE4_MOTOR_RAMP_TIME,
        )
        return True

    def _drive_camera_edge_turn(self, error, command_prefix, ramp_time):
        """Recover image margin when a powered-inner-wheel arc is saturated."""
        side = "right" if error > 0 else "left"
        threshold = (
            CAMERA_EDGE_TURN_RELEASE_ERROR
            if getattr(self, "phase45_edge_turn_side", None) == side
            else CAMERA_EDGE_TURN_ERROR
        )
        if abs(error) < threshold:
            self.phase45_edge_turn_side = None
            return False
        self.phase45_edge_turn_side = side
        self._set_forward_pivot_turn(
            side, CAMERA_EDGE_TURN_SPEED, speed_inner=0.0,
            ramp_time=ramp_time, cmd_type=f"{command_prefix}_edge_{side}",
        )
        return True

    def _drive_phase4_candidate_capture(self, command_prefix="phase4_candidate_capture"):
        """Move forward while proportionally steering a candidate to center."""
        last_dir = getattr(self, "phase4_last_candidate_dir", None)
        try:
            error = float(last_dir) - CONE_CENTER_POSITION
        except (TypeError, ValueError):
            return False
        if self._drive_camera_edge_turn(error, command_prefix, PHASE4_MOTOR_RAMP_TIME):
            return True
        if abs(error) <= float(PHASE4_ALIGN_STOP_DEADBAND):
            self.set_motors(
                PHASE4_CANDIDATE_INNER_SPEED,
                True,
                PHASE4_CANDIDATE_INNER_SPEED,
                True,
                ramp_time=PHASE4_MOTOR_RAMP_TIME,
                cmd_type=f"{command_prefix}_forward",
            )
            return True
        turn_delta = min(
            float(PHASE4_TRACK_ROTATE_CLAMP),
            max(
                float(PHASE4_TRACK_MIN_ROTATE_SPEED),
                abs(error) * float(PHASE4_TRACK_ROTATE_GAIN),
            ),
            max(
                0.0,
                float(PHASE4_CANDIDATE_OUTER_SPEED)
                - float(PHASE4_CANDIDATE_INNER_SPEED),
            ),
        )
        speed_outer = float(PHASE4_CANDIDATE_INNER_SPEED) + turn_delta
        turn_side = "right" if error > 0.0 else "left"
        self._set_forward_pivot_turn(
            turn_side,
            speed_outer,
            cmd_type=f"{command_prefix}_arc",
            speed_inner=PHASE4_CANDIDATE_INNER_SPEED,
            ramp_time=PHASE4_MOTOR_RAMP_TIME,
        )
        return True

    def _drive_phase4_candidate_observe(self):
        """Immediately reduce search rotation and center a credible candidate."""
        return self._drive_phase4_candidate_capture(
            command_prefix="phase4_candidate_observe"
        )

    def _phase4_motor_candidate(self, snapshot):
        evidence = evaluate_cone_candidate(snapshot)
        if not evidence["candidate"]:
            return None

        direction = self._snapshot_float(
            snapshot,
            "cone_direction",
            CONE_CENTER_POSITION,
        )
        return {
            "direction": max(0.0, min(1.0, direction)),
            "occupancy": float(evidence["occupancy"]),
            "tiny": bool(evidence["tiny"]),
            "strict": bool(evidence["strict"]),
            "weak": bool(evidence["weak"]),
        }

    @staticmethod
    def _phase4_motor_candidate_consistent(previous, current):
        if not previous:
            return True
        if abs(float(current["direction"]) - float(previous["direction"])) > 0.22:
            return False
        previous_occ = float(previous.get("occupancy", 0.0))
        current_occ = float(current.get("occupancy", 0.0))
        if previous_occ > 0.0 and current_occ > 0.0:
            ratio = max(previous_occ / current_occ, current_occ / previous_occ)
            if ratio > 2.4:
                return False
        return True

    def _phase4_new_snapshot_candidate(self, snapshot, now):
        sequence = int(self._snapshot_float(snapshot, "cone_sequence", 0.0))
        if sequence <= int(getattr(self, "phase4_motor_candidate_seq", 0)):
            return False
        self.phase4_motor_candidate_seq = sequence
        state = getattr(self, "phase4_search_state", "drive")
        if state == "observe":
            self.phase4_observe_frame_count = int(
                getattr(self, "phase4_observe_frame_count", 0)
            ) + 1

        candidate = self._phase4_motor_candidate(snapshot)
        previous = getattr(self, "phase4_motor_track_signature", None)
        last_track_at = float(getattr(self, "phase4_motor_track_updated_at", 0.0))
        if previous and last_track_at > 0.0 and float(now) - last_track_at > float(
            PHASE4_CANDIDATE_CAPTURE_SEC
        ):
            previous = None
            self.phase4_motor_track_signature = None
            self.phase4_motor_track_count = 0
        if candidate is not None:
            consistent = self._phase4_motor_candidate_consistent(previous, candidate)
            if not consistent:
                previous_occ = float((previous or {}).get("occupancy", 0.0))
                # A tiny high-score grass fragment must not steal a credible
                # cone track. Treat it as a missed frame instead.
                if (
                    previous_occ >= float(CAMERA_TINY_OCCUPANCY_THRESHOLD)
                    and bool(candidate.get("tiny"))
                ):
                    candidate = None
                else:
                    self.phase4_motor_track_count = 0
                    previous = None

        if candidate is not None:
            if bool(candidate.get("tiny")) and state not in ("observe", "track"):
                # Collect a temporal track while preserving the normal search
                # command. A tiny single frame must not move the rover at all.
                self.phase4_motor_track_count = int(
                    getattr(self, "phase4_motor_track_count", 0)
                ) + 1
                self.phase4_motor_track_signature = candidate
                self.phase4_motor_track_updated_at = float(now)
                self.phase4_motor_missed_frames = 0
                if self.phase4_motor_track_count >= int(
                    CAMERA_TINY_MIN_CONSISTENT_FRAMES
                ):
                    self.phase4_search_state = "track"
                    self.phase4_last_candidate_dir = float(candidate["direction"])
                    self.phase4_candidate_active_until = (
                        float(now) + float(PHASE4_CANDIDATE_CAPTURE_SEC)
                    )
                return True

            # A non-tiny candidate is safe enough for one gentle steering
            # response.  Temporal confirmation is still required before P5.
            self.phase4_last_candidate_dir = float(candidate["direction"])
            if state not in ("observe", "track"):
                self.phase4_search_state = "observe"
                state = "observe"
                self.phase4_observe_frame_count = 1
                self.phase4_candidate_active_until = (
                    float(now) + float(PHASE4_CANDIDATE_CAPTURE_SEC)
                )
            elif state == "track" and float(now) > float(
                getattr(self, "phase4_candidate_active_until", now)
            ):
                self.phase4_candidate_active_until = (
                    float(now) + float(PHASE4_CANDIDATE_CAPTURE_SEC)
                )

            self.phase4_motor_track_count = int(
                getattr(self, "phase4_motor_track_count", 0)
            ) + 1
            self.phase4_motor_track_signature = candidate
            self.phase4_motor_track_updated_at = float(now)
            self.phase4_motor_missed_frames = 0
            required = (
                CAMERA_TINY_MIN_CONSISTENT_FRAMES
                if bool(candidate.get("tiny"))
                else PHASE4_CANDIDATE_TRACK_CONFIRM_FRAMES
            )
            if (
                int(self.phase4_motor_track_count) >= int(required)
                and int(getattr(self, "phase4_observe_frame_count", 0))
                >= int(PHASE4_CANDIDATE_CAPTURE_MIN_FRAMES)
            ):
                self.phase4_search_state = "track"
                self.phase4_last_candidate_dir = float(candidate["direction"])
            return True

        if state in ("observe", "track") or (
            previous and bool(previous.get("tiny"))
        ):
            self.phase4_motor_missed_frames = int(
                getattr(self, "phase4_motor_missed_frames", 0)
            ) + 1
            if (
                state == "drive"
                and self.phase4_motor_missed_frames > 1
            ):
                self.phase4_motor_track_signature = None
                self.phase4_motor_track_count = 0
        return False

    def _update_phase45_filtered_cone_dir(
        self,
        raw_cone_direction,
        cone_seen,
        cone_sequence=None,
        now=None,
    ):
        if not cone_seen:
            return getattr(self, "phase45_filtered_cone_dir", None)
        try:
            cdir = float(raw_cone_direction)
        except (TypeError, ValueError):
            return getattr(self, "phase45_filtered_cone_dir", None)
        cdir = max(0.0, min(1.0, cdir))
        try:
            sequence = int(cone_sequence) if cone_sequence is not None else None
        except (TypeError, ValueError):
            sequence = None
        update_time = time.time() if now is None else float(now)
        if (
            sequence is not None
            and sequence == getattr(self, "phase45_filtered_cone_seq", None)
        ):
            # The motor loop runs much faster than the camera. Reusing one
            # observation must not apply the EMA repeatedly and pull steering
            # toward the same (possibly edge-clipped) centroid.
            self.phase45_last_seen_time = update_time
            return getattr(self, "phase45_filtered_cone_dir", None)
        prev = getattr(self, "phase45_filtered_cone_dir", None)
        if prev is None:
            filtered = cdir
        else:
            alpha = PHASE45_CONE_DIR_FILTER_ALPHA
            filtered = (1.0 - alpha) * float(prev) + alpha * cdir
        self.phase45_filtered_cone_dir = filtered
        self.phase45_filtered_cone_seq = sequence
        self.phase45_last_seen_time = update_time
        return filtered

    @staticmethod
    def _phase45_cone_seen(snapshot, threshold):
        try:
            cone_probability = float(snapshot.get("cone_probability", 0.0))
        except (TypeError, ValueError):
            cone_probability = 0.0
        evidence = evaluate_cone_candidate(snapshot)
        return bool(
            evidence["close_reached"]
            or cone_probability > float(threshold)
            or evidence["weak"]
        )

    def _phase5_approach_speed(self, snapshot):
        """Select a grass-safe approach speed from normalized red occupancy."""
        debug = dict(snapshot.get("cone_debug", {}) or {})
        occupancy = max(
            self._snapshot_float(debug, "occupancy", 0.0),
            self._snapshot_float(debug, "frame_red_occupancy", 0.0),
        )
        self.phase5_last_proximity_occupancy = occupancy
        if occupancy >= float(PHASE5_NEAR_OCCUPANCY_THRESHOLD):
            return self._clamp_percent(PHASE5_NEAR_SPEED)
        if occupancy >= float(PHASE5_MID_OCCUPANCY_THRESHOLD):
            return self._clamp_percent(PHASE5_MID_SPEED)
        return self._clamp_percent(PHASE5_BASE_SPEED)

    def _drive_phase5_reacquire(self, now):
        """Continue a gentle corrective arc across a short detection gap."""
        last_seen = getattr(self, "phase45_last_seen_time", None)
        last_dir = getattr(self, "phase45_filtered_cone_dir", None)
        if last_seen is None or last_dir is None:
            self.stop_motors()
            return False
        if float(now) - float(last_seen) > float(PHASE5_REACQUIRE_GRACE_SEC):
            self.stop_motors()
            return False
        try:
            error = float(last_dir) - CONE_CENTER_POSITION
        except (TypeError, ValueError):
            self.stop_motors()
            return False
        if abs(error) <= float(PHASE5_STEER_DEADBAND):
            self.stop_motors()
            return False
        turn_side = "right" if error > 0.0 else "left"
        self._set_forward_pivot_turn(
            turn_side,
            PHASE5_REACQUIRE_OUTER_SPEED,
            cmd_type=f"phase5_reacquire_{turn_side}",
            speed_inner=PHASE5_REACQUIRE_INNER_SPEED,
            ramp_time=PHASE5_MOTOR_RAMP_TIME,
        )
        return True

    def _drive_phase4_camera(self, snapshot):
        """Apply one production Phase4 search/alignment motor decision."""
        now = time.time()
        if not self._camera_observation_fresh(snapshot, now):
            self.stop_motors()
            return
        if evaluate_cone_candidate(snapshot)["close_reached"]:
            self.stop_motors()
            return
        self._phase4_new_snapshot_candidate(snapshot, now)
        state = getattr(self, "phase4_search_state", "drive")
        candidate = self._phase4_motor_candidate(snapshot)
        if (
            candidate is not None
            and (not candidate["tiny"] or state == "track")
            and int(getattr(self, "phase4_motor_missed_frames", 0)) == 0
        ):
            # Fresh visual evidence owns the target even after capture timers
            # expire. Do not replace a visible cone with the search direction.
            self.phase4_last_candidate_dir = candidate["direction"]
            prefix = "phase4_candidate_observe" if state == "observe" else "phase4_candidate_capture"
            self._drive_phase4_candidate_capture(prefix)
            return
        capture_until = float(
            getattr(self, "phase4_candidate_active_until", now) or now
        )
        if state == "observe":
            observed_frames = int(getattr(self, "phase4_observe_frame_count", 0))
            if (
                now < capture_until
                and observed_frames < int(PHASE4_CANDIDATE_CAPTURE_MAX_FRAMES)
            ):
                self._drive_phase4_candidate_observe()
                return
            trusted = getattr(self, "phase4_motor_track_signature", None)
            if trusted and not bool(trusted.get("tiny")):
                self.phase4_last_candidate_dir = float(trusted["direction"])
                self.phase4_search_state = "reacquire"
                self.phase4_reacquire_until = now + float(PHASE4_REACQUIRE_TURN_SEC)
                if self._drive_phase4_toward_last_candidate():
                    return
            self.phase4_search_state = "drive"
        if state == "track":
            if now < capture_until and int(
                getattr(self, "phase4_motor_missed_frames", 0)
            ) <= 1 and self._drive_phase4_candidate_capture():
                return
            self.phase4_search_state = "reacquire"
            self.phase4_reacquire_until = now + float(PHASE4_REACQUIRE_TURN_SEC)
            state = "reacquire"
        if state == "reacquire":
            reacquire_until = getattr(self, "phase4_reacquire_until", now)
            if now < float(reacquire_until) and self._drive_phase4_toward_last_candidate():
                return
            self.phase4_search_state = "drive"

        # With no candidate, preserve the established counter-clockwise arc.
        self.phase4_search_state = "drive"
        self._set_forward_pivot_turn(
            "left",
            PHASE4_SEARCH_OUTER_SPEED,
            cmd_type="phase4_search_arc",
            speed_inner=PHASE4_SEARCH_INNER_SPEED,
            ramp_time=PHASE4_MOTOR_RAMP_TIME,
        )

    def _drive_phase5_camera(self, snapshot):
        """Apply one production Phase5 visual-approach motor decision."""
        now = time.time()
        if not self._camera_observation_fresh(snapshot, now):
            self.stop_motors()
            return
        close_reached = evaluate_cone_candidate(snapshot)["close_reached"]
        if close_reached and cone_centered_for_final_ram(snapshot):
            self.stop_motors()
            return
        cone_seen = self._phase45_cone_seen(
            snapshot,
            CONE_PROBABILITY_THRESHOLD_PHASE5,
        )
        cone_direction = snapshot.get("cone_direction", CONE_CENTER_POSITION)
        self._update_phase45_filtered_cone_dir(
            cone_direction,
            cone_seen,
            cone_sequence=snapshot.get("cone_sequence"),
            now=now,
        )
        if not cone_seen:
            self._drive_phase5_reacquire(now)
            return
        # Steering follows the current image; the filtered history is retained
        # only to recover from a brief missed detection.
        steer_direction = self._snapshot_float(snapshot, "cone_direction", CONE_CENTER_POSITION)
        error = steer_direction - CONE_CENTER_POSITION
        if self._drive_camera_edge_turn(error, "phase5_approach", PHASE5_MOTOR_RAMP_TIME):
            return
        approach_speed = self._phase5_approach_speed(snapshot)
        if abs(error) <= float(PHASE5_STEER_DEADBAND):
            self.set_motors(
                approach_speed,
                True,
                approach_speed,
                True,
                ramp_time=PHASE5_MOTOR_RAMP_TIME,
                cmd_type="phase5_approach_forward",
            )
            return

        turn_delta = min(
            float(PHASE5_TURN_CLAMP),
            abs(error) * float(APPROACH_TURN_GAIN),
        )
        inner_speed = self._clamp_percent(
            max(
                float(GRASS_MIN_MOTOR_SPEED),
                float(approach_speed) - turn_delta,
            )
        )
        outer_speed = self._clamp_percent(
            float(approach_speed) + turn_delta
        )
        if error > 0.0:
            speed_left, speed_right = outer_speed, inner_speed
            cmd_type = "phase5_approach_steer_right"
        else:
            speed_left, speed_right = inner_speed, outer_speed
            cmd_type = "phase5_approach_steer_left"
        self.set_motors(
            speed_left,
            True,
            speed_right,
            True,
            ramp_time=PHASE5_MOTOR_RAMP_TIME,
            cmd_type=cmd_type,
        )

    def _record_motor_command(self, cmd_type, motor1_speed, motor1_forward, motor2_speed, motor2_forward):
        self.last_motor_command = {
            "type": cmd_type,
            "updated_ms": int(time.time() * 1000),
            "motor1_speed": float(motor1_speed),
            "motor1_forward": int(bool(motor1_forward)),
            "motor2_speed": float(motor2_speed),
            "motor2_forward": int(bool(motor2_forward)),
        }

    def move_motor_thread(self):
        while not self._shutdown_active():
            try:
                snapshot = self.st.snapshot()
                phase = Phase(snapshot["phase"])
                obstacle_dist = snapshot["obstacle_dist"]
                direction = snapshot["direction"]
                cone_direction = snapshot["cone_direction"]
                last_phase = getattr(self, "_motor_last_phase", None)
                if last_phase != phase:
                    if phase not in (Phase.PHASE4, Phase.PHASE5) or last_phase not in (Phase.PHASE4, Phase.PHASE5):
                        self._reset_phase45_camera_track()
                    if phase == Phase.PHASE4:
                        self._reset_phase4_search_cycle()
                    self._motor_last_phase = phase

                if phase in PHASES_STOP_MOTORS:
                    self.stop_motors()
                    time.sleep(MOTOR_IDLE_SLEEP)
                    continue

                obstacle_detected = (
                    phase not in PHASES_SKIP_OBSTACLE
                    and bool(snapshot.get("obstacle_valid", False))
                    and obstacle_dist is not None
                    and 0 < obstacle_dist < OBSTACLE_AVOID_DIST
                )
                if obstacle_detected:
                    self.obstacle_detect_count += 1
                else:
                    self.obstacle_detect_count = 0

                if self.obstacle_detect_count >= OBSTACLE_CONFIRM_COUNT:
                    print(f"Obstacle Detected! {obstacle_dist:.1f}cm")
                    self.stop_motors()
                    time.sleep(OBSTACLE_PAUSE_TIME)
                    turn_fast = self._clamp_percent(OBSTACLE_SPEED)
                    turn_slow = self._clamp_percent(OBSTACLE_SPEED * 0.35)
                    self._set_forward_diff_turn("right", turn_fast, turn_slow, cmd_type="obstacle_forward_turn")
                    time.sleep(OBSTACLE_TURN_TIME)
                    self.stop_motors()
                    time.sleep(OBSTACLE_PAUSE_TIME)
                    self.obstacle_detect_count = 0
                    continue

                if phase == Phase.PHASE1 and direction == PARACHUTE_DIRECTION:
                    self.set_motors(
                        PARACHUTE_SEPARATION_SPEED,
                        True,
                        PARACHUTE_SEPARATION_SPEED,
                        True,
                        ramp_time=PHASE1_SOFTSTART_RAMP_TIME,
                        step_interval=PHASE1_SOFTSTART_STEP,
                        cmd_type="phase1_parachute_separation",
                    )
                    time.sleep(PARACHUTE_MOTOR_PULSE)
                    continue

                if phase == Phase.PHASE2:
                    if self.phase2_stage == PHASE2_STAGE_ESCAPE:
                        self.set_motors(
                            PHASE2_SPEED,
                            True,
                            PHASE2_SPEED,
                            True,
                            ramp_time=PHASE2_RAMP_TIME,
                            cmd_type="phase2_escape_forward",
                        )
                    elif self.phase2_stage == PHASE2_STAGE_OFFSET:
                        offset_mode = getattr(self, "phase2_offset_mode", "")
                        if offset_mode == PHASE2_OFFSET_MODE_REORIENT:
                            self._drive_phase2_forward_reorient()
                        elif offset_mode in (PHASE2_OFFSET_MODE_BNO_WAIT, PHASE2_OFFSET_MODE_READINESS):
                            self.stop_motors()
                        else:
                            hold = self._phase2_offset_hold_speeds(snapshot)
                            if hold is None:
                                self.stop_motors()
                            else:
                                speed_l, speed_r, heading_error = hold
                                cmd_type = (
                                    "phase2_offset_heading_hold_straight"
                                    if abs(heading_error) <= float(PHASE2_OFFSET_HOLD_DEADBAND_DEG)
                                    else "phase2_offset_heading_hold"
                                )
                                self.set_motors(
                                    speed_l,
                                    True,
                                    speed_r,
                                    True,
                                    ramp_time=PHASE2_RAMP_TIME,
                                    cmd_type=cmd_type,
                                )
                    elif self.phase2_stage == PHASE2_STAGE_CALIBRATION:
                        elapsed = 0.0
                        if self.phase2_stage_start is not None:
                            elapsed = time.time() - self.phase2_stage_start
                        speed_l, motor1_forward, speed_r, motor2_forward = (
                            self._phase2_calibration_pattern(elapsed)
                        )
                        self.set_motors(
                            speed_l,
                            motor1_forward,
                            speed_r,
                            motor2_forward,
                            ramp_time=PHASE2_RAMP_TIME,
                            cmd_type="phase2_calibration_forward_arc",
                        )
                    else:
                        self.stop_motors()
                    time.sleep(MOTOR_LOOP_INTERVAL)
                    continue

                if phase == Phase.PHASE3:
                    if not bool(getattr(self, "phase3_heading_entry_ready", False)):
                        self.stop_motors()
                        time.sleep(MOTOR_IDLE_SLEEP)
                        continue
                    self._drive_phase3_navigation(snapshot, direction)

                if phase == Phase.PHASE4:
                    self._drive_phase4_camera(snapshot)
                elif phase == Phase.PHASE5:
                    self._drive_phase5_camera(snapshot)

                if phase in (Phase.PHASE4, Phase.PHASE5):
                    time.sleep(PHASE45_MOTOR_LOOP_INTERVAL)
                elif phase == Phase.PHASE3:
                    time.sleep(PHASE3_MOTOR_LOOP_INTERVAL)
                else:
                    time.sleep(MOTOR_LOOP_INTERVAL)
            except Exception as e:
                print(f"Exception in move_motor_thread: {e}")
                try:
                    self.stop_motors()
                except Exception as stop_exc:
                    print(f"Emergency motor stop failed: {stop_exc}")
                time.sleep(MOTOR_LOOP_INTERVAL)
        self.stop_motors()

    def _ramp_pwm(self, pwm_dev, start_speed, target_speed, ramp_time, step_interval=MOTOR_RAMP_STEP):
        if self._shutdown_active():
            if pwm_dev is not None:
                pwm_dev.value = 0
            return 0.0
        if pwm_dev is None:
            return target_speed
        if ramp_time <= 0 or step_interval <= 0:
            pwm_dev.value = max(PWM_DUTY_MIN, min(PWM_DUTY_MAX, target_speed / PWM_PERCENT_MAX))
            return target_speed
        steps = max(1, int(ramp_time / step_interval))
        step_duration = ramp_time / steps
        for step in range(1, steps + 1):
            if self._shutdown_active():
                pwm_dev.value = 0
                return 0.0
            duty = start_speed + (target_speed - start_speed) * (step / steps)
            pwm_dev.value = max(PWM_DUTY_MIN, min(PWM_DUTY_MAX, duty / PWM_PERCENT_MAX))
            time.sleep(step_duration)
        return target_speed

    def _ramp_pwm_dual(
        self,
        pwm_a,
        start_a,
        target_a,
        pwm_b,
        start_b,
        target_b,
        ramp_time,
        step_interval=MOTOR_RAMP_STEP,
    ):
        if self._shutdown_active():
            if pwm_a is not None:
                pwm_a.value = 0
            if pwm_b is not None:
                pwm_b.value = 0
            return 0.0, 0.0
        if pwm_a is None and pwm_b is None:
            return start_a, start_b
        if ramp_time <= 0 or step_interval <= 0:
            if pwm_a is not None:
                pwm_a.value = max(PWM_DUTY_MIN, min(PWM_DUTY_MAX, target_a / PWM_PERCENT_MAX))
            if pwm_b is not None:
                pwm_b.value = max(PWM_DUTY_MIN, min(PWM_DUTY_MAX, target_b / PWM_PERCENT_MAX))
            return target_a, target_b
        steps = max(1, int(ramp_time / step_interval))
        step_duration = ramp_time / steps
        for step in range(1, steps + 1):
            if self._shutdown_active():
                if pwm_a is not None:
                    pwm_a.value = 0
                if pwm_b is not None:
                    pwm_b.value = 0
                return 0.0, 0.0
            duty_a = start_a + (target_a - start_a) * (step / steps)
            duty_b = start_b + (target_b - start_b) * (step / steps)
            if pwm_a is not None:
                pwm_a.value = max(PWM_DUTY_MIN, min(PWM_DUTY_MAX, duty_a / PWM_PERCENT_MAX))
            if pwm_b is not None:
                pwm_b.value = max(PWM_DUTY_MIN, min(PWM_DUTY_MAX, duty_b / PWM_PERCENT_MAX))
            time.sleep(step_duration)
        return target_a, target_b

    def set_motor(
        self,
        motor_pwm,
        motor_dir,
        speed,
        forward,
        invert=False,
        ramp_time=MOTOR_RAMP_TIME,
        step_interval=MOTOR_RAMP_STEP,
    ):
        if self._shutdown_active():
            self.stop_motors()
            return
        if motor_pwm is None or motor_dir is None:
            return
        state = self.motor_state.setdefault(motor_pwm, {"speed": 0.0, "direction": True})
        current_speed = state["speed"]
        current_direction = state["direction"]

        if current_speed > 0 and forward != current_direction:
            current_speed = self._ramp_pwm(
                motor_pwm,
                current_speed,
                0.0,
                ramp_time / RAMP_HALF_DIVISOR,
                step_interval,
            )

        motor_dir.value = forward_to_dir_value(forward, invert)
        target_speed = self._clamp_percent(speed)
        devices = getattr(self, "devices", {})
        if motor_pwm is devices.get(DEVICE_MOTOR_1_PWM):
            target_speed = self._apply_motor_speed_scale(target_speed, 1)
        elif motor_pwm is devices.get(DEVICE_MOTOR_2_PWM):
            target_speed = self._apply_motor_speed_scale(target_speed, 2)
        current_speed = self._ramp_pwm(motor_pwm, current_speed, target_speed, ramp_time, step_interval)
        state["speed"] = current_speed
        state["direction"] = forward

    def set_motors(
        self,
        speed_left,
        forward_left,
        speed_right,
        forward_right,
        ramp_time=MOTOR_RAMP_TIME,
        step_interval=MOTOR_RAMP_STEP,
        cmd_type="set_motors",
    ):
        if self._shutdown_active():
            self.stop_motors()
            return
        # Callers always use logical wheel order (left, right). The current
        # airframe wiring is physical MTR1=left and MTR2=right.
        (
            speed_motor_1,
            forward_motor_1,
            speed_motor_2,
            forward_motor_2,
        ) = map_logical_wheels_to_physical(
            speed_left,
            forward_left,
            speed_right,
            forward_right,
        )
        motor_1_pwm = self.devices.get(DEVICE_MOTOR_1_PWM)
        motor_1_dir = self.devices.get(DEVICE_MOTOR_1_DIR)
        motor_2_pwm = self.devices.get(DEVICE_MOTOR_2_PWM)
        motor_2_dir = self.devices.get(DEVICE_MOTOR_2_DIR)
        if motor_1_pwm is None or motor_1_dir is None or motor_2_pwm is None or motor_2_dir is None:
            return

        state_motor_1 = self.motor_state.setdefault(
            motor_1_pwm, {"speed": 0.0, "direction": True}
        )
        state_motor_2 = self.motor_state.setdefault(
            motor_2_pwm, {"speed": 0.0, "direction": True}
        )
        current_motor_1 = state_motor_1["speed"]
        current_motor_2 = state_motor_2["speed"]

        if (
            current_motor_1 > 0
            and forward_motor_1 != state_motor_1["direction"]
        ) or (
            current_motor_2 > 0
            and forward_motor_2 != state_motor_2["direction"]
        ):
            current_motor_1, current_motor_2 = self._ramp_pwm_dual(
                motor_1_pwm,
                current_motor_1,
                0.0,
                motor_2_pwm,
                current_motor_2,
                0.0,
                ramp_time / RAMP_HALF_DIVISOR,
                step_interval,
            )

        motor_1_dir.value = motor_forward_to_dir_value(1, forward_motor_1)
        motor_2_dir.value = motor_forward_to_dir_value(2, forward_motor_2)

        target_motor_1 = self._apply_motor_speed_scale(speed_motor_1, 1)
        target_motor_2 = self._apply_motor_speed_scale(speed_motor_2, 2)
        target_motor_1, target_motor_2 = apply_turn_speed_floor(
            target_motor_1,
            target_motor_2,
            turning=(
                min(speed_left, speed_right) > 0
                and speed_left != speed_right
                and bool(forward_left) == bool(forward_right)
            ),
        )
        current_motor_1, current_motor_2 = self._ramp_pwm_dual(
            motor_1_pwm,
            current_motor_1,
            target_motor_1,
            motor_2_pwm,
            current_motor_2,
            target_motor_2,
            ramp_time,
            step_interval,
        )

        state_motor_1["speed"] = current_motor_1
        state_motor_1["direction"] = forward_motor_1
        state_motor_2["speed"] = current_motor_2
        state_motor_2["direction"] = forward_motor_2
        self._record_motor_command(
            cmd_type,
            current_motor_1,
            forward_motor_1,
            current_motor_2,
            forward_motor_2,
        )

    def stop_motors(self):
        motor_1_pwm = self.devices.get(DEVICE_MOTOR_1_PWM)
        motor_2_pwm = self.devices.get(DEVICE_MOTOR_2_PWM)
        if motor_1_pwm:
            motor_1_pwm.value = 0
        if motor_2_pwm:
            motor_2_pwm.value = 0
        for state in self.motor_state.values():
            state["speed"] = 0.0
        self._record_motor_command("stop", 0.0, True, 0.0, True)
