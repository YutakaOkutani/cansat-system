import time
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
MAIN_PY_LIBRARY_DIR = PROJECT_ROOT / "lib"
if not MAIN_PY_LIBRARY_DIR.exists():
    raise FileNotFoundError(f"main.py library directory not found: {MAIN_PY_LIBRARY_DIR}")
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mission.const import (
    CAMERA_FRAME_STALE_STOP_SEC,
    CAMERA_HORIZONTAL_FOV_DEG,
    CAMERA_MIXED_CONFIRM_FRAMES,
    CAMERA_WEAK_CONFIRM_FRAMES,
    CAMERA_WEAK_MAX_MISSED_FRAMES,
    CAMERA_TINY_OCCUPANCY_THRESHOLD,
    CONE_CENTER_POSITION,
    CONE_PHASE4_BBOX_CENTER_Y_TOLERANCE,
    CONE_PHASE4_BBOX_SIZE_RATIO_MAX,
    CONE_PHASE4_BBOX_VERTICAL_OVERLAP_MIN,
    CONE_PHASE4_BEARING_CONSISTENCY_TOLERANCE_DEG,
    CONE_PHASE4_CENTER_TOLERANCE,
    CONE_PHASE4_DIRECTION_CONSISTENCY_TOLERANCE,
    CONE_PROBABILITY_THRESHOLD,
    CONE_PROBABILITY_THRESHOLD_PHASE4,
    DEVICE_LED_GREEN,
    DEVICE_LED_RED,
    GPS_ACTIVE_DETECT,
    GPS_PHASE45_MAX_DISTANCE,
    PHASE4_CANDIDATE_CAPTURE_SEC,
    Phase,
    TIMEOUT_PHASE_4,
)
from mission.cone_candidate import camera_has_visible_cone, evaluate_cone_candidate
from mission.nav import calc_distance_and_azimuth
from mission.phases.base import BasePhaseHandler


class Phase4Handler(BasePhaseHandler):
    @staticmethod
    def _float_value(value, default=0.0):
        try:
            return float(value)
        except (TypeError, ValueError):
            return float(default)

    def _fresh_camera_frame(self, snapshot, now):
        sequence = int(self._float_value(snapshot.get("cone_sequence", 0), 0.0))
        updated_at = self._float_value(snapshot.get("cone_updated_at", 0.0), 0.0)
        age = float("inf") if updated_at <= 0.0 else max(0.0, now - updated_at)
        return (
            bool(snapshot.get("cone_valid", False))
            and sequence > 0
            and age <= float(CAMERA_FRAME_STALE_STOP_SEC)
        ), sequence, age

    @staticmethod
    def _angle_diff_deg(target, current):
        return (float(target) - float(current) + 180.0) % 360.0 - 180.0

    def _candidate_signature(self, snapshot, cone_dir):
        debug = dict(snapshot.get("cone_debug", {}) or {})
        heading = None
        bearing = None
        bbox_x = self._float_value(debug.get("bbox_x", 0.0))
        bbox_y = self._float_value(debug.get("bbox_y", 0.0))
        bbox_w = self._float_value(debug.get("bbox_width", 0.0))
        bbox_h = self._float_value(debug.get("bbox_height", 0.0))
        reported_height_frac = self._float_value(
            debug.get("bbox_height_frac", 0.0)
        )
        reported_bottom_frac = self._float_value(
            debug.get("bbox_bottom_frac", 0.0)
        )
        reported_width_frac = self._float_value(
            debug.get("bbox_width_frac", 0.0)
        )
        bbox_valid = (
            bbox_w > 0.0
            and bbox_h > 0.0
            and (reported_height_frac > 0.0 or reported_bottom_frac > 0.0)
        )
        if bbox_valid:
            if reported_height_frac > 0.0:
                frame_height = bbox_h / reported_height_frac
            else:
                frame_height = (bbox_y + bbox_h) / reported_bottom_frac
            center_y = (bbox_y + 0.5 * bbox_h) / max(frame_height, 1.0)
            height_frac = bbox_h / max(frame_height, 1.0)
            bottom_frac = (bbox_y + bbox_h) / max(frame_height, 1.0)
        else:
            center_y = 0.0
            height_frac = 0.0
            bottom_frac = 0.0

        # Bearing consistency uses the physical image X axis.  Motor control
        # direction is kept separately so mount compensation cannot leak into
        # visual tracking geometry.
        image_direction = None
        snapshot_image_direction = snapshot.get("cone_image_direction")
        if (
            snapshot_image_direction is not None
            and bool(snapshot.get("cone_image_direction_valid", False))
        ):
            image_direction = self._float_value(
                snapshot_image_direction,
                CONE_CENTER_POSITION,
            )
        elif bbox_valid and reported_width_frac > 0.0:
            frame_width = bbox_w / reported_width_frac
            image_direction = (bbox_x + 0.5 * bbox_w) / max(frame_width, 1.0)
        elif bool(int(self._float_value(debug.get("image_direction_valid", 0.0)))):
            image_direction = self._float_value(
                debug.get("image_direction", CONE_CENTER_POSITION),
                CONE_CENTER_POSITION,
            )
        else:
            # Backward-compatible fallback: the control and image conventions
            # are now identical unless explicit mount inversion is configured.
            image_direction = float(cone_dir)
        image_direction = max(0.0, min(1.0, float(image_direction)))

        if bool(snapshot.get("angle_valid", False)):
            try:
                heading = float(snapshot.get("angle")) % 360.0
                bearing = (
                    heading
                    + (image_direction - CONE_CENTER_POSITION)
                    * float(CAMERA_HORIZONTAL_FOV_DEG)
                ) % 360.0
            except (TypeError, ValueError):
                heading = None
                bearing = None

        return {
            "direction": float(cone_dir),
            "image_direction": image_direction,
            "heading": heading,
            "bearing": bearing,
            "bbox_valid": bbox_valid,
            "bbox_x": bbox_x,
            "bbox_y": bbox_y,
            "bbox_width": bbox_w,
            "bbox_height": bbox_h,
            "center_y": center_y,
            "height_frac": height_frac,
            "bottom_frac": bottom_frac,
            "occupancy": self._float_value(debug.get("occupancy", 0.0)),
        }

    def _candidate_consistency(self, previous, current):
        if not previous:
            return True, "new"

        previous_bearing = previous.get("bearing")
        current_bearing = current.get("bearing")
        if previous_bearing is not None and current_bearing is not None:
            bearing_error = abs(
                self._angle_diff_deg(current_bearing, previous_bearing)
            )
            if bearing_error > float(
                CONE_PHASE4_BEARING_CONSISTENCY_TOLERANCE_DEG
            ):
                return False, "bearing"
        elif abs(
            float(current["direction"]) - float(previous.get("direction", 0.5))
        ) > float(CONE_PHASE4_DIRECTION_CONSISTENCY_TOLERANCE):
            return False, "direction"

        if bool(previous.get("bbox_valid")) and bool(current.get("bbox_valid")):
            if abs(
                float(current["center_y"]) - float(previous.get("center_y", 0.0))
            ) > float(CONE_PHASE4_BBOX_CENTER_Y_TOLERANCE):
                return False, "bbox_y"

            previous_h = max(float(previous.get("bbox_height", 0.0)), 1.0)
            current_h = max(float(current.get("bbox_height", 0.0)), 1.0)
            previous_w = max(float(previous.get("bbox_width", 0.0)), 1.0)
            current_w = max(float(current.get("bbox_width", 0.0)), 1.0)
            size_ratio = max(
                previous_h / current_h,
                current_h / previous_h,
                previous_w / current_w,
                current_w / previous_w,
            )
            if size_ratio > float(CONE_PHASE4_BBOX_SIZE_RATIO_MAX):
                return False, "bbox_size"

            previous_top = float(previous.get("center_y", 0.0)) - 0.5 * float(
                previous.get("height_frac", 0.0)
            )
            previous_bottom = float(previous.get("bottom_frac", 0.0))
            current_top = float(current.get("center_y", 0.0)) - 0.5 * float(
                current.get("height_frac", 0.0)
            )
            current_bottom = float(current.get("bottom_frac", 0.0))
            overlap = max(
                0.0,
                min(previous_bottom, current_bottom)
                - max(previous_top, current_top),
            )
            union = max(previous_bottom, current_bottom) - min(
                previous_top,
                current_top,
            )
            vertical_overlap = overlap / max(union, 1e-6)
            if vertical_overlap < float(CONE_PHASE4_BBOX_VERTICAL_OVERLAP_MIN):
                return False, "bbox_overlap"

        return True, "consistent"

    @staticmethod
    def _reset_strong_confirmation(controller):
        controller.phase4_detect_confirm_count = 0
        controller.phase4_detect_confirm_marker = None
        controller.phase4_detect_track_signature = None
        controller.phase4_track_has_strict = False
        controller.phase4_candidate_missed_frames = 0

    @staticmethod
    def _reset_weak_confirmation(controller):
        controller.phase4_weak_confirm_count = 0
        controller.phase4_weak_confirm_marker = None
        controller.phase4_weak_missed_frames = 0
        controller.phase4_weak_track_signature = None

    @staticmethod
    def _arm_candidate_capture(controller, now, cone_dir):
        # The motor thread owns observe/track transitions.  Phase logic only
        # publishes the latest candidate and a hard observation deadline.
        controller.phase4_phase_candidate_dir = float(cone_dir)
        capture_sec = float(getattr(controller, "phase4_candidate_capture_sec", 0.0))
        if capture_sec <= 0.0:
            capture_sec = float(PHASE4_CANDIDATE_CAPTURE_SEC)
        active_until = float(
            getattr(controller, "phase4_candidate_active_until", 0.0) or 0.0
        )
        if active_until <= float(now):
            controller.phase4_candidate_active_until = float(now) + capture_sec

    def _fallback_to_p3(self, controller, current_snapshot, reason):
        fallback_dir = current_snapshot["angle"] if current_snapshot["angle_valid"] else current_snapshot["direction"]
        print(reason)
        controller.cone_phase_decision = "p4_fallback_to_p3"
        controller.st.update_navigation(direction=fallback_dir, phase=int(Phase.PHASE3))
        controller.searching_flag = False
        self._reset_strong_confirmation(controller)
        self._reset_weak_confirmation(controller)
        controller.phase4_candidate_active_until = 0.0
        controller.phase4_search_state = "drive"
        controller.time_phase3_start = time.time()

    def execute(self, controller, snapshot):
        led_red = controller.devices.get(DEVICE_LED_RED)
        led_green = controller.devices.get(DEVICE_LED_GREEN)
        entry_marker = getattr(controller, "phase_entry_time", None)
        if (
            entry_marker is not None
            and getattr(controller, "phase4_camera_recovery_marker", None) != entry_marker
        ):
            controller.phase4_camera_recovery_marker = entry_marker
            controller.phase4_last_processed_cone_seq = 0
            self._reset_strong_confirmation(controller)
            self._reset_weak_confirmation(controller)
            controller.phase4_candidate_active_until = 0.0
            controller.phase4_search_state = "drive"
            recovery_started = getattr(controller, "camera_recovery_started_at", None)
            # Preserve a campaign started by the camera thread just after this
            # P4 entry, but discard exhausted/stale state inherited from P3/P5.
            if recovery_started is None or float(recovery_started) < float(entry_marker):
                controller.reset_camera_recovery_window()
        current_snapshot = controller.st.snapshot()
        visible_cone = camera_has_visible_cone(current_snapshot, time.time())
        cone_prob = current_snapshot["cone_probability"]
        cone_dir = current_snapshot.get("cone_direction", CONE_CENTER_POSITION)
        evidence = evaluate_cone_candidate(current_snapshot)
        now = time.time()
        camera_fresh, cone_sequence, _camera_age = self._fresh_camera_frame(current_snapshot, now)
        cone_reached_effective = bool(
            camera_fresh and evidence["close_reached"]
        )
        controller.cone_phase_decision = "p4_precheck"
        controller.cone_phase_threshold = float(CONE_PROBABILITY_THRESHOLD_PHASE4)
        controller.cone_phase_reached_probability_threshold = 0.0
        controller.cone_phase_center_tolerance = float(CONE_PHASE4_CENTER_TOLERANCE)
        controller.cone_phase_direction_tolerance = float(
            CONE_PHASE4_DIRECTION_CONSISTENCY_TOLERANCE
        )
        controller.cone_phase_required_confirm_frames = int(CAMERA_MIXED_CONFIRM_FRAMES)
        controller.cone_phase_detected = False
        controller.cone_phase_reached_effective = bool(cone_reached_effective)
        controller.cone_phase_centered = False
        controller.cone_phase_direction_consistent = False
        controller.cone_phase_confirm_count = int(getattr(controller, "phase4_detect_confirm_count", 0))
        print("p4 : camera searching")
        if led_red:
            led_red.off()
        if led_green:
            led_green.on()
        if (
            hasattr(controller, "target_lat")
            and hasattr(controller, "target_lng")
            and current_snapshot.get("gps_detect") == GPS_ACTIVE_DETECT
        ):
            dist_m, azimuth = calc_distance_and_azimuth(
                current_snapshot["lat"],
                current_snapshot["lng"],
                controller.target_lat,
                controller.target_lng,
            )
            controller.st.update_navigation(distance=dist_m, azimuth=azimuth)
            if (
                dist_m > GPS_PHASE45_MAX_DISTANCE
                and not getattr(controller, "phase3_arrived_latched", False)
                and not visible_cone
            ):
                self._fallback_to_p3(
                    controller,
                    current_snapshot,
                    f"Phase4 GPS fallback: target is {dist_m:.1f}m away (> {GPS_PHASE45_MAX_DISTANCE:.1f}m)",
                )
                return
        if not controller.searching_flag:
            controller.searching_flag = True
            controller.time_start_searching_cone = now
            controller.camera_phase4_attempts += 1
            controller.camera_phase4_start = controller.time_start_searching_cone
        else:
            if now - controller.time_start_searching_cone >= TIMEOUT_PHASE_4 and camera_fresh and not visible_cone:
                print("Phase4 TIMEOUT: stopping camera phases and giving up")
                controller.cone_phase_decision = "p4_timeout_to_p7_give_up"
                controller.searching_flag = False
                controller.transition_to_give_up("PHASE4_TIMEOUT_GIVE_UP")
                return
        if not camera_fresh:
            controller.cone_phase_decision = (
                "p4_wait_first_camera_frame"
                if cone_sequence <= 0
                else "p4_camera_stale_wait"
            )
            return
        if cone_sequence == int(getattr(controller, "phase4_last_processed_cone_seq", 0)):
            if int(getattr(controller, "phase4_detect_confirm_count", 0)) > 0:
                controller.cone_phase_required_confirm_frames = int(
                    CAMERA_MIXED_CONFIRM_FRAMES
                    if bool(getattr(controller, "phase4_track_has_strict", False))
                    else CAMERA_WEAK_CONFIRM_FRAMES
                )
            controller.cone_phase_decision = "p4_wait_new_frame"
            return
        controller.phase4_last_processed_cone_seq = cone_sequence
        if cone_reached_effective:
            controller.cone_phase_detected = True
            controller.cone_phase_reached_effective = True
            controller.cone_phase_centered = True
            controller.cone_phase_direction_consistent = True
            controller.cone_phase_confirm_count = 1
            controller.cone_phase_required_confirm_frames = 1
            controller.cone_phase_decision = "p4_close_reached_to_p5"
            print("Phase4 -> Phase5: close_reached_ok detected")
            controller.searching_flag = False
            self._reset_strong_confirmation(controller)
            self._reset_weak_confirmation(controller)
            controller.phase4_candidate_active_until = 0.0
            controller.phase5_last_processed_cone_seq = 0
            controller.phase5_entry_reason = "phase4_close_reached"
            controller.st.update_navigation(phase=int(Phase.PHASE5))
            return

        # Phase4開始時は比較的近距離(数m)のため、誤検知低減のために
        # Phase5より厳しめのprobability + 数フレーム連続確認を要求する。
        try:
            cone_dir_val = float(cone_dir)
        except (TypeError, ValueError):
            cone_dir_val = CONE_CENTER_POSITION
        centered = abs(cone_dir_val - CONE_CENTER_POSITION) <= CONE_PHASE4_CENTER_TOLERANCE
        strict_detect = bool(evidence["strict"])
        loose_detect = cone_prob > CONE_PROBABILITY_THRESHOLD
        weak_detect = bool(evidence["weak"])
        signature = self._candidate_signature(current_snapshot, cone_dir_val)
        candidate_detect = bool(cone_reached_effective or strict_detect or weak_detect)
        consistent, consistency_reason = self._candidate_consistency(
            getattr(controller, "phase4_detect_track_signature", None),
            signature,
        )

        controller.cone_phase_detected = candidate_detect
        controller.cone_phase_centered = bool(centered)
        controller.cone_phase_direction_consistent = bool(candidate_detect and consistent)

        if candidate_detect:
            current_is_strict = bool(cone_reached_effective or strict_detect)
            previous_marker = getattr(controller, "phase4_detect_confirm_marker", None)
            previous_signature = getattr(
                controller,
                "phase4_detect_track_signature",
                None,
            )
            preserve_large_track = bool(
                not consistent
                and previous_signature
                and float(previous_signature.get("occupancy", 0.0))
                >= float(CAMERA_TINY_OCCUPANCY_THRESHOLD)
                and 0.0 < float(signature.get("occupancy", 0.0))
                < float(CAMERA_TINY_OCCUPANCY_THRESHOLD)
            )
            if preserve_large_track:
                controller.phase4_candidate_missed_frames = int(
                    getattr(controller, "phase4_candidate_missed_frames", 0)
                ) + 1
                controller.cone_phase_decision = (
                    f"p4_track_hold_{consistency_reason}"
                )
                if (
                    controller.phase4_candidate_missed_frames
                    > CAMERA_WEAK_MAX_MISSED_FRAMES
                ):
                    self._reset_strong_confirmation(controller)
                candidate_detect = False
                controller.cone_phase_detected = False
            elif consistent:
                controller.phase4_detect_confirm_count = int(
                    getattr(controller, "phase4_detect_confirm_count", 0)
                ) + 1
                controller.phase4_track_has_strict = bool(
                    getattr(controller, "phase4_track_has_strict", False)
                    or current_is_strict
                )
                if previous_marker is None:
                    controller.phase4_detect_confirm_marker = cone_dir_val
                else:
                    controller.phase4_detect_confirm_marker = (
                        0.7 * self._float_value(previous_marker, cone_dir_val)
                        + 0.3 * cone_dir_val
                    )
                if cone_reached_effective:
                    controller.cone_phase_decision = "p4_reached_confirm"
                elif strict_detect:
                    controller.cone_phase_decision = "p4_strict_confirm"
                else:
                    controller.cone_phase_decision = "p4_weak_confirm"
            else:
                controller.phase4_detect_confirm_count = 1
                controller.phase4_track_has_strict = current_is_strict
                controller.phase4_detect_confirm_marker = cone_dir_val
                prefix = "p4_track_reset" if current_is_strict else "p4_weak_track_reset"
                controller.cone_phase_decision = f"{prefix}_{consistency_reason}"
            if not preserve_large_track:
                controller.phase4_detect_track_signature = signature
                controller.phase4_candidate_missed_frames = 0
                self._arm_candidate_capture(controller, now, cone_dir_val)
        else:
            controller.cone_phase_decision = (
                "p4_low_confidence_hold" if loose_detect else "p4_no_detection"
            )
            if int(getattr(controller, "phase4_detect_confirm_count", 0)) > 0:
                controller.phase4_candidate_missed_frames = int(
                    getattr(controller, "phase4_candidate_missed_frames", 0)
                ) + 1
                if (
                    controller.phase4_candidate_missed_frames
                    > CAMERA_WEAK_MAX_MISSED_FRAMES
                ):
                    self._reset_strong_confirmation(controller)

        has_strict = bool(getattr(controller, "phase4_track_has_strict", False))
        required_frames = (
            CAMERA_MIXED_CONFIRM_FRAMES if has_strict else CAMERA_WEAK_CONFIRM_FRAMES
        )
        controller.cone_phase_confirm_count = int(
            getattr(controller, "phase4_detect_confirm_count", 0)
        )
        controller.cone_phase_required_confirm_frames = int(required_frames)
        # Legacy fields remain mirrored for existing logs and external tooling.
        controller.phase4_weak_confirm_count = (
            controller.cone_phase_confirm_count if not has_strict else 0
        )
        controller.phase4_weak_missed_frames = int(
            getattr(controller, "phase4_candidate_missed_frames", 0)
        )

        confirmed = controller.cone_phase_confirm_count >= required_frames
        if confirmed:
            if cone_reached_effective:
                controller.cone_phase_decision = "p4_reached_to_p5"
                print("Phase4 -> Phase5: close-range visual reached detected")
            elif not has_strict:
                controller.cone_phase_decision = "p4_weak_confirmed_to_p5"
                print(
                    "Phase4 -> Phase5: temporally stable weak cone candidate "
                    f"(confirm={controller.cone_phase_confirm_count})"
                )
            else:
                controller.cone_phase_decision = "p4_confirmed_to_p5"
                print(
                    "Phase4 -> Phase5: cone detected "
                    f"(prob={cone_prob:.2f}, confirm={controller.phase4_detect_confirm_count})"
                )
            controller.searching_flag = False
            self._reset_strong_confirmation(controller)
            self._reset_weak_confirmation(controller)
            controller.phase4_candidate_active_until = 0.0
            controller.phase5_last_processed_cone_seq = 0
            controller.phase5_entry_reason = "phase4_detected"
            controller.st.update_navigation(phase=int(Phase.PHASE5))
            return

        if loose_detect and not candidate_detect and controller.led_blink_timer % 10 == 0:
            print(
                f"Phase4: low-confidence cone ignored (prob={cone_prob:.2f}, dir={cone_dir_val:.2f})"
            )


def run_standalone():
    from mission.run import run_single_phase

    run_single_phase(Phase.PHASE4)


if __name__ == "__main__":
    run_standalone()
