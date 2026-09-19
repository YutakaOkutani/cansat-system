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
    CAMERA_DEAD_TIMEOUT,
    CAMERA_FRAME_STALE_STOP_SEC,
    CAMERA_PHASE5_MAX_ATTEMPTS,
    CONE_PHASE5_REACH_CONFIRM_FRAMES,
    CONE_LOST_COUNT_LIMIT,
    CONE_PROBABILITY_THRESHOLD,
    CONE_PROBABILITY_THRESHOLD_PHASE5,
    DEVICE_LED_GREEN,
    DEVICE_LED_RED,
    GPS_ACTIVE_DETECT,
    GPS_PHASE45_MAX_DISTANCE,
    LED_INTERVAL_PHASE5,
    Phase,
    PHASE5_RAM_CENTER_TOLERANCE,
    TIMEOUT_PHASE_5,
)
from mission.cone_candidate import camera_has_visible_cone, cone_centered_for_final_ram, evaluate_cone_candidate
from mission.nav import calc_distance_and_azimuth
from mission.phases.base import BasePhaseHandler

class Phase5Handler(BasePhaseHandler):
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
        ), sequence

    def _fallback_to_p3(self, controller, current_snapshot, reason):
        fallback_dir = current_snapshot["angle"] if current_snapshot["angle_valid"] else current_snapshot["direction"]
        print(reason)
        controller.cone_phase_decision = "p5_fallback_to_p3"
        controller.st.update_navigation(direction=fallback_dir, phase=int(Phase.PHASE3))
        controller.time_phase3_start = time.time()

    def execute(self, controller, snapshot):
        led_red = controller.devices.get(DEVICE_LED_RED)
        led_green = controller.devices.get(DEVICE_LED_GREEN)
        entry_marker = getattr(controller, "phase_entry_time", None)
        need_phase5_init = False
        if entry_marker is not None:
            need_phase5_init = getattr(controller, "phase5_entry_marker", None) != entry_marker
        else:
            need_phase5_init = getattr(controller, "time_camera_start", 0.0) <= 0.0
        if need_phase5_init:
            entry_reason = str(getattr(controller, "phase5_entry_reason", "unknown"))
            timeout_limit = TIMEOUT_PHASE_5
            print(f"p5 : approaching ({entry_reason}, timeout={timeout_limit:.1f}s)")
            controller.phase5_entry_marker = entry_marker
            controller.phase5_timeout_limit_sec = float(timeout_limit)
            controller.time_camera_start = time.time()
            controller.count_cone_lost = 0
            controller.phase5_reach_confirm_count = 0
            controller.phase5_last_processed_cone_seq = 0
            controller.camera_phase5_attempts += 1
            controller.camera_phase5_start = controller.time_camera_start
        controller.cone_phase_decision = "p5_precheck"
        controller.cone_phase_threshold = float(CONE_PROBABILITY_THRESHOLD_PHASE5)
        controller.cone_phase_reached_probability_threshold = 0.0
        controller.cone_phase_center_tolerance = float(PHASE5_RAM_CENTER_TOLERANCE)
        controller.cone_phase_direction_tolerance = 0.0
        controller.cone_phase_required_confirm_frames = int(CONE_PHASE5_REACH_CONFIRM_FRAMES)
        controller.cone_phase_detected = False
        controller.cone_phase_reached_effective = False
        controller.cone_phase_centered = False
        controller.cone_phase_direction_consistent = False
        controller.cone_phase_confirm_count = int(getattr(controller, "phase5_reach_confirm_count", 0))
        timeout_limit = float(getattr(controller, "phase5_timeout_limit_sec", TIMEOUT_PHASE_5))

        controller.led_blink_timer += 1
        if (controller.led_blink_timer // LED_INTERVAL_PHASE5) % 2 == 0:
            if led_red:
                led_red.on()
            if led_green:
                led_green.off()
        else:
            if led_red:
                led_red.off()
            if led_green:
                led_green.on()

        current_snapshot = controller.st.snapshot()
        visible_cone = camera_has_visible_cone(current_snapshot, time.time())
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
                    f"Phase5 GPS fallback: target is {dist_m:.1f}m away (> {GPS_PHASE45_MAX_DISTANCE:.1f}m)",
                )
                return
        # close_reached_ok は専用の近距離品質ゲートを通過済みなので、
        # 一般候補の probability では再度ゲートしない。
        cone_prob = current_snapshot["cone_probability"]
        now = time.time()
        camera_fresh, cone_sequence = self._fresh_camera_frame(current_snapshot, now)
        evidence = evaluate_cone_candidate(current_snapshot)
        centered = cone_centered_for_final_ram(current_snapshot)
        controller.cone_phase_centered = bool(camera_fresh and centered)
        is_reach_effective = bool(
            camera_fresh and evidence["close_reached"] and centered
        )
        weak_detect = bool(camera_fresh and evidence["weak"])
        is_det = (
            camera_fresh
            and (
                cone_prob > CONE_PROBABILITY_THRESHOLD_PHASE5
                or weak_detect
                or evidence["close_reached"]
            )
        )
        controller.cone_phase_detected = bool(is_det)
        controller.cone_phase_reached_effective = bool(is_reach_effective)
        camera_dead = (
            controller.camera_dead_since is not None
            and now - controller.camera_dead_since >= CAMERA_DEAD_TIMEOUT
        )
        if camera_dead and not visible_cone and (
            controller.camera_phase5_attempts >= CAMERA_PHASE5_MAX_ATTEMPTS
            or (controller.camera_phase5_start is not None and now - controller.camera_phase5_start >= timeout_limit)
        ):
            self._fallback_to_p3(controller, current_snapshot, "Camera DEAD: Fallback to Phase3 (GPS/Straight)")
            return
        if not camera_fresh:
            controller.cone_phase_detected = False
            controller.cone_phase_reached_effective = False
            controller.cone_phase_decision = (
                "p5_wait_first_camera_frame"
                if cone_sequence <= 0
                else "p5_camera_stale_wait"
            )
            return
        if cone_sequence == int(getattr(controller, "phase5_last_processed_cone_seq", 0)):
            controller.cone_phase_decision = "p5_wait_new_frame"
            return
        controller.phase5_last_processed_cone_seq = cone_sequence

        if not is_det:
            controller.cone_phase_decision = "p5_cone_lost_counting"
            controller.count_cone_lost += 1
            controller.phase5_reach_confirm_count = 0
            if cone_prob > CONE_PROBABILITY_THRESHOLD and controller.led_blink_timer % 10 == 0:
                print(f"Phase5: weak visual ignored (prob={cone_prob:.2f})")
        else:
            controller.cone_phase_decision = (
                "p5_tracking_weak" if weak_detect else "p5_tracking"
            )
            if evidence["close_reached"] and not centered:
                controller.cone_phase_decision = "p5_align_before_ram"
            controller.count_cone_lost = 0

        if controller.count_cone_lost >= CONE_LOST_COUNT_LIMIT:
            controller.cone_phase_decision = "p5_cone_lost_to_p4"
            print("Phase5 -> Phase4: cone lost")
            controller.phase4_last_processed_cone_seq = 0
            controller.st.update_navigation(phase=int(Phase.PHASE4))
            return

        if is_reach_effective:
            controller.phase5_reach_confirm_count = getattr(controller, "phase5_reach_confirm_count", 0) + 1
            controller.cone_phase_confirm_count = int(controller.phase5_reach_confirm_count)
            controller.cone_phase_decision = "p5_reached_confirm"
            if controller.phase5_reach_confirm_count < CONE_PHASE5_REACH_CONFIRM_FRAMES:
                return
            controller.cone_phase_decision = "p5_reached_to_p6"
            print(
                f"Reached Cone! (Visual confirmation x{controller.phase5_reach_confirm_count})"
            )
            controller.mission_end_reason = "GOAL_REACHED"
            controller.st.update_navigation(phase=int(Phase.PHASE6))
            return
        else:
            controller.phase5_reach_confirm_count = 0
            controller.cone_phase_confirm_count = 0

        if now - controller.time_camera_start >= timeout_limit and not visible_cone:
            controller.cone_phase_decision = "p5_timeout_to_p7_give_up"
            controller.transition_to_give_up("PHASE5_VISUAL_LOST_TIMEOUT")
            return



def run_standalone():
    from mission.run import run_single_phase

    run_single_phase(Phase.PHASE5)


if __name__ == "__main__":
    run_standalone()
