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
    GOAL_MAX_DISTANCE_SPREAD_CM,
    GOAL_OBSERVATION_TIMEOUT_SEC,
    CAMERA_FRAME_STALE_STOP_SEC,
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
    GOAL_CENTER_TOLERANCE,
    TIMEOUT_PHASE_5,
)
from mission.cone_candidate import camera_has_visible_cone, cone_centered_for_final_approach, evaluate_cone_candidate
from mission.goal import final_entry_evidence
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
            controller.phase5_last_sonar_seq = 0
            controller.phase5_goal_distance = None
            controller.phase5_goal_wait_since = None
            controller.phase5_last_processed_cone_seq = 0
            controller.camera_phase5_attempts += 1
            controller.camera_phase5_start = controller.time_camera_start
        controller.cone_phase_decision = "p5_precheck"
        controller.cone_phase_threshold = float(CONE_PROBABILITY_THRESHOLD_PHASE5)
        controller.cone_phase_reached_probability_threshold = 0.0
        controller.cone_phase_center_tolerance = float(GOAL_CENTER_TOLERANCE)
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
        close_hold = current_snapshot.get('cone_close_track', {}).get('hold', False)
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
                and not close_hold
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
        centered = cone_centered_for_final_approach(current_snapshot)
        controller.cone_phase_centered = bool(camera_fresh and centered)
        is_reach_effective, goal_reason, goal_distance = final_entry_evidence(
            current_snapshot, now, time.monotonic()
        )
        controller.goal_decision = goal_reason
        if goal_reason == 'close_track_camera_sonar_matched':
            controller.cone_phase_centered = True
        controller.goal_confirm_count = int(getattr(controller, "phase5_reach_confirm_count", 0))
        if close_hold and not is_reach_effective:
            controller.stop_motors()
            controller.count_cone_lost = 0
            controller.cone_phase_decision = 'p5_close_track_observe'
            # P6 owns the finite stopped recovery window; never resume search
            # merely because a close cone has become clipped or ambiguous.
            controller.phase6_motion_until = 0.0
            controller.st.update_navigation(phase=int(Phase.PHASE6))
            return
        # Transfer a visual-close range failure to the bounded P6 recovery window.
        # The motor interlock holds position when the cone is visually close.
        if camera_fresh and evidence["close_reached"] and not is_reach_effective:
            if getattr(controller, "phase5_goal_wait_since", None) is None:
                controller.phase5_goal_wait_since = time.monotonic()
            if time.monotonic() - controller.phase5_goal_wait_since >= GOAL_OBSERVATION_TIMEOUT_SEC:
                controller.goal_decision = "bounded_range_recovery"
                controller.phase6_motion_until = 0.0
                controller.stop_motors()
                controller.st.update_navigation(phase=int(Phase.PHASE6))
                return
        else:
            controller.phase5_goal_wait_since = None
        waiting_for_image = (
            goal_reason == "observations_not_aligned"
            and cone_sequence == int(getattr(controller, "phase5_last_processed_cone_seq", 0))
        )
        if not is_reach_effective and not waiting_for_image:
            controller.phase5_reach_confirm_count = 0
            controller.phase5_goal_distance = None
            controller.goal_confirm_count = 0
        weak_detect = bool(camera_fresh and evidence["weak"])
        is_det = (
            camera_fresh
            and (
                cone_prob > CONE_PROBABILITY_THRESHOLD_PHASE5
                or weak_detect
                or evidence["close_reached"]
                or (close_hold and is_reach_effective)
            )
        )
        controller.cone_phase_detected = bool(is_det)
        controller.cone_phase_reached_effective = bool(is_reach_effective)
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
                controller.cone_phase_decision = "p5_align_before_approach"
            controller.count_cone_lost = 0

        if controller.count_cone_lost >= CONE_LOST_COUNT_LIMIT:
            controller.cone_phase_decision = "p5_cone_lost_to_p4"
            print("Phase5 -> Phase4: cone lost")
            controller.phase4_last_processed_cone_seq = 0
            controller.st.update_navigation(phase=int(Phase.PHASE4))
            return

        if is_reach_effective:
            sonar_sequence = int(current_snapshot["sonar_sequence"])
            if sonar_sequence == getattr(controller, "phase5_last_sonar_seq", 0):
                controller.goal_decision = "wait_new_sonar"
                return
            controller.phase5_last_sonar_seq = sonar_sequence
            previous_distance = getattr(controller, "phase5_goal_distance", None)
            if previous_distance is not None and abs(goal_distance - previous_distance) > GOAL_MAX_DISTANCE_SPREAD_CM:
                controller.phase5_reach_confirm_count = 0
            controller.phase5_goal_distance = goal_distance
            controller.phase5_reach_confirm_count = getattr(controller, "phase5_reach_confirm_count", 0) + 1
            controller.cone_phase_confirm_count = int(controller.phase5_reach_confirm_count)
            controller.goal_confirm_count = controller.phase5_reach_confirm_count
            controller.cone_phase_decision = "p5_reached_confirm"
            if controller.phase5_reach_confirm_count < CONE_PHASE5_REACH_CONFIRM_FRAMES:
                return
            controller.cone_phase_decision = "p5_reached_to_p6"
            print(
                f"Final approach ready (camera/range confirmation x{controller.phase5_reach_confirm_count})"
            )
            controller.goal_decision = "final_approach_ready"
            controller.phase6_motion_until = 0.0
            controller.stop_motors()
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
