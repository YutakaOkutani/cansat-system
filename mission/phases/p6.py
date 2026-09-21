"""Bounded range-controlled approach; success is decided after stopping."""
import time
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mission.const import (
    DEVICE_LED_GREEN, DEVICE_LED_RED, GOAL_CONFIRM_SAMPLES,
    GOAL_MAX_DISTANCE_SPREAD_CM, GOAL_OBSERVATION_TIMEOUT_SEC,
    GOAL_PULSE_SEC, GOAL_SETTLE_SEC, GOAL_STOP_DISTANCE_CM,
    PHASE6_APPROACH_TIMEOUT_SEC, Phase,
)
from mission.goal import goal_evidence
from mission.phases.base import BasePhaseHandler


class Phase6Handler(BasePhaseHandler):
    @staticmethod
    def _stop(controller):
        controller.phase6_motion_until = 0.0
        controller.stop_motors()

    def _finish(self, controller, reason):
        self._stop(controller)
        controller.mission_end_reason = reason
        controller.goal_decision = reason.lower()
        controller.st.update_navigation(phase=int(Phase.PHASE7))

    def _retry(self, controller, reason):
        self._stop(controller)
        controller.goal_decision = reason
        controller.phase5_entry_marker = None
        controller.time_camera_start = 0.0
        controller.phase5_entry_reason = reason
        controller.st.update_navigation(phase=int(Phase.PHASE5))

    def execute(self, controller, snapshot):
        now = time.monotonic()
        marker = getattr(controller, "phase_entry_time", None)
        if (not hasattr(controller, "phase6_stage")
                or getattr(controller, "phase6_entry_marker", None) != marker):
            self._stop(controller)
            controller.phase6_entry_marker = marker
            controller.phase6_start_time = now
            controller.phase6_stage = "settle"
            controller.phase6_observe_after = now + GOAL_SETTLE_SEC
            controller.phase6_last_pair = (0, 0)
            controller.phase6_distances = []
            controller.phase6_wait_since = now
            controller.goal_confirm_count = 0
        for key in (DEVICE_LED_RED, DEVICE_LED_GREEN):
            if controller.devices.get(key):
                controller.devices[key].on()
        if getattr(controller, "mission_total_timeout_triggered", False):
            self._finish(controller, "MISSION_TOTAL_TIMEOUT")
            return
        if now - controller.phase6_start_time >= PHASE6_APPROACH_TIMEOUT_SEC:
            self._retry(controller, "goal_approach_retry")
            return

        snapshot = controller.st.snapshot()
        matched, reason, distance = goal_evidence(snapshot, time.time(), now)
        controller.goal_decision = reason
        if not matched:
            was_moving = controller.phase6_stage == "move"
            self._stop(controller)
            # Faster sonar updates can exceed the pairing skew while waiting
            # for the next image. Keep stopped confirmations across that gap;
            # otherwise a ~1.8 fps camera can never accumulate three pairs.
            if was_moving or reason != "observations_not_aligned":
                controller.phase6_distances = []
                controller.goal_confirm_count = 0
            controller.phase6_stage = "settle"
            if was_moving:
                controller.phase6_observe_after = now + GOAL_SETTLE_SEC
            if now - controller.phase6_wait_since >= GOAL_OBSERVATION_TIMEOUT_SEC:
                self._retry(controller, "goal_observation_retry")
            return

        if controller.phase6_stage == "move":
            if now < controller.phase6_motion_until and distance > GOAL_STOP_DISTANCE_CM:
                return
            self._stop(controller)
            controller.phase6_stage = "settle"
            controller.phase6_observe_after = now + GOAL_SETTLE_SEC
            controller.phase6_distances = []
            controller.goal_confirm_count = 0
            return

        self._stop(controller)
        if now < controller.phase6_observe_after:
            return
        # Both observations must be acquired after the stopped settling period.
        camera_age = time.time() - float(snapshot.get("cone_updated_at", 0.0))
        if (float(snapshot["sonar_observed_monotonic"]) < controller.phase6_observe_after
                or camera_age < 0 or now - camera_age < controller.phase6_observe_after):
            return
        pair = (int(snapshot["cone_sequence"]), int(snapshot["sonar_sequence"]))
        previous = controller.phase6_last_pair
        if pair[0] <= previous[0] or pair[1] <= previous[1]:
            return
        controller.phase6_last_pair = pair
        controller.phase6_wait_since = now
        distances = controller.phase6_distances
        distances.append(distance)
        if max(distances) - min(distances) > GOAL_MAX_DISTANCE_SPREAD_CM:
            distances[:] = [distance]
        # Near and far measurements must not be combined into a goal decision.
        if distance > GOAL_STOP_DISTANCE_CM:
            distances[:] = []
            controller.goal_confirm_count = 0
            controller.phase6_stage = "move"
            controller.phase6_motion_until = now + GOAL_PULSE_SEC
            controller.goal_decision = "range_approach_pulse"
            return
        controller.goal_confirm_count = len(distances)
        controller.goal_decision = "stopped_proximity_confirm"
        if len(distances) >= GOAL_CONFIRM_SAMPLES:
            self._finish(controller, "GOAL_PROXIMITY_CONFIRMED")


def run_standalone():
    from mission.run import run_single_phase
    run_single_phase(Phase.PHASE6)


if __name__ == "__main__":
    run_standalone()
