"""Bounded range-controlled approach; success is decided after stopping."""
import time
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mission.const import (
    DEVICE_LED_GREEN, DEVICE_LED_RED, GOAL_CONFIRM_SAMPLES,
    GOAL_OBSERVATION_TIMEOUT_SEC, GOAL_PULSE_SEC, GOAL_SETTLE_SEC,
    GOAL_STOP_DISTANCE_CM, PHASE6_APPROACH_TIMEOUT_SEC, Phase,
    GOAL_VOTE_WINDOW_SIZE, GOAL_VOTE_WINDOW_SEC, GOAL_RESUME_DISTANCE_CM,
    GOAL_FAR_CONFIRM_SAMPLES, GOAL_NEAR_PULSE_SEC, GOAL_NEAR_PULSE_DISTANCE_CM,
    GOAL_MAX_PULSES, GOAL_PROGRESS_MIN_CM, GOAL_PROGRESS_PULSES, GOAL_MAX_DISTANCE_SPREAD_CM,
    GOAL_SETTLE_HEADING_DEG, GOAL_ALIGN_PULSE_SEC, CAMERA_FRAME_STALE_STOP_SEC,
)
from mission.goal import goal_evidence, alignment_evidence
from mission.close_track import fresh_heading, heading_delta, number
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

    def _settle(self, c, now):
        self._stop(c)
        c.phase6_stage = 'settle'
        c.phase6_observe_after = now + GOAL_SETTLE_SEC
        c.phase6_votes = []
        c.phase6_far_count = 0
        c.goal_confirm_count = 0
        c.phase6_align_side = None
        c.phase6_align_count = 0
        c.phase6_heading = None
        c.phase6_settle_started = now

    def _pulse(self, c, now, distance, action):
        if c.phase6_pulses >= GOAL_MAX_PULSES:
            self._finish(c, 'GOAL_MOTION_LIMIT')
            return
        # Translation must make measurable progress over a bounded group of
        # pulses. Alignment has a separate action but shares the total budget.
        if action == 'forward':
            c.phase6_progress.append(distance)
            if len(c.phase6_progress) > GOAL_PROGRESS_PULSES:
                if c.phase6_progress[0] - distance < GOAL_PROGRESS_MIN_CM:
                    self._finish(c, 'GOAL_NO_PROGRESS')
                    return
                c.phase6_progress = [distance]
        self._settle(c, now)
        c.phase6_pulses += 1
        c.phase6_action = action
        c.phase6_stage = 'move'
        duration = (GOAL_ALIGN_PULSE_SEC if action != 'forward' else
                    GOAL_NEAR_PULSE_SEC if distance <= GOAL_NEAR_PULSE_DISTANCE_CM else GOAL_PULSE_SEC)
        c.phase6_motion_started = now
        c.phase6_motion_until = now + duration
        c.goal_decision = 'range_approach_pulse' if action == 'forward' else 'bounded_align_' + action

    def execute(self, controller, snapshot):
        c = controller
        now, wall = time.monotonic(), time.time()
        marker = getattr(c, 'phase_entry_time', None)
        if not hasattr(c, 'phase6_stage') or getattr(c, 'phase6_entry_marker', None) != marker:
            self._settle(c, now)
            c.phase6_entry_marker = marker
            c.phase6_start_time = now
            c.phase6_last_pair = (0, 0)
            c.phase6_wait_since = now
            c.phase6_pulses = 0
            c.phase6_progress = []
            c.phase6_camera_recovery_requested = False
        for key in (DEVICE_LED_RED, DEVICE_LED_GREEN):
            if c.devices.get(key):
                c.devices[key].on()
        if getattr(c, 'mission_total_timeout_triggered', False):
            self._finish(c, 'MISSION_TOTAL_TIMEOUT')
            return
        if now - c.phase6_start_time >= PHASE6_APPROACH_TIMEOUT_SEC:
            self._finish(c, 'GOAL_APPROACH_TIMEOUT')
            return

        snapshot = c.st.snapshot()
        camera_age = wall - number(snapshot, 'cone_updated_at', 0)
        # Independent of the acquisition worker: a blocked driver cannot keep
        # P6 alive indefinitely. Recovery is consumed by that worker if it returns.
        if camera_age > CAMERA_FRAME_STALE_STOP_SEC:
            if not c.phase6_camera_recovery_requested:
                c.camera_recovery_requested = True
                c.phase6_camera_recovery_requested = True
            if now - c.phase6_start_time >= GOAL_OBSERVATION_TIMEOUT_SEC and camera_age >= GOAL_OBSERVATION_TIMEOUT_SEC:
                self._finish(c, 'GOAL_CAMERA_TIMEOUT')
                return
        # Expire stopped votes even while no usable sensor pair arrives.
        c.phase6_votes = [(t, d) for t, d in c.phase6_votes if now - t <= GOAL_VOTE_WINDOW_SEC]
        c.goal_confirm_count = sum(d <= GOAL_STOP_DISTANCE_CM for _, d in c.phase6_votes)
        matched, reason, distance = goal_evidence(snapshot, wall, now)
        action = 'forward'
        if not matched and reason == 'cone_off_axis':
            aligned, _, distance = alignment_evidence(snapshot, wall, now)
            if aligned:
                matched = True
                action = 'left' if snapshot['cone_direction'] < .5 else 'right'
        c.goal_decision = reason

        if number(snapshot, 'angle_motion_monotonic', 0) > c.phase6_settle_started:
            self._settle(c, now)
            c.goal_decision = 'settling_after_heading_change'
            return
        if fresh_heading(snapshot, now):
            heading = number(snapshot, 'angle')
            if c.phase6_heading is not None and heading_delta(heading, c.phase6_heading) > GOAL_SETTLE_HEADING_DEG:
                self._settle(c, now)
                c.phase6_heading = heading
                c.goal_decision = 'settling_after_heading_change'
                return
            if c.phase6_heading is None:
                c.phase6_heading = heading

        if not matched:
            was_moving = c.phase6_stage == 'move'
            self._stop(c)
            c.phase6_far_count = 0
            c.phase6_align_count = 0
            if was_moving:
                self._settle(c, now)
            # Missing/ambiguous observations are never votes. Previously good
            # stopped votes survive only their short window, not a movement.
            if now - c.phase6_wait_since >= GOAL_OBSERVATION_TIMEOUT_SEC:
                self._finish(c, 'GOAL_PROXIMITY_UNCONFIRMED')
            return
        if c.phase6_stage == 'move':
            if (now < c.phase6_motion_until and distance > GOAL_STOP_DISTANCE_CM
                    and action == c.phase6_action):
                return
            self._settle(c, now)
            return
        self._stop(c)
        if now < c.phase6_observe_after:
            return
        if (number(snapshot, 'sonar_observed_monotonic') < c.phase6_observe_after
                or camera_age < 0 or now - camera_age < c.phase6_observe_after):
            return
        pair = (int(snapshot['cone_sequence']), int(snapshot['sonar_sequence']))
        previous = c.phase6_last_pair
        if pair[0] <= previous[0] or pair[1] <= previous[1]:
            return
        c.phase6_last_pair = pair
        c.phase6_wait_since = now
        if action != 'forward':
            c.phase6_votes = []
            c.goal_confirm_count = 0
            c.phase6_far_count = 0
            c.phase6_align_count = c.phase6_align_count + 1 if c.phase6_align_side == action else 1
            c.phase6_align_side = action
            if c.phase6_align_count >= GOAL_FAR_CONFIRM_SAMPLES:
                self._pulse(c, now, distance, action)
            return
        c.phase6_align_count = 0
        if c.phase6_votes and abs(c.phase6_votes[-1][1] - distance) > GOAL_MAX_DISTANCE_SPREAD_CM:
            self._settle(c, now)
            c.goal_decision = 'settling_after_range_jump'
            return
        c.phase6_votes.append((now, distance))
        c.phase6_votes = c.phase6_votes[-GOAL_VOTE_WINDOW_SIZE:]
        c.goal_confirm_count = sum(d <= GOAL_STOP_DISTANCE_CM for _, d in c.phase6_votes)
        c.goal_decision = 'stopped_proximity_confirm'
        if distance <= GOAL_STOP_DISTANCE_CM and c.goal_confirm_count >= GOAL_CONFIRM_SAMPLES:
            self._finish(c, 'GOAL_PROXIMITY_CONFIRMED')
            return
        c.phase6_far_count = c.phase6_far_count + 1 if distance > GOAL_RESUME_DISTANCE_CM else 0
        if c.phase6_far_count >= GOAL_FAR_CONFIRM_SAMPLES:
            self._pulse(c, now, distance, 'forward')


def run_standalone():
    from mission.run import run_single_phase
    run_single_phase(Phase.PHASE6)


if __name__ == "__main__":
    run_standalone()
