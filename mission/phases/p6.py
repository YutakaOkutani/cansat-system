"""Bounded range-controlled approach; success is decided after stopping."""
import time
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mission.diagnostics import elapsed
from mission.const import (
    DEVICE_LED_GREEN, DEVICE_LED_RED, GOAL_CONFIRM_SAMPLES,
    GOAL_OBSERVATION_TIMEOUT_SEC, GOAL_SETTLE_SEC,
    GOAL_STOP_DISTANCE_CM, PHASE6_APPROACH_TIMEOUT_SEC, Phase,
    GOAL_VOTE_WINDOW_SIZE, GOAL_VOTE_WINDOW_SEC, GOAL_RESUME_DISTANCE_CM,
    GOAL_FAR_CONFIRM_SAMPLES,
    GOAL_MAX_PULSES, GOAL_PROGRESS_MIN_CM, GOAL_MAX_DISTANCE_SPREAD_CM,
    GOAL_SETTLE_HEADING_DEG, CAMERA_FRAME_STALE_STOP_SEC,
    GOAL_MAX_FORWARD_PULSES, GOAL_MAX_ALIGN_PULSES, GOAL_RECOVERY_DISTANCE_CM,
    GOAL_RECOVERY_SAMPLES, GOAL_MAX_REAPPROACHES, SONAR_MAX_DISTANCE,
    GOAL_REAPPROACH_WAIT_SEC, GOAL_CAMERA_TIMEOUT_SEC,
)
from mission.goal import goal_evidence, alignment_evidence
from mission.approach import settings, output_seconds, motion_profile, motion_evidence
from mission.close_track import fresh_heading, heading_delta, number, cropped_region
from mission.cone_candidate import evaluate_cone_candidate
from mission.phases.base import BasePhaseHandler


class Phase6Handler(BasePhaseHandler):
    @staticmethod
    def _stop(controller, reason='phase6_stopped_observation'):
        controller.phase6_motion_until = 0.0
        controller.phase6_motion = None
        controller.stop_motors(reason=reason)

    def _finish(self, controller, reason):
        self._stop(controller)
        controller.phase6_gate = 'terminal'
        controller.mission_end_reason = reason
        controller.goal_decision = reason.lower()
        controller.st.update_navigation(phase=int(Phase.PHASE7))

    def _settle(self, c, now, reason='phase6_stopped_observation'):
        self._stop(c, reason)
        c.phase6_stage = 'settle'
        c.phase6_observe_after = now + GOAL_SETTLE_SEC
        c.phase6_votes = []
        c.phase6_recovery_samples = []
        c.phase6_far_count = 0
        c.goal_confirm_count = 0
        c.phase6_align_side = None
        c.phase6_align_count = 0
        c.phase6_heading = None
        c.phase6_settle_started = now

    def _pulse(self, c, now, distance, action):
        counter = 'phase6_forward_pulses' if action == 'forward' else 'phase6_align_pulses'
        limit = GOAL_MAX_FORWARD_PULSES if action == 'forward' else GOAL_MAX_ALIGN_PULSES
        if c.phase6_pulses >= GOAL_MAX_PULSES or getattr(c, counter, 0) >= limit:
            self._finish(c, 'GOAL_MOTION_LIMIT')
            return
        cfg = settings(c)
        output = output_seconds(c, action, now)
        output_limit = cfg.forward_output_limit_sec if action == 'forward' else cfg.align_output_limit_sec
        if output >= output_limit:
            self._finish(c, 'GOAL_OUTPUT_TIME_LIMIT')
            return
        # Progress is assessed only from stopped, independent valid observations,
        # after actual output. Unused requests are not evidence of a stuck rover.
        key = 'forward' if action == 'forward' else 'align'
        metric = distance if action == 'forward' else abs(number(c.phase6_eval_snapshot, 'cone_image_direction', .5)-.5)
        anchor_name = 'phase6_' + key + '_progress_anchor'
        anchor = getattr(c, anchor_name, None)
        window = cfg.progress_output_sec if action == 'forward' else cfg.align_progress_output_sec
        minimum = GOAL_PROGRESS_MIN_CM if action == 'forward' else .02
        c.phase6_progress_metric = metric
        c.phase6_progress_delta = '' if anchor is None else anchor[0] - metric
        c.phase6_progress_output = 0.0 if anchor is None else output - anchor[1]
        if anchor is not None and output - anchor[1] >= window:
            if anchor[0] - metric < minimum:
                boost_name = 'phase6_' + key + '_boost'
                if getattr(c, boost_name, False) or (action == 'forward' and distance <= 8):
                    self._finish(c, 'GOAL_NO_PROGRESS')
                    return
                setattr(c, boost_name, True)
            anchor = None
        if anchor is None:
            setattr(c, anchor_name, (metric, output))
        duration, speed, floor = motion_profile(c, distance, action)
        duration = min(duration, output_limit - output)
        self._settle(c, now)
        c.phase6_pulses += 1
        setattr(c, counter, getattr(c, counter, 0) + 1)
        c.phase6_action = action
        c.phase6_stage = 'move'
        c.phase6_gate = 'pulse_requested'
        c.phase6_motion_started = now
        snapshot = c.phase6_eval_snapshot
        c.phase6_motion = dict(
            action=action, requested_at=now, until=now+duration, speed=speed,
            duration=duration, distance_floor=floor,
            cone_sequence=int(snapshot.get('cone_sequence', 0)),
            sonar_sequence=int(snapshot.get('sonar_sequence', 0)),
            heading=number(snapshot, 'angle') if fresh_heading(snapshot, now) else None,
            heading_travel=number(snapshot, 'heading_travel_deg', 0))
        c.phase6_motion_until = now + duration
        c.phase6_requested_duration = duration
        c.phase6_requested_pwm = speed
        c.goal_decision = 'range_approach_pulse' if action == 'forward' else 'bounded_align_' + action

    def _reapproach(self, c, snapshot, now, wall):
        """Only stopped, fresh, repeatedly far observations can release to P5."""
        evidence = evaluate_cone_candidate(snapshot)
        matched, _, distance = goal_evidence(
            snapshot, wall, now, require_center=False,
            max_distance=SONAR_MAX_DISTANCE * 100.0 - 0.001)
        eligible = (
            matched and distance >= GOAL_RECOVERY_DISTANCE_CM
            and not evidence['close_reached'] and not cropped_region(snapshot)
            and not snapshot.get('cone_close_track', {}).get('hold')
            and now >= c.phase6_observe_after
            and number(snapshot, 'sonar_observed_monotonic', 0) >= c.phase6_observe_after
            and now - (wall - number(snapshot, 'cone_updated_at', 0)) >= c.phase6_observe_after
        )
        if not eligible:
            c.phase6_recovery_samples = []
            return False
        pair = (int(snapshot['cone_sequence']), int(snapshot['sonar_sequence']))
        c.phase6_recovery_samples = [s for s in c.phase6_recovery_samples
                                     if now - s[0] <= GOAL_VOTE_WINDOW_SEC]
        previous = c.phase6_recovery_last_pair
        if pair[0] > previous[0] and pair[1] > previous[1]:
            c.phase6_recovery_last_pair = pair
            samples = c.phase6_recovery_samples
            if samples and max(abs(distance - s[1]) for s in samples) > GOAL_MAX_DISTANCE_SPREAD_CM:
                samples.clear()
            samples.append((now, distance))
            c.phase6_recovery_samples = samples[-GOAL_RECOVERY_SAMPLES:]
        if (len(c.phase6_recovery_samples) < GOAL_RECOVERY_SAMPLES
                or now - c.phase6_wait_since < GOAL_REAPPROACH_WAIT_SEC):
            return False
        if getattr(c, 'phase6_reapproaches', 0) >= GOAL_MAX_REAPPROACHES:
            self._finish(c, 'GOAL_REAPPROACH_LIMIT')
            return True
        self._settle(c, now)
        c.phase6_reapproaches = getattr(c, 'phase6_reapproaches', 0) + 1
        c.phase6_elapsed_used = now - c.phase6_start_time
        c.phase6_gate = 'reapproach_p5'
        c.goal_decision = 'far_range_reapproach'
        c.phase5_entry_reason = 'phase6_far_range_recovery'
        c.st.update_navigation(phase=int(Phase.PHASE5))
        return True

    def execute(self, controller, snapshot):
        controller.phase6_gate = 'evaluating'
        controller.phase6_evidence_matched = ''
        controller.phase6_eval_snapshot = snapshot
        try:
            self._execute(controller, snapshot)
        finally:
            c = controller
            now, wall = time.monotonic(), time.time()
            used = c.phase6_eval_snapshot
            c.phase6_diagnostics = {
                'Phase6Stage': getattr(c, 'phase6_stage', 'inactive'),
                'Phase6Action': getattr(c, 'phase6_action', ''),
                'Phase6Gate': c.phase6_gate,
                'Phase6ElapsedSec': round(now - getattr(c, 'phase6_start_time', now), 3),
                'Phase6WaitSec': round(now - getattr(c, 'phase6_wait_since', now), 3),
                'Phase6SettleRemainingSec': round(max(0, getattr(c, 'phase6_observe_after', now) - now), 3),
                'Phase6PulseRequestedCount': getattr(c, 'phase6_pulses', 0),
                'Phase6VoteCount': len(getattr(c, 'phase6_votes', [])),
                'Phase6RequestedDurationSec': getattr(c, 'phase6_requested_duration', ''),
                'Phase6RequestedPWM': getattr(c, 'phase6_requested_pwm', ''),
                'Phase6DistanceBand': getattr(c, 'phase6_distance_band', ''),
                'Phase6ForwardOutputSec': round(output_seconds(c, 'forward', now), 4),
                'Phase6AlignOutputSec': round(output_seconds(c, 'right', now), 4),
                'Phase6ForwardBoost': int(getattr(c, 'phase6_forward_boost', False)),
                'Phase6AlignBoost': int(getattr(c, 'phase6_align_boost', False)),
                'Phase6Reapproaches': getattr(c, 'phase6_reapproaches', 0),
                'Phase6ProgressMetric': getattr(c, 'phase6_progress_metric', ''),
                'Phase6ProgressDelta': getattr(c, 'phase6_progress_delta', ''),
                'Phase6ProgressOutputSec': getattr(c, 'phase6_progress_output', ''),
                'GoalEvalConeSeq': used.get('cone_sequence', 0),
                'GoalEvalSonarSeq': used.get('sonar_sequence', 0),
                'GoalEvalDistanceCm': used.get('sonar_distance_cm', ''),
                'GoalEvalSampleSkewSec': round(abs(number(used, 'cone_updated_at', 0) - number(used, 'sonar_observed_at', 0)), 3),
                'GoalEvalReason': getattr(c, 'goal_decision', ''),
                'GoalEvalMatched': c.phase6_evidence_matched,
                'GoalEvalConfirmCount': getattr(c, 'goal_confirm_count', 0),
                'GoalEvalElapsedSec': elapsed(c, wall),
            }
            # A periodic heartbeat distinguishes waiting from a blocked loop.
            if now - getattr(c, 'phase6_last_status_at', float('-inf')) >= 1.0 or c.phase6_gate == 'terminal':
                c.phase6_last_status_at = now
                print(f"p6: {c.phase6_gate}; evidence={getattr(c, 'goal_decision', '')}; "
                      f"confirm={getattr(c, 'goal_confirm_count', 0)}/{GOAL_CONFIRM_SAMPLES}; "
                      f"elapsed={c.phase6_diagnostics['Phase6ElapsedSec']:.1f}s/{PHASE6_APPROACH_TIMEOUT_SEC:.0f}s",
                      flush=True)

    def _execute(self, controller, snapshot):
        c = controller
        now, wall = time.monotonic(), time.time()
        marker = getattr(c, 'phase_entry_time', None)
        if not hasattr(c, 'phase6_stage') or getattr(c, 'phase6_entry_marker', None) != marker:
            self._settle(c, now)
            c.phase6_entry_marker = marker
            elapsed_used = max(getattr(c, 'phase6_elapsed_used', 0.0),
                               getattr(c, 'phase6_resume_elapsed', 0.0),
                               getattr(c, 'phase_elapsed_totals', {}).get(Phase.PHASE6, 0.0))
            c.phase6_start_time = now - elapsed_used
            c.phase6_resume_elapsed = 0.0
            c.phase6_last_pair = (0, 0)
            c.phase6_wait_since = now
            c.phase6_pulses = max(getattr(c, 'phase6_pulses', 0), getattr(c, 'phase6_resume_pulses', 0))
            c.phase6_resume_pulses = 0
            c.phase6_forward_progress_anchor = None
            c.phase6_align_progress_anchor = None
            c.phase6_camera_recovery_requested = False
            c.phase6_recovery_samples = []
            c.phase6_recovery_last_pair = (0, 0)
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
        c.phase6_eval_snapshot = snapshot
        camera_age = wall - number(snapshot, 'cone_updated_at', 0)
        # Independent of the acquisition worker: a blocked driver cannot keep
        # P6 alive indefinitely. Recovery is consumed by that worker if it returns.
        if camera_age > CAMERA_FRAME_STALE_STOP_SEC:
            if not c.phase6_camera_recovery_requested:
                c.camera_recovery_requested = True
                c.phase6_camera_recovery_requested = True
            if now - c.phase6_start_time >= GOAL_CAMERA_TIMEOUT_SEC and camera_age >= GOAL_CAMERA_TIMEOUT_SEC:
                self._finish(c, 'GOAL_CAMERA_TIMEOUT')
                return
        # Expire stopped votes even while no usable sensor pair arrives.
        c.phase6_votes = [(t, d) for t, d in c.phase6_votes if now - t <= GOAL_VOTE_WINDOW_SEC]
        c.goal_confirm_count = sum(d <= GOAL_STOP_DISTANCE_CM for _, d in c.phase6_votes)
        if c.phase6_stage == 'move':
            moving, move_reason, _ = motion_evidence(c, snapshot, wall, now)
            c.goal_decision = move_reason
            c.phase6_evidence_matched = int(moving)
            if moving:
                c.phase6_gate = 'pulse_active'
                return
            self._settle(c, now, 'phase6:' + move_reason)
            c.phase6_gate = 'pulse_finished_settle'
            return
        matched, reason, distance = goal_evidence(snapshot, wall, now)
        action = 'forward'
        if not matched and reason == 'cone_off_axis':
            aligned, _, distance = alignment_evidence(snapshot, wall, now)
            if aligned:
                matched = True
                action = 'left' if snapshot['cone_direction'] < .5 else 'right'
        c.goal_decision = reason
        c.phase6_evidence_matched = int(matched)

        if number(snapshot, 'angle_motion_monotonic', 0) > c.phase6_settle_started:
            self._settle(c, now)
            c.phase6_gate = 'heading_settle'
            c.goal_decision = 'settling_after_heading_change'
            return
        if fresh_heading(snapshot, now):
            heading = number(snapshot, 'angle')
            if c.phase6_heading is not None and heading_delta(heading, c.phase6_heading) > GOAL_SETTLE_HEADING_DEG:
                self._settle(c, now)
                c.phase6_heading = heading
                c.phase6_gate = 'heading_settle'
                c.goal_decision = 'settling_after_heading_change'
                return
            if c.phase6_heading is None:
                c.phase6_heading = heading

        if not matched:
            c.phase6_gate = 'evidence_rejected'
            was_moving = c.phase6_stage == 'move'
            self._stop(c)
            c.phase6_far_count = 0
            c.phase6_align_count = 0
            if was_moving:
                self._settle(c, now)
            if self._reapproach(c, snapshot, now, wall):
                return
            # Missing/ambiguous observations are never votes. Previously good
            # stopped votes survive only their short window, not a movement.
            if now - c.phase6_wait_since >= GOAL_OBSERVATION_TIMEOUT_SEC:
                self._finish(c, 'GOAL_PROXIMITY_UNCONFIRMED')
            return
        c.phase6_recovery_samples = []
        self._stop(c)
        if now < c.phase6_observe_after:
            c.phase6_gate = 'settle_wait'
            return
        if (number(snapshot, 'sonar_observed_monotonic') < c.phase6_observe_after
                or camera_age < 0 or now - camera_age < c.phase6_observe_after):
            c.phase6_gate = 'post_stop_sample_wait'
            return
        pair = (int(snapshot['cone_sequence']), int(snapshot['sonar_sequence']))
        previous = c.phase6_last_pair
        if pair[0] <= previous[0] or pair[1] <= previous[1]:
            c.phase6_gate = 'new_pair_wait'
            return
        c.phase6_last_pair = pair
        c.phase6_wait_since = now
        if action != 'forward':
            c.phase6_gate = 'alignment_confirm_wait'
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
            c.phase6_gate = 'range_jump_settle'
            c.goal_decision = 'settling_after_range_jump'
            return
        c.phase6_votes.append((now, distance))
        c.phase6_votes = c.phase6_votes[-GOAL_VOTE_WINDOW_SIZE:]
        c.goal_confirm_count = sum(d <= GOAL_STOP_DISTANCE_CM for _, d in c.phase6_votes)
        c.phase6_gate = 'vote_accepted'
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
