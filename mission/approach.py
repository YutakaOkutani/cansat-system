"""Distance-dependent P6 motion and a shared, bounded motor authorization."""
from dataclasses import dataclass
import math

from mission.const import (GOAL_STOP_DISTANCE_CM, GOAL_CENTER_TOLERANCE,
    GOAL_SETTLE_HEADING_DEG, PHASE6_APPROACH_SPEED, GOAL_PULSE_SEC,
    GOAL_NEAR_PULSE_SEC, GOAL_ALIGN_PULSE_SEC, MISSION_TIMEOUT_TOTAL, PHASE6_APPROACH_TIMEOUT_SEC)
from mission.goal import goal_evidence, alignment_evidence
from mission.close_track import fresh_heading, heading_delta, number


@dataclass(frozen=True)
class ApproachConfig:
    # Near-goal PWM and 50 ms motion deliberately remain unchanged.
    far_forward_sec: float = .4
    far_forward_boost_sec: float = .6
    far_forward_pwm: float = 70.0
    far_forward_boost_pwm: float = 75.0
    mid_forward_sec: float = GOAL_PULSE_SEC
    mid_forward_boost_sec: float = .25
    far_align_sec: float = .15
    far_align_boost_sec: float = .30
    far_align_pwm: float = 65.0
    far_align_boost_pwm: float = 75.0
    near_align_boost_sec: float = .10
    near_align_boost_pwm: float = 55.0
    forward_output_limit_sec: float = 60.0
    align_output_limit_sec: float = 20.0
    progress_output_sec: float = 3.0
    align_progress_output_sec: float = .6
    max_turn_deg: float = 15.0


DEFAULT_APPROACH = ApproachConfig()


def settings(c):
    return getattr(getattr(c, 'mission_config', None), 'approach', DEFAULT_APPROACH)


def output_seconds(c, action, now):
    """Include an in-flight output without counting requested-but-unused motion."""
    state = getattr(c, 'phase6_output', {})
    key = 'forward' if action == 'forward' else 'align'
    value = float(state.get(key, 0.0))
    if state.get('active') == key:
        value += max(0.0, now - state['updated_at'])
    return value


def motion_profile(c, distance, action):
    cfg = settings(c)
    boost = bool(getattr(c, 'phase6_forward_boost' if action == 'forward' else 'phase6_align_boost', False))
    # Upgrade only after a larger distance is independently observed; downgrade
    # immediately on a closer echo, including while an output is in flight.
    previous = getattr(c, 'phase6_distance_band', None)
    if distance <= 8:
        band = 'near'
    elif distance <= 20:
        band = 'near' if previous == 'near' and distance < 10 else 'mid'
    else:
        band = 'mid' if previous in ('near', 'mid') and distance < 22 else 'far'
    c.phase6_distance_band = band
    if action == 'forward':
        if band == 'far':
            return (cfg.far_forward_boost_sec if boost else cfg.far_forward_sec,
                    cfg.far_forward_boost_pwm if boost else cfg.far_forward_pwm, 20.0)
        if band == 'mid':
            return (cfg.mid_forward_boost_sec if boost else cfg.mid_forward_sec,
                    PHASE6_APPROACH_SPEED, 8.0)
        return GOAL_NEAR_PULSE_SEC, PHASE6_APPROACH_SPEED, GOAL_STOP_DISTANCE_CM
    if band == 'far':
        return (cfg.far_align_boost_sec if boost else cfg.far_align_sec,
                cfg.far_align_boost_pwm if boost else cfg.far_align_pwm, 20.0)
    return (cfg.near_align_boost_sec if boost else GOAL_ALIGN_PULSE_SEC,
            cfg.near_align_boost_pwm if boost else PHASE6_APPROACH_SPEED, 6.0)


def motion_evidence(c, snapshot, wall, now):
    """Only active motion may reuse a still-fresh authorized camera observation.

    Entry, stopped voting and success always use synchronized sensor pairs.
    The authorization has a fixed deadline; repeated motor ticks never renew it.
    """
    start = getattr(c, 'mission_start_time', None)
    if start is not None and wall - start >= MISSION_TIMEOUT_TOTAL:
        return False, 'mission_timeout', float('nan')
    start = getattr(c, 'phase6_start_time', None)
    if start is not None and now - start >= PHASE6_APPROACH_TIMEOUT_SEC:
        return False, 'approach_timeout', float('nan')
    action = getattr(c, 'phase6_action', 'forward')
    pulse = getattr(c, 'phase6_motion', None)
    until = min(float(getattr(c, 'phase6_motion_until', 0)), pulse['until'] if pulse else float('inf'))
    if now >= until:
        return False, 'pulse_deadline_or_not_authorized', float('nan')
    # Keep the original near-motion evidence gate. Only the longer far/mid
    # commands need a continuation lease across camera/sonar sampling skew.
    relaxed = (pulse is not None and pulse['action'] == action and now >= pulse['requested_at']
               and (pulse['distance_floor'] == 20
                    or (action == 'forward' and pulse['distance_floor'] == 8
                        and pulse['duration'] > GOAL_PULSE_SEC)))
    matched, reason, distance = (alignment_evidence(snapshot, wall, now, require_sync=not relaxed)
        if action != 'forward' else goal_evidence(snapshot, wall, now, require_sync=not relaxed))
    if not matched:
        return False, 'evidence:' + reason, distance
    if distance <= GOAL_STOP_DISTANCE_CM:
        return False, 'stop_distance', distance
    if pulse:
        if (int(snapshot.get('cone_sequence', 0)) < pulse['cone_sequence']
                or int(snapshot.get('sonar_sequence', 0)) < pulse['sonar_sequence']):
            return False, 'observation_sequence_regressed', distance
        if distance <= pulse['distance_floor'] and (action == 'forward' or pulse['distance_floor'] == 20):
            return False, 'closer_distance_band', distance
        cfg = settings(c)
        limit = cfg.forward_output_limit_sec if action == 'forward' else cfg.align_output_limit_sec
        if output_seconds(c, action, now) >= limit:
            return False, 'output_time_limit', distance
    if action != 'forward':
        direction = number(snapshot, 'cone_direction', float('nan'))
        if not math.isfinite(direction) or not (direction < .5-GOAL_CENTER_TOLERANCE if action == 'left'
                                               else direction > .5+GOAL_CENTER_TOLERANCE):
            return False, 'alignment_direction_mismatch', distance
    # Far alignment intentionally rotates. Near alignment retains the old shock
    # interlock; stationary confirmation always retains its original IMU gates.
    far_turn = pulse and action != 'forward' and pulse['distance_floor'] == 20
    if far_turn:
        if pulse['heading'] is not None:
            if not fresh_heading(snapshot, now):
                return False, 'heading_stale', distance
            travel = number(snapshot, 'heading_travel_deg', 0) - pulse['heading_travel']
            if max(travel, heading_delta(number(snapshot, 'angle'), pulse['heading'])) >= settings(c).max_turn_deg:
                return False, 'turn_angle_limit', distance
    elif (number(snapshot, 'angle_motion_monotonic', 0) > getattr(c, 'phase6_motion_started', now)
          or (pulse and pulse['heading'] is not None and fresh_heading(snapshot, now)
              and heading_delta(number(snapshot, 'angle'), pulse['heading']) > GOAL_SETTLE_HEADING_DEG)):
        return False, 'heading_changed', distance
    return True, 'drive_authorized', distance
