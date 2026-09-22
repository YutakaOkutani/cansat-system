"""Shared camera/range evidence for final approach and its motor interlock."""
import math
from mission.cone_candidate import camera_has_visible_cone, evaluate_cone_candidate
from mission.close_track import close_track_evidence, cropped_region
from mission.const import (
    GOAL_ENTRY_DISTANCE_CM, GOAL_MAX_SAMPLE_SKEW_SEC, GOAL_MIN_OCCUPANCY,
    GOAL_CENTER_TOLERANCE, SONAR_MIN_DISTANCE_CM, SONAR_STALE_TIMEOUT_SEC,
)


def goal_evidence(snapshot, now, monotonic_now):
    try:
        distance = float(snapshot.get('sonar_distance_cm', float('nan')))
        age = monotonic_now - float(snapshot.get('sonar_observed_monotonic', 0))
        sonar_time = float(snapshot.get('sonar_observed_at', 0))
        camera_time = float(snapshot.get('cone_updated_at', 0))
        direction = float(snapshot.get('cone_image_direction', float('nan')))
        evidence = evaluate_cone_candidate(snapshot)
        continuation = close_track_evidence(snapshot, now, monotonic_now)
        if snapshot.get('cone_close_track', {}).get('hold') and cropped_region(snapshot) and not continuation:
            return False, 'close_identity_unconfirmed', distance
        if not continuation and not camera_has_visible_cone(snapshot, now):
            return False, 'camera_missing', distance
        if not (continuation or evidence['candidate'] or evidence['close_reached']):
            return False, 'cone_unconfirmed', distance
        if not snapshot.get('cone_image_direction_valid', False) or not math.isfinite(direction):
            return False, 'image_direction_invalid', distance
        if not continuation and abs(direction - 0.5) > GOAL_CENTER_TOLERANCE:
            return False, 'cone_off_axis', distance
        if not evidence['close_reached'] and evidence['occupancy'] < GOAL_MIN_OCCUPANCY:
            return False, 'visual_range_mismatch', distance
        if not snapshot.get('sonar_valid', False) or int(snapshot.get('sonar_sequence', 0)) <= 0:
            return False, 'sonar_invalid', distance
        if not 0 <= age < SONAR_STALE_TIMEOUT_SEC:
            return False, 'sonar_stale', distance
        if not math.isfinite(distance) or not SONAR_MIN_DISTANCE_CM <= distance <= GOAL_ENTRY_DISTANCE_CM:
            return False, 'sonar_out_of_range', distance
        if not math.isfinite(sonar_time) or not math.isfinite(camera_time) or abs(camera_time - sonar_time) > GOAL_MAX_SAMPLE_SKEW_SEC:
            return False, 'observations_not_aligned', distance
        return True, 'close_track_camera_sonar_matched' if continuation else 'camera_sonar_matched', distance
    except (TypeError, ValueError, OverflowError):
        return False, 'invalid_observation', float('nan')
