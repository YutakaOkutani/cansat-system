"""Bounded continuity for an identified cone clipped by the image edges.

CanSatState owns the tracker under its lock. Consumers only read its published
evidence and recheck live sensor freshness; they cannot advance confirmations.
"""
import math

from mission.cone_candidate import evaluate_cone_candidate
from mission.const import (
    CAMERA_FRAME_STALE_STOP_SEC, CAMERA_HORIZONTAL_FOV_DEG,
    CONE_CLOSE_TRACK_CONFIRM_FRAMES, CONE_CLOSE_TRACK_DISTANCE_CM,
    CONE_CLOSE_TRACK_MAX_SEC, CONE_CLOSE_TRACK_MAX_TURN_DEG,
    CONE_CLOSE_TRACK_MIN_OCCUPANCY, CONE_CLOSE_TRACK_MAX_HUE_CHANGE,
    CONE_CLOSE_TRACK_MAX_SV_CHANGE, GOAL_CENTER_TOLERANCE,
    CONE_CLOSE_TRACK_MIN_HUE, CONE_CLOSE_TRACK_MIN_SV, CONE_CLOSE_TRACK_SURFACE_HUE,
    CONE_CLOSE_TRACK_MIN_ROI_SUPPORT, CONE_CLOSE_TRACK_WIDTH_FRAC,
    CONE_CLOSE_TRACK_MIN_EDGES, CONE_CLOSE_TRACK_BEARING_TOLERANCE_DEG,
    CONE_CLOSE_TRACK_OCCUPANCY_RATIO_MIN, CONE_CLOSE_TRACK_OCCUPANCY_RATIO_MAX,
    GOAL_ENTRY_DISTANCE_CM, GOAL_MAX_SAMPLE_SKEW_SEC, SONAR_MIN_DISTANCE_CM, SONAR_STALE_TIMEOUT_SEC,
    BNO_HEADING_RECOVERY_STALE_SEC,
)


def number(source, key, default=float('nan')):
    try:
        return float(source.get(key, default))
    except (ValueError, TypeError, OverflowError):
        return default


def heading_delta(a, b):
    return abs((a - b + 180) % 360 - 180)


def cropped_region(snapshot):
    d = snapshot.get('cone_debug', {})
    return (number(d, 'bbox_width_frac') > CONE_CLOSE_TRACK_WIDTH_FRAC
            and number(d, 'edge_touch_count') >= CONE_CLOSE_TRACK_MIN_EDGES
            and number(d, 'occupancy') >= CONE_CLOSE_TRACK_MIN_OCCUPANCY)


def fresh_range(snapshot, now, mono, limit=CONE_CLOSE_TRACK_DISTANCE_CM):
    distance = number(snapshot, 'sonar_distance_cm')
    return (snapshot.get('sonar_valid', False)
            and number(snapshot, 'sonar_sequence') > 0
            and 0 <= mono - number(snapshot, 'sonar_observed_monotonic') < SONAR_STALE_TIMEOUT_SEC
            and 0 <= now - number(snapshot, 'sonar_observed_at') < SONAR_STALE_TIMEOUT_SEC
            and SONAR_MIN_DISTANCE_CM <= distance <= limit)


def fresh_heading(snapshot, mono):
    return (snapshot.get('angle_valid', False)
            and math.isfinite(number(snapshot, 'angle'))
            and 0 <= mono - number(snapshot, 'angle_observed_monotonic') <= BNO_HEADING_RECOVERY_STALE_SEC)


def close_track_evidence(snapshot, now, mono):
    """Return live continuation eligibility, distinct from a latched stop."""
    track = snapshot.get('cone_close_track', {})
    if not track.get('eligible', False):
        return False
    return bool(
        track.get('sequence') == snapshot.get('cone_sequence')
        and snapshot.get('cone_valid', False)
        and 0 <= now - number(snapshot, 'cone_updated_at') <= CAMERA_FRAME_STALE_STOP_SEC
        and mono <= number(track, 'deadline')
        and fresh_range(snapshot, now, mono)
        and fresh_heading(snapshot, mono)
        and heading_delta(number(snapshot, 'angle'), number(track, 'center_heading')) <= CONE_CLOSE_TRACK_MAX_TURN_DEG
    )


class CloseConeTrack:
    def __init__(self):
        self.reset()

    def reset(self):
        self.count = 0
        self.previous = None
        self.anchor = None
        self.center = None
        self.deadline = 0.0
        self.hold = False
        self.result = self._result('inactive')

    def _result(self, reason, eligible=False, sequence=0):
        deadline = min(self.deadline, self.center[2] + CONE_CLOSE_TRACK_MAX_SEC) if self.center else self.deadline
        return dict(anchor_present=self.anchor is not None, center_present=self.center is not None,
                    hold=self.hold, eligible=eligible, reason=reason,
                    count=self.count, sequence=sequence, deadline=deadline,
                    center_heading=self.center[0] if self.center else float('nan'),
                    center_travel=self.center[1] if self.center else float('nan'))

    def suspend(self, reason):
        """Stop using evidence now, but retain identity until its fixed expiry."""
        self.result = self._result(reason, sequence=self.result['sequence'])

    def observe_heading(self, s):
        # Check every IMU sample: turn-away/turn-back cannot hide an excursion.
        if self.center and heading_delta(number(s, 'angle'), self.center[0]) > CONE_CLOSE_TRACK_MAX_TURN_DEG:
            self.invalidate('heading_excursion')

    def invalidate(self, reason):
        """Revoke identity on contradiction or expiry, retaining the stop."""
        sequence = self.result['sequence']
        self.count = 0
        self.previous = None
        self.anchor = None
        self.center = None
        self.deadline = 0.0
        self.result = self._result(reason, sequence=sequence)

    def update(self, s, now, mono):
        if s.get('phase') not in (4, 5, 6):
            self.reset()
            return self.result
        seq = s.get('cone_sequence', 0)
        e = evaluate_cone_candidate(s)
        d = s.get('cone_debug', {})
        clipped = cropped_region(s)
        camera_ok = (s.get('cone_valid', False) and seq > 0
                     and 0 <= now - number(s, 'cone_updated_at') <= CAMERA_FRAME_STALE_STOP_SEC)
        range_ok = fresh_range(s, now, mono)
        # A near, frame-filling object warrants a stop even without identity.
        # Sensor loss/expiry never releases this stop into a search arc.
        if camera_ok and clipped and range_ok:
            self.hold = True
        if mono > self.deadline and self.anchor:
            self.invalidate('identity_expired')
        self.observe_heading(s)
        reason = 'collecting'
        eligible = False
        color_ok = (e['hue'] >= CONE_CLOSE_TRACK_MIN_HUE and e['sv'] >= CONE_CLOSE_TRACK_MIN_SV
                    and (e['roi_support'] >= CONE_CLOSE_TRACK_MIN_ROI_SUPPORT
                         or (clipped and e['hue'] >= CONE_CLOSE_TRACK_SURFACE_HUE))
                    and number(d, 'roi_negative_support', 0) <= e['roi_absolute_support']
                    and number(d, 'ground_penalty', 1) >= 0.5)
        current = dict(time=now, direction=number(s, 'cone_image_direction'),
                       hue=e['hue'], sv=e['sv'], occupancy=e['occupancy'],
                       bearing=(number(s, 'angle') + (number(s, 'cone_image_direction') - 0.5)
                                * CAMERA_HORIZONTAL_FOV_DEG) % 360)
        valid = (camera_ok and fresh_heading(s, mono)
                 and s.get('cone_image_direction_valid', False)
                 and 0 <= current['direction'] <= 1)
        continuous = self.previous is not None and valid
        if continuous:
            p = self.previous
            continuous = (0 < now - p['time'] <= CAMERA_FRAME_STALE_STOP_SEC
                          and abs(current['hue'] - p['hue']) <= CONE_CLOSE_TRACK_MAX_HUE_CHANGE
                          and abs(current['sv'] - p['sv']) <= CONE_CLOSE_TRACK_MAX_SV_CHANGE
                          and CONE_CLOSE_TRACK_OCCUPANCY_RATIO_MIN
                          <= current['occupancy'] / max(p['occupancy'], 1e-6)
                          <= CONE_CLOSE_TRACK_OCCUPANCY_RATIO_MAX
                          and heading_delta(current['bearing'], p['bearing']) <= CONE_CLOSE_TRACK_BEARING_TOLERANCE_DEG)
        if not continuous:
            self.count = 0
            if self.anchor is None:
                self.center = None

        # Acquire only from ordinary, not fully clipped observations. A single
        # close_reached shortcut cannot arm the history.
        ordinary = valid and not clipped and (e['candidate'] or e['close_reached'])
        if ordinary:
            self.count += 1
            if self.count >= CONE_CLOSE_TRACK_CONFIRM_FRAMES:
                self.anchor = current.copy()
                self.deadline = mono + CONE_CLOSE_TRACK_MAX_SEC
                reason = 'identified'
        elif not clipped:
            self.count = 0
            reason = 'target_temporarily_missing'

        # Keep an ordinary centered view seen during identity acquisition.
        # It grants nothing until the independent multi-frame identity exists.
        # A clipped candidate can establish centering only after acquisition.
        if ((self.anchor or ordinary) and valid and (e['candidate'] or e['close_reached'])
                and (self.center is None or not clipped)
                and abs(current['direction'] - 0.5) <= GOAL_CENTER_TOLERANCE
                and (ordinary or mono <= self.deadline)
                and fresh_range(s, now, mono, GOAL_ENTRY_DISTANCE_CM)):
            self.center = (number(s, 'angle'), number(s, 'heading_travel_deg'), mono)

        # Strong contradictory negative evidence revokes the identity. A weak
        # ROI, missing echo or brief color dropout merely suspends its use.
        if clipped and number(d, 'roi_negative_support', 0) > e['roi_absolute_support']:
            self.invalidate('negative_region_contradiction')

        if clipped:
            reason = 'close_identity_unconfirmed'
            if self.anchor and self.center and valid and color_ok and range_ok:
                eligible = (bool(d.get('close_region_ok')) and bool(d.get('dominant_close'))
                            and mono <= self.deadline
                            and mono <= self.center[2] + CONE_CLOSE_TRACK_MAX_SEC
                            and abs(current['hue'] - self.anchor['hue']) <= CONE_CLOSE_TRACK_MAX_HUE_CHANGE
                            and abs(current['sv'] - self.anchor['sv']) <= CONE_CLOSE_TRACK_MAX_SV_CHANGE
                            and heading_delta(number(s, 'angle'), self.center[0]) <= CONE_CLOSE_TRACK_MAX_TURN_DEG
                            and abs(number(s, 'sonar_observed_at') - number(s, 'cone_updated_at')) <= GOAL_MAX_SAMPLE_SKEW_SEC)
                reason = 'close_track_continuation' if eligible else 'close_track_expired_or_inconsistent'
        self.previous = current if valid and (ordinary or clipped) else None
        self.result = self._result(reason, eligible, seq)
        return self.result
