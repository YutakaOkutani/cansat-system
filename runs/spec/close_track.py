"""Near-cone continuity, rejection, and real phase/motor integration."""
import unittest
import json
from pathlib import Path
from unittest.mock import patch

from mission.const import Phase
from mission.goal import goal_evidence
from mission.st import CanSatState
from mission.phases.p4 import Phase4Handler
from mission.phases.p5 import Phase5Handler
from mission.phases.p6 import Phase6Handler
from runs.spec.cone_phase_diagnostics import _VisionController
from runs.spec.goal_approach import Motor


def observe(st, now, *, clipped=False, distance=2.6, heading=100,
            direction=0.5, hue=0.93, sv=0.70, negative=0,
            probability=None, valid=True, sonar_valid=True):
    debug = dict(strict_red_ok=0 if clipped else 1,
                 candidate_probability=0.124 if clipped else 0.6,
                 cone_shape_score=0.49 if clipped else 0.7,
                 hue_redness_score=hue, sv_score=sv, roi_support_ratio=0.25,
                 roi_absolute_support=0.05, roi_negative_support=negative,
                 occupancy=0.72 if clipped else 0.50,
                 bbox_width_frac=1.0 if clipped else 0.70,
                 bbox_height_frac=1.0, bbox_bottom_frac=1.0,
                 bbox_height=480, edge_touch_count=4 if clipped else 2,
                 close_region_ok=1, dominant_close=1, close_reached_ok=0,
                 ground_penalty=1)
    with patch('time.time', return_value=now), patch('time.monotonic', return_value=now):
        st.update_imu(angle=heading, angle_valid=True)
        st.update_sonar(sonar_valid=sonar_valid, sonar_distance_cm=distance,
                        sonar_sequence=st.snapshot()['sonar_sequence'] + 1,
                        sonar_observed_at=now, sonar_observed_monotonic=now)
        st.update_cone(cone_valid=valid, cone_probability=(0.0784 if clipped else 0.8)
                       if probability is None else probability,
                       cone_direction=direction, cone_image_direction=direction,
                       cone_is_reached=False, cone_debug=debug,
                       observation_time=now, observation_accepted=True)
    return st.snapshot()


def acquired(st, direction=0.5):
    for now in (100, 100.4, 100.8):
        observe(st, now, direction=direction)


def run_phase(handler, ctrl, now):
    with patch('time.time', return_value=now), patch('time.monotonic', return_value=now):
        handler.execute(ctrl, ctrl.st.snapshot())


class CloseTrackTest(unittest.TestCase):
    def state(self):
        st = CanSatState()
        st.update_navigation(phase=5)
        return st

    def test_identified_centered_cone_survives_full_frame_shape_rejection(self):
        st = self.state()
        acquired(st)
        s = observe(st, 101.2, clipped=True, direction=0.60)
        self.assertEqual(s['cone_probability'], 0.0784)
        self.assertTrue(s['cone_close_track']['hold'])
        self.assertEqual(goal_evidence(s, 101.2, 101.2)[:2],
                         (True, 'close_track_camera_sonar_matched'))

    def test_red_screen_at_startup_never_acquires_identity_even_with_high_score(self):
        st = self.state()
        for i in range(15):
            now = 100 + i * 0.4
            s = observe(st, now, clipped=True, probability=0.98)
            self.assertFalse(goal_evidence(s, now, now)[0])
            self.assertTrue(s['cone_close_track']['hold'])

    def test_single_frame_and_off_axis_history_cannot_authorize_goal(self):
        for count, direction in ((1, 0.5), (3, 0.7)):
            st = self.state()
            for i in range(count):
                observe(st, 100 + i * 0.4, direction=direction)
            now = 100 + count * 0.4
            s = observe(st, now, clipped=True, direction=direction)
            self.assertFalse(goal_evidence(s, now, now)[0])

    def test_different_color_negative_roi_or_turn_cannot_continue(self):
        for change in ({'hue': 0.65}, {'sv': 0.4}, {'negative': 0.2},
                       {'heading': 112}, {'valid': False}, {'sonar_valid': False}):
            with self.subTest(change=change):
                st = self.state()
                acquired(st)
                observe(st, 101.2, clipped=True)
                s = observe(st, 101.6, clipped=True, **change)
                self.assertFalse(goal_evidence(s, 101.6, 101.6)[0])
                self.assertTrue(s['cone_close_track']['hold'])

    def test_live_heading_range_and_camera_age_rechecked_between_frames(self):
        for change in ({'angle': 120}, {'heading_travel_deg': 30},
                       {'sonar_valid': False}, {'sonar_observed_monotonic': 99},
                       {'angle_valid': False}, {'cone_updated_at': 99}):
            st = self.state()
            acquired(st)
            s = observe(st, 101.2, clipped=True)
            s.update(change)
            self.assertFalse(goal_evidence(s, 101.2, 101.2)[0])

    def test_history_expires_without_releasing_stop(self):
        st = self.state()
        acquired(st)
        for i in range(1, 18):
            now = 100.8 + 0.4 * i
            s = observe(st, now, clipped=True, probability=0.98)
        self.assertFalse(goal_evidence(s, now, now)[0])
        self.assertTrue(s['cone_close_track']['hold'])

    def test_turning_away_and_back_between_camera_frames_invalidates_history(self):
        st = self.state()
        acquired(st)
        observe(st, 101.2, clipped=True)
        with patch('time.monotonic', return_value=101.3):
            st.update_imu(angle=112, angle_valid=True)
        s = observe(st, 101.6, clipped=True, heading=100)
        self.assertFalse(goal_evidence(s, 101.6, 101.6)[0])

    def test_full_phase_sequence_requires_two_then_three_stopped_pairs(self):
        ctrl = _VisionController(Phase.PHASE5)
        acquired(ctrl.st)
        for now in (101.2, 101.6):
            observe(ctrl.st, now, clipped=True)
            run_phase(Phase5Handler(), ctrl, now)
        self.assertEqual(ctrl.st.snapshot()['phase'], 6)
        run_phase(Phase6Handler(), ctrl, 101.6)
        for now in (102, 102.4):
            observe(ctrl.st, now, clipped=True)
            run_phase(Phase6Handler(), ctrl, now)
            self.assertEqual(ctrl.mission_end_reason, 'RUNNING')
        # Polling a repeated pair does not manufacture the third confirmation.
        run_phase(Phase6Handler(), ctrl, 102.41)
        self.assertEqual(ctrl.goal_confirm_count, 2)
        observe(ctrl.st, 102.8, clipped=True)
        run_phase(Phase6Handler(), ctrl, 102.8)
        self.assertEqual(ctrl.mission_end_reason, 'GOAL_PROXIMITY_CONFIRMED')
        self.assertEqual(ctrl.st.snapshot()['phase'], 7)

    def test_uncertain_close_object_holds_both_phases_and_motor_even_after_loss(self):
        for phase, handler, drive in ((4, Phase4Handler(), '_drive_phase4_camera'),
                                      (5, Phase5Handler(), '_drive_phase5_camera')):
            ctrl = _VisionController(Phase(phase))
            observe(ctrl.st, 100, clipped=True)
            for i in range(12):
                now = 100.4 + i * 0.4
                s = observe(ctrl.st, now, clipped=True, valid=False, sonar_valid=False)
                run_phase(handler, ctrl, now)
                self.assertEqual(ctrl.st.snapshot()['phase'], phase)
                m = Motor()
                getattr(m, drive)(s)
                self.assertEqual(m.commands, ['stop'])

    def test_goal_range_limits_unchanged_and_phase_exit_clears_history(self):
        st = self.state()
        acquired(st)
        s = observe(st, 101.2, clipped=True, distance=1.9)
        self.assertFalse(goal_evidence(s, 101.2, 101.2)[0])
        st.update_navigation(phase=7)
        self.assertFalse(st.snapshot()['cone_close_track']['hold'])
        st.update_navigation(phase=4)
        s = observe(st, 101.6, clipped=True)
        self.assertFalse(goal_evidence(s, 101.6, 101.6)[0])

    def test_field_log_clipped_frames_retain_identity_without_rewriting_probability(self):
        frames = json.loads((Path(__file__).parent / 'fixtures/close_track_20260922.json').read_text())['frames']
        st = self.state()
        for frame in frames:
            now = frame['time']
            with patch('time.time', return_value=now), patch('time.monotonic', return_value=now):
                st.update_imu(angle=frame['heading'], angle_valid=True)
                st.update_sonar(sonar_distance_cm=frame['distance'], sonar_valid=True,
                                sonar_sequence=frame['sequence'],
                                sonar_observed_at=now - frame['sonar_age'],
                                sonar_observed_monotonic=now - frame['sonar_age'])
                st.update_cone(cone_probability=frame['probability'], cone_valid=True,
                               cone_image_direction=frame['direction'], cone_direction=frame['direction'],
                               cone_is_reached=frame['reached'], cone_debug=frame['debug'],
                               observation_accepted=True, observation_time=frame['camera_time'])
            if frame['sequence'] >= 54:
                with self.subTest(sequence=frame['sequence']):
                    s = st.snapshot()
                    self.assertEqual(s['cone_probability'], 0.0784)
                    self.assertTrue(s['cone_close_track']['hold'])
                    self.assertTrue(goal_evidence(s, now, now)[0])

    def test_brief_invalid_sensor_between_frames_revokes_identity(self):
        for sensor in ('heading', 'range'):
            st = self.state()
            acquired(st)
            observe(st, 101.2, clipped=True)
            if sensor == 'heading':
                st.update_imu(angle_valid=False)
            else:
                st.update_sonar(sonar_valid=False)
            s = observe(st, 101.6, clipped=True, probability=0.98)
            self.assertFalse(goal_evidence(s, 101.6, 101.6)[0])
            self.assertTrue(s['cone_close_track']['hold'])

    def test_continuation_above_three_cm_uses_bounded_p6_pulse_and_live_interlock(self):
        ctrl = _VisionController(Phase.PHASE6)
        acquired(ctrl.st)
        observe(ctrl.st, 101.2, clipped=True, distance=5)
        run_phase(Phase6Handler(), ctrl, 101.2)
        observe(ctrl.st, 101.6, clipped=True, distance=5)
        run_phase(Phase6Handler(), ctrl, 101.6)
        self.assertEqual(ctrl.phase6_stage, 'move')
        self.assertLessEqual(ctrl.phase6_motion_until, 101.75)
        self.assertEqual(ctrl.mission_end_reason, 'RUNNING')
        m = Motor()
        m.phase6_motion_until = ctrl.phase6_motion_until
        with patch('time.time', return_value=101.6), patch('time.monotonic', return_value=101.6):
            m._drive_phase6_approach(ctrl.st.snapshot())
            self.assertEqual(m.commands[-1], 'phase6_range_approach')
            ctrl.st.update_imu(angle_valid=False)
            m._drive_phase6_approach(ctrl.st.snapshot())
            self.assertEqual(m.commands[-1], 'stop')
