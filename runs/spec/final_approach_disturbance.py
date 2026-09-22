"""Final approach under rocking, range outliers and acquisition stalls."""
import unittest
from unittest.mock import patch

from mission.goal import goal_evidence
from mission.phases.p5 import Phase5Handler
from mission.phases.p6 import Phase6Handler
from mission.const import Phase
from runs.spec.close_track import observe, acquired, run_phase
from runs.spec.cone_phase_diagnostics import _VisionController
from runs.spec.phase6_flow import Controller, execute
from runs.spec.goal_approach import Motor


class FinalDisturbanceTest(unittest.TestCase):
    def test_center_at_29cm_survives_one_invalid_echo(self):
        c = _VisionController(Phase.PHASE5)
        for t in (100, 100.4, 100.8):
            observe(c.st, t, distance=29)
        observe(c.st, 101.2, clipped=True, sonar_valid=False)
        self.assertFalse(goal_evidence(c.st.snapshot(), 101.2, 101.2)[0])
        s = observe(c.st, 101.6, clipped=True, direction=0.60)
        self.assertTrue(goal_evidence(s, 101.6, 101.6)[0])

    def test_color_dropout_is_not_a_vote_and_recovers_without_new_identity(self):
        c = _VisionController(Phase.PHASE5)
        acquired(c.st)
        s = observe(c.st, 101.2, clipped=True, hue=0.65)
        self.assertFalse(goal_evidence(s, 101.2, 101.2)[0])
        s = observe(c.st, 101.6, clipped=True)
        self.assertTrue(goal_evidence(s, 101.6, 101.6)[0])

    def test_small_heading_oscillations_do_not_accumulate_into_target_loss(self):
        c = _VisionController(Phase.PHASE5)
        acquired(c.st)
        for i in range(10):
            with patch('time.monotonic', return_value=100.9 + i * .02):
                c.st.update_imu(angle=102 if i % 2 else 98, angle_valid=True)
        s = observe(c.st, 101.2, clipped=True)
        self.assertTrue(goal_evidence(s, 101.2, 101.2)[0])

    def test_near_off_axis_target_hands_over_before_continuous_steering(self):
        c = _VisionController(Phase.PHASE5)
        for t in (100, 100.4):
            s = observe(c.st, t, distance=20, direction=0.65)
            run_phase(Phase5Handler(), c, t)
            m = Motor()
            with patch('time.time', return_value=t), patch('time.monotonic', return_value=t):
                m._drive_phase5_camera(s)
            self.assertEqual(m.commands[-1], 'stop')
        self.assertEqual(c.st.snapshot()['phase'], 6)

    def test_three_near_votes_with_one_boundary_outlier_confirm_while_stopped(self):
        c = Controller()
        c.observe(100); execute(c, 100)
        for t, distance in ((100.4, 2.8), (100.8, 3.2), (101.2, 2.9)):
            c.observe(t, distance=distance); execute(c, t)
            self.assertEqual(c.phase6_motion_until, 0)
            self.assertEqual(c.mission_end_reason, 'RUNNING')
        c.observe(101.6, distance=2.85); execute(c, 101.6)
        self.assertEqual(c.mission_end_reason, 'GOAL_PROXIMITY_CONFIRMED')

    def test_far_latest_value_cannot_complete_near_votes(self):
        c = Controller()
        c.observe(100); execute(c, 100)
        for t, distance in ((100.4, 2.8), (100.8, 2.9), (101.2, 3.2)):
            c.observe(t, distance=distance); execute(c, t)
        self.assertEqual(c.mission_end_reason, 'RUNNING')
        self.assertEqual(c.phase6_motion_until, 0)

    def test_single_far_outlier_never_commands_motion(self):
        c = Controller()
        c.observe(100, distance=4.5); execute(c, 100)
        c.observe(100.4, distance=4.5); execute(c, 100.4)
        self.assertEqual(c.phase6_motion_until, 0)
        c.observe(100.8, distance=4.5); execute(c, 100.8)
        self.assertGreater(c.phase6_motion_until, 100.8)
        self.assertLessEqual(c.phase6_motion_until, 100.95)

    def test_camera_stall_has_bounded_terminal_failure(self):
        c = Controller()
        c.observe(100); execute(c, 100)
        execute(c, 102)
        self.assertEqual(c.phase6_motion_until, 0)
        execute(c, 107)
        self.assertEqual(c.st.snapshot()['phase'], 7)
        self.assertEqual(c.mission_end_reason, 'GOAL_CAMERA_TIMEOUT')

    def test_shaking_after_two_votes_discards_pre_shake_votes(self):
        c = Controller()
        for t, heading in ((100, 100), (100.4, 100), (100.8, 100), (101.2, 112)):
            c.observe(t)
            with patch('time.monotonic', return_value=t):
                c.st.update_imu(angle=heading, angle_valid=True)
            execute(c, t)
        self.assertEqual(c.mission_end_reason, 'RUNNING')
        self.assertEqual(c.goal_confirm_count, 0)

    def test_turn_away_and_back_between_camera_frames_revokes_identity(self):
        c = _VisionController(Phase.PHASE5)
        acquired(c.st)
        for t, heading in ((100.9, 114), (101, 100)):
            with patch('time.monotonic', return_value=t):
                c.st.update_imu(angle=heading, angle_valid=True)
        s = observe(c.st, 101.2, clipped=True)
        self.assertFalse(goal_evidence(s, 101.2, 101.2)[0])

    def test_large_red_surface_low_roi_requires_prior_identification(self):
        for prior_identity in (False, True):
            c = _VisionController(Phase.PHASE5)
            if prior_identity:
                acquired(c.st)
            s = observe(c.st, 101.2, clipped=True)
            debug = dict(s['cone_debug'], roi_support_ratio=.02)
            with patch('time.time', return_value=101.2), patch('time.monotonic', return_value=101.2):
                c.st.update_cone(cone_debug=debug, cone_valid=True,
                                observation_time=101.2, observation_accepted=True)
            self.assertEqual(goal_evidence(c.st.snapshot(), 101.2, 101.2)[0], prior_identity)

    def test_motion_resets_votes_and_no_progress_has_finite_exit(self):
        c = Controller()
        c.observe(100, distance=4.5); execute(c, 100)
        now = 100
        for _ in range(40):
            now += .4
            c.observe(now, distance=4.5); execute(c, now)
            self.assertEqual(c.goal_confirm_count, 0)
            if c.st.snapshot()['phase'] == 7:
                break
        self.assertEqual(c.mission_end_reason, 'GOAL_NO_PROGRESS')
        self.assertEqual(c.phase6_motion_until, 0)

    def test_new_good_camera_with_no_echo_exits_unconfirmed(self):
        c = Controller()
        c.observe(100); execute(c, 100)
        for now in (100.4, 102, 104, 106.1):
            c.observe(now)
            c.st.update_sonar(sonar_valid=False)
            execute(c, now)
        self.assertEqual(c.mission_end_reason, 'GOAL_PROXIMITY_UNCONFIRMED')
        self.assertEqual(c.phase6_motion_until, 0)

    def test_expired_votes_cannot_be_carried_into_recovery(self):
        c = Controller()
        for now in (100, 100.4, 100.8, 104.1):
            c.observe(now); execute(c, now)
        self.assertEqual(c.goal_confirm_count, 1)
        self.assertEqual(c.mission_end_reason, 'RUNNING')

    def test_large_range_jump_restarts_settling(self):
        c = Controller()
        for now, d in ((100, 2.8), (100.4, 2.8), (100.8, 2.9), (101.2, 20)):
            c.observe(now, distance=d); execute(c, now)
        self.assertEqual(c.goal_confirm_count, 0)
        self.assertEqual(c.phase6_motion_until, 0)
        self.assertEqual(c.goal_decision, 'settling_after_range_jump')

    def test_off_axis_alignment_is_bounded_and_never_a_goal_vote(self):
        c = Controller()
        for now in (100, 100.4, 100.8):
            c.observe(now, distance=20, direction=.65); execute(c, now)
        self.assertEqual(c.phase6_action, 'right')
        self.assertLessEqual(c.phase6_motion_until, 100.86)
        self.assertEqual(c.goal_confirm_count, 0)
        m = Motor()
        m.phase6_action = c.phase6_action
        m.phase6_motion_until = c.phase6_motion_until
        with patch('time.time', return_value=100.8), patch('time.monotonic', return_value=100.8):
            m._drive_phase6_approach(c.st.snapshot())
            self.assertEqual(m.commands[-1], 'phase6_align_right')
            c.observe(100.8, distance=5, direction=.65)
            m._drive_phase6_approach(c.st.snapshot())
            self.assertEqual(m.commands[-1], 'stop')
        with patch('time.time', return_value=100.9), patch('time.monotonic', return_value=100.9):
            m._drive_phase6_approach(c.st.snapshot())
            self.assertEqual(m.commands[-1], 'stop')

    def test_alignment_has_total_pulse_budget(self):
        c = Controller()
        c.observe(100, distance=20, direction=.65); execute(c, 100)
        now = 100
        for _ in range(110):
            now += .31
            c.observe(now, distance=20, direction=.65); execute(c, now)
            if c.st.snapshot()['phase'] == 7:
                break
        self.assertEqual(c.mission_end_reason, 'GOAL_MOTION_LIMIT')
        self.assertEqual(c.phase6_motion_until, 0)

    def test_imu_excursion_between_phase_ticks_discards_votes(self):
        c = Controller()
        for now in (100, 100.4, 100.8):
            c.observe(now)
            with patch('time.monotonic', return_value=now):
                c.st.update_imu(angle=100, angle_valid=True)
            execute(c, now)
        self.assertEqual(c.goal_confirm_count, 2)
        for now, heading in ((100.9, 112), (101, 100)):
            with patch('time.monotonic', return_value=now):
                c.st.update_imu(angle=heading, angle_valid=True)
        c.observe(101.2); execute(c, 101.2)
        self.assertEqual(c.goal_confirm_count, 0)
        self.assertEqual(c.phase6_motion_until, 0)
        self.assertEqual(c.mission_end_reason, 'RUNNING')

    def test_center_seen_during_identity_acquisition_is_retained(self):
        c = _VisionController(Phase.PHASE5)
        for now, direction in ((100, .5), (100.4, .60), (100.8, .62)):
            observe(c.st, now, distance=29, direction=direction)
        s = observe(c.st, 101.2, clipped=True, direction=.62)
        self.assertTrue(goal_evidence(s, 101.2, 101.2)[0])

    def test_alignment_respects_existing_camera_control_direction_mapping(self):
        c = Controller()
        for now in (100, 100.4, 100.8):
            c.observe(now, distance=20, direction=.65)
            c.st.update_cone(cone_direction=.35)
            execute(c, now)
        self.assertEqual(c.phase6_action, 'left')
        m = Motor()
        m.phase6_action = c.phase6_action
        m.phase6_motion_until = c.phase6_motion_until
        with patch('time.time', return_value=100.8), patch('time.monotonic', return_value=100.8):
            m._drive_phase6_approach(c.st.snapshot())
        self.assertEqual(m.commands[-1], 'phase6_align_left')

    def test_motor_stops_on_imu_shock_without_waiting_for_phase_tick(self):
        from runs.spec.goal_approach import observation
        m = Motor()
        m.phase6_motion_started = 100
        with patch('time.time', return_value=100.05), patch('time.monotonic', return_value=100.05):
            m._drive_phase6_approach(dict(observation(now=100.05), angle_motion_monotonic=100.04))
        self.assertEqual(m.commands[-1], 'stop')
