"""Regression for the 2026-09-28 small-target / short-echo handoff."""
import unittest
from unittest.mock import patch

from mission.const import (
    Phase, PHASE6_APPROACH_TIMEOUT_SEC, GOAL_MAX_ALIGN_PULSES,
    GOAL_MAX_FORWARD_PULSES, GOAL_MAX_REAPPROACHES,
)
from mission.goal import final_entry_evidence
from mission.phases.p5 import Phase5Handler
from mission.phases.p6 import Phase6Handler
from runs.spec.goal_approach import observation, Motor
from runs.spec.phase6_flow import Controller, execute
from runs.spec.cone_phase_diagnostics import _VisionController


class FarTargetRecoveryTest(unittest.TestCase):
    def test_logged_small_centered_target_does_not_enter_p6_or_stop_for_handoff(self):
        # At P6 entry the attached log had occupancy .012168 and sonar 41.90 cm.
        s = observation(distance=41.90)
        s['cone_debug']['occupancy'] = .012168
        self.assertEqual(final_entry_evidence(s, 100, 100)[1], 'small_target_range_unconfirmed')
        c = _VisionController(Phase.PHASE5)
        m = Motor()
        for t in (100, 100.4, 100.8, 101.2):
            c.st.update_cone(cone_direction=.5, cone_image_direction=.5, cone_probability=.41,
                            cone_valid=True, cone_debug=s['cone_debug'], observation_time=t,
                            observation_accepted=True)
            c.st.update_sonar(sonar_distance_cm=41.9, sonar_valid=True,
                             sonar_sequence=c.st.snapshot()['cone_sequence'],
                             sonar_observed_at=t, sonar_observed_monotonic=t)
            with patch('time.time', return_value=t), patch('time.monotonic', return_value=t):
                Phase5Handler().execute(c, c.st.snapshot())
                m._drive_phase5_camera(c.st.snapshot())
            self.assertEqual(c.st.snapshot()['phase'], 5)
            self.assertEqual(m.commands[-1], 'phase5_approach_forward')

    def test_existing_large_and_near_entry_paths_remain_available(self):
        for distance, occupancy, reached in ((60, .02, False), (30, .012, False),
                                              (50, .012, True)):
            s = observation(distance=distance)
            s['cone_debug']['occupancy'] = occupancy
            s['cone_is_reached'] = reached
            self.assertTrue(final_entry_evidence(s, 100, 100)[0])

    def far(self, c, t, distance=86, direction=.665625):
        c.observe(t, distance=distance, direction=direction, reached=False)
        # Same modest visual footprint as the failed run; not a near/clipped cone.
        c.st.update_cone(cone_debug={'strict_red_ok': 1, 'occupancy': .012})
        execute(c, t)

    def recovered(self):
        c = Controller()
        for t in (100, 105, 105.5, 106.1):
            self.far(c, t)
        return c

    def test_repeated_far_off_axis_evidence_returns_to_p5_stopped(self):
        c = self.recovered()
        self.assertEqual(c.st.snapshot()['phase'], 5)
        self.assertEqual(c.phase6_gate, 'reapproach_p5')
        self.assertEqual(c.phase6_motion_until, 0)
        self.assertEqual(c.goal_confirm_count, 0)
        self.assertEqual(c.mission_end_reason, 'RUNNING')
        self.assertEqual(c.phase6_reapproaches, 1)

    def test_repeated_pair_or_single_far_outlier_cannot_release_to_p5(self):
        for case in ('repeated', 'outlier', 'jump', 'near_visual', 'invalid', 'stale', 'skew'):
            with self.subTest(case=case):
                c = Controller()
                self.far(c, 100)
                if case == 'repeated':
                    self.far(c, 105.9)
                    execute(c, 106)
                    execute(c, 106.1)
                else:
                    for t in (105, 105.5, 106.1):
                        c.observe(t, distance=86, direction=.65, reached=False)
                        if case == 'outlier' and t < 106:
                            c.st.update_sonar(sonar_valid=False)
                        elif case == 'jump':
                            c.st.update_sonar(sonar_distance_cm=300 if t == 105.5 else 86)
                        elif case == 'near_visual':
                            c.st.update_cone(cone_is_reached=True)
                        elif case == 'invalid':
                            c.st.update_sonar(sonar_valid=False)
                        elif case == 'stale':
                            c.st.update_sonar(sonar_observed_monotonic=t-1)
                        elif case == 'skew':
                            c.st.update_sonar(sonar_observed_at=t-.4)
                        execute(c, t)
                self.assertEqual(c.st.snapshot()['phase'], 6)
                self.assertEqual(c.mission_end_reason, 'RUNNING')
                c.observe(120.1, distance=86, reached=False)
                c.st.update_sonar(sonar_valid=False)
                execute(c, 120.1)
                self.assertEqual(c.mission_end_reason, 'GOAL_PROXIMITY_UNCONFIRMED')
                self.assertEqual(c.phase6_motion_until, 0)

    def test_reentry_keeps_elapsed_pulse_and_retry_budgets(self):
        c = self.recovered()
        c.phase6_pulses = 10
        c.phase6_forward_pulses = 7
        c.phase6_align_pulses = 3
        used = c.phase6_elapsed_used
        c.phase_entry_time = 200
        c.st.update_navigation(phase=6)
        c.observe(200, distance=30)
        execute(c, 200)
        self.assertAlmostEqual(200-c.phase6_start_time, used)
        self.assertEqual((c.phase6_pulses, c.phase6_forward_pulses, c.phase6_align_pulses), (10, 7, 3))
        self.assertEqual(c.phase6_reapproaches, 1)
        execute(c, 200 + PHASE6_APPROACH_TIMEOUT_SEC-used+.01)
        self.assertEqual(c.mission_end_reason, 'GOAL_APPROACH_TIMEOUT')

    def test_retry_limit_is_terminal_and_never_success(self):
        c = Controller()
        c.phase6_reapproaches = GOAL_MAX_REAPPROACHES
        for t in (100, 105, 105.5, 106.1):
            self.far(c, t)
        self.assertEqual(c.mission_end_reason, 'GOAL_REAPPROACH_LIMIT')
        self.assertEqual(c.phase6_motion_until, 0)

    def test_slow_forward_progress_can_exceed_old_twelve_pulse_limit(self):
        c = Controller()
        c.observe(100, distance=30)
        execute(c, 100)
        distance = 30
        previous = 0
        for i in range(1, 100):
            t = 100+i*.4
            c.observe(t, distance=distance)
            execute(c, t)
            if c.phase6_pulses > previous:
                previous = c.phase6_pulses
                distance -= 1
            if previous >= 13:
                break
        self.assertEqual(c.phase6_pulses, 13)
        self.assertEqual(c.mission_end_reason, 'RUNNING')

    def test_action_limits_do_not_spend_each_others_allowance(self):
        for action, field, limit in (('forward', 'phase6_forward_pulses', GOAL_MAX_FORWARD_PULSES),
                                     ('right', 'phase6_align_pulses', GOAL_MAX_ALIGN_PULSES)):
            c = Controller()
            c.observe(100, distance=30)
            execute(c, 100)
            setattr(c, field, limit)
            c.phase6_pulses = limit
            Phase6Handler()._pulse(c, 101, 30, action)
            self.assertEqual(c.mission_end_reason, 'GOAL_MOTION_LIMIT')
            self.assertEqual(c.phase6_motion_until, 0)
        c = Controller()
        c.observe(100, distance=30)
        execute(c, 100)
        c.phase6_align_pulses = GOAL_MAX_ALIGN_PULSES
        c.phase6_pulses = GOAL_MAX_ALIGN_PULSES
        Phase6Handler()._pulse(c, 101, 30, 'forward')
        self.assertEqual(c.phase6_forward_pulses, 1)
        self.assertEqual(c.mission_end_reason, 'RUNNING')
