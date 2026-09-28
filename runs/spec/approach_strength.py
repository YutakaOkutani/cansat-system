"""P6 integration tests use the real motor mapper with fake GPIO and clocks."""
from dataclasses import replace
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from mission.approach import DEFAULT_APPROACH, motion_profile, output_seconds
from mission.const import (DEVICE_MOTOR_1_PWM, DEVICE_MOTOR_2_PWM, DEVICE_MOTOR_1_DIR,
    DEVICE_MOTOR_2_DIR, MISSION_TIMEOUT_TOTAL, MISSION_PHASE3_CUMULATIVE_BUDGET,
    MISSION_PHASE4_CUMULATIVE_BUDGET, MISSION_PHASE5_CUMULATIVE_BUDGET,
    TIMEOUT_PHASE_4, TIMEOUT_PHASE_5, PHASE6_APPROACH_TIMEOUT_SEC)
from mission.goal import goal_evidence
from mission.phases.p6 import Phase6Handler
from runs.spec.phase6_flow import Controller
from runs.spec.goal_approach import motor


class Pin:
    value = 0.0


class DrivenController(Controller, motor.MotorManager):
    def __init__(self):
        super().__init__()
        self.devices = {key: Pin() for key in (DEVICE_MOTOR_1_PWM, DEVICE_MOTOR_2_PWM,
                                              DEVICE_MOTOR_1_DIR, DEVICE_MOTOR_2_DIR)}
        self.motor_state = {}
        self.mission_start_time = 100

    def stop_motors(self, reason='unspecified'):
        motor.MotorManager.stop_motors(self, reason)

    def tick(self, t, distance=30, direction=.5, *, heading=None, phase=True, camera=True):
        with patch('time.time', return_value=t), patch('time.monotonic', return_value=t):
            if camera:
                self.observe(t, distance=distance, direction=direction, reached=False)
            else:
                self.st.update_sonar(sonar_distance_cm=distance, sonar_valid=True,
                    sonar_sequence=self.st.snapshot()['sonar_sequence']+1,
                    sonar_observed_at=t, sonar_observed_monotonic=t)
            if heading is not None:
                self.st.update_imu(angle=heading, angle_valid=True)
            if phase:
                Phase6Handler().execute(self, self.st.snapshot())
            self._drive_phase6_approach(self.st.snapshot())

    def output(self):
        return tuple(self.devices[k].value*100 for k in (DEVICE_MOTOR_1_PWM, DEVICE_MOTOR_2_PWM))


def started(distance=30, direction=.5, heading=None):
    c = DrivenController()
    for t in (100, 100.4, 100.8):
        c.tick(t, distance, direction, heading=heading)
    return c


class ApproachStrengthTest(unittest.TestCase):
    def test_twenty_minute_budget_preserves_navigation_reserve(self):
        self.assertEqual(MISSION_TIMEOUT_TOTAL, 1200)
        self.assertEqual((TIMEOUT_PHASE_4, MISSION_PHASE4_CUMULATIVE_BUDGET), (120, 180))
        self.assertEqual((TIMEOUT_PHASE_5, MISSION_PHASE5_CUMULATIVE_BUDGET), (180, 240))
        self.assertEqual(PHASE6_APPROACH_TIMEOUT_SEC, 180)
        self.assertEqual(MISSION_PHASE3_CUMULATIVE_BUDGET, 375)

    def test_far_forward_actually_outputs_70_percent_for_longer_than_old_pulse(self):
        c = started()
        self.assertEqual(c.output(), (70, 70))
        c.tick(101.16, camera=False, phase=False)
        self.assertEqual(c.output(), (70, 70))
        # A fresh echo with a .36 s older camera may sustain this fixed lease,
        # but is never valid for a new motion request or a successful arrival.
        self.assertFalse(goal_evidence(c.st.snapshot(), 101.16, 101.16)[0])
        c.tick(101.21, camera=False, phase=False)
        self.assertEqual(c.output(), (0, 0))
        self.assertAlmostEqual(output_seconds(c, 'forward', 101.21), .41)
        c.tick(101.22, phase=False)
        self.assertEqual(c.output(), (0, 0))  # Cannot restart the canceled lease.

    def test_each_closer_band_interrupts_without_waiting_for_phase_handler(self):
        for initial, closer in ((30, 20), (15, 8), (5, 3)):
            c = started(initial)
            c.tick(100.82, closer, phase=False)
            self.assertEqual(c.output(), (0, 0))
            self.assertEqual(c.phase6_motion_until, 0)
            self.assertEqual(c.mission_end_reason, 'RUNNING')

    def test_near_motion_and_success_remain_short_and_stopped(self):
        c = started(5)
        self.assertAlmostEqual(c.phase6_requested_duration, .05)
        self.assertEqual(c.output(), (65, 65))
        c.tick(100.83, 2.9)
        self.assertEqual(c.output(), (0, 0))
        self.assertEqual(c.goal_confirm_count, 0)
        for t in (101.2, 101.4, 101.6):
            c.tick(t, 2.9)
            self.assertEqual(c.output(), (0, 0))
        self.assertEqual(c.mission_end_reason, 'GOAL_PROXIMITY_CONFIRMED')

    def test_far_pivot_is_stronger_and_expected_rotation_does_not_trigger_shock_stop(self):
        c = started(30, .7, heading=100)
        self.assertEqual(c.output(), (65, 0))
        c.tick(100.85, 30, .68, heading=108)
        self.assertEqual(c.output(), (65, 0))
        c.tick(100.9, 30, .65, heading=116, phase=False)
        self.assertEqual(c.output(), (0, 0))
        self.assertEqual(c.phase6_motor_gate, 'turn_angle_limit')

    def test_centered_image_stops_turn_and_forward_still_stops_for_shock(self):
        c = started(30, .7)
        c.tick(100.85, 30, .5, phase=False)
        self.assertEqual(c.output(), (0, 0))
        c = started(30, heading=100)
        c.tick(100.85, 30, heading=108, phase=False)
        self.assertEqual(c.output(), (0, 0))
        self.assertEqual(c.phase6_motor_gate, 'heading_changed')

    def test_near_pivot_keeps_original_output_and_no_turn_inside_six_cm(self):
        c = started(15, .7)
        self.assertEqual(c.output(), (45, 0))
        self.assertAlmostEqual(c.phase6_requested_duration, .05)
        c.tick(100.82, 5, .7, phase=False)
        self.assertEqual(c.output(), (0, 0))

    def test_original_near_sync_gate_and_six_cm_alignment_boundary_are_retained(self):
        c = started(5)
        snapshot = dict(c.st.snapshot(), cone_updated_at=100.51,
                        sonar_observed_at=100.84, sonar_observed_monotonic=100.84)
        with patch('time.time', return_value=100.84), patch('time.monotonic', return_value=100.84):
            c._drive_phase6_approach(snapshot)
        self.assertEqual(c.output(), (0, 0))
        self.assertEqual(c.phase6_motor_gate, 'evidence:observations_not_aligned')
        c = started(6, .7)
        self.assertEqual(c.output(), (45, 0))

    def test_invalid_or_stale_observations_stop_mid_pulse(self):
        for changes in ({'sonar_valid': False}, {'cone_valid': False},
                        {'sonar_observed_monotonic': 99}, {'cone_updated_at': 99}):
            c = started()
            snapshot = dict(c.st.snapshot(), **changes)
            with patch('time.time', return_value=100.9), patch('time.monotonic', return_value=100.9):
                c._drive_phase6_approach(snapshot)
            self.assertEqual(c.output(), (0, 0))

    def test_forward_boost_requires_actual_output_and_remains_bounded(self):
        c = DrivenController()
        for i in range(701):
            c.tick(100 + i*.05, 30)
            if c.mission_end_reason != 'RUNNING':
                break
        self.assertTrue(c.phase6_forward_boost)
        self.assertEqual(c.phase6_requested_pwm, 75)
        self.assertAlmostEqual(c.phase6_requested_duration, .6)
        self.assertEqual(c.mission_end_reason, 'GOAL_NO_PROGRESS')
        self.assertGreaterEqual(output_seconds(c, 'forward', 100+i*.05), 6)
        self.assertEqual(c.output(), (0, 0))

    def test_turn_boost_uses_image_progress_and_has_finite_failure(self):
        c = DrivenController()
        for i in range(501):
            c.tick(100+i*.05, 30, .7)
            if c.mission_end_reason != 'RUNNING':
                break
        self.assertTrue(c.phase6_align_boost)
        self.assertEqual(c.phase6_requested_pwm, 75)
        self.assertAlmostEqual(c.phase6_requested_duration, .30)
        self.assertEqual(c.mission_end_reason, 'GOAL_NO_PROGRESS')

    def test_output_budget_stops_from_motor_worker_without_new_phase_tick(self):
        c = DrivenController()
        c.mission_config = SimpleNamespace(approach=replace(DEFAULT_APPROACH, forward_output_limit_sec=.2))
        for t in (100, 100.4, 100.8):
            c.tick(t)
        self.assertAlmostEqual(c.phase6_requested_duration, .2)
        c.tick(101.01, phase=False)
        self.assertEqual(c.output(), (0, 0))
        for t in (101.1, 101.5, 101.9):
            c.tick(t)
        self.assertEqual(c.mission_end_reason, 'GOAL_OUTPUT_TIME_LIMIT')

    def test_motor_independently_obeys_mission_and_p6_deadlines(self):
        for mission in (True, False):
            c = started()
            if mission:
                c.mission_start_time = 100.9 - 1200
            else:
                c.phase6_start_time = 100.9 - 180
            c.tick(100.9, phase=False)
            self.assertEqual(c.output(), (0, 0))
            self.assertEqual(c.phase6_motor_gate, 'mission_timeout' if mission else 'approach_timeout')

    def test_settings_control_real_far_output_and_progress_does_not_boost(self):
        c = DrivenController()
        c.mission_config = SimpleNamespace(approach=replace(DEFAULT_APPROACH, far_forward_pwm=72))
        distance, previous = 55, 0
        for i in range(201):
            c.tick(100+i*.05, distance)
            if getattr(c, 'phase6_pulses', 0) > previous:
                previous = c.phase6_pulses
                self.assertEqual(c.output(), (72, 72))
                distance -= 1
        self.assertGreater(output_seconds(c, 'forward', 110), 3)
        self.assertFalse(getattr(c, 'phase6_forward_boost', False))
        self.assertEqual(c.mission_end_reason, 'RUNNING')

    def test_distance_bands_have_hysteresis_and_near_never_boosts_forward(self):
        c = SimpleNamespace(phase6_forward_boost=True)
        self.assertEqual(motion_profile(c, 8, 'forward')[:2], (.05, 45))
        self.assertEqual(motion_profile(c, 9, 'forward')[:2], (.05, 45))
        self.assertEqual(motion_profile(c, 10, 'forward')[0], .25)
        self.assertEqual(motion_profile(c, 21, 'forward')[0], .25)
        self.assertEqual(motion_profile(c, 22, 'forward')[:2], (.6, 75))
        self.assertEqual(motion_profile(c, 20, 'forward')[0], .25)
