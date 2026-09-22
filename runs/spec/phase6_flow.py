import sys
import unittest
from pathlib import Path
from unittest.mock import patch

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mission.const import GOAL_STOP_DISTANCE_CM, PHASE6_APPROACH_TIMEOUT_SEC, Phase
from mission.phases.p6 import Phase6Handler
from mission.st import CanSatState


class Controller:
    def __init__(self):
        self.devices = {}
        self.st = CanSatState()
        self.st.update_navigation(phase=int(Phase.PHASE6))
        self.phase_entry_time = 1.0
        self.mission_end_reason = 'RUNNING'
        self.mission_total_timeout_triggered = False
        self.stop_calls = 0

    def stop_motors(self):
        self.stop_calls += 1

    def observe(self, now, distance=2.01, direction=0.5, reached=True):
        self.st.update_cone(cone_direction=direction, cone_image_direction=direction, cone_probability=0.8,
                            cone_is_reached=reached, cone_valid=True,
                            cone_debug={'strict_red_ok': 1, 'occupancy': 0.1},
                            observation_time=now, observation_accepted=True)
        self.st.update_sonar(sonar_distance_cm=distance, sonar_valid=True,
                             sonar_sequence=self.st.snapshot()['cone_sequence'],
                             sonar_observed_at=now, sonar_observed_monotonic=now)


def execute(controller, now):
    with patch('mission.phases.p6.time.time', return_value=now), patch('mission.phases.p6.time.monotonic', return_value=now):
        Phase6Handler().execute(controller, controller.st.snapshot())


class Phase6FlowTest(unittest.TestCase):
    def test_stopped_new_observations_confirm_proximity_not_contact(self):
        ctrl = Controller()
        ctrl.observe(100)
        execute(ctrl, 100)
        self.assertEqual(ctrl.mission_end_reason, 'RUNNING')
        for now in (100.4, 100.6):
            ctrl.observe(now)
            execute(ctrl, now)
            self.assertEqual(ctrl.mission_end_reason, 'RUNNING')
        ctrl.observe(100.8)
        execute(ctrl, 100.8)
        self.assertEqual(ctrl.mission_end_reason, 'GOAL_PROXIMITY_CONFIRMED')
        self.assertEqual(ctrl.st.snapshot()['phase'], int(Phase.PHASE7))
        self.assertEqual(ctrl.phase6_motion_until, 0)

    def test_camera_at_1_7_fps_with_faster_sonar_confirms_only_new_frames(self):
        ctrl = Controller()
        ctrl.observe(100, distance=2.01)
        execute(ctrl, 100)
        for index in range(1, 10):
            now = 100 + index * 0.2
            if index % 3 == 0:
                ctrl.observe(now, distance=2.01)
            ctrl.st.update_sonar(sonar_distance_cm=2.01, sonar_valid=True,
                                 sonar_sequence=100 + index,
                                 sonar_observed_at=now, sonar_observed_monotonic=now)
            execute(ctrl, now)
            if index < 9:
                self.assertEqual(ctrl.mission_end_reason, 'RUNNING')
        self.assertEqual(ctrl.mission_end_reason, 'GOAL_PROXIMITY_CONFIRMED')

    def test_five_cm_now_requires_further_approach(self):
        ctrl = Controller()
        ctrl.observe(100, distance=5)
        execute(ctrl, 100)
        ctrl.observe(100.4, distance=5)
        execute(ctrl, 100.4)
        self.assertEqual(ctrl.phase6_motion_until, 0)
        ctrl.observe(100.6, distance=5)
        execute(ctrl, 100.6)
        self.assertEqual(ctrl.phase6_stage, 'move')
        self.assertEqual(ctrl.mission_end_reason, 'RUNNING')

    def test_repeated_frame_or_echo_cannot_confirm(self):
        ctrl = Controller()
        ctrl.observe(100)
        execute(ctrl, 100)
        ctrl.observe(100.4)
        execute(ctrl, 100.4)
        for now in (100.41, 100.42, 100.43):
            execute(ctrl, now)
        self.assertEqual(ctrl.goal_confirm_count, 1)
        self.assertEqual(ctrl.mission_end_reason, 'RUNNING')

    def test_far_range_authorizes_only_a_bounded_pulse(self):
        ctrl = Controller()
        ctrl.observe(100, distance=30)
        execute(ctrl, 100)
        ctrl.observe(100.4, distance=30)
        execute(ctrl, 100.4)
        ctrl.observe(100.8, distance=30)
        execute(ctrl, 100.8)
        self.assertGreater(ctrl.phase6_motion_until, 100.8)
        self.assertLess(ctrl.phase6_motion_until, 101)
        ctrl.observe(101, distance=28)
        execute(ctrl, 101)
        self.assertEqual(ctrl.phase6_motion_until, 0)
        self.assertEqual(ctrl.mission_end_reason, 'RUNNING')

    def test_close_range_interrupts_pulse_and_waits_for_stopped_samples(self):
        ctrl = Controller()
        ctrl.observe(100, distance=30)
        execute(ctrl, 100)
        ctrl.observe(100.4, distance=30)
        execute(ctrl, 100.4)
        ctrl.observe(100.6, distance=30)
        execute(ctrl, 100.6)
        self.assertGreater(ctrl.phase6_motion_until, 100.6)
        ctrl.observe(100.65, distance=GOAL_STOP_DISTANCE_CM)
        execute(ctrl, 100.65)
        self.assertEqual(ctrl.phase6_motion_until, 0)
        self.assertEqual(ctrl.goal_confirm_count, 0)

    def test_missing_stale_off_axis_or_blind_zone_stops(self):
        for case in ('missing', 'stale', 'off_axis', 'blind_zone'):
            with self.subTest(case=case):
                ctrl = Controller()
                ctrl.observe(100, distance=30)
                execute(ctrl, 100)
                ctrl.observe(100.4, distance=30)
                execute(ctrl, 100.4)
                if case == 'missing':
                    ctrl.st.update_sonar(sonar_valid=False)
                elif case == 'off_axis':
                    ctrl.observe(100.5, direction=0.8)
                elif case == 'blind_zone':
                    ctrl.observe(100.5, distance=0.5)
                execute(ctrl, 101 if case == 'stale' else 100.5)
                self.assertEqual(ctrl.phase6_motion_until, 0)
                self.assertEqual(ctrl.goal_confirm_count, 0)
                execute(ctrl, 104)
                self.assertEqual(ctrl.mission_end_reason, 'RUNNING')
                self.assertEqual(ctrl.st.snapshot()['phase'], int(Phase.PHASE6))
                execute(ctrl, 107)
                self.assertEqual(ctrl.mission_end_reason, 'GOAL_CAMERA_TIMEOUT')
                self.assertEqual(ctrl.st.snapshot()['phase'], int(Phase.PHASE7))

    def test_timeout_is_not_success(self):
        for global_timeout in (False, True):
            ctrl = Controller()
            ctrl.observe(100)
            execute(ctrl, 100)
            ctrl.mission_total_timeout_triggered = global_timeout
            execute(ctrl, 100 + PHASE6_APPROACH_TIMEOUT_SEC)
            self.assertEqual(ctrl.mission_end_reason, 'MISSION_TOTAL_TIMEOUT' if global_timeout else 'GOAL_APPROACH_TIMEOUT')
            self.assertEqual(ctrl.phase6_motion_until, 0)

    def test_old_before_stop_frame_is_not_confirmation(self):
        ctrl = Controller()
        ctrl.observe(100)
        execute(ctrl, 100)
        execute(ctrl, 100.4)
        self.assertEqual(ctrl.goal_confirm_count, 0)


if __name__ == '__main__':
    unittest.main()
