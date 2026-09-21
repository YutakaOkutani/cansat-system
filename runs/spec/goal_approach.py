import importlib.util
import unittest
from pathlib import Path
from unittest.mock import patch
from mission.goal import goal_evidence
from mission.const import Phase, GOAL_STOP_DISTANCE_CM
from mission.phases.p5 import Phase5Handler
from mission.phases.p7 import Phase7Handler
from mission.st import CanSatState

path = Path(__file__).resolve().parents[2] / 'mission/mgr/mtr_mgr.py'
spec = importlib.util.spec_from_file_location('goal_motor_under_test', path)
motor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(motor)


def observation(distance=30, now=100):
    return dict(cone_valid=True, cone_sequence=1, cone_updated_at=now,
                cone_direction=0.5, cone_image_direction=0.5, cone_image_direction_valid=True,
                cone_probability=0.8, cone_is_reached=False,
                cone_debug={'strict_red_ok': 1, 'occupancy': 0.02},
                sonar_valid=True, sonar_sequence=1, sonar_distance_cm=distance,
                sonar_observed_at=now, sonar_observed_monotonic=now)


class Motor(motor.MotorManager):
    def __init__(self):
        self.phase6_motion_until = 100.15
        self.commands = []

    def stop_motors(self):
        self.commands.append('stop')

    def set_motors(self, *args, **kwargs):
        self.commands.append(kwargs['cmd_type'])


class GoalApproachTest(unittest.TestCase):
    def test_moderate_visual_size_can_match_without_close_reached(self):
        self.assertTrue(goal_evidence(observation(), 100, 100)[0])

    def test_range_alone_and_incompatible_evidence_never_match(self):
        for changes in (
            {'cone_probability': 0, 'cone_debug': {}},
            {'cone_image_direction': 0.8}, {'cone_image_direction_valid': False},
            {'cone_debug': {'strict_red_ok': 1, 'occupancy': 0.001}},
            {'sonar_observed_at': 99.5}, {'sonar_observed_monotonic': 99},
            {'sonar_observed_monotonic': 101}, {'sonar_distance_cm': float('nan')},
            {'sonar_valid': False}, {'sonar_sequence': 0},
        ):
            with self.subTest(changes=changes):
                self.assertFalse(goal_evidence(dict(observation(), **changes), 100, 100)[0])

    def test_motor_deadline_independently_stops_when_phase_handler_stalls(self):
        ctrl = Motor()
        with patch.object(motor.time, 'time', return_value=100), patch.object(motor.time, 'monotonic', return_value=100):
            ctrl._drive_phase6_approach(observation())
        self.assertEqual(ctrl.commands[-1], 'phase6_range_approach')
        with patch.object(motor.time, 'time', return_value=100.16), patch.object(motor.time, 'monotonic', return_value=100.16):
            ctrl._drive_phase6_approach(observation())
        self.assertEqual(ctrl.commands[-1], 'stop')

    def test_motor_stops_for_near_range_lost_target_stale_echo_or_shutdown(self):
        for changes in ({'sonar_distance_cm': GOAL_STOP_DISTANCE_CM}, {'sonar_valid': False},
                        {'cone_valid': False}, {'sonar_observed_monotonic': 99}):
            ctrl = Motor()
            with patch.object(motor.time, 'time', return_value=100), patch.object(motor.time, 'monotonic', return_value=100):
                ctrl._drive_phase6_approach(dict(observation(), **changes))
            self.assertEqual(ctrl.commands, ['stop'])
        ctrl = Motor()
        ctrl._shutdown_requested = True
        with patch.object(motor.time, 'time', return_value=100), patch.object(motor.time, 'monotonic', return_value=100):
            ctrl._drive_phase6_approach(observation())
        self.assertEqual(ctrl.commands, ['stop'])

    def test_phase5_stops_for_range_confirmation_even_without_close_occupancy(self):
        ctrl = Motor()
        with patch.object(motor.time, 'time', return_value=100), patch.object(motor.time, 'monotonic', return_value=100):
            ctrl._drive_phase5_camera(observation())
        self.assertEqual(ctrl.commands, ['stop'])

    def test_phase7_records_proximity_without_announcing_verified_contact(self):
        from types import SimpleNamespace
        calls = []
        ctrl = SimpleNamespace(devices={}, mission_end_reason='GOAL_PROXIMITY_CONFIRMED',
            stop_motors=lambda: calls.append('stop'),
            _resolve_phase7_arrival_reason=lambda: 'GOAL_PROXIMITY_CONFIRMED',
            request_shutdown=lambda reason: calls.append(reason))
        Phase7Handler().execute(ctrl, {})
        self.assertEqual(calls, ['stop', 'GOAL_PROXIMITY_CONFIRMED'])

class RemovedAvoidanceTest(unittest.TestCase):
    def test_near_echo_never_preempts_phase2_3_4_motion(self):
        from mission.const import PHASE2_STAGE_ESCAPE
        for phase in (Phase.PHASE2, Phase.PHASE3, Phase.PHASE4):
            with self.subTest(phase=phase):
                ctrl = Motor()
                ctrl.st = CanSatState()
                ctrl.st.update_navigation(phase=int(phase))
                ctrl.st.update_sonar(sonar_distance_cm=5, sonar_valid=True)
                ctrl._motor_last_phase = phase
                ctrl.phase2_stage = PHASE2_STAGE_ESCAPE
                ctrl.phase3_heading_entry_ready = True
                ctrl._drive_phase3_navigation = lambda *_: ctrl.commands.append('navigation')
                ctrl._drive_phase4_camera = lambda *_: ctrl.commands.append('search')
                iterations = []
                def end_after_four(_duration):
                    iterations.append(1)
                    if len(iterations) >= 4:
                        ctrl._shutdown_requested = True
                with patch.object(motor.time, 'sleep', side_effect=end_after_four):
                    ctrl.move_motor_thread()
                expected = {Phase.PHASE2: 'phase2_escape_forward', Phase.PHASE3: 'navigation', Phase.PHASE4: 'search'}[phase]
                self.assertEqual(ctrl.commands, [expected] * 4 + ['stop'])
