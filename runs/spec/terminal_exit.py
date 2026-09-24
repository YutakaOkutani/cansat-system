"""Phase7 is terminal for every outcome, including blocked driver cleanup."""
import subprocess
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from mission.const import Phase, PHASE7_ERROR_EXIT_CODE, RECOVERY_ERROR_EXIT_CODE
from mission.phases.p7 import Phase7Handler
from mission.shutdown import ProcessExitDeadline
from runs.spec.controller_exception_safety import CanSatController


class TerminalController(CanSatController):
    def __init__(self, reason, phase=6):
        self.current_phase = phase
        self.st = SimpleNamespace(snapshot=lambda: {'phase': self.current_phase})
        self.devices = {}
        self.mission_end_reason = reason
        self.phase7_arrival_reason = 'RUNNING'
        self.mission_total_timeout_triggered = reason == 'MISSION_TOTAL_TIMEOUT'
        self.mission_start_time = 0
        self._shutdown_requested = False
        self.led_blink_timer = 0
        self.events = []
        self.phase_handlers = {
            Phase.PHASE6: SimpleNamespace(execute=self.to_terminal),
            Phase.PHASE7: Phase7Handler(),
        }

    def to_terminal(self, *_):
        self.current_phase = 7

    def setup_hardware(self):
        pass

    def signal_led(self, *_):
        pass

    def prepare_mission_radio_control(self, *_):
        pass

    def initialize_phase(self, *_):
        pass

    def _sync_phase_time_tracking(self, *_):
        pass

    def _handle_timeout_transitions(self, *_):
        return False

    def check_radio_failsafe(self):
        if self.current_phase == 7:
            raise AssertionError('Terminal phase must precede periodic work')

    def stop_motors(self, **_kwargs):
        self.events.append('stop')

    def signal_give_up(self):
        self.events.append('give_up')

    def restore_mission_radio(self, *_):
        self.events.append('radio')

    def close_hardware(self):
        self.events.append('close')

    def _write_final_log_row(self):
        self.events.append('log')


class TerminalExitTest(unittest.TestCase):
    def test_every_reason_exits_in_transition_iteration_without_sleep(self):
        for reason in ('GOAL_PROXIMITY_CONFIRMED', 'GOAL_APPROACH_TIMEOUT',
                       'GOAL_PROXIMITY_UNCONFIRMED', 'GOAL_CAMERA_TIMEOUT',
                       'GOAL_MOTION_LIMIT', 'GOAL_NO_PROGRESS', 'MISSION_TOTAL_TIMEOUT',
                       'PHASE4_TIMEOUT_GIVE_UP', 'PHASE5_VISUAL_LOST_TIMEOUT',
                       'UNRECOGNIZED_FAILURE', 'RUNNING'):
            with self.subTest(reason=reason):
                c = TerminalController(reason)
                with patch('time.sleep', side_effect=AssertionError('Unexpected next iteration')):
                    c.run(start_phase=Phase.PHASE6)
                self.assertEqual(c.current_phase, 7)
                self.assertTrue(c._shutdown_requested)
                self.assertEqual(c.lifecycle_diagnostics['Phase7HandlerEntered'], 1)
                self.assertEqual(c.lifecycle_diagnostics['ShutdownCompleted'], 1)
                self.assertEqual(c.events.count('close'), 1)
                self.assertEqual(c.mission_end_reason, 'PHASE7_EXIT' if reason == 'RUNNING' else reason)

    def test_phase7_precedes_subset_filter_and_radio_work(self):
        c = TerminalController('GOAL_APPROACH_TIMEOUT', phase=7)
        c.run(start_phase=Phase.PHASE7, allowed_phases=(Phase.PHASE6,))
        self.assertEqual(c.lifecycle_diagnostics['Phase7HandlerEntered'], 1)
        self.assertEqual(c.lifecycle_diagnostics['ShutdownReason'], 'GOAL_APPROACH_TIMEOUT')

    def test_terminal_reason_is_not_overwritten_by_global_timeout(self):
        c = TerminalController('GOAL_PROXIMITY_CONFIRMED', phase=7)
        self.assertFalse(CanSatController._handle_timeout_transitions(c, Phase.PHASE7))
        self.assertEqual(c.mission_end_reason, 'GOAL_PROXIMITY_CONFIRMED')

    def test_controller_timeout_dispatches_terminal_immediately(self):
        c = TerminalController('MISSION_TOTAL_TIMEOUT')
        def timeout(_):
            c.current_phase = 7
            return True
        c._handle_timeout_transitions = timeout
        c.loop_once()
        self.assertEqual(c.lifecycle_diagnostics['ShutdownCompleted'], 1)

    def test_worker_stop_is_set_before_led_and_led_failure_still_cleans_up(self):
        c = TerminalController('GOAL_APPROACH_TIMEOUT')
        def failed_led():
            self.assertTrue(c._shutdown_requested)
            raise RuntimeError('LED failed')
        c.signal_give_up = failed_led
        with self.assertRaisesRegex(RuntimeError, 'LED failed'):
            c.run(start_phase=Phase.PHASE6)
        self.assertEqual(c.lifecycle_diagnostics['ShutdownCompleted'], 1)
        self.assertEqual(c.lifecycle_diagnostics['RunExceptionType'], 'RuntimeError')
        self.assertEqual(c.events.count('close'), 1)

    def test_blocked_cleanup_or_foreign_thread_cannot_keep_process_alive(self):
        for blockage in ('close', 'foreign_thread', 'log'):
            with self.subTest(blockage=blockage):
                code = f'''
import threading
from runs.spec.terminal_exit import TerminalController
from mission.shutdown import ProcessExitDeadline
c = TerminalController('GOAL_APPROACH_TIMEOUT')
gate = threading.Event()
if {blockage!r} == 'close':
    c.close_hardware = gate.wait
elif {blockage!r} == 'log':
    c._write_final_log_row = gate.wait
else:
    threading.Thread(target=gate.wait, daemon=False).start()
c.arm_process_exit_deadline = ProcessExitDeadline(c, timeout=0.15).arm
c.run()
'''
                result = subprocess.run([sys.executable, '-c', code],
                                        cwd=Path(__file__).resolve().parents[2],
                                        capture_output=True, text=True, timeout=5)
                self.assertEqual(result.returncode, PHASE7_ERROR_EXIT_CODE, result.stderr)
                self.assertIn('Shutdown deadline exceeded', result.stdout)

    def test_completed_mission_returns_without_forced_exit(self):
        code = '''
from runs.spec.terminal_exit import TerminalController
from mission.shutdown import ProcessExitDeadline
c = TerminalController('GOAL_PROXIMITY_UNCONFIRMED')
c.arm_process_exit_deadline = ProcessExitDeadline(c, timeout=2).arm
c.run()
print('RUN_RETURNED')
'''
        result = subprocess.run([sys.executable, '-c', code], capture_output=True,
                                text=True, timeout=5, cwd=Path(__file__).resolve().parents[2])
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('RUN_RETURNED', result.stdout)
        self.assertNotIn('deadline exceeded', result.stdout)

    def test_runner_preserves_failure_recovery_before_phase7(self):
        from mission.run import _run_controller
        c = TerminalController('RUNNING')
        c.run = lambda **_: (_ for _ in ()).throw(RuntimeError('transient failure'))
        with self.assertRaisesRegex(RuntimeError, 'transient failure'):
            _run_controller(c)
        c._terminal_phase_reached = True
        with self.assertRaises(SystemExit) as exit_result:
            _run_controller(c)
        self.assertEqual(exit_result.exception.code, PHASE7_ERROR_EXIT_CODE)

    def test_runner_maps_caught_cleanup_error_to_terminal_exit(self):
        from mission.run import _run_controller
        c = TerminalController('GOAL_APPROACH_TIMEOUT')
        c.close_hardware = lambda: (_ for _ in ()).throw(RuntimeError('close failure'))
        with self.assertRaises(SystemExit) as exit_result:
            _run_controller(c)
        self.assertEqual(exit_result.exception.code, PHASE7_ERROR_EXIT_CODE)
        self.assertTrue(c.lifecycle_diagnostics['ShutdownError'])

    def test_preterminal_deadline_still_exits_with_restartable_failure(self):
        code = '''
import threading
from runs.spec.terminal_exit import TerminalController
from mission.shutdown import ProcessExitDeadline
c = TerminalController('RUN_EXIT')
c.close_hardware = threading.Event().wait
c.arm_process_exit_deadline = ProcessExitDeadline(c, timeout=0.15).arm
c.request_shutdown('RUN_EXIT')
'''
        result = subprocess.run([sys.executable, '-c', code], capture_output=True,
                                text=True, timeout=5, cwd=Path(__file__).resolve().parents[2])
        self.assertEqual(result.returncode, 1, result.stderr)

    def test_service_restart_exception_matches_executable_exit_code(self):
        import configparser
        config = configparser.ConfigParser(interpolation=None)
        config.read(Path(__file__).resolve().parents[2] / 'deploy/systemd/cansat.service.example')
        self.assertEqual(config['Service']['Restart'], 'on-failure')
        self.assertEqual(config['Service']['RestartPreventExitStatus'],
                         f'{PHASE7_ERROR_EXIT_CODE} {RECOVERY_ERROR_EXIT_CODE}')


if __name__ == '__main__':
    unittest.main()
