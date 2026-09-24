"""Record termination triggers independently of goal outcomes and sampled PWM."""
import csv
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from analysis.final_approach import write_final_approach_report
from mission.const import LOG_HEADER
from mission.diagnostics import FINAL_DIAGNOSTIC_DEFAULTS
from runs.spec.controller_exception_safety import CanSatController
from runs.spec.log_schema import _LogOnlyController
from runs.spec.phase6_flow import Controller, execute
from runs.spec.goal_approach import motor


class FinalDiagnosticsTest(unittest.TestCase):
    def controller(self, close=None):
        c = CanSatController.__new__(CanSatController)
        c._shutdown_requested = False
        c.mission_end_reason = 'GOAL_APPROACH_TIMEOUT'
        c.phase7_arrival_reason = 'RUNNING'
        c.mission_start_time = 100
        c.current_phase = 6
        c.st = SimpleNamespace(snapshot=lambda: {'phase': c.current_phase})
        c.setup_hardware = lambda: None
        c.signal_led = lambda _: None
        c.prepare_mission_radio_control = lambda _: None
        c.initialize_phase = lambda _: None
        c.restore_mission_radio = lambda _: None
        c.stop_motors = lambda **_kwargs: None
        c.close_hardware = close or (lambda: None)
        c.rows = []
        c._write_final_log_row = lambda: c.rows.append(dict(getattr(c, 'lifecycle_diagnostics', {})))
        return c

    def test_ctrl_c_after_phase7_transition_preserves_both_reasons(self):
        c = self.controller()
        def interrupted_transition():
            c.current_phase = 7
            raise KeyboardInterrupt()
        c.loop_once = interrupted_transition
        c.run()
        self.assertEqual(c.mission_end_reason, 'GOAL_APPROACH_TIMEOUT')
        last = c.rows[-1]
        self.assertEqual(last['ShutdownReason'], 'KEYBOARD_INTERRUPT')
        self.assertEqual(last['InterruptPhase'], 7)
        self.assertEqual(last['InterruptCount'], 1)
        self.assertEqual(last['ShutdownCompleted'], 1)
        self.assertEqual(last['RunFinallyReached'], 1)
        self.assertNotIn('Phase7HandlerEntered', last)

    def test_ctrl_c_during_cleanup_records_incomplete_stage(self):
        c = self.controller(close=lambda: (_ for _ in ()).throw(KeyboardInterrupt()))
        c.loop_once = lambda: c.request_shutdown(c.mission_end_reason)
        c.run()
        last = c.rows[-1]
        self.assertEqual(last['ShutdownReason'], 'GOAL_APPROACH_TIMEOUT')
        self.assertEqual(last['InterruptStage'], 'hardware_close')
        self.assertEqual(last['InterruptCount'], 1)
        self.assertFalse(last.get('ShutdownCompleted', 0))
        self.assertTrue(any(r.get('ShutdownStage') == 'hardware_close' for r in c.rows))

    def test_exception_is_not_reported_as_successful_mission(self):
        c = self.controller()
        c.mission_end_reason = 'RUNNING'
        c.loop_once = lambda: (_ for _ in ()).throw(ValueError('failure'))
        with self.assertRaises(ValueError):
            c.run()
        self.assertEqual(c.rows[-1]['RunExceptionType'], 'ValueError')
        self.assertEqual(c.rows[-1]['ShutdownReason'], 'RUN_EXIT')
        self.assertEqual(c.rows[-1]['ShutdownCompleted'], 1)

    def test_short_pulse_survives_subsequent_stop_samples(self):
        c = motor.MotorManager()
        c.mission_start_time = 100
        c.phase6_pulses = 4
        for t, cmd in ((101, 'phase6_align_right'), (101.04, 'stop'), (101.2, 'stop')):
            stop_reason = 'phase6:stop_distance' if t == 101.04 else 'unspecified'
            with patch.object(motor.time, 'time', return_value=t), patch.object(motor.time, 'monotonic', return_value=t):
                c._record_motor_command(cmd, 45 if t == 101 else 0, True, 0, True, stop_reason=stop_reason)
        self.assertEqual(c.last_motor_command['type'], 'stop')
        self.assertEqual(c.motor_diagnostics['Phase6PulseStartedCount'], 1)
        self.assertEqual(c.motor_diagnostics['Phase6LastPulseId'], 4)
        self.assertAlmostEqual(c.motor_diagnostics['Phase6LastPulseDurationSec'], .04)
        self.assertEqual(c.motor_diagnostics['Phase6LastPulseStopReason'], 'phase6:stop_distance')

    def test_phase6_wait_gate_and_used_sequences_are_frozen(self):
        c = Controller()
        c.observe(100)
        execute(c, 100)
        self.assertEqual(c.phase6_diagnostics['Phase6Gate'], 'settle_wait')
        c.observe(100.4)
        execute(c, 100.4)
        self.assertEqual(c.phase6_diagnostics['GoalEvalConfirmCount'], 1)
        used = c.phase6_diagnostics['GoalEvalConeSeq']
        c.observe(100.5)
        self.assertEqual(c.phase6_diagnostics['GoalEvalConeSeq'], used)
        execute(c, 100.6)
        execute(c, 100.61)
        self.assertEqual(c.phase6_diagnostics['Phase6Gate'], 'new_pair_wait')

    def test_schema_defaults_do_not_claim_shutdown_or_interrupt(self):
        c = _LogOnlyController()
        values = c._build_log_row()
        self.assertEqual(len(values), len(LOG_HEADER))
        row = dict(zip(LOG_HEADER, values))
        self.assertTrue(set(FINAL_DIAGNOSTIC_DEFAULTS).issubset(row))
        self.assertEqual(row['ShutdownCompleted'], 0)
        self.assertEqual(row['InterruptCount'], 0)
        c.lifecycle_diagnostics = {'ShutdownReason': 'KEYBOARD_INTERRUPT', 'InterruptCount': 1}
        row = dict(zip(LOG_HEADER, c._build_log_row()))
        self.assertEqual(row['ShutdownReason'], 'KEYBOARD_INTERRUPT')
        self.assertEqual(row['InterruptCount'], 1)

    def test_old_and_new_analysis_distinguishes_unknown_from_no_interrupt(self):
        for modern in (False, True):
            with self.subTest(modern=modern), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                row = {'ElapsedSec': '38.26', 'Phase': '7', 'MissionEndReason': 'GOAL_APPROACH_TIMEOUT'}
                if modern:
                    row.update(ShutdownReason='KEYBOARD_INTERRUPT', InterruptCount='1', ShutdownCompleted='1')
                with (root / 'mission.csv').open('w', newline='') as stream:
                    writer = csv.DictWriter(stream, fieldnames=list(row))
                    writer.writeheader()
                    writer.writerow(row)
                write_final_approach_report(root / 'mission.csv', root)
                report = (root / 'final_approach_summary.txt').read_text()
                self.assertIn('Phase 7 first recorded: 38.26 sec', report)
                self.assertIn('Phase7HandlerEntered: unknown', report)
                self.assertIn('InterruptCount: ' + ('1' if modern else 'unknown'), report)
                self.assertIn('ShutdownReason: ' + ('KEYBOARD_INTERRUPT' if modern else 'unknown'), report)

    def test_analysis_does_not_resurrect_a_cleared_cleanup_device(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / 'mission.csv').write_text(
                'Phase,HardwareCloseDevice\n7,camera\n7,\n', encoding='utf-8')
            write_final_approach_report(root / 'mission.csv', root)
            report = (root / 'final_approach_summary.txt').read_text()
            self.assertIn('HardwareCloseDevice: (none recorded)', report)


if __name__ == '__main__':
    unittest.main()
