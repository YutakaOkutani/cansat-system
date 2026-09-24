"""Durable full-mission recovery, without device access."""
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from mission.config import MissionConfig, TargetConfig, RadioConfig
from mission.const import Phase, RECOVERY_ERROR_EXIT_CODE, PHASE6_APPROACH_TIMEOUT_SEC
from mission.recovery import RecoveryStore, RecoveryError
from mission.st import CanSatState
from runs.spec.controller_exception_safety import CanSatController
from runs.spec.phase6_flow import execute
from runs.spec.terminal_exit import TerminalController


def controller(phase=0, now=100):
    c = CanSatController.__new__(CanSatController)
    c.st = CanSatState()
    c.st.update_navigation(phase=phase)
    c.run_id = 'new-run'
    c.run_bundle = SimpleNamespace(record_recovery=Mock())
    c.phase_elapsed_totals = {p: 0 for p in Phase}
    c.last_phase_observed = Phase(phase)
    c.phase_entry_time = now
    c.mission_start_time = now
    c.mission_end_reason = 'RUNNING'
    c.phase2_stage = 'escape'
    c.phase2_start_time = now
    c.phase2_stage_start = now
    c.devices = {}
    c.mission_total_timeout_triggered = False
    c.stop_motors = Mock()
    return c


class MissionRecoveryTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'mission-state.json'
        self.config = MissionConfig(TargetConfig(35, 139),
                                    RadioConfig('off', 3, 180, False, True), Path('fixture.toml'))
        config_patch = patch('mission.run.load_mission_config', return_value=self.config)
        config_patch.start()
        self.addCleanup(config_patch.stop)

    def seed(self, phase=0, visit=0, stage='escape', pulses=0):
        store = RecoveryStore(self.path)
        with patch('time.time', return_value=100):
            c = controller()
            store.bind(c, self.config)
        record = dict(store.record, phase=phase, visit_elapsed=visit,
                      mission_elapsed=40, phase2_stage=stage, phase6_pulses=pulses,
                      stage_elapsed=3)
        record['phase_elapsed'][str(phase)] = visit
        store._write(record)
        return record

    def resume(self, phase=0, now=110):
        store = RecoveryStore(self.path)
        store.load(self.config)
        c = controller(phase, now)
        store.bind(c, self.config)
        with patch('time.time', return_value=now):
            store.restore(c)
        c._recovery_ready = True
        return store, c

    def test_seed_exists_before_hardware_and_resume_links_new_run(self):
        self.seed()
        store, c = self.resume()
        self.assertEqual(c.lifecycle_diagnostics['RecoveryCount'], 1)
        self.assertEqual(c.lifecycle_diagnostics['RecoveryMissionId'], 'new-run')
        self.assertEqual(c.lifecycle_diagnostics['RecoveryFromPhase'], 0)
        self.assertTrue(c._recovery_starting)
        self.assertEqual(store.record['phase'], 0)

    def test_p0_to_p6_restore_same_phase_and_no_sensor_evidence(self):
        for phase in range(7):
            with self.subTest(phase=phase):
                self.seed(phase, visit=8)
                store, c = self.resume(phase)
                self.assertEqual(c.st.snapshot()['phase'], phase)
                self.assertFalse(c.st.snapshot()['cone_valid'])
                self.assertEqual(c._current_phase_elapsed(Phase(phase), 110), 8)
                self.assertEqual(110 - c.mission_start_time, 50)

    def test_initial_p6_save_and_handler_keep_pulse_and_time_limits(self):
        self.seed(6, visit=PHASE6_APPROACH_TIMEOUT_SEC, pulses=4)
        store, c = self.resume(6)
        with patch('time.time', return_value=110):
            store.save(c, force=True)
        self.assertEqual(store.record['phase6_pulses'], 4)
        execute(c, 110)
        self.assertEqual(c.phase6_pulses, 4)
        self.assertEqual(c.st.snapshot()['phase'], 7)
        self.assertEqual(c.mission_end_reason, 'GOAL_APPROACH_TIMEOUT')
        c.stop_motors.assert_called()

    def test_budgets_are_not_double_counted_on_second_restart(self):
        self.seed(6, visit=8)
        store, c = self.resume(6)
        with patch('time.time', return_value=112):
            store.save(c, force=True)
        self.assertEqual(store.record['visit_elapsed'], 10)
        self.assertEqual(store.record['phase_elapsed']['6'], 10)
        store, c = self.resume(6, now=120)
        with patch('time.time', return_value=122):
            store.save(c, force=True)
        self.assertEqual(store.record['visit_elapsed'], 12)
        self.assertEqual(store.record['phase_elapsed']['6'], 12)
        self.assertEqual(store.record['mission_elapsed'], 62)
        c.st.update_navigation(phase=7)
        with patch('time.time', return_value=123):
            store.save(c, force=True)
        self.assertEqual(store.record['visit_elapsed'], 0)
        self.assertEqual(store.record['phase_elapsed']['6'], 13)

    def test_p2_does_not_repeat_completed_escape(self):
        for stage in ('calibration', 'offset'):
            self.seed(2, stage=stage)
            _, c = self.resume(2)
            self.assertEqual(c.phase2_stage, stage)
            self.assertEqual(c.phase2_stage_start, 107)
            if stage == 'offset':
                self.assertFalse(c.bno_heading_offset_verified)
                self.assertEqual(c.phase2_entry_ready_count, 0)

    def test_p1_restores_only_remaining_active_duration(self):
        self.seed(1, visit=4)
        _, c = self.resume(1)
        self.assertEqual(c.time_phase1_start, 106)

    def test_motor_worker_cannot_drive_during_restore(self):
        from runs.spec.goal_approach import motor
        c = SimpleNamespace(_recovery_starting=True, stop_motors=Mock(),
                            _shutdown_active=Mock(side_effect=[False, True]))
        with patch.object(motor.time, 'sleep'):
            motor.MotorManager.move_motor_thread(c)
        self.assertEqual(c.stop_motors.call_args_list[0].kwargs,
                         {'reason': 'recovery_initializing'})

    def test_recovery_identity_reaches_csv_and_analysis(self):
        import csv
        from mission.const import LOG_HEADER
        from analysis.final_approach import write_final_approach_report
        from runs.spec.log_schema import _LogOnlyController
        self.seed(6)
        _, c = self.resume(6)
        log = _LogOnlyController()
        log.lifecycle_diagnostics = c.lifecycle_diagnostics
        log.st.update_navigation(phase=6)
        csv_path = self.path.parent / 'recovered.csv'
        with csv_path.open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(LOG_HEADER)
            writer.writerow(log._build_log_row())
        write_final_approach_report(csv_path, self.path.parent)
        summary = (self.path.parent / 'final_approach_summary.txt').read_text()
        self.assertIn('RecoveryCount: 1', summary)
        self.assertIn('RecoveryFromPhase: 6', summary)
        self.assertIn('RecoveryFromRunId: new-run', summary)

    def test_atomic_replace_failure_preserves_previous_checkpoint(self):
        previous = self.seed(6)
        store = RecoveryStore(self.path)
        with patch('mission.recovery.os.replace', side_effect=OSError('power lost')):
            with self.assertRaises(RecoveryError):
                store._write(dict(previous, phase=7))
        self.assertEqual(RecoveryStore(self.path).load(self.config)['phase'], 6)

    def test_corrupt_or_mismatched_state_refuses_new_mission(self):
        valid = self.seed()
        for record in ('{', dict(valid, phase=8), dict(valid, mission_elapsed=float('nan')),
                       dict(valid, config={}), dict(valid, phase6_pulses=-1)):
            with self.subTest(record=record):
                self.path.write_text(record if isinstance(record, str) else json.dumps(record))
                with self.assertRaises(RecoveryError):
                    RecoveryStore(self.path).load(self.config)

    def test_backwards_clock_refuses_resume(self):
        self.seed()
        with self.assertRaises(RecoveryError):
            self.resume(now=90)

    def test_lock_prevents_duplicate_run_and_live_reset(self):
        with RecoveryStore(self.path):
            with self.assertRaises(RecoveryError):
                with RecoveryStore(self.path):
                    self.fail('Acquired duplicate lock')
        with RecoveryStore(self.path):
            pass

    def test_phase7_preflight_never_constructs_hardware_for_any_reason(self):
        from mission.run import run_full_mission
        for reason in ('GOAL_PROXIMITY_CONFIRMED', 'GOAL_APPROACH_TIMEOUT', 'unknown'):
            record = self.seed(7)
            RecoveryStore(self.path)._write(dict(record, reason=reason))
            with patch('mission.recovery.RecoveryStore', side_effect=lambda: RecoveryStore(self.path)), \
                 patch('mission.run._build_controller') as build:
                run_full_mission()
                build.assert_not_called()

    def test_main_passes_p0_to_p6_saved_phase_to_runner(self):
        from mission.run import run_full_mission
        for phase in range(7):
            self.seed(phase)
            c = controller(phase)
            with patch('mission.recovery.RecoveryStore', side_effect=lambda: RecoveryStore(self.path)), \
                 patch('mission.run._build_controller', return_value=c), \
                 patch('mission.run._run_controller') as run:
                run_full_mission()
                run.assert_called_once_with(c, start_phase=Phase(phase))

    def test_main_returns_nonrestart_exit_for_invalid_state(self):
        from mission.run import run_full_mission
        self.path.write_text('{')
        with patch('mission.recovery.RecoveryStore', side_effect=lambda: RecoveryStore(self.path)), \
             patch('mission.run._build_controller') as build:
            with self.assertRaises(SystemExit) as caught:
                run_full_mission()
            self.assertEqual(caught.exception.code, RECOVERY_ERROR_EXIT_CODE)
            build.assert_not_called()

    def test_terminal_checkpoint_precedes_led_and_cleanup(self):
        c = TerminalController('GOAL_APPROACH_TIMEOUT', phase=7)
        c.recovery_store = SimpleNamespace(save=lambda *a, **k: c.events.append('checkpoint'))
        c._finish_terminal_phase()
        self.assertLess(c.events.index('stop'), c.events.index('checkpoint'))
        self.assertLess(c.events.index('checkpoint'), c.events.index('give_up'))
        self.assertTrue(c._shutdown_requested)

    def test_reset_allows_new_mission_from_every_saved_phase(self):
        for phase in range(8):
            with self.subTest(phase=phase):
                self.seed(phase)
                with RecoveryStore(self.path) as store:
                    store.reset()
                    self.assertIsNone(store.load(self.config))
                    store.bind(controller(), self.config)
                self.assertEqual(RecoveryStore(self.path).load(self.config)['phase'], 0)

    def test_reset_hooks_orderly_reboot_and_poweroff_and_waits_for_writers(self):
        import configparser
        config = configparser.ConfigParser(interpolation=None)
        root = Path(__file__).resolve().parents[2] / 'deploy/systemd'
        config.read(root / 'cansat-reset-on-reboot.service.example')
        self.assertEqual(set(config['Install']['WantedBy'].split()),
                         {'reboot.target', 'poweroff.target'})
        self.assertEqual(config['Unit']['DefaultDependencies'], 'no')
        writers = {'cansat.service', 'cansat.timer', 'cansat-resume.service'}
        self.assertTrue(writers <= set(config['Unit']['Conflicts'].split()))
        self.assertTrue(writers <= set(config['Unit']['After'].split()))
        self.assertTrue({'shutdown.target', 'umount.target', 'reboot.target', 'poweroff.target'}
                        <= set(config['Unit']['Before'].split()))
        # Manual reset must not itself request an OS reboot or poweroff.
        for dependency in ('Wants', 'Requires'):
            self.assertFalse({'reboot.target', 'poweroff.target'}
                             & set(config['Unit'].get(dependency, '').split()))
        self.assertTrue(config['Service']['ExecStart'].endswith(' -m mission.recovery reset'))
        mission = configparser.ConfigParser(interpolation=None)
        mission.read(root / 'cansat.service.example')
        self.assertEqual(config['Service']['User'], mission['Service']['User'])
        self.assertEqual(config['Service']['WorkingDirectory'], mission['Service']['WorkingDirectory'])
        self.assertNotIn('ExecStop', mission['Service'])


if __name__ == '__main__':
    unittest.main()
