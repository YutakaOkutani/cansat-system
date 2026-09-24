"""Real camera cleanup methods with a deliberately blocked fake driver."""
import ast
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from mission import const
from mission.diagnostics import lifecycle
from runs.spec.controller_exception_safety import CanSatController
from runs.spec.camera_recovery import _CameraRecoveryController, sns_mgr_under_test

# Load the owning class without importing GPIO drivers. The method bodies and
# timeout are the production source, not a reimplementation of cleanup.
source = Path(__file__).resolve().parents[2] / 'mission/mgr/hw_mgr.py'
node = next(n for n in ast.parse(source.read_text()).body
            if isinstance(n, ast.ClassDef) and n.name == 'HardwareManager')
namespace = dict(vars(const), threading=threading, lifecycle=lifecycle)
exec(compile(ast.Module(body=[node], type_ignores=[]), str(source), 'exec'), namespace)
HardwareManager = namespace['HardwareManager']


class CameraShutdownTest(unittest.TestCase):
    def test_blocked_close_still_stops_and_writes_final_log(self):
        gate = threading.Event()
        detector = SimpleNamespace(close=lambda: gate.wait(2))
        c = HardwareManager()
        c.devices = {const.DEVICE_DETECTOR: detector}
        c._shutdown_checkpoint = lambda stage, **kw: CanSatController._shutdown_checkpoint(c, stage, **kw)
        c._shutdown_requested = False
        c.mission_end_reason = 'GOAL_CAMERA_TIMEOUT'
        c.phase7_arrival_reason = 'RUNNING'
        c.st = SimpleNamespace(snapshot=lambda: {'phase': 7})
        c._resolve_phase7_arrival_reason = lambda: c.mission_end_reason
        actions = []
        c.restore_mission_radio = lambda _: None
        c.stop_motors = lambda **_kwargs: actions.append('stop')
        c._write_final_log_row = lambda: actions.append('final_log')
        try:
            with patch.dict(namespace, CAMERA_CLOSE_TIMEOUT_SEC=.01):
                CanSatController.request_shutdown(c, c.mission_end_reason)
                self.assertEqual(actions[0], 'stop')
                self.assertEqual(actions[-1], 'final_log')
                self.assertEqual(c.lifecycle_diagnostics['ShutdownError'], 'camera_close_timeout')
                self.assertEqual(c.lifecycle_diagnostics['ShutdownCompleted'], 1)
                self.assertIsNone(c.devices[const.DEVICE_DETECTOR])
                self.assertTrue(c._camera_close_worker.is_alive())
                self.assertFalse(c._release_camera_detector())
        finally:
            gate.set()
            c._camera_close_worker.join(1)
        self.assertTrue(c._release_camera_detector())

    def test_recreation_waits_for_old_camera_close(self):
        c = _CameraRecoveryController()
        c._release_camera_detector = lambda: False
        with patch.object(sns_mgr_under_test.dc, 'detector', create=True) as factory:
            self.assertFalse(c._try_reinit_camera(force=True, reason='test_stall'))
            factory.assert_not_called()

    def test_new_terminal_reasons_remain_distinct_in_phase7(self):
        for reason in ('GOAL_CAMERA_TIMEOUT', 'GOAL_PROXIMITY_UNCONFIRMED',
                       'GOAL_MOTION_LIMIT', 'GOAL_NO_PROGRESS'):
            c = SimpleNamespace(mission_end_reason=reason)
            self.assertEqual(CanSatController._resolve_phase7_arrival_reason(c), reason)
