import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

sys.modules.setdefault("serial", types.SimpleNamespace())
sys.modules.setdefault("lib.bno055", types.SimpleNamespace())
sys.modules.setdefault("lib.cone_detect", types.SimpleNamespace())
sys.modules.setdefault(
    "mission.gps_util",
    types.SimpleNamespace(
        coerce_gga_metrics=lambda *_args, **_kwargs: None,
        gga_quality_ok=lambda *_args, **_kwargs: False,
        open_gps_serial=lambda *_args, **_kwargs: None,
        parse_gga_sentence=lambda *_args, **_kwargs: None,
    ),
)

SNS_MGR_PATH = PROJECT_ROOT / "mission" / "mgr" / "sns_mgr.py"
spec = importlib.util.spec_from_file_location("camera_recovery_under_test", SNS_MGR_PATH)
sns_mgr_under_test = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sns_mgr_under_test)
SensorManager = sns_mgr_under_test.SensorManager
from mission.st import CanSatState


class _Detector:
    def set_roi_img(self, _roi):
        pass


class _LiveDetector(_Detector):
    probability = 0.4
    cone_direction = 0.2
    is_reached = False
    is_detected = True
    occupancy = 0.02
    frame_red_occupancy = 0.03
    debug_method = "test:hue"
    debug_scores = {"image_direction": 0.2, "image_direction_valid": 1}

    def detect_cone(self):
        return True


class _CameraRecoveryController(SensorManager):
    def __init__(self):
        self.devices = {"detector": None}
        self.roi_img = None
        self.roi_references = []
        self.camera_fail_count = 5
        self.camera_last_reinit = 0.0
        self.camera_dead_since = None
        self.camera_recovery_started_at = None
        self.camera_reinit_attempt_count = 0
        self.camera_recovery_exhausted = False

    def _release_camera_detector(self):
        self.devices["detector"] = None


class CameraRecoveryTest(unittest.TestCase):
    def test_camera_control_direction_is_not_unconditionally_mirrored(self):
        ctrl = _CameraRecoveryController()
        ctrl.camera_control_invert_x = False
        self.assertAlmostEqual(ctrl._transform_cone_direction_for_control(0.2), 0.2)

        ctrl.camera_control_invert_x = True
        self.assertAlmostEqual(ctrl._transform_cone_direction_for_control(0.2), 0.8)

    def test_cone_detection_stores_image_and_control_x_without_extra_flip(self):
        ctrl = _CameraRecoveryController()
        ctrl.st = CanSatState()
        ctrl.devices["detector"] = _LiveDetector()
        ctrl.camera_control_invert_x = False
        ctrl.camera_fail_count = 0
        ctrl._record_camera_recovered = lambda: None

        ctrl.cone_detect()

        snapshot = ctrl.st.snapshot()
        self.assertAlmostEqual(snapshot["cone_image_direction"], 0.2)
        self.assertAlmostEqual(snapshot["cone_direction"], 0.2)

    def test_camera_is_not_initialized_through_phase3(self):
        ctrl = _CameraRecoveryController()
        ctrl.setup_calls = 0
        ctrl.release_calls = 0
        ctrl._camera_runtime_active = False

        def setup_camera():
            ctrl.setup_calls += 1

        def release_camera():
            ctrl.release_calls += 1
            ctrl.devices["detector"] = None

        ctrl._setup_camera_detector = setup_camera
        ctrl._release_camera_detector = release_camera

        for phase in (0, 1, 2, 3):
            self.assertFalse(ctrl._sync_camera_runtime_for_phase(phase))

        self.assertEqual(ctrl.setup_calls, 0)
        self.assertEqual(ctrl.release_calls, 0)
        self.assertFalse(ctrl._camera_runtime_active)

    def test_camera_activates_at_phase4_and_is_released_on_p3_fallback(self):
        ctrl = _CameraRecoveryController()
        ctrl.setup_calls = 0
        ctrl.release_calls = 0
        ctrl._camera_runtime_active = False

        def setup_camera():
            ctrl.setup_calls += 1
            ctrl.devices["detector"] = _Detector()

        def release_camera():
            ctrl.release_calls += 1
            ctrl.devices["detector"] = None

        ctrl._setup_camera_detector = setup_camera
        ctrl._release_camera_detector = release_camera

        self.assertTrue(ctrl._sync_camera_runtime_for_phase(4))
        self.assertEqual(ctrl.setup_calls, 1)
        self.assertTrue(ctrl._camera_runtime_active)

        self.assertTrue(ctrl._sync_camera_runtime_for_phase(5))
        self.assertTrue(ctrl._sync_camera_runtime_for_phase(6))
        self.assertEqual(ctrl.setup_calls, 1)
        self.assertFalse(ctrl._sync_camera_runtime_for_phase(3))
        self.assertEqual(ctrl.release_calls, 1)
        self.assertIsNone(ctrl.devices["detector"])
        self.assertFalse(ctrl._camera_runtime_active)

    def test_fresh_phase4_window_clears_stale_exhaustion(self):
        ctrl = _CameraRecoveryController()
        ctrl.camera_fail_count = 9
        ctrl.camera_last_reinit = 110.0
        ctrl.camera_dead_since = 90.0
        ctrl.camera_recovery_started_at = 90.0
        ctrl.camera_reinit_attempt_count = 3
        ctrl.camera_recovery_exhausted = True

        ctrl.reset_camera_recovery_window()

        self.assertEqual(ctrl.camera_fail_count, 0)
        self.assertEqual(ctrl.camera_last_reinit, 0.0)
        self.assertIsNone(ctrl.camera_dead_since)
        self.assertIsNone(ctrl.camera_recovery_started_at)
        self.assertEqual(ctrl.camera_reinit_attempt_count, 0)
        self.assertFalse(ctrl.camera_recovery_exhausted)

    def test_recovery_continues_with_backoff_after_three_attempts(self):
        ctrl = _CameraRecoveryController()

        with patch.object(sns_mgr_under_test.dc, "detector", side_effect=_Detector, create=True):
            ctrl._begin_camera_recovery(100.0)
            with patch.object(sns_mgr_under_test.time, "time", return_value=100.0):
                self.assertTrue(ctrl._try_reinit_camera())
            with patch.object(sns_mgr_under_test.time, "time", return_value=104.0):
                self.assertFalse(ctrl._try_reinit_camera())
            with patch.object(sns_mgr_under_test.time, "time", return_value=105.0):
                self.assertTrue(ctrl._try_reinit_camera())
            with patch.object(sns_mgr_under_test.time, "time", return_value=110.0):
                self.assertTrue(ctrl._try_reinit_camera())
            with patch.object(sns_mgr_under_test.time, "time", return_value=115.0):
                self.assertFalse(ctrl._try_reinit_camera())

            with patch.object(sns_mgr_under_test.time, "time", return_value=125.0):
                self.assertTrue(ctrl._try_reinit_camera())

        self.assertEqual(ctrl.camera_reinit_attempt_count, 4)

    def test_detector_recreation_is_not_a_recovery_until_a_valid_frame(self):
        ctrl = _CameraRecoveryController()
        ctrl._begin_camera_recovery(100.0)

        with patch.object(sns_mgr_under_test.dc, "detector", side_effect=_Detector, create=True):
            with patch.object(sns_mgr_under_test.time, "time", return_value=100.0):
                ctrl._try_reinit_camera()

        self.assertIsNotNone(ctrl.camera_dead_since)
        self.assertEqual(ctrl.camera_reinit_attempt_count, 1)

        ctrl._record_camera_recovered()

        self.assertIsNone(ctrl.camera_dead_since)
        self.assertIsNone(ctrl.camera_recovery_started_at)
        self.assertEqual(ctrl.camera_reinit_attempt_count, 0)
        self.assertFalse(ctrl.camera_recovery_exhausted)


if __name__ == "__main__":
    unittest.main()
