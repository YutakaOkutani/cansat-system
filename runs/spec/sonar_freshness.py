import importlib.util
import sys
import types
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mission.const import DEFAULT_SONAR_DIST_CM, SONAR_STALE_TIMEOUT_SEC
from mission.st import CanSatState

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
spec = importlib.util.spec_from_file_location("sonar_sensor_manager_under_test", SNS_MGR_PATH)
sensor_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sensor_module)


class _SonarController(sensor_module.SensorManager):
    def __init__(self):
        self.st = CanSatState()


from lib.sonar import SonarSample


class SonarFreshnessTest(unittest.TestCase):
    def test_cached_sample_does_not_refresh_timestamp(self):
        ctrl = _SonarController()
        sample = SonarSample(1, 25.0, 1000.0, 100.0)
        self.assertTrue(ctrl._update_sonar_state(sample, now=100.0))
        self.assertTrue(ctrl._update_sonar_state(sample, now=100.2))
        self.assertFalse(ctrl._update_sonar_state(sample, now=100.5))
        state = ctrl.st.snapshot()
        self.assertFalse(state['sonar_valid'])
        self.assertEqual(state['sonar_sequence'], 1)
        self.assertEqual(state['sonar_observed_monotonic'], 100.0)

    def test_no_echo_invalidates_immediately_and_recovers(self):
        ctrl = _SonarController()
        ctrl._update_sonar_state(SonarSample(1, 25.0, 1000.0, 100.0), now=100.0)
        self.assertFalse(ctrl._update_sonar_state(SonarSample(2, None, 1000.1, 100.1), now=100.1))
        self.assertFalse(ctrl.st.snapshot()['sonar_valid'])
        self.assertTrue(ctrl._update_sonar_state(SonarSample(3, 5.0, 1000.2, 100.2), now=100.2))

    def test_invalid_and_blind_zone_samples_are_rejected(self):
        ctrl = _SonarController()
        for distance in (None, float('nan'), float('inf'), -1, 0, 1, 400, 9999):
            with self.subTest(distance=distance):
                self.assertFalse(ctrl._update_sonar_state(SonarSample(1, distance, 1000, 100), now=100))
        self.assertFalse(ctrl._update_sonar_state(None, now=100))
        self.assertFalse(ctrl._update_sonar_state(SonarSample(1, 20, 1000, 101), now=100))


if __name__ == '__main__':
    unittest.main()
