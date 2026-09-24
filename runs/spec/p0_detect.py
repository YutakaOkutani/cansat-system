import sys
import time
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mission.const import (
    PHASE0_DROP_TO_PHASE1_DELAY_SEC, PHASE0_IMPACT_CONFIRM_SAMPLES,
    PHASE0_IMPACT_DELTA_THRESHOLD, Phase,
)
from mission.phases.p0 import Phase0Handler


class _FakeLed:
    def off(self):
        return None


class _FakeState:
    def __init__(self):
        self.phase = int(Phase.PHASE0)

    def update_navigation(self, phase=None, **_kwargs):
        if phase is not None:
            self.phase = phase


class _FakeController:
    def __init__(self):
        self.devices = {"led_red": _FakeLed(), "led_green": _FakeLed()}
        self.led_blink_timer = 0
        self.phase_entry_time = time.time()
        self.phase0_entry_marker = None
        self.phase0_max_altitude = None
        self.phase0_drop_detect_time = None
        self.phase0_drop_detect_reason = None
        self.phase0_exit_reason = ""
        self.phase0_exit_detail = ""
        self.phase0_acc_baseline = None
        self.phase0_impact_confirm_count = 0
        self.phase0_wait_log_counter = 0
        self.time_phase1_start = None
        self.radio_restore_reasons = []
        self.bmp_last_valid_time = 0.0
        self.bmp_stale_sec = 99.0
        self.bno_last_acc_time = 0.0
        self.bno_acc_stale_sec = 99.0
        self.st = _FakeState()

    def toggle_led(self, *_args, **_kwargs):
        return None

    def restore_mission_radio(self, reason):
        self.radio_restore_reasons.append(reason)
        return True


class Phase0DetectionTest(unittest.TestCase):
    def setUp(self):
        self.handler = Phase0Handler()
        self.ctrl = _FakeController()

    def test_ignores_unavailable_sensor_defaults(self):
        snapshot = {"alt": 0.0, "fall": 0.0}

        self.handler.execute(self.ctrl, snapshot)

        self.assertEqual(self.ctrl.st.phase, int(Phase.PHASE0))
        self.assertIsNone(self.ctrl.phase0_max_altitude)

    def test_initializes_max_altitude_only_after_valid_bmp(self):
        snapshot = {"alt": 123.4, "fall": 0.0}
        self.ctrl.bmp_last_valid_time = time.time()
        self.ctrl.bmp_stale_sec = 0.0

        self.handler.execute(self.ctrl, snapshot)

        self.assertEqual(self.ctrl.phase0_max_altitude, 123.4)
        self.assertEqual(self.ctrl.st.phase, int(Phase.PHASE0))

    def test_transitions_on_altitude_drop_with_valid_bmp(self):
        self.ctrl.bmp_last_valid_time = time.time()
        self.ctrl.bmp_stale_sec = 0.0
        self.ctrl.phase0_max_altitude = 120.0
        self.ctrl.phase0_entry_marker = self.ctrl.phase_entry_time
        self.ctrl.phase0_drop_detect_time = time.time() - PHASE0_DROP_TO_PHASE1_DELAY_SEC

        self.handler.execute(self.ctrl, {"alt": 40.0, "fall": 0.0})

        self.assertEqual(self.ctrl.st.phase, int(Phase.PHASE1))
        self.assertIsNotNone(self.ctrl.time_phase1_start)
        self.assertEqual(self.ctrl.radio_restore_reasons, ["phase0_release_altitude"])
        self.assertEqual(self.ctrl.phase0_exit_reason, "altitude")
        self.assertIn("altitude_drop", self.ctrl.phase0_exit_detail)

    def observe_altitude(self, altitude):
        self.ctrl.bmp_last_valid_time = time.time()
        self.ctrl.bmp_stale_sec = 0.0
        self.ctrl.bno_last_acc_time = time.time()
        self.ctrl.bno_acc_stale_sec = 0.0
        self.handler.execute(self.ctrl, {"alt": altitude, "fall": 9.8})

    def test_ground_ascent_and_small_descent_do_not_trigger(self):
        for altitude in (0, 0, 0, 30, 100, 90):
            self.observe_altitude(altitude)
            self.assertEqual(self.ctrl.st.phase, int(Phase.PHASE0))
            self.assertIsNone(self.ctrl.phase0_drop_detect_time)
        self.assertEqual(self.ctrl.phase0_max_altitude, 100)
        self.assertEqual(self.ctrl.phase0_altitude_drop, 10)

    def test_descent_threshold_inclusive_and_transition_after_delay(self):
        for altitude in (80, 79):
            with self.subTest(altitude=altitude):
                self.setUp()
                for value in (0, 30, 100, altitude):
                    self.observe_altitude(value)
                self.assertEqual(self.ctrl.phase0_drop_detect_reason, "altitude")
                self.assertEqual(self.ctrl.st.phase, int(Phase.PHASE0))
                self.ctrl.phase0_drop_detect_time -= PHASE0_DROP_TO_PHASE1_DELAY_SEC
                self.observe_altitude(altitude)
                self.assertEqual(self.ctrl.st.phase, int(Phase.PHASE1))

    def test_peak_tracks_small_fluctuations(self):
        for altitude, peak in ((-2, -2), (0, 0), (-1, 0), (30, 30), (29, 30), (31, 31)):
            self.observe_altitude(altitude)
            self.assertEqual(self.ctrl.phase0_max_altitude, peak)
            self.assertEqual(self.ctrl.phase0_altitude_drop, peak - altitude)
            self.assertIsNone(self.ctrl.phase0_drop_detect_time)

    def test_stale_bmp_does_not_poison_peak(self):
        self.observe_altitude(100)
        self.ctrl.bmp_stale_sec = 99.0
        for altitude in (10000, float("inf"), float("nan"), -10000):
            self.handler.execute(self.ctrl, {"alt": altitude, "fall": 9.8})
            self.assertEqual(self.ctrl.phase0_max_altitude, 100)
            self.assertIsNone(self.ctrl.phase0_current_altitude)
            self.assertIsNone(self.ctrl.phase0_altitude_drop)
            self.assertIsNone(self.ctrl.phase0_drop_detect_time)
        self.observe_altitude(99)
        self.assertEqual(self.ctrl.phase0_altitude_drop, 1)

    def test_altitude_hold_cancels_when_descent_recovers(self):
        for altitude in (100, 79, 90):
            self.observe_altitude(altitude)
        self.assertIsNone(self.ctrl.phase0_drop_detect_time)
        self.assertIsNone(self.ctrl.phase0_drop_detect_reason)

    def test_peak_resets_on_phase0_reentry(self):
        self.observe_altitude(100)
        self.ctrl.phase_entry_time += 1.0
        self.observe_altitude(0)
        self.assertEqual(self.ctrl.phase0_max_altitude, 0)
        self.assertEqual(self.ctrl.phase0_altitude_drop, 0)
        self.assertIsNone(self.ctrl.phase0_drop_detect_time)

    def test_latches_impact_after_confirmation_and_delay(self):
        self.ctrl.bno_last_acc_time = time.time()
        self.ctrl.bno_acc_stale_sec = 0.0
        self.ctrl.phase0_entry_marker = self.ctrl.phase_entry_time
        self.ctrl.phase0_acc_baseline = 9.8

        for _ in range(PHASE0_IMPACT_CONFIRM_SAMPLES):
            self.handler.execute(self.ctrl, {"alt": 0.0, "fall": 9.8 + PHASE0_IMPACT_DELTA_THRESHOLD + 1})
        self.assertEqual(self.ctrl.st.phase, int(Phase.PHASE0))

        self.ctrl.phase0_drop_detect_time = time.time() - PHASE0_DROP_TO_PHASE1_DELAY_SEC
        self.handler.execute(self.ctrl, {"alt": 0.0, "fall": 9.8 + PHASE0_IMPACT_DELTA_THRESHOLD + 1})

        self.assertEqual(self.ctrl.st.phase, int(Phase.PHASE1))
        self.assertIsNotNone(self.ctrl.time_phase1_start)
        self.assertEqual(self.ctrl.radio_restore_reasons, ["phase0_release_impact"])
        self.assertEqual(self.ctrl.phase0_exit_reason, "impact")
        self.assertIn("delta", self.ctrl.phase0_exit_detail)

    def test_restores_radio_on_phase0_timeout(self):
        self.ctrl.phase0_entry_marker = self.ctrl.phase_entry_time
        self.ctrl.phase_entry_time = time.time() - 999.0

        self.handler.execute(self.ctrl, {"alt": 0.0, "fall": 0.0})

        self.assertEqual(self.ctrl.st.phase, int(Phase.PHASE1))
        self.assertEqual(self.ctrl.radio_restore_reasons, ["phase0_timeout"])
        self.assertEqual(self.ctrl.phase0_exit_reason, "timeout")
        self.assertIn("timeout", self.ctrl.phase0_exit_detail)

    def test_latched_impact_survives_return_to_stationary_during_delay(self):
        self.ctrl.bno_last_acc_time = time.time()
        self.ctrl.bno_acc_stale_sec = 0.0
        self.ctrl.phase0_entry_marker = self.ctrl.phase_entry_time
        self.ctrl.phase0_acc_baseline = 9.8

        for _ in range(PHASE0_IMPACT_CONFIRM_SAMPLES):
            self.handler.execute(self.ctrl, {"alt": 0.0, "fall": 9.8 + PHASE0_IMPACT_DELTA_THRESHOLD + 1})
        self.assertEqual(self.ctrl.phase0_drop_detect_reason, "impact")

        self.ctrl.phase0_drop_detect_time = time.time() - PHASE0_DROP_TO_PHASE1_DELAY_SEC
        self.handler.execute(self.ctrl, {"alt": 0.0, "fall": 9.8})

        self.assertEqual(self.ctrl.st.phase, int(Phase.PHASE1))

    def test_stationary_gravity_does_not_trigger_impact(self):
        self.ctrl.bno_last_acc_time = time.time()
        self.ctrl.bno_acc_stale_sec = 0.0

        for _ in range(PHASE0_IMPACT_CONFIRM_SAMPLES + 3):
            self.handler.execute(self.ctrl, {"alt": 0.0, "fall": 9.8})

        self.assertEqual(self.ctrl.st.phase, int(Phase.PHASE0))
        self.assertIsNone(self.ctrl.phase0_drop_detect_time)


if __name__ == "__main__":
    unittest.main()
