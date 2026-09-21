import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mission.const import Phase


def _load_controller_class_without_hardware_dependencies():
    manager_module = types.ModuleType("mission.mgr")
    for name in ("HardwareManager", "LedManager", "MotorManager", "RadioManager", "SensorManager"):
        setattr(manager_module, name, type(name, (), {}))

    phases_module = types.ModuleType("mission.phases")
    for name in (
        "Phase0Handler",
        "Phase1Handler",
        "Phase2Handler",
        "Phase3Handler",
        "Phase4Handler",
        "Phase5Handler",
        "Phase6Handler",
        "Phase7Handler",
    ):
        setattr(phases_module, name, type(name, (), {}))

    state_module = types.ModuleType("mission.st")
    state_module.CanSatState = type("CanSatState", (), {})

    ctrl_path = PROJECT_ROOT / "mission" / "ctrl.py"
    spec = importlib.util.spec_from_file_location("controller_under_test", ctrl_path)
    module = importlib.util.module_from_spec(spec)
    original_modules = {
        name: sys.modules.get(name)
        for name in ("mission.mgr", "mission.phases", "mission.st")
    }
    try:
        sys.modules["mission.mgr"] = manager_module
        sys.modules["mission.phases"] = phases_module
        sys.modules["mission.st"] = state_module
        spec.loader.exec_module(module)
    finally:
        for name, original in original_modules.items():
            if original is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = original
    return module.CanSatController


CanSatController = _load_controller_class_without_hardware_dependencies()


class _RunHarness:
    run = CanSatController.run

    def __init__(self, failure_stage):
        self.failure_stage = failure_stage
        self._shutdown_requested = False
        self.shutdown_reasons = []

    def setup_hardware(self):
        if self.failure_stage == "setup":
            raise RuntimeError("setup failed")

    def signal_led(self, _count):
        if self.failure_stage == "led":
            raise RuntimeError("led failed")

    def prepare_mission_radio_control(self, _start_phase):
        if self.failure_stage == "radio":
            raise RuntimeError("radio failed")

    def initialize_phase(self, _phase):
        if self.failure_stage == "phase":
            raise RuntimeError("phase failed")

    def request_shutdown(self, reason):
        self._shutdown_requested = True
        self.shutdown_reasons.append(reason)


class _OffsetLearningHarness:
    _normalize_heading_deg = CanSatController._normalize_heading_deg
    _angle_diff_deg = CanSatController._angle_diff_deg
    _update_heading_offset_estimate = CanSatController._update_heading_offset_estimate

    def __init__(self):
        self.bno_heading_offset_deg = 0.0
        self.bno_heading_offset_valid = False
        self.bno_heading_offset_verified = False
        self.bno_heading_offset_candidate_deg = None
        self.bno_heading_offset_candidate_count = 0


class _ShutdownHarness:
    request_shutdown = CanSatController.request_shutdown

    def __init__(self):
        self._shutdown_requested = False
        self.mission_end_reason = "RUNNING"
        self.phase7_arrival_reason = "RUNNING"
        self.actions = []
        self.st = types.SimpleNamespace(snapshot=lambda: {"phase": int(Phase.PHASE4)})

    def restore_mission_radio(self, reason):
        self.actions.append(("radio", reason))

    def stop_motors(self):
        self.actions.append(("motors", "stop"))

    def close_hardware(self):
        self.actions.append(("hardware", "close"))

    def _write_final_log_row(self):
        self.actions.append(("log", "final"))


class _TimeoutHarness:
    _handle_timeout_transitions = CanSatController._handle_timeout_transitions
    _current_phase_elapsed = CanSatController._current_phase_elapsed
    _commit_active_phase_elapsed = CanSatController._commit_active_phase_elapsed

    def __init__(self):
        self.mission_start_time = 0.0
        self.phase_elapsed_totals = {phase: 0.0 for phase in Phase}
        self.last_phase_observed = Phase.PHASE4
        self.phase_entry_time = 0.0
        self.mission_total_timeout_triggered = False
        self.mission_end_reason = "RUNNING"
        self.cone_phase_decision = "p4_precheck"
        self.terminal_phase = None
        self.motor_stopped = False
        self.st = types.SimpleNamespace(
            snapshot=lambda: {
                "cone_valid": False,
                "cone_sequence": 0,
                "cone_updated_at": 0.0,
            }
        )

    def transition_to_give_up(self, reason):
        self.mission_end_reason = reason
        self.terminal_phase = Phase.PHASE7
        self.motor_stopped = True

    def initialize_phase(self, phase):
        self.terminal_phase = Phase(phase)


class ControllerExceptionSafetyTest(unittest.TestCase):
    def test_dead_camera_waits_past_phase_budget_but_obeys_global_deadline(self):
        for phase in (Phase.PHASE4, Phase.PHASE5):
            harness = _TimeoutHarness()
            harness.last_phase_observed = phase
            time_module = CanSatController._handle_timeout_transitions.__globals__["time"]
            with patch.object(time_module, "time", return_value=100.0):
                self.assertFalse(harness._handle_timeout_transitions(phase))
            self.assertEqual(harness.mission_end_reason, "RUNNING")
            with patch.object(time_module, "time", return_value=1000.0):
                self.assertTrue(harness._handle_timeout_transitions(phase))
            self.assertEqual(harness.mission_end_reason, "MISSION_TOTAL_TIMEOUT")
            self.assertEqual(harness.terminal_phase, Phase.PHASE7)

    def test_phase5_fresh_image_without_cone_never_forces_final_ram(self):
        harness = _TimeoutHarness()
        harness.last_phase_observed = Phase.PHASE5
        harness.st = types.SimpleNamespace(
            snapshot=lambda: {
                "cone_valid": True,
                "cone_sequence": 12,
                "cone_updated_at": 99.8,
            }
        )
        time_module = CanSatController._handle_timeout_transitions.__globals__["time"]

        with patch.object(time_module, "time", return_value=100.0):
            handled = harness._handle_timeout_transitions(Phase.PHASE5)

        self.assertTrue(handled)
        self.assertEqual(harness.terminal_phase, Phase.PHASE7)
        self.assertEqual(harness.mission_end_reason, "PHASE5_VISUAL_LOST_TIMEOUT")

    def test_visible_cone_overrides_phase_budgets_but_not_global_deadline(self):
        for phase in (Phase.PHASE4, Phase.PHASE5):
            for now, should_end in ((100.0, False), (1000.0, True)):
                with self.subTest(phase=phase, now=now):
                    harness = _TimeoutHarness()
                    harness.last_phase_observed = phase
                    harness.st = types.SimpleNamespace(snapshot=lambda: {
                        "cone_valid": True, "cone_sequence": 12,
                        "cone_updated_at": now - 0.2,
                        "cone_probability": 0.5, "cone_direction": 0.85,
                    })
                    time_module = CanSatController._handle_timeout_transitions.__globals__["time"]
                    with patch.object(time_module, "time", return_value=now):
                        handled = harness._handle_timeout_transitions(phase)
                    self.assertEqual(handled, should_end)
                    self.assertEqual(harness.terminal_phase, Phase.PHASE7 if should_end else None)

    def test_shutdown_stops_workers_and_releases_hardware_before_final_log(self):
        harness = _ShutdownHarness()

        harness.request_shutdown("PHASE_SUBSET_COMPLETED")

        self.assertTrue(harness._shutdown_requested)
        self.assertEqual(harness.mission_end_reason, "PHASE_SUBSET_COMPLETED")
        self.assertEqual(
            harness.actions,
            [
                ("radio", "shutdown_PHASE_SUBSET_COMPLETED"),
                ("motors", "stop"),
                ("hardware", "close"),
                ("log", "final"),
            ],
        )

    def test_startup_failures_always_request_shutdown(self):
        for stage in ("setup", "led", "radio", "phase"):
            with self.subTest(stage=stage):
                harness = _RunHarness(stage)
                with self.assertRaisesRegex(RuntimeError, f"{stage} failed"):
                    harness.run(start_phase=Phase.PHASE0)
                self.assertEqual(harness.shutdown_reasons, ["RUN_EXIT"])

    def test_unverified_offset_requires_six_consistent_observations(self):
        harness = _OffsetLearningHarness()

        harness._update_heading_offset_estimate("bno", 100.0)
        harness._update_heading_offset_estimate("bno", 180.0)
        self.assertEqual(harness.bno_heading_offset_candidate_count, 1)
        self.assertFalse(harness.bno_heading_offset_verified)

        for _ in range(5):
            harness._update_heading_offset_estimate("bno", 180.0)

        self.assertTrue(harness.bno_heading_offset_verified)
        self.assertAlmostEqual(harness.bno_heading_offset_deg, 180.0)

    def test_verified_offset_can_follow_large_field_drift_in_capped_steps(self):
        harness = _OffsetLearningHarness()
        harness.bno_heading_offset_deg = 100.0
        harness.bno_heading_offset_valid = True
        harness.bno_heading_offset_verified = True

        harness._update_heading_offset_estimate("bno", 180.0)

        self.assertAlmostEqual(harness.bno_heading_offset_deg, 109.0)


if __name__ == "__main__":
    unittest.main()
