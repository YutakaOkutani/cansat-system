import sys
import unittest
from pathlib import Path
from unittest.mock import patch


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mission.const import (
    PHASE6_RAM_DURATION_SEC,
    PHASE6_RAM_RAMP_TIME,
    PHASE6_RAM_SPEED,
    Phase,
)
from mission.phases.p6 import Phase6Handler
from mission.st import CanSatState


class _Phase6Controller:
    def __init__(self):
        self.devices = {}
        self.st = CanSatState()
        self.st.update_navigation(phase=int(Phase.PHASE6))
        self.phase_entry_time = 1.0
        self.phase6_entry_marker = None
        self.phase6_start_time = None
        self.mission_total_timeout_triggered = False
        self.mission_end_reason = "GOAL_REACHED"
        self.motor_commands = []

    def set_motors(self, *args, **kwargs):
        self.motor_commands.append((args, kwargs))

    def stop_motors(self):
        self.motor_commands.append(
            ((0.0, True, 0.0, True), {"cmd_type": "stop"})
        )


class Phase6FlowTest(unittest.TestCase):
    def test_final_ram_duration_is_full_goal_push(self):
        self.assertEqual(PHASE6_RAM_DURATION_SEC, 5.0)
        self.assertEqual(PHASE6_RAM_SPEED, 75)

    def test_ram_continues_past_previous_deadline(self):
        controller = _Phase6Controller()
        handler = Phase6Handler()
        for now in (100.0, 103.0, 104.9):
            with patch("mission.phases.p6.time.time", return_value=now):
                handler.execute(controller, controller.st.snapshot())
            self.assertEqual(controller.st.snapshot()["phase"], int(Phase.PHASE6))
            self.assertEqual(controller.motor_commands[-1][1]["cmd_type"], "phase6_final_ram")

    def test_total_timeout_stops_ram_immediately(self):
        controller = _Phase6Controller()
        handler = Phase6Handler()
        with patch("mission.phases.p6.time.time", return_value=100.0):
            handler.execute(controller, controller.st.snapshot())
        controller.mission_total_timeout_triggered = True
        with patch("mission.phases.p6.time.time", return_value=101.0):
            handler.execute(controller, controller.st.snapshot())
        self.assertEqual(controller.motor_commands[-1][1]["cmd_type"], "stop")
        self.assertEqual(controller.st.snapshot()["phase"], int(Phase.PHASE7))

    def test_cone_loss_during_final_ram_never_falls_back(self):
        controller = _Phase6Controller()
        handler = Phase6Handler()

        with patch("mission.phases.p6.time.time", return_value=100.0):
            handler.execute(controller, controller.st.snapshot())

        controller.st.update_cone(
            cone_probability=0.0,
            cone_is_reached=False,
            cone_valid=True,
            cone_status="ok",
            observation_time=101.0,
            observation_accepted=True,
        )
        with patch("mission.phases.p6.time.time", return_value=101.0):
            handler.execute(controller, controller.st.snapshot())

        self.assertEqual(controller.st.snapshot()["phase"], int(Phase.PHASE6))
        self.assertEqual(
            controller.motor_commands[-1][1]["cmd_type"],
            "phase6_final_ram",
        )

    def test_final_ram_stops_before_transitioning_to_phase7(self):
        controller = _Phase6Controller()
        handler = Phase6Handler()

        with patch("mission.phases.p6.time.time", return_value=100.0):
            handler.execute(controller, controller.st.snapshot())

        args, kwargs = controller.motor_commands[-1]
        self.assertEqual((args[0], args[2]), (PHASE6_RAM_SPEED, PHASE6_RAM_SPEED))
        self.assertEqual(kwargs["ramp_time"], PHASE6_RAM_RAMP_TIME)
        self.assertEqual(controller.st.snapshot()["phase"], int(Phase.PHASE6))

        with patch(
            "mission.phases.p6.time.time",
            return_value=100.0 + PHASE6_RAM_DURATION_SEC,
        ):
            handler.execute(controller, controller.st.snapshot())

        self.assertEqual(controller.motor_commands[-1][1]["cmd_type"], "stop")
        self.assertEqual(controller.st.snapshot()["phase"], int(Phase.PHASE7))


if __name__ == "__main__":
    unittest.main()
