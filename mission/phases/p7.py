import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
MAIN_PY_LIBRARY_DIR = PROJECT_ROOT / "lib"
if not MAIN_PY_LIBRARY_DIR.exists():
    raise FileNotFoundError(f"main.py library directory not found: {MAIN_PY_LIBRARY_DIR}")
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mission.const import DEVICE_LED_GREEN, DEVICE_LED_RED, Phase
from mission.phases.base import BasePhaseHandler


class Phase7Handler(BasePhaseHandler):
    def execute(self, controller, snapshot):
        led_red = controller.devices.get(DEVICE_LED_RED)
        led_green = controller.devices.get(DEVICE_LED_GREEN)
        controller.stop_motors()
        mission_end_reason = getattr(controller, "mission_end_reason", "RUNNING")
        if mission_end_reason == "RUNNING":
            controller.mission_end_reason = "PHASE7_EXIT"
            mission_end_reason = controller.mission_end_reason
        controller.phase7_arrival_reason = controller._resolve_phase7_arrival_reason()

        if mission_end_reason == "GOAL_PROXIMITY_CONFIRMED" and not getattr(controller, "mission_total_timeout_triggered", False):
            print("p7 : Goal proximity confirmed; physical contact is unverified")
            if led_red:
                led_red.on()
            if led_green:
                led_green.on()
            controller.request_shutdown(mission_end_reason)
            return

        give_up = getattr(controller, "mission_total_timeout_triggered", False) or mission_end_reason not in {
            "GOAL_REACHED",
            "PHASE7_EXIT",
        }

        if give_up:
            print(f"p7 : Give up ({controller.phase7_arrival_reason})")
            controller.signal_give_up()
        else:
            print(f"p7 : Goal!! ({controller.phase7_arrival_reason})")
            if led_red:
                led_red.on()
            if led_green:
                led_green.on()
        controller.request_shutdown(mission_end_reason)


def run_standalone():
    from mission.run import run_single_phase

    run_single_phase(Phase.PHASE7)


if __name__ == "__main__":
    run_standalone()
