import signal
import sys
import time
from pathlib import Path

from gpiozero import LED
from gpiozero.pins.lgpio import LGPIOFactory

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mission.const import (
    DEVICE_LED_GREEN,
    DEVICE_LED_RED,
    GPS_INACTIVE_DETECT,
    MAIN_LOOP_INTERVAL,
    PIN_LED_GREEN,
    PIN_LED_RED,
    Phase,
)
from mission.mgr.led_mgr import LedManager
from mission.phases.p0 import Phase0Handler
from mission.phases.p1 import Phase1Handler
from mission.phases.p2 import Phase2Handler
from mission.phases.p3 import Phase3Handler
from mission.phases.p4 import Phase4Handler
from mission.phases.p5 import Phase5Handler
from mission.phases.p6 import Phase6Handler
from mission.phases.p7 import Phase7Handler


class DummyState:
    def __init__(self):
        self.data = {
            "phase": int(Phase.PHASE0),
            "alt": 100.0,
            "fall": 0.0,
            "gps_detect": GPS_INACTIVE_DETECT,
            "lat": 0.0,
            "lng": 0.0,
            "direction": 0.0,
            "angle": 0.0,
            "angle_valid": False,
            "cone_probability": 0.0,
            "cone_is_reached": False,
        }

    def snapshot(self):
        return dict(self.data)

    def update_navigation(self, **kwargs):
        self.data.update(kwargs)


class LedPatternHarness(LedManager):
    def __init__(self):
        self.pin_factory = LGPIOFactory()
        self.devices = {
            DEVICE_LED_RED: LED(PIN_LED_RED, pin_factory=self.pin_factory),
            DEVICE_LED_GREEN: LED(PIN_LED_GREEN, pin_factory=self.pin_factory),
        }
        self.st = DummyState()
        self.led_blink_timer = 0
        self.bno_calib = {"valid": False, "value": (0, 0, 0, 0)}

        self.time_phase1_start = None
        self.phase0_entry_marker = None
        self.phase0_max_altitude = None
        self.phase0_current_altitude = None
        self.phase0_altitude_drop = None
        self.phase2_start_time = None
        self.phase2_stage = "straight"
        self.phase2_stage_start = None
        self.time_phase3_start = time.time()
        self.time_phase4_start = 0.0
        self.time_phase5_start = 0.0
        self.time_start_searching_cone = 0.0
        self.time_camera_start = 0.0
        self.phase_entry_time = None
        self.last_phase_observed = None
        self.searching_flag = False
        self.count_cone_lost = 0
        self.phase5_entry_marker = None
        self.camera_dead_since = None
        self.camera_phase4_attempts = 0
        self.camera_phase5_attempts = 0
        self.camera_phase4_start = None
        self.camera_phase5_start = None
        self.phase5_reach_confirm_count = 0
        self.mission_end_reason = "RUNNING"
        self.mission_total_timeout_triggered = False

    def stop_motors(self):
        return None

    def set_motors(self, *args, **kwargs):
        return None

    def _weighted_heading(self, snapshot):
        return None, "INVALID", 0.0

    def _angle_diff_deg(self, target_deg, current_deg):
        return 0.0

    def all_off(self):
        led_red = self.devices.get(DEVICE_LED_RED)
        led_green = self.devices.get(DEVICE_LED_GREEN)
        if led_red:
            led_red.off()
        if led_green:
            led_green.off()

    def close(self):
        self.all_off()
        for dev in self.devices.values():
            try:
                dev.close()
            except Exception:
                pass

    def _set_phase_context(self, phase):
        self.st.update_navigation(phase=int(phase))
        self.phase_entry_time = time.time()
        if phase == Phase.PHASE3:
            self.time_phase3_start = time.time()
        if phase == Phase.PHASE4:
            self.searching_flag = False
            self.camera_dead_since = None
            self.st.update_navigation(cone_probability=0.0, cone_is_reached=False)
        if phase == Phase.PHASE5:
            self.camera_dead_since = None
            self.st.update_navigation(cone_probability=1.0, cone_is_reached=False)
        if phase == Phase.PHASE6:
            self.phase6_entry_marker = None
            self.phase6_start_time = None

    def run_handler_for(self, handler, phase, duration, loop_dt=MAIN_LOOP_INTERVAL):
        self._set_phase_context(phase)
        end = time.time() + duration
        while time.time() < end:
            self.led_blink_timer += 1
            handler.execute(self, self.st.snapshot())
            time.sleep(loop_dt)

    def run_phase7_once(self, total_timeout=False, hold_sec=2.0):
        self._set_phase_context(Phase.PHASE7)
        self.mission_total_timeout_triggered = total_timeout
        handler = Phase7Handler()
        try:
            handler.execute(self, self.st.snapshot())
        except SystemExit:
            pass
        if not total_timeout:
            time.sleep(hold_sec)
            self.all_off()
        self.mission_total_timeout_triggered = False


HARNESS = None


def safe_exit(signum=None, frame=None):
    global HARNESS
    print("\nLED test stopped. Turning all LEDs off.")
    if HARNESS is not None:
        HARNESS.close()
    raise SystemExit(0)


def run_demo():
    global HARNESS
    HARNESS = LedPatternHarness()
    signal.signal(signal.SIGINT, safe_exit)

    print("--- LED test using production mission handlers/managers ---")
    print("Covers startup signal, Phase0-7 patterns, and give-up indication.")
    print("Note: LED_INTERVAL_PHASE3_NEAR exists in constants but is not used by current production phase handlers.\n")

    try:
        print("[COMMON] Startup signal (LedManager.signal_led)")
        HARNESS.signal_led(3)
        time.sleep(0.5)

        print("[PH0] red blink / green off")
        HARNESS.run_handler_for(Phase0Handler(), Phase.PHASE0, duration=6.0)
        HARNESS.all_off()
        time.sleep(0.5)

        print("[PH1] red on / green off")
        HARNESS.run_handler_for(Phase1Handler(), Phase.PHASE1, duration=3.0)
        HARNESS.all_off()
        HARNESS.time_phase1_start = None
        time.sleep(0.5)

        print("[PH2] red blink / green on")
        HARNESS.phase2_start_time = None
        HARNESS.run_handler_for(Phase2Handler(), Phase.PHASE2, duration=6.0)
        HARNESS.all_off()
        time.sleep(0.5)

        print("[PH3] red off / green blink")
        HARNESS.time_phase3_start = time.time()
        HARNESS.run_handler_for(Phase3Handler(), Phase.PHASE3, duration=6.0)
        HARNESS.all_off()
        time.sleep(0.5)

        print("[PH4] red off / green on")
        HARNESS.run_handler_for(Phase4Handler(), Phase.PHASE4, duration=4.0)
        HARNESS.all_off()
        time.sleep(0.5)

        print("[PH5] alternate red/green blink")
        HARNESS.run_handler_for(Phase5Handler(), Phase.PHASE5, duration=6.0)
        HARNESS.all_off()
        time.sleep(0.5)

        print("[PH6] red on / green on")
        HARNESS.run_handler_for(Phase6Handler(), Phase.PHASE6, duration=2.0)
        HARNESS.all_off()
        time.sleep(0.5)

        print("[PH7 normal] red on / green on then exit")
        HARNESS.run_phase7_once(total_timeout=False, hold_sec=2.0)
        time.sleep(0.5)

        print("[PH7 give-up] red on / green off")
        HARNESS.run_phase7_once(total_timeout=True, hold_sec=0.0)
        time.sleep(0.5)

        print("LED pattern demo completed. Turning all LEDs off.")
        HARNESS.close()
        HARNESS = None
    except Exception as exc:
        print(f"Error during LED demo: {exc}")
        if HARNESS is not None:
            HARNESS.close()
            HARNESS = None
        raise


if __name__ == "__main__":
    run_demo()
