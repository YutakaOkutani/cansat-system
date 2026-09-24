import os
import sys
import time

if __package__ is None or __package__ == "":
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from mission.const import (
    DEVICE_LED_GREEN,
    DEVICE_LED_RED,
    DROP_ALTITUDE_DIFF_THRESHOLD,
    IMPACT_FALL_THRESHOLD,
    LED_INTERVAL_PHASE0,
    PHASE0_ACCEL_BASELINE_ALPHA,
    PHASE0_DROP_TO_PHASE1_DELAY_SEC,
    PHASE0_IMPACT_CONFIRM_SAMPLES,
    PHASE0_IMPACT_DELTA_THRESHOLD,
    PHASE0_SENSOR_STALE_TIMEOUT,
    Phase,
    TIMEOUT_PHASE_0,
)
from mission.phases.base import BasePhaseHandler


class Phase0Handler(BasePhaseHandler):
    def execute(self, controller, snapshot):
        led_red = controller.devices.get(DEVICE_LED_RED)
        led_green = controller.devices.get(DEVICE_LED_GREEN)

        entry_marker = getattr(controller, "phase_entry_time", None)
        if getattr(controller, "phase0_entry_marker", None) != entry_marker:
            controller.phase0_entry_marker = entry_marker
            controller.phase0_max_altitude = None
            controller.phase0_current_altitude = None
            controller.phase0_altitude_drop = None
            controller.phase0_drop_detect_time = None
            controller.phase0_drop_detect_reason = None
            controller.phase0_exit_reason = ""
            controller.phase0_exit_detail = ""
            controller.phase0_acc_baseline = None
            controller.phase0_impact_confirm_count = 0
            controller.phase0_wait_log_counter = 0
            print("p0 : falling")

        controller.toggle_led(led_red, controller.led_blink_timer, interval=LED_INTERVAL_PHASE0)
        if led_green:
            led_green.off()

        now = time.time()
        bmp_stale_sec = float(getattr(controller, "bmp_stale_sec", TIMEOUT_PHASE_0 + 1.0))
        bno_acc_stale_sec = float(getattr(controller, "bno_acc_stale_sec", TIMEOUT_PHASE_0 + 1.0))
        bmp_valid = (
            getattr(controller, "bmp_last_valid_time", 0.0) > 0.0
            and bmp_stale_sec <= PHASE0_SENSOR_STALE_TIMEOUT
        )
        acc_valid = (
            getattr(controller, "bno_last_acc_time", 0.0) > 0.0
            and bno_acc_stale_sec <= PHASE0_SENSOR_STALE_TIMEOUT
            and snapshot["fall"] is not None
        )

        # SensorManager validates finite/range values before publishing altitude.
        # Only fresh, valid BMP observations may update the Phase0 peak.
        current_altitude = snapshot["alt"] if bmp_valid else None
        max_altitude = controller.phase0_max_altitude
        altitude_drop = None
        if current_altitude is not None:
            if max_altitude is None:
                max_altitude = current_altitude
                print(f"Start Altitude: {current_altitude:.2f}m")
            else:
                max_altitude = max(max_altitude, current_altitude)
            altitude_drop = max_altitude - current_altitude
        controller.phase0_max_altitude = max_altitude
        controller.phase0_current_altitude = current_altitude
        controller.phase0_altitude_drop = altitude_drop

        fall_norm = float(snapshot["fall"] or 0.0)
        acc_baseline = getattr(controller, "phase0_acc_baseline", None)
        if acc_valid and fall_norm > 0.0:
            if acc_baseline is None:
                acc_baseline = fall_norm
            elif abs(fall_norm - acc_baseline) < PHASE0_IMPACT_DELTA_THRESHOLD:
                alpha = max(0.0, min(1.0, float(PHASE0_ACCEL_BASELINE_ALPHA)))
                acc_baseline = (1.0 - alpha) * acc_baseline + alpha * fall_norm
            controller.phase0_acc_baseline = acc_baseline

        is_drop = altitude_drop is not None and altitude_drop >= DROP_ALTITUDE_DIFF_THRESHOLD
        impact_delta = 0.0
        if acc_valid and acc_baseline is not None:
            impact_delta = abs(fall_norm - acc_baseline)
        # Backward-compatible absolute guard remains useful for very large shocks,
        # but normal detection uses baseline-subtracted acceleration so gravity at
        # rest (about 9.8m/s^2) does not consume the threshold budget.
        impact_sample = (
            acc_valid
            and (
                impact_delta >= PHASE0_IMPACT_DELTA_THRESHOLD
                or fall_norm > IMPACT_FALL_THRESHOLD
            )
        )
        if impact_sample:
            controller.phase0_impact_confirm_count = (
                int(getattr(controller, "phase0_impact_confirm_count", 0)) + 1
            )
        else:
            controller.phase0_impact_confirm_count = 0
        is_impact = (
            controller.phase0_impact_confirm_count >= int(PHASE0_IMPACT_CONFIRM_SAMPLES)
        )

        if not bmp_valid or not acc_valid:
            controller.phase0_wait_log_counter = int(getattr(controller, "phase0_wait_log_counter", 0)) + 1
            if controller.phase0_wait_log_counter % 25 == 0:
                print(
                    "Phase0 sensor wait: "
                    f"bmp_valid={int(bmp_valid)} bmp_stale={bmp_stale_sec:.2f}s "
                    f"acc_valid={int(acc_valid)} acc_stale={bno_acc_stale_sec:.2f}s "
                    f"alt={snapshot['alt']:.2f} fall={fall_norm:.2f}"
                )

        detect_reason = None
        detect_detail = ""
        if is_drop:
            detect_reason = "altitude"
            detect_detail = (
                f"current_altitude={current_altitude:.2f}m "
                f"max_altitude={max_altitude:.2f}m altitude_drop={altitude_drop:.2f}m"
            )
        elif is_impact:
            detect_reason = "impact"
            detect_detail = (
                f"fall={fall_norm:.2f}m/s^2 "
                f"baseline={float(acc_baseline or 0.0):.2f} "
                f"delta={impact_delta:.2f}"
            )

        latched_reason = getattr(controller, "phase0_drop_detect_reason", None)
        if detect_reason is None and latched_reason == "impact":
            detect_reason = "impact"
            detect_detail = (
                f"latched impact; fall={fall_norm:.2f}m/s^2 "
                f"baseline={float(acc_baseline or 0.0):.2f}"
            )

        if detect_reason is not None:
            if controller.phase0_drop_detect_time is None:
                controller.phase0_drop_detect_time = now
                controller.phase0_drop_detect_reason = detect_reason
                print(
                    f"Detected Phase0 release candidate ({detect_reason}: {detect_detail}); "
                    f"(waiting {PHASE0_DROP_TO_PHASE1_DELAY_SEC:.1f}s before Phase1)"
                )
            elif now - controller.phase0_drop_detect_time >= PHASE0_DROP_TO_PHASE1_DELAY_SEC:
                reason = getattr(controller, "phase0_drop_detect_reason", None) or detect_reason
                controller.phase0_exit_reason = reason
                controller.phase0_exit_detail = detect_detail
                phase0_start = entry_marker if entry_marker is not None else now
                phase0_elapsed = max(0.0, now - phase0_start)
                print(f"PHASE0_EXIT reason={reason} elapsed={phase0_elapsed:.2f}s detail={detect_detail}")
                print(f"Phase0 release hold complete ({reason}: {detect_detail}) -> Phase1")
                restore_radio = getattr(controller, "restore_mission_radio", None)
                if callable(restore_radio):
                    restore_radio(f"phase0_release_{reason}")
                controller.st.update_navigation(phase=int(Phase.PHASE1))
                controller.time_phase1_start = now
                return
        else:
            if controller.phase0_drop_detect_time is not None:
                print("Phase0 release hold canceled: signal returned below threshold")
            controller.phase0_drop_detect_time = None
            controller.phase0_drop_detect_reason = None

        if controller.time_phase1_start is None:
            phase0_start = entry_marker if entry_marker is not None else now
            if now - phase0_start > TIMEOUT_PHASE_0:
                detail = f"elapsed={now - phase0_start:.2f}s timeout={TIMEOUT_PHASE_0:.2f}s"
                controller.phase0_exit_reason = "timeout"
                controller.phase0_exit_detail = detail
                print(f"PHASE0_EXIT reason=timeout detail={detail}")
                print("Phase0 TIMEOUT: Force proceed (Sensor failure?)")
                restore_radio = getattr(controller, "restore_mission_radio", None)
                if callable(restore_radio):
                    restore_radio("phase0_timeout")
                controller.st.update_navigation(phase=int(Phase.PHASE1))
                controller.time_phase1_start = now


def _print_direct_run_help():
    print("p0.py is a phase handler module and does not run mission logic by itself.")
    print("Run mission from project root with: python3 main.py")


if __name__ == "__main__":
    _print_direct_run_help()
