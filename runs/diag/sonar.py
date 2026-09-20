import argparse
import math
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mission.const import (
    PIN_ECHO,
    PIN_TRIG,
    SONAR_MAX_DISTANCE,
    SONAR_MIN_DISTANCE_CM,
    SONAR_STALE_TIMEOUT_SEC,
)


def parse_args():
    parser = argparse.ArgumentParser(description="Ultrasonic distance sensor diagnostic")
    parser.add_argument("--interval", type=float, default=0.2, help="Polling interval in seconds")
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Stop after this many seconds; 0 runs until Ctrl+C",
    )
    return parser.parse_args()


def _valid_distance_cm(distance_m):
    try:
        distance_m = float(distance_m)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(distance_m) or not SONAR_MIN_DISTANCE_CM / 100.0 <= distance_m < float(SONAR_MAX_DISTANCE):
        return None
    return distance_m * 100.0


def main():
    args = parse_args()
    interval = max(0.05, float(args.interval))
    duration = max(0.0, float(args.duration))

    from lib.sonar import SonarSensor
    from gpiozero.pins.lgpio import LGPIOFactory

    print("Ultrasonic sensor diagnostic")
    print(f"  trigger GPIO : {PIN_TRIG}")
    print(f"  echo GPIO    : {PIN_ECHO}")
    print(f"  max distance : {SONAR_MAX_DISTANCE:.1f} m")
    print(f"  stale limit  : {SONAR_STALE_TIMEOUT_SEC:.1f} s")
    print("Purpose: forward ranging / goal proximity; no automatic obstacle avoidance.")
    print("Wiring: direct ECHO connection is supported when its output meets 3.3 V GPIO input limits.")
    print("Wiring: for 5 V ECHO, use an external divider or level shifter; the current PCB has none.")
    print("Check the module's output specification at its supply voltage; successful readings alone are not proof.")
    print("Place a flat object in front of the sensor and move it. Ctrl+C to exit.")

    pin_factory = None
    sensor = None
    valid_count = 0
    invalid_count = 0
    consecutive_invalid = 0
    last_sequence = 0
    last_valid_at = None
    last_valid_cm = None
    started_at = time.monotonic()

    try:
        pin_factory = LGPIOFactory()
        sensor = SonarSensor(
            echo=PIN_ECHO,
            trigger=PIN_TRIG,
            max_distance=SONAR_MAX_DISTANCE,
            pin_factory=pin_factory,
        )

        while duration <= 0.0 or time.monotonic() - started_at < duration:
            now = time.monotonic()
            try:
                sample = sensor.read_sample()
                distance_cm = _valid_distance_cm(
                    sample.distance_cm / 100.0 if sample.distance_cm is not None else None
                )
                if sample.sequence == last_sequence and now - sample.observed_monotonic < SONAR_STALE_TIMEOUT_SEC:
                    time.sleep(interval)
                    continue
                last_sequence = sample.sequence
                if not 0 <= now - sample.observed_monotonic < SONAR_STALE_TIMEOUT_SEC:
                    distance_cm = None
            except Exception as exc:
                distance_cm = None
                error_detail = f"{type(exc).__name__}: {exc}"
            else:
                error_detail = "no valid echo"

            timestamp = time.strftime("%H:%M:%S")
            if distance_cm is not None:
                valid_count += 1
                consecutive_invalid = 0
                last_valid_at = now
                last_valid_cm = distance_cm
                print(
                    f"[{timestamp}] VALID distance={distance_cm:7.2f} cm "
                    f"sequence={sample.sequence}"
                )
            else:
                invalid_count += 1
                consecutive_invalid += 1
                stale_sec = now - last_valid_at if last_valid_at is not None else float("inf")
                last_text = f"{last_valid_cm:.2f} cm" if last_valid_cm is not None else "none"
                stale_text = f"{stale_sec:.2f}s" if math.isfinite(stale_sec) else "never"
                state = "INVALID"
                print(
                    f"[{timestamp}] {state} last={last_text} age={stale_text} "
                    f"failures={consecutive_invalid} ({error_detail})"
                )

            time.sleep(interval)
    except KeyboardInterrupt:
        print("\nDiagnostic stopped by user.")
    finally:
        if sensor is not None:
            try:
                sensor.close()
            except Exception:
                pass
        if pin_factory is not None:
            try:
                pin_factory.close()
            except Exception:
                pass

    print(f"Summary: valid={valid_count}, invalid={invalid_count}")
    if valid_count == 0:
        print("No valid echo was received. Check 5V/GND, GPIO wiring, target angle, and distance.")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
