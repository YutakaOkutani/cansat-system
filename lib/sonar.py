"""Timestamp individual echo attempts, never a cached DistanceSensor average."""
import math
import threading
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class SonarSample:
    sequence: int
    distance_cm: float | None
    observed_at: float
    observed_monotonic: float


class SonarSensor:
    def __init__(self, *, echo, trigger, max_distance, pin_factory):
        from gpiozero import DistanceSensor

        self._lock = threading.Lock()
        self._sample = SonarSample(0, None, 0.0, 0.0)
        owner = self

        class TimestampedDistanceSensor(DistanceSensor):
            def _read(self):
                # gpiozero owns trigger/echo timing and its bounded echo timeout.
                # Capture the individual attempt before its smoothing queue drops
                # None or retains previous readings. This override is covered by
                # driver-contract specs and must be checked on gpiozero upgrades.
                try:
                    value = super()._read()
                except Exception:
                    owner._publish(None)
                    raise
                distance = None
                if value is not None and math.isfinite(value) and 0 < value < 1:
                    distance = value * max_distance * 100.0
                owner._publish(distance)
                return value

        self._device = TimestampedDistanceSensor(
            echo=echo, trigger=trigger, max_distance=max_distance,
            queue_len=1, partial=True, pin_factory=pin_factory,
        )

    def _publish(self, distance_cm):
        with self._lock:
            self._sample = SonarSample(
                self._sample.sequence + 1, distance_cm,
                time.time(), time.monotonic(),
            )

    def read_sample(self):
        with self._lock:
            return self._sample

    def close(self):
        self._device.close()
