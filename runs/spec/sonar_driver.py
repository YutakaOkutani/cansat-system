import sys
import types
import unittest
from unittest.mock import patch
from lib.sonar import SonarSensor


class FakeDistanceSensor:
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.next_value = 0.025
        self.closed = False

    def _read(self):
        return self.next_value

    def close(self):
        self.closed = True


class SonarDriverTest(unittest.TestCase):
    def test_attempt_metadata_is_independent_of_polling_and_cached_queue(self):
        with patch.dict(sys.modules, {'gpiozero': types.SimpleNamespace(DistanceSensor=FakeDistanceSensor)}):
            sensor = SonarSensor(echo=24, trigger=23, max_distance=4, pin_factory=None)
        self.assertEqual(sensor.read_sample().sequence, 0)
        with patch('lib.sonar.time.time', return_value=1000), patch('lib.sonar.time.monotonic', return_value=100):
            sensor._device._read()
        first = sensor.read_sample()
        self.assertEqual(first.distance_cm, 10)
        self.assertEqual(first.observed_monotonic, 100)
        self.assertEqual(sensor.read_sample(), first)
        sensor._device.next_value = None
        sensor._device._read()
        self.assertEqual(sensor.read_sample().sequence, 2)
        self.assertIsNone(sensor.read_sample().distance_cm)
        sensor.close()
        self.assertTrue(sensor._device.closed)

    def test_max_range_is_not_a_valid_return(self):
        with patch.dict(sys.modules, {'gpiozero': types.SimpleNamespace(DistanceSensor=FakeDistanceSensor)}):
            sensor = SonarSensor(echo=24, trigger=23, max_distance=4, pin_factory=None)
        for value in (0, 1, float('nan'), None):
            sensor._device.next_value = value
            sensor._device._read()
            self.assertIsNone(sensor.read_sample().distance_cm)
