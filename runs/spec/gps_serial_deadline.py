"""GPS serial regression tests; no hardware or NMEA dependency required."""
import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
spec = importlib.util.spec_from_file_location('gps_deadline_under_test', ROOT / 'mission/gps_util.py')
gps = importlib.util.module_from_spec(spec)
with patch.dict(sys.modules, {'serial': types.SimpleNamespace(), 'pynmea2': types.SimpleNamespace()}):
    spec.loader.exec_module(gps)


class Clock:
    now = 0.0

    def monotonic(self):
        return self.now


class Input:
    def __init__(self, clock, data=None, delay=0.01):
        self.clock = clock
        self.data = data
        self.delay = delay
        self.timeout = 0.2
        self.closed = False

    def read(self, size):
        if self.delay > self.timeout:
            self.clock.now += self.timeout
            return b''
        self.clock.now += self.delay
        if self.data is None:
            return b'x'  # Never ends and never emits a newline.
        byte, self.data = self.data[:size], self.data[size:]
        return byte

    def reset_input_buffer(self):
        pass

    def close(self):
        self.closed = True


class GPSSerialDeadlineTest(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.time_patch = patch.object(gps, 'time', self.clock)
        self.time_patch.start()
        self.addCleanup(self.time_patch.stop)

    def test_continuous_corrupt_input_has_deadline(self):
        stream = Input(self.clock)
        result = gps.read_gps_line(stream, max_seconds=0.15)
        self.assertTrue(result)
        self.assertLessEqual(self.clock.now, 0.151)
        self.assertEqual(stream.timeout, 0.2)

    def test_fast_corrupt_input_has_size_limit(self):
        stream = Input(self.clock, delay=0)
        self.assertEqual(len(gps.read_gps_line(stream)), 512)

    def test_valid_lines_remain_separate(self):
        stream = Input(self.clock, b'$GNGGA,first\r\n$GNGGA,second\r\n')
        self.assertEqual(gps.read_gps_line(stream), b'$GNGGA,first\r\n')
        self.assertEqual(gps.read_gps_line(stream), b'$GNGGA,second\r\n')

    def test_silent_input_and_exception_restore_timeout(self):
        stream = Input(self.clock, delay=1)
        self.assertEqual(gps.read_gps_line(stream, max_seconds=0.1), b'')
        self.assertEqual(stream.timeout, 0.2)
        with patch.object(stream, 'read', side_effect=OSError('disconnected')):
            with self.assertRaises(OSError):
                gps.read_gps_line(stream)
        self.assertEqual(stream.timeout, 0.2)

    def test_discovery_advances_after_continuous_garbage(self):
        streams = []
        attempts = []
        logs = []
        def connect(port, baud, timeout):
            attempts.append(baud)
            stream = Input(self.clock, None if baud == 9600 else b'$VALID\r\n')
            streams.append(stream)
            return stream
        with patch.object(gps, 'serial', types.SimpleNamespace(Serial=connect)), \
             patch.object(gps, '_unique_port_candidates', return_value=['/dev/serial0']), \
             patch.object(gps, 'GPS_BAUDRATE_CANDIDATES', [9600, 38400]), \
             patch.object(gps, 'warmup_serial_for_nmea', return_value={
                 'ok': False, 'total_bytes': 100, 'approx_sentences': 0, 'last_text': 'xxx'}), \
             patch.object(gps, '_is_parseable_nmea', side_effect=lambda line: line == '$VALID'):
            stream, port, baud = gps.open_gps_serial(log=logs.append)
        self.assertEqual(attempts, [9600, 38400])
        self.assertTrue(streams[0].closed)
        self.assertIs(stream, streams[1])
        self.assertEqual(baud, 38400)
        self.assertLess(self.clock.now, 2.2)
        self.assertIn('GPS probing: /dev/serial0 @ 38400', logs)


if __name__ == '__main__':
    unittest.main()
