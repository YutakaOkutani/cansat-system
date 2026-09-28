import time
from decimal import Decimal, InvalidOperation, localcontext
from pathlib import Path

import pynmea2
import serial

from mission.const import (
    GPS_BAUDRATE,
    GPS_BAUDRATE_CANDIDATES,
    GPS_BUFFER_CLEAR_INTERVAL,
    GPS_BUFFER_CLEAR_THRESHOLD,
    GPS_COORD_LAT_OFFSET_DEG,
    GPS_COORD_LNG_OFFSET_DEG,
    GPS_FIX_LOSS_TIMEOUT,
    GPS_MAX_HDOP,
    GPS_MAX_SPEED_MPS,
    GPS_MIN_FIX_QUAL,
    GPS_MIN_SATELLITES,
    GPS_PROBE_SECONDS,
    GPS_STARTUP_READ_SIZE,
    GPS_STARTUP_SYNC_SECONDS,
    GPS_SERIAL_DISCOVERY_TIMEOUT,
    GPS_SERIAL_PORT,
    GPS_SERIAL_PORT_CANDIDATES,
    GPS_SERIAL_TIMEOUT,
    GPS_STABLE_FIX_COUNT,
    NMEA_GGA_PREFIXES,
    NMEA_SENTENCE_GGA,
)
from mission.nav import calc_distance_and_azimuth


def _is_parseable_nmea(line):
    if not line or not line.startswith("$"):
        return False
    try:
        pynmea2.parse(line, check=True)
        return True
    except Exception:
        return False


def read_gps_line(serial_obj, max_seconds=GPS_SERIAL_TIMEOUT, max_bytes=512):
    """Bound a line read even when corrupt input arrives without newlines.

    Return partial input at the deadline/size limit, as diagnostic data.
    A serial read timeout alone does not bound IOBase.readline().
    """
    deadline = time.monotonic() + max(0.0, max_seconds)
    original_timeout = serial_obj.timeout
    data = bytearray()
    try:
        while len(data) < max_bytes:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            read_timeout = min(remaining, original_timeout) if original_timeout is not None else remaining
            if serial_obj.timeout != read_timeout:
                serial_obj.timeout = read_timeout
            byte = serial_obj.read(1)
            if not byte:
                break
            data.extend(byte)
            if byte == b"\n":
                break
        return bytes(data)
    finally:
        serial_obj.timeout = original_timeout


def probe_nmea(serial_obj, probe_seconds=GPS_PROBE_SECONDS):
    deadline = time.monotonic() + probe_seconds
    last_line = ""
    while time.monotonic() < deadline:
        try:
            line_bytes = read_gps_line(serial_obj, max_seconds=max(0.0, deadline - time.monotonic()))
        except Exception:
            return False, last_line
        if not line_bytes:
            continue
        line = line_bytes.decode("utf-8", errors="ignore").strip()
        if line:
            last_line = line[-160:]
        if _is_parseable_nmea(line):
            return True, last_line
    return False, last_line


def warmup_serial_for_nmea(serial_obj, warmup_seconds=GPS_STARTUP_SYNC_SECONDS, read_size=GPS_STARTUP_READ_SIZE):
    start = time.time()
    total_bytes = 0
    approx_sentences = 0
    last_text = ""
    carry = ""

    while time.time() - start < warmup_seconds:
        try:
            chunk = serial_obj.read(read_size)
        except Exception:
            return {
                "ok": False,
                "total_bytes": total_bytes,
                "approx_sentences": approx_sentences,
                "last_text": last_text,
            }
        if not chunk:
            continue

        total_bytes += len(chunk)
        text = chunk.decode("utf-8", errors="ignore")
        if not text:
            continue

        approx_sentences += text.count("$")
        last_text = (last_text + text)[-160:]
        carry += text.replace("\r", "\n")
        while "\n" in carry:
            line, carry = carry.split("\n", 1)
            line = line.strip()
            if _is_parseable_nmea(line):
                return {
                    "ok": True,
                    "total_bytes": total_bytes,
                    "approx_sentences": approx_sentences,
                    "last_text": line,
                }

    return {
        "ok": False,
        "total_bytes": total_bytes,
        "approx_sentences": approx_sentences,
        "last_text": last_text.strip(),
    }


def _unique_port_candidates():
    ports = [GPS_SERIAL_PORT] + [port for port in GPS_SERIAL_PORT_CANDIDATES if port != GPS_SERIAL_PORT]
    unique = []
    seen = set()
    for port in ports:
        try:
            real = str(Path(port).resolve(strict=False))
        except Exception:
            real = port
        key = (port, real) if not real else real
        if key in seen:
            continue
        seen.add(key)
        unique.append(port)
    return unique


def open_gps_serial(log=print):
    ports = _unique_port_candidates()
    bauds = [GPS_BAUDRATE] + [baud for baud in GPS_BAUDRATE_CANDIDATES if baud != GPS_BAUDRATE]
    for port in ports:
        for baud in bauds:
            if log:
                log(f"GPS probing: {port} @ {baud}")
            try:
                serial_obj = serial.Serial(port, baud, timeout=GPS_SERIAL_DISCOVERY_TIMEOUT)
                try:
                    serial_obj.reset_input_buffer()
                except Exception:
                    pass
                is_primary_candidate = port == GPS_SERIAL_PORT and baud == GPS_BAUDRATE
                if is_primary_candidate:
                    warmup = warmup_serial_for_nmea(serial_obj)
                    if warmup["ok"]:
                        serial_obj.timeout = GPS_SERIAL_TIMEOUT
                        if log:
                            log(
                                "GPS serial opened: "
                                f"{port} @ {baud} after startup sync "
                                f"({warmup['total_bytes']} bytes, ~{warmup['approx_sentences']} sentences)"
                            )
                        return serial_obj, port, baud
                    if log:
                        log(
                            "GPS startup sync failed: "
                            f"{port} @ {baud} "
                            f"({warmup['total_bytes']} bytes, ~{warmup['approx_sentences']} sentences, "
                            f"last: {warmup['last_text'] or 'none'!r})"
                        )
                    try:
                        serial_obj.reset_input_buffer()
                    except Exception:
                        pass
                ok, last_line = probe_nmea(serial_obj)
                if ok:
                    serial_obj.timeout = GPS_SERIAL_TIMEOUT
                    if log:
                        log(f"GPS serial opened: {port} @ {baud}")
                    return serial_obj, port, baud
                try:
                    serial_obj.close()
                except Exception:
                    pass
                if log:
                    if last_line:
                        hint = " (baud mismatch or wrong UART?)" if "$" not in last_line else ""
                        log(f"No NMEA at {port} @ {baud}{hint} (last: {last_line!r})")
                    else:
                        log(f"No NMEA at {port} @ {baud}")
            except Exception as exc:
                if log:
                    log(f"GPS Serial Open Failed: {port} @ {baud}: {exc}")
    return None, None, None


def coerce_gga_metrics(gps_qual, num_sats, hdop):
    try:
        gps_fix_qual_val = int(gps_qual) if gps_qual is not None else 0
    except (TypeError, ValueError):
        gps_fix_qual_val = 0
    try:
        gps_sats_val = int(num_sats) if num_sats is not None else 0
    except (TypeError, ValueError):
        gps_sats_val = 0
    try:
        gps_hdop_val = float(hdop) if hdop is not None else 0.0
    except (TypeError, ValueError):
        gps_hdop_val = 0.0
    return gps_fix_qual_val, gps_sats_val, gps_hdop_val


def gga_quality_ok(gps_qual, num_sats, hdop):
    try:
        qual_ok = gps_qual is not None and int(gps_qual) >= GPS_MIN_FIX_QUAL
    except (TypeError, ValueError):
        qual_ok = False
    try:
        sats_ok = num_sats is not None and int(num_sats) >= GPS_MIN_SATELLITES
    except (TypeError, ValueError):
        sats_ok = False
    try:
        hdop_ok = hdop is not None and float(hdop) <= GPS_MAX_HDOP
    except (TypeError, ValueError):
        hdop_ok = True
    return qual_ok, sats_ok, hdop_ok


def apply_gps_coordinate_offset(lat, lng):
    """Apply static coordinate offsets to absorb repeatable field bias."""
    return float(lat) + float(GPS_COORD_LAT_OFFSET_DEG), float(lng) + float(GPS_COORD_LNG_OFFSET_DEG)


def _decimal_coordinate_from_dm(dm_value, direction, degree_digits, offset_deg=0.0):
    """Convert NMEA ddmm.mmmm text into a decimal-degree string without truncating source precision."""
    if not dm_value or "." not in dm_value:
        return None

    try:
        deg_text = dm_value[:degree_digits]
        min_text = dm_value[degree_digits:]
        degrees = Decimal(deg_text)
        minutes = Decimal(min_text)
        decimal_value = degrees + (minutes / Decimal("60"))
        if direction in ("S", "W"):
            decimal_value = -decimal_value
        decimal_value += Decimal(str(offset_deg))
    except (InvalidOperation, ValueError):
        return None

    frac_digits = max(0, len(min_text.partition(".")[2]))
    output_places = max(6, frac_digits + 2)
    quantum = Decimal(1).scaleb(-output_places)
    with localcontext() as ctx:
        ctx.prec = max(28, output_places + 8)
        quantized = decimal_value.quantize(quantum)
    return format(quantized, f".{output_places}f")


def format_latitude_from_nmea(dm_value, direction, offset_deg=0.0):
    return _decimal_coordinate_from_dm(dm_value, direction, degree_digits=2, offset_deg=offset_deg)


def format_longitude_from_nmea(dm_value, direction, offset_deg=0.0):
    return _decimal_coordinate_from_dm(dm_value, direction, degree_digits=3, offset_deg=offset_deg)


def parse_gga_sentence(line):
    if not line or not line.startswith(NMEA_GGA_PREFIXES):
        return None
    try:
        msg = pynmea2.parse(line, check=True)
    except Exception:
        return None
    if getattr(msg, "sentence_type", "") != NMEA_SENTENCE_GGA:
        return None

    lat_val = getattr(msg, "latitude", None)
    lng_val = getattr(msg, "longitude", None)
    if lat_val is None or lng_val is None:
        return None

    lat_corr, lng_corr = apply_gps_coordinate_offset(lat_val, lng_val)
    return {
        "msg": msg,
        "lat": lat_corr,
        "lng": lng_corr,
        "lat_text": format_latitude_from_nmea(getattr(msg, "lat", None), getattr(msg, "lat_dir", None), GPS_COORD_LAT_OFFSET_DEG),
        "lng_text": format_longitude_from_nmea(getattr(msg, "lon", None), getattr(msg, "lon_dir", None), GPS_COORD_LNG_OFFSET_DEG),
        "gps_qual": getattr(msg, "gps_qual", None),
        "num_sats": getattr(msg, "num_sats", None),
        "hdop": getattr(msg, "horizontal_dil", None),
        "raw": line,
    }


class RobustGPSReader:
    """Shared standalone GPS reader using the same gating logic as SensorManager."""

    def __init__(self):
        self.ser = None
        self.selected_port = None
        self.selected_baud = None
        self.last_buffer_clear = time.time()
        self.last_fix_time = 0.0
        self.last_valid_fix_time = 0.0
        self.last_valid_latlng = None
        self.stable_count = 0

    def open_serial(self):
        self.ser, self.selected_port, self.selected_baud = open_gps_serial()
        return self.ser

    def ensure_serial(self):
        if self.ser is None or not self.ser.is_open:
            self.open_serial()

    def read_fix(self):
        self.ensure_serial()
        if self.ser is None:
            time.sleep(1)
            return None

        now = time.time()
        if self.last_valid_fix_time > 0 and now - self.last_valid_fix_time > GPS_FIX_LOSS_TIMEOUT:
            self.stable_count = 0

        if (
            self.ser.in_waiting > GPS_BUFFER_CLEAR_THRESHOLD
            and now - self.last_buffer_clear >= GPS_BUFFER_CLEAR_INTERVAL
        ):
            try:
                self.ser.reset_input_buffer()
                print("GPS buffer cleared to drop stale data.")
            except Exception:
                pass
            self.last_buffer_clear = now
            self.stable_count = 0

        try:
            line_bytes = read_gps_line(self.ser)
        except Exception as exc:
            print(f"GPS read error: {exc}")
            try:
                if self.ser is not None:
                    self.ser.close()
            except Exception:
                pass
            self.ser = None
            time.sleep(1)
            return None

        if not line_bytes:
            return None

        line = line_bytes.decode("utf-8", errors="ignore").strip()
        parsed = parse_gga_sentence(line)
        if parsed is None:
            return None

        lat = parsed["lat"]
        lng = parsed["lng"]
        gps_qual = parsed["gps_qual"]
        num_sats = parsed["num_sats"]
        hdop = parsed["hdop"]
        qual_ok, sats_ok, hdop_ok = gga_quality_ok(gps_qual, num_sats, hdop)
        if not (qual_ok and sats_ok and hdop_ok and (lat != 0.0 or lng != 0.0)):
            self.stable_count = 0
            return None

        speed_ok = True
        if self.last_valid_latlng is not None:
            dist, _ = calc_distance_and_azimuth(
                self.last_valid_latlng[0], self.last_valid_latlng[1], lat, lng
            )
            dt = now - self.last_fix_time if self.last_fix_time > 0 else 0.0
            if dt > 0:
                speed = dist / dt
                if speed > GPS_MAX_SPEED_MPS:
                    speed_ok = False

        if not speed_ok:
            self.stable_count = 0
            return None

        self.stable_count += 1
        self.last_fix_time = now
        if self.stable_count < GPS_STABLE_FIX_COUNT:
            return None

        self.last_valid_fix_time = now
        self.last_valid_latlng = (lat, lng)
        msg = parsed["msg"]
        try:
            timestamp = msg.timestamp.strftime("%H:%M:%S")
        except Exception:
            timestamp = str(getattr(msg, "timestamp", ""))
        try:
            altitude = float(getattr(msg, "altitude", 0.0) or 0.0)
        except Exception:
            altitude = 0.0

        return {
            "lat": lat,
            "lng": lng,
            "lat_text": parsed.get("lat_text"),
            "lng_text": parsed.get("lng_text"),
            "gps_qual": gps_qual,
            "num_sats": num_sats,
            "hdop": hdop,
            "altitude": altitude,
            "timestamp": timestamp,
            "raw": parsed["raw"],
            "port": self.selected_port,
            "baud": self.selected_baud,
        }
