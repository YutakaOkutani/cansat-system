import threading
import time

from lib.cone_diagnostics import normalize_cone_diagnostics

from mission.const import (
    CONE_CENTER_POSITION,
    DEFAULT_FLOAT_VALUE,
    DEFAULT_SONAR_DIST_CM,
    DEFAULT_PHASE,
    DEFAULT_VECTOR3,
    SONAR_STALE_TIMEOUT_SEC,
)


class CanSatState:
    def __init__(self):
        self.lock = threading.Lock()
        self.acc = list(DEFAULT_VECTOR3)
        self.gyro = list(DEFAULT_VECTOR3)
        self.mag = list(DEFAULT_VECTOR3)
        self.lat = DEFAULT_FLOAT_VALUE
        self.lng = DEFAULT_FLOAT_VALUE
        self.gps_heading = DEFAULT_FLOAT_VALUE
        self.gps_heading_valid = False
        self.gps_speed_mps = DEFAULT_FLOAT_VALUE
        self.gps_fix_qual = 0
        self.gps_sats = 0
        self.gps_hdop = DEFAULT_FLOAT_VALUE
        self.gps_fix_seq = 0
        self.nav_heading = DEFAULT_FLOAT_VALUE
        self.nav_heading_source = ""
        self.heading_diff = DEFAULT_FLOAT_VALUE
        self.heading_trust = DEFAULT_FLOAT_VALUE
        self.bno_trusted = False
        self.bno_offset_deg = DEFAULT_FLOAT_VALUE
        self.bno_offset_valid = False
        self.gps_heading_baseline_m = DEFAULT_FLOAT_VALUE
        self.arrival_inside = False
        self.arrival_confirm_count = 0
        self.phase3_arrived_latched = False
        self.alt = DEFAULT_FLOAT_VALUE
        self.pres = DEFAULT_FLOAT_VALUE
        self.distance = DEFAULT_FLOAT_VALUE
        self.azimuth = DEFAULT_FLOAT_VALUE
        self.angle = DEFAULT_FLOAT_VALUE
        self.angle_valid = False
        self.direction = DEFAULT_FLOAT_VALUE
        self.fall = DEFAULT_FLOAT_VALUE
        self.cone_direction = CONE_CENTER_POSITION
        self.cone_image_direction = CONE_CENTER_POSITION
        self.cone_image_direction_valid = False
        self.cone_probability = DEFAULT_FLOAT_VALUE
        self.cone_method = ""
        self.cone_valid = False
        self.cone_status = "not_started"
        self.cone_sequence = 0
        self.cone_updated_at = 0.0
        self.cone_last_valid_at = 0.0
        self.cone_debug = normalize_cone_diagnostics()
        self.sonar_distance_cm = DEFAULT_SONAR_DIST_CM
        self.sonar_sequence = 0
        self.sonar_observed_at = 0.0
        self.sonar_observed_monotonic = 0.0
        self.sonar_valid = False
        self.sonar_stale_sec = SONAR_STALE_TIMEOUT_SEC + 1.0
        self.phase = DEFAULT_PHASE
        self.gps_detect = 0
        self.cone_is_reached = False

    def update_imu(self, acc=None, gyro=None, mag=None, fall=None, angle=None, angle_valid=None):
        with self.lock:
            if acc is not None:
                self.acc = acc
            if gyro is not None:
                self.gyro = gyro
            if mag is not None:
                self.mag = mag
            if fall is not None:
                self.fall = fall
            if angle is not None:
                self.angle = angle
            if angle_valid is not None:
                self.angle_valid = angle_valid

    def update_gps(
        self,
        lat=None,
        lng=None,
        gps_detect=None,
        gps_heading=None,
        gps_heading_valid=None,
        gps_speed_mps=None,
        gps_fix_qual=None,
        gps_sats=None,
        gps_hdop=None,
        gps_heading_baseline_m=None,
        gps_fix_accepted=False,
    ):
        with self.lock:
            if lat is not None:
                self.lat = lat
            if lng is not None:
                self.lng = lng
            if gps_detect is not None:
                self.gps_detect = gps_detect
            if gps_heading is not None:
                self.gps_heading = gps_heading
            if gps_heading_valid is not None:
                self.gps_heading_valid = gps_heading_valid
            if gps_speed_mps is not None:
                self.gps_speed_mps = gps_speed_mps
            if gps_fix_qual is not None:
                self.gps_fix_qual = gps_fix_qual
            if gps_sats is not None:
                self.gps_sats = gps_sats
            if gps_hdop is not None:
                self.gps_hdop = gps_hdop
            if gps_heading_baseline_m is not None:
                self.gps_heading_baseline_m = gps_heading_baseline_m
            if gps_fix_accepted:
                self.gps_fix_seq += 1

    def update_barometer(self, alt=None, pres=None):
        with self.lock:
            if alt is not None:
                self.alt = alt
            if pres is not None:
                self.pres = pres

    def update_navigation(
        self,
        distance=None,
        azimuth=None,
        direction=None,
        phase=None,
        nav_heading=None,
        nav_heading_source=None,
        heading_diff=None,
        heading_trust=None,
        bno_trusted=None,
        bno_offset_deg=None,
        bno_offset_valid=None,
        arrival_inside=None,
        arrival_confirm_count=None,
        phase3_arrived_latched=None,
    ):
        with self.lock:
            if distance is not None:
                self.distance = distance
            if azimuth is not None:
                self.azimuth = azimuth
            if direction is not None:
                self.direction = direction
            if phase is not None:
                self.phase = phase
            if nav_heading is not None:
                self.nav_heading = nav_heading
            if nav_heading_source is not None:
                self.nav_heading_source = nav_heading_source
            if heading_diff is not None:
                self.heading_diff = heading_diff
            if heading_trust is not None:
                self.heading_trust = heading_trust
            if bno_trusted is not None:
                self.bno_trusted = bno_trusted
            if bno_offset_deg is not None:
                self.bno_offset_deg = bno_offset_deg
            if bno_offset_valid is not None:
                self.bno_offset_valid = bno_offset_valid
            if arrival_inside is not None:
                self.arrival_inside = arrival_inside
            if arrival_confirm_count is not None:
                self.arrival_confirm_count = arrival_confirm_count
            if phase3_arrived_latched is not None:
                self.phase3_arrived_latched = phase3_arrived_latched

    def update_cone(
        self,
        cone_direction=None,
        cone_image_direction=None,
        cone_probability=None,
        cone_is_reached=None,
        cone_method=None,
        cone_valid=None,
        cone_status=None,
        cone_debug=None,
        observation_time=None,
        observation_accepted=False,
    ):
        with self.lock:
            if cone_direction is not None:
                self.cone_direction = cone_direction
            if cone_image_direction is not None:
                self.cone_image_direction = cone_image_direction
                self.cone_image_direction_valid = True
            if cone_probability is not None:
                self.cone_probability = cone_probability
            if cone_is_reached is not None:
                self.cone_is_reached = cone_is_reached
            if cone_method is not None:
                self.cone_method = cone_method
            if cone_valid is not None:
                self.cone_valid = bool(cone_valid)
            if cone_status is not None:
                self.cone_status = str(cone_status)
            if observation_time is not None or observation_accepted or cone_valid is not None:
                observed_at = float(observation_time) if observation_time is not None else time.time()
                self.cone_updated_at = observed_at
                if observation_accepted:
                    self.cone_sequence += 1
                    if self.cone_valid:
                        self.cone_last_valid_at = observed_at

            diagnostics = dict(self.cone_debug)
            if cone_debug is not None:
                diagnostics.update(cone_debug)
            diagnostics.update(
                {
                    "valid": int(self.cone_valid),
                    "status": self.cone_status,
                    "sequence": self.cone_sequence,
                    "direction": self.cone_direction,
                    "probability": self.cone_probability,
                    "method": self.cone_method,
                    "is_reached": int(bool(self.cone_is_reached)),
                }
            )
            if self.cone_image_direction_valid:
                diagnostics.update(
                    {
                        "image_direction": self.cone_image_direction,
                        "image_direction_valid": 1,
                    }
                )
            self.cone_debug = normalize_cone_diagnostics(diagnostics)

    def update_sonar(self, sonar_distance_cm=None, sonar_valid=None, sonar_stale_sec=None,
                     sonar_sequence=None, sonar_observed_at=None, sonar_observed_monotonic=None):
        with self.lock:
            if sonar_sequence is not None:
                self.sonar_sequence = int(sonar_sequence)
            if sonar_observed_at is not None:
                self.sonar_observed_at = float(sonar_observed_at)
            if sonar_observed_monotonic is not None:
                self.sonar_observed_monotonic = float(sonar_observed_monotonic)
            if sonar_distance_cm is not None:
                self.sonar_distance_cm = sonar_distance_cm
            if sonar_valid is not None:
                self.sonar_valid = bool(sonar_valid)
            if sonar_stale_sec is not None:
                self.sonar_stale_sec = float(sonar_stale_sec)

    def snapshot(self):
        with self.lock:
            return {
                "acc": list(self.acc),
                "gyro": list(self.gyro),
                "mag": list(self.mag),
                "lat": self.lat,
                "lng": self.lng,
                "gps_heading": self.gps_heading,
                "gps_heading_valid": self.gps_heading_valid,
                "gps_speed_mps": self.gps_speed_mps,
                "gps_fix_qual": self.gps_fix_qual,
                "gps_sats": self.gps_sats,
                "gps_hdop": self.gps_hdop,
                "gps_fix_seq": self.gps_fix_seq,
                "nav_heading": self.nav_heading,
                "nav_heading_source": self.nav_heading_source,
                "heading_diff": self.heading_diff,
                "heading_trust": self.heading_trust,
                "bno_trusted": self.bno_trusted,
                "bno_offset_deg": self.bno_offset_deg,
                "bno_offset_valid": self.bno_offset_valid,
                "gps_heading_baseline_m": self.gps_heading_baseline_m,
                "arrival_inside": self.arrival_inside,
                "arrival_confirm_count": self.arrival_confirm_count,
                "phase3_arrived_latched": self.phase3_arrived_latched,
                "alt": self.alt,
                "pres": self.pres,
                "distance": self.distance,
                "azimuth": self.azimuth,
                "angle": self.angle,
                "angle_valid": self.angle_valid,
                "direction": self.direction,
                "fall": self.fall,
                "cone_direction": self.cone_direction,
                "cone_image_direction": self.cone_image_direction,
                "cone_image_direction_valid": self.cone_image_direction_valid,
                "cone_probability": self.cone_probability,
                "cone_method": self.cone_method,
                "cone_valid": self.cone_valid,
                "cone_status": self.cone_status,
                "cone_sequence": self.cone_sequence,
                "cone_updated_at": self.cone_updated_at,
                "cone_last_valid_at": self.cone_last_valid_at,
                "cone_debug": dict(self.cone_debug),
                "sonar_sequence": self.sonar_sequence,
                "sonar_observed_at": self.sonar_observed_at,
                "sonar_observed_monotonic": self.sonar_observed_monotonic,
                "sonar_distance_cm": self.sonar_distance_cm,
                "sonar_valid": self.sonar_valid,
                "sonar_stale_sec": self.sonar_stale_sec,
                "phase": self.phase,
                "gps_detect": self.gps_detect,
                "cone_is_reached": self.cone_is_reached,
            }
