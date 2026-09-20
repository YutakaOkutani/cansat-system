import csv
import glob
import math
import os
import threading
import time

import cv2
from gpiozero import DigitalOutputDevice, LED, PWMOutputDevice
from gpiozero.pins.lgpio import LGPIOFactory

from lib import bmp180, bno055
from lib.sonar import SonarSensor
from lib import cone_detect as dc

from mission.const import (
    BNO_FUSION_OK_STATES,
    BNO_INIT_READY_TIMEOUT,
    BNO_INIT_SAMPLE_INTERVAL,
    BNO_SETUP_RETRY_COUNT,
    BNO_SETUP_RETRY_INTERVAL,
    DEVICE_BMP,
    DEVICE_BNO,
    DEVICE_DETECTOR,
    DEVICE_KEYS,
    DEVICE_LED_GREEN,
    DEVICE_LED_RED,
    DEVICE_MOTOR_1_DIR,
    DEVICE_MOTOR_1_PWM,
    DEVICE_MOTOR_2_DIR,
    DEVICE_MOTOR_2_PWM,
    DEVICE_SONAR,
    LOG_HEADER,
    PIN_ECHO,
    PIN_EN1,
    PIN_EN2,
    PIN_LED_GREEN,
    PIN_LED_RED,
    PIN_PH1,
    PIN_PH2,
    PIN_TRIG,
    PWM_FREQ,
    ROI_GLOB_PATTERNS,
    SONAR_MAX_DISTANCE,
)
from mission.paths import ROI_PRIMARY_REFERENCE


class HardwareManager:
    def close_hardware(self):
        """Release GPIO/camera resources so subset runners can exit cleanly."""
        self._release_camera_detector()
        closed_ids = set()
        for key, device in list(self.devices.items()):
            if device is None or id(device) in closed_ids:
                continue
            closed_ids.add(id(device))
            try:
                close_fn = getattr(device, "close", None)
                if callable(close_fn):
                    close_fn()
            except Exception as exc:
                print(f"Hardware close error ({key}): {exc}")
            finally:
                self.devices[key] = None

    def _bno_has_live_sample(self, bno):
        try:
            acc = bno.getAcc()
            mag = bno.getMag()
            euler = bno.getEuler()
            sys_status = bno.getSystemStatus()
            sys_error = bno.getSystemError()
        except Exception:
            return False, "sample_read_error"

        if not all(packet.get("valid", False) for packet in (acc, mag, euler, sys_status, sys_error)):
            return False, "sample_invalid"

        try:
            acc_norm = math.sqrt(sum(float(v) * float(v) for v in acc["value"][:3]))
            mag_norm = math.sqrt(sum(float(v) * float(v) for v in mag["value"][:3]))
            heading = float(euler["value"][0])
        except Exception:
            return False, "sample_parse_error"

        sys_state = int(sys_status["value"])
        sys_err = int(sys_error["value"])
        vectors_live = acc_norm > 0.5 and mag_norm > 0.5
        heading_live = math.isfinite(heading) and abs(heading) > 0.01
        fusion_ready = sys_state in BNO_FUSION_OK_STATES and sys_err == 0
        if vectors_live and (heading_live or fusion_ready):
            return True, "ready"
        return False, (
            f"acc_norm={acc_norm:.3f} mag_norm={mag_norm:.3f} "
            f"heading={heading:.3f} sys={sys_state} err={sys_err}"
        )

    def _wait_for_bno_ready(self, bno, timeout_sec=BNO_INIT_READY_TIMEOUT):
        deadline = time.time() + max(0.1, float(timeout_sec))
        last_reason = "timeout"
        while time.time() < deadline:
            ready, reason = self._bno_has_live_sample(bno)
            if ready:
                return True, "ready"
            last_reason = reason
            time.sleep(BNO_INIT_SAMPLE_INTERVAL)
        return False, last_reason

    def _classify_roi_reference(self, path):
        name = os.path.basename(path).lower()
        label = "positive"
        if any(token in name for token in ("false_", "fake_", "grass", "sky", "bg_")):
            label = "negative"

        weight = 1.0
        if "close" in name or "closeup" in name:
            weight *= 0.45
        if "3m" in name:
            weight *= 1.00
        if "5m" in name:
            weight *= 1.20
        if "sky" in name:
            weight *= 1.35
        if "grass" in name:
            weight *= 1.15
        if "backlit" in name:
            weight *= 1.10
        return label, weight

    def _load_roi_images(self):
        roi_images = []
        seen_paths = set()
        candidate_paths = []
        roi_path_1 = str(getattr(self, "roi_path_1", ROI_PRIMARY_REFERENCE))
        roi_path_2 = str(getattr(self, "roi_path_2", roi_path_1))
        for base_path in (roi_path_1, roi_path_2):
            if base_path and base_path not in candidate_paths:
                candidate_paths.append(base_path)
        roi_dir = os.path.dirname(roi_path_1)
        for pattern in ROI_GLOB_PATTERNS:
            for path in sorted(glob.glob(os.path.join(roi_dir, pattern))):
                if path not in candidate_paths:
                    candidate_paths.append(path)

        for path in candidate_paths:
            if not path or path in seen_paths or not os.path.exists(path):
                continue
            seen_paths.add(path)
            img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
            if img is None:
                print(f"WARNING: Failed to load ROI image: {path}")
                continue
            label, weight = self._classify_roi_reference(path)
            roi_images.append({
                "path": path,
                "image": img,
                "label": label,
                "weight": float(weight),
            })
            print(f"Loaded ROI image: {path} ({label}, weight={weight:.2f})")
        return roi_images

    def _release_camera_detector(self):
        detector = self.devices.get(DEVICE_DETECTOR)
        if detector is None:
            return
        try:
            close_fn = getattr(detector, "close", None)
            if callable(close_fn):
                close_fn()
        except Exception as exc:
            print(f"Camera: Detector close error {exc}.")
        finally:
            self.devices[DEVICE_DETECTOR] = None

    def _setup_bno_device(self):
        try:
            for i in range(BNO_SETUP_RETRY_COUNT):
                bno = bno055.BNO055()
                if bno.setUp():
                    ready, reason = self._wait_for_bno_ready(bno)
                    if ready:
                        self.devices[DEVICE_BNO] = bno
                        print("BNO055: OK")
                        return True
                    print(f"BNO055: setup succeeded but sensor stayed inactive ({reason})")
                print(f"BNO055: Retry {i + 1}...")
                time.sleep(BNO_SETUP_RETRY_INTERVAL)
            print("WARNING: BNO055 Init Failed.")
        except Exception as exc:
            print(f"BNO055: Critical Error {exc}.")
        return False

    def _setup_camera_detector(self):
        print("Camera: Initializing...")
        try:
            self._release_camera_detector()
            detector = dc.detector()
            roi_images = self._load_roi_images()
            roi_img = roi_images[0] if roi_images else None
            if roi_images:
                pos_count = sum(1 for item in roi_images if item["label"] == "positive")
                neg_count = sum(1 for item in roi_images if item["label"] == "negative")
                print(f"Camera: using {len(roi_images)} ROI image(s) (positive={pos_count}, negative={neg_count})")
            else:
                configured_roi_path = getattr(self, "roi_path_1", ROI_PRIMARY_REFERENCE)
                print(f"WARNING: ROI image not found at configured path: {configured_roi_path}")
                print("Switching to DEFAULT RED detection.")
            self.roi_img = roi_img
            self.roi_references = list(roi_images)
            run_bundle = getattr(self, "run_bundle", None)
            if run_bundle is not None:
                try:
                    run_bundle.snapshot_roi_inputs(self.roi_references)
                except Exception as exc:
                    print(f"WARNING: ROI input snapshot failed: {exc}")
            detector.set_roi_img(roi_images if roi_images else roi_img)
            detector.capture_reached_path = str(
                getattr(self, "capture_reached_path", os.path.join(".", "log", "capture_reached.png"))
            )
            set_evidence_dir = getattr(detector, "set_evidence_dir", None)
            if callable(set_evidence_dir):
                set_evidence_dir(getattr(self, "camera_evidence_dir", None))
            self.devices[DEVICE_DETECTOR] = detector
            print("Camera: OK (Initialized)")
            return True
        except Exception as exc:
            print(f"Camera: Critical Init Error {exc}. Proceeding without Vision.")
            self.devices[DEVICE_DETECTOR] = None
            return False

    def _setup_gpio_devices(self, include_sonar=True):
        print("GPIOZero: Initializing devices...")
        try:
            pin_factory = LGPIOFactory()
            self.devices[DEVICE_LED_RED] = LED(PIN_LED_RED, pin_factory=pin_factory)
            self.devices[DEVICE_LED_GREEN] = LED(PIN_LED_GREEN, pin_factory=pin_factory)
            self.devices[DEVICE_MOTOR_1_PWM] = PWMOutputDevice(
                PIN_EN1,
                pin_factory=pin_factory,
                frequency=PWM_FREQ,
                initial_value=0,
            )
            self.devices[DEVICE_MOTOR_1_DIR] = DigitalOutputDevice(
                PIN_PH1,
                pin_factory=pin_factory,
                initial_value=False,
            )
            self.devices[DEVICE_MOTOR_2_PWM] = PWMOutputDevice(
                PIN_EN2,
                pin_factory=pin_factory,
                frequency=PWM_FREQ,
                initial_value=0,
            )
            self.devices[DEVICE_MOTOR_2_DIR] = DigitalOutputDevice(
                PIN_PH2,
                pin_factory=pin_factory,
                initial_value=False,
            )
            if include_sonar:
                self.devices[DEVICE_SONAR] = SonarSensor(
                    echo=PIN_ECHO,
                    trigger=PIN_TRIG,
                    max_distance=SONAR_MAX_DISTANCE,
                    pin_factory=pin_factory,
                )
            self.motor_state = {}
            for key in (DEVICE_MOTOR_1_PWM, DEVICE_MOTOR_2_PWM):
                pwm_dev = self.devices.get(key)
                if pwm_dev:
                    self.motor_state[pwm_dev] = {"speed": 0.0, "direction": True}
            self.stop_motors()
            print("GPIOZero: OK")
            return True
        except Exception as exc:
            print(f"GPIOZero Setup Error {exc}.")
            return False

    def setup_hardware(self):
        print("--- Robust Setup Start ---")
        self.devices = {key: None for key in DEVICE_KEYS}

        self._setup_bno_device()

        try:
            bmp = bmp180.BMP180(oss=3)
            if bmp.setUp():
                self.devices[DEVICE_BMP] = bmp
                print("BMP180: OK")
            else:
                print("WARNING: BMP180 Init Failed.")
        except Exception as exc:
            print(f"BMP180: Critical Error {exc}.")

        # Vision is not needed before Phase4.  In particular, do not create or
        # open a camera pipeline during P0-P3: a loose CSI connector must stay
        # isolated from navigation and motor control.  camera_thread activates
        # the detector only after the mission actually enters P4/P5.
        self.devices[DEVICE_DETECTOR] = None
        self._camera_runtime_active = False
        print("Camera: Deferred until Phase4.")

        self._setup_gpio_devices(include_sonar=True)

        # Create the log file before background threads start writing to it.
        # This avoids a race where data rows are appended and then overwritten
        # by the header initialization.
        self.init_log_file()
        self.start_threads()
        print("--- Setup Finished (Ready to Die Trying) ---")

    def start_threads(self):
        try:
            threading.Thread(target=self.move_motor_thread, daemon=True).start()
            threading.Thread(target=self.bno_thread, daemon=True).start()
            threading.Thread(target=self.sonar_thread, daemon=True).start()
            threading.Thread(target=self.log_thread, daemon=True).start()
            threading.Thread(target=self.gps_thread, daemon=True).start()
            threading.Thread(target=self.camera_thread, daemon=True).start()
        except Exception as exc:
            print(f"Thread Start Error {exc}.")

    def init_log_file(self):
        try:
            with open(self.log_path, "w", newline="") as file_obj:
                writer = csv.writer(file_obj)
                writer.writerow(LOG_HEADER)
                file_obj.flush()
                # Avoid fsync stalls on Windows/OneDrive test environments.
                if os.name != "nt":
                    os.fsync(file_obj.fileno())
        except Exception:
            print("Log File Init Failed. No logging.")
