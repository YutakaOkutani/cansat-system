import argparse
import sys
import time
from pathlib import Path

# Allow running this file directly (e.g. `python runs/diag/motor.py`) by adding
# the repository root to sys.path so `mission` can be imported.
REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from mission.const import (
    CAMERA_EDGE_TURN_SPEED,
    CAMERA_EDGE_TURN_INNER_SPEED,
    GRASS_MIN_MOTOR_SPEED,
    MANUAL_TURN_SPEED_RATIO,
    MOTOR_LEFT_MTR_INDEX,
    MOTOR_RIGHT_MTR_INDEX,
    MOTOR_SPEED_OFFSET_1,
    MOTOR_SPEED_OFFSET_2,
    MOTOR_SPEED_SCALE_1,
    MOTOR_SPEED_SCALE_2,
    MOTOR_DRIVE_MIN_SPEED,
    PARACHUTE_SEPARATION_SPEED,
    PHASE1_SOFTSTART_RAMP_TIME,
    PHASE1_SOFTSTART_STEP,
    PHASE2_CALIB_ARC_INNER_SPEED,
    PHASE2_CALIB_ARC_OUTER_SPEED,
    PHASE2_FORWARD_REORIENT_INNER_SPEED,
    PHASE2_FORWARD_REORIENT_OUTER_SPEED,
    PHASE2_OFFSET_CORRECTION_BASE_SPEED,
    PHASE2_OFFSET_HOLD_MAX_DELTA,
    PHASE2_OFFSET_SPEED,
    PHASE2_RAMP_TIME,
    PHASE2_SPEED,
    PHASE3_FORWARD_RAMP_TIME,
    PHASE3_FORWARD_SPEED,
    PHASE3_GPS_ARC_BASE_SPEED,
    PHASE3_GPS_ARC_MAX_DELTA,
    PHASE3_LARGE_ERROR_INNER_SPEED,
    PHASE3_LARGE_ERROR_OUTER_SPEED,
    PHASE3_TURN_INNER_SPEED,
    PHASE3_TURN_OUTER_SPEED,
    PHASE3_TURN_RAMP_TIME,
    PHASE4_ALIGN_FORWARD_SPEED,
    PHASE4_ALIGN_INNER_SPEED,
    PHASE4_ALIGN_PIVOT_SPEED,
    PHASE4_MOTOR_RAMP_TIME,
    PHASE5_BASE_SPEED,
    PHASE5_MID_SPEED,
    PHASE5_NEAR_SPEED,
    PHASE5_MOTOR_RAMP_TIME,
    PHASE5_TURN_CLAMP,
    PHASE6_APPROACH_RAMP_TIME,
    GOAL_PULSE_SEC,
    PHASE6_APPROACH_SPEED,
    PIN_EN1,
    PIN_EN2,
    PIN_PH1,
    PIN_PH2,
    PWM_FREQ,
    PHASE4_SEARCH_INNER_SPEED,
    PHASE4_SEARCH_OUTER_SPEED,
)
from mission.motor_map import (
    apply_drive_speed_floor,
    get_manual_drive_pattern,
    map_logical_wheels_to_physical,
    motor_forward_to_dir_value,
)

DEFAULT_SPEED = 100  # Default duty for manual control (0-100)
SPEED_STEP = 5  # Duty adjustment step for interactive test (0-100)
COMMAND_BUFFER_SEC = 0.25  # Delay before applying a new command to avoid regen spikes
DEFAULT_RAMP_TIME = 0.6
DEFAULT_STEP_INTERVAL = 0.05

# gpiozero devices (created in setup())
pin_factory = None
motor_1_pwm = None
motor_1_dir = None
motor_2_pwm = None
motor_2_dir = None
motor_state = {
    'A': {'speed': 0.0, 'direction': 1},
    'B': {'speed': 0.0, 'direction': 1},
}


def _command(
    label,
    speed_left,
    forward_left,
    speed_right,
    forward_right,
    ramp_time,
    step_interval=DEFAULT_STEP_INTERVAL,
):
    return {
        "label": label,
        "speed_left": float(speed_left),
        "forward_left": bool(forward_left),
        "speed_right": float(speed_right),
        "forward_right": bool(forward_right),
        "ramp_time": float(ramp_time),
        "step_interval": float(step_interval),
    }


def _wasd_commands(
    straight_speed,
    turn_outer,
    turn_inner,
    ramp_straight,
    ramp_turn=None,
    step_interval=DEFAULT_STEP_INTERVAL,
):
    """Build logical-wheel WASD commands from one production output profile."""
    if ramp_turn is None:
        ramp_turn = ramp_straight
    return {
        "w": _command(
            "Forward",
            straight_speed,
            True,
            straight_speed,
            True,
            ramp_straight,
            step_interval,
        ),
        "s": _command(
            "Backward (diagnostic)",
            straight_speed,
            False,
            straight_speed,
            False,
            ramp_straight,
            step_interval,
        ),
        "a": _command(
            "Left",
            turn_inner,
            True,
            turn_outer,
            True,
            ramp_turn,
            step_interval,
        ),
        "d": _command(
            "Right",
            turn_outer,
            True,
            turn_inner,
            True,
            ramp_turn,
            step_interval,
        ),
    }


def _straight_only_commands(
    speed,
    ramp_time,
    step_interval=DEFAULT_STEP_INTERVAL,
    minimum_turn_speed=0.0,
):
    """Expose straight-only phases through WASD without hiding that A/D are diagnostic."""
    turn_inner = max(
        float(speed) * float(MANUAL_TURN_SPEED_RATIO),
        float(minimum_turn_speed),
    )
    # Preserve the configured turn ratio when applying a grass-safe inner-wheel floor.
    turn_outer = max(
        float(speed),
        turn_inner / float(MANUAL_TURN_SPEED_RATIO),
    )
    commands = _wasd_commands(
        speed,
        turn_outer,
        turn_inner,
        ramp_time,
        ramp_time,
        step_interval,
    )
    commands["a"]["label"] = "Left (diagnostic; phase is straight-only)"
    commands["d"]["label"] = "Right (diagnostic; phase is straight-only)"
    return commands


def _profile(name, description, commands):
    return {
        "name": name,
        "description": description,
        "commands": commands,
    }


_gps_arc_delta = float(PHASE3_GPS_ARC_MAX_DELTA)
_gps_arc_outer = min(100.0, float(PHASE3_GPS_ARC_BASE_SPEED) + _gps_arc_delta / 2.0)
_gps_arc_inner = max(0.0, float(PHASE3_GPS_ARC_BASE_SPEED) - _gps_arc_delta / 2.0)
_phase5_outer = min(100.0, float(PHASE5_BASE_SPEED) + float(PHASE5_TURN_CLAMP))
_phase5_inner = max(
    float(GRASS_MIN_MOTOR_SPEED),
    float(PHASE5_BASE_SPEED) - float(PHASE5_TURN_CLAMP),
)
_phase2_offset_outer = min(
    100.0,
    float(PHASE2_OFFSET_CORRECTION_BASE_SPEED) + float(PHASE2_OFFSET_HOLD_MAX_DELTA),
)
_phase2_offset_inner = max(
    0.0,
    float(PHASE2_OFFSET_CORRECTION_BASE_SPEED) - float(PHASE2_OFFSET_HOLD_MAX_DELTA),
)

PHASE_DRIVE_PROFILES = {
    "1": (
        _profile(
            "separation",
            "P1 parachute separation; W is the production 100% output, A/D are diagnostic turns.",
            _straight_only_commands(
                PARACHUTE_SEPARATION_SPEED,
                PHASE1_SOFTSTART_RAMP_TIME,
                PHASE1_SOFTSTART_STEP,
            ),
        ),
    ),
    "2": (
        _profile(
            "escape",
            "P2 parachute escape straight output; A/D use the manual ratio for field diagnosis.",
            _straight_only_commands(PHASE2_SPEED, PHASE2_RAMP_TIME),
        ),
        _profile(
            "calibration_arc",
            "P2 grass-torque magnetic-calibration arc; A/D reproduce the two production directions.",
            _wasd_commands(
                PHASE2_CALIB_ARC_OUTER_SPEED,
                PHASE2_CALIB_ARC_OUTER_SPEED,
                PHASE2_CALIB_ARC_INNER_SPEED,
                PHASE2_RAMP_TIME,
            ),
        ),
        _profile(
            "offset_hold_max",
            "P2 straight offset collection and its maximum heading correction.",
            _wasd_commands(
                PHASE2_OFFSET_SPEED,
                _phase2_offset_outer,
                _phase2_offset_inner,
                PHASE2_RAMP_TIME,
            ),
        ),
        _profile(
            "reorient",
            "P2 grass-torque retry reorientation forward arc.",
            _wasd_commands(
                PHASE2_FORWARD_REORIENT_OUTER_SPEED,
                PHASE2_FORWARD_REORIENT_OUTER_SPEED,
                PHASE2_FORWARD_REORIENT_INNER_SPEED,
                PHASE2_RAMP_TIME,
            ),
        ),
    ),
    "3": (
        _profile(
            "navigation",
            "P3 verified-BNO straight and normal turn outputs.",
            _wasd_commands(
                PHASE3_FORWARD_SPEED,
                PHASE3_TURN_OUTER_SPEED,
                PHASE3_TURN_INNER_SPEED,
                PHASE3_FORWARD_RAMP_TIME,
                PHASE3_TURN_RAMP_TIME,
            ),
        ),
        _profile(
            "large_arc",
            "P3 large-heading-error high-torque arc; both wheels stay powered.",
            _wasd_commands(
                PHASE3_FORWARD_SPEED,
                PHASE3_LARGE_ERROR_OUTER_SPEED,
                PHASE3_LARGE_ERROR_INNER_SPEED,
                PHASE3_FORWARD_RAMP_TIME,
                PHASE3_TURN_RAMP_TIME,
            ),
        ),
        _profile(
            "gps_arc_max",
            "P3 GPS-only maximum correction and straight probe output.",
            _wasd_commands(
                PHASE3_FORWARD_SPEED,
                _gps_arc_outer,
                _gps_arc_inner,
                PHASE3_FORWARD_RAMP_TIME,
                PHASE3_TURN_RAMP_TIME,
            ),
        ),
    ),
    "4": (
        _profile(
            "search_arc",
            "P4 production camera-search arc; A reproduces phase4_search_arc.",
            _wasd_commands(
                PHASE4_SEARCH_OUTER_SPEED,
                PHASE4_SEARCH_OUTER_SPEED,
                PHASE4_SEARCH_INNER_SPEED,
                PHASE4_MOTOR_RAMP_TIME,
            ),
        ),
        _profile(
            "camera_align",
            "P4 production camera-centering forward and phase4_camera_align_arc outputs.",
            _wasd_commands(
                PHASE4_ALIGN_FORWARD_SPEED,
                PHASE4_ALIGN_PIVOT_SPEED,
                PHASE4_ALIGN_INNER_SPEED,
                PHASE4_MOTOR_RAMP_TIME,
            ),
        ),
    ),
    "5": (
        _profile(
            "approach",
            "P5 cone-approach forward and differential arc; use edge_recovery for a cone at the image edge.",
            _wasd_commands(
                PHASE5_BASE_SPEED,
                _phase5_outer,
                _phase5_inner,
                PHASE5_MOTOR_RAMP_TIME,
            ),
        ),
        _profile(
            "approach_mid",
            "P5 medium-distance approach and maximum steering outputs.",
            _wasd_commands(
                PHASE5_MID_SPEED,
                min(100.0, PHASE5_MID_SPEED + PHASE5_TURN_CLAMP),
                max(GRASS_MIN_MOTOR_SPEED, PHASE5_MID_SPEED - PHASE5_TURN_CLAMP),
                PHASE5_MOTOR_RAMP_TIME,
            ),
        ),
        _profile(
            "approach_near",
            "P5 near-cone approach and maximum steering before final alignment.",
            _wasd_commands(
                PHASE5_NEAR_SPEED,
                min(100.0, PHASE5_NEAR_SPEED + PHASE5_TURN_CLAMP),
                max(GRASS_MIN_MOTOR_SPEED, PHASE5_NEAR_SPEED - PHASE5_TURN_CLAMP),
                PHASE5_MOTOR_RAMP_TIME,
            ),
        ),
    ),
    "6": (
        _profile(
            "final_approach",
            f"P6 range approach (production pulse: {GOAL_PULSE_SEC:.1f}s; "
            "diagnostic: runs until stop); A/D are diagnostic turns.",
            _straight_only_commands(
                PHASE6_APPROACH_SPEED,
                PHASE6_APPROACH_RAMP_TIME,
                minimum_turn_speed=GRASS_MIN_MOTOR_SPEED,
            ),
        ),
    ),
    "7": (
        _profile(
            "stopped",
            "P7 shutdown state; every WASD key keeps both motors stopped.",
            {
                key: _command("Stopped", 0.0, True, 0.0, True, 0.0)
                for key in ("w", "a", "s", "d")
            },
        ),
    ),
}

for _phase, _speed, _ramp in (
    ("4", PHASE4_ALIGN_FORWARD_SPEED, PHASE4_MOTOR_RAMP_TIME),
    ("5", PHASE5_BASE_SPEED, PHASE5_MOTOR_RAMP_TIME),
):
    PHASE_DRIVE_PROFILES[_phase] += (
        _profile(
            "edge_recovery",
            "Image-edge recovery: moderated forward arc; W/S are diagnostic straight commands.",
            _wasd_commands(_speed, CAMERA_EDGE_TURN_SPEED, CAMERA_EDGE_TURN_INNER_SPEED, _ramp),
        ),
    )

def setup():
    """Initialize gpiozero devices."""
    global pin_factory, motor_1_pwm, motor_1_dir, motor_2_pwm, motor_2_dir
    from gpiozero import PWMOutputDevice, DigitalOutputDevice
    from gpiozero.pins.lgpio import LGPIOFactory

    pin_factory = LGPIOFactory()
    motor_1_pwm = PWMOutputDevice(PIN_EN1, pin_factory=pin_factory, frequency=PWM_FREQ, initial_value=0)
    motor_1_dir = DigitalOutputDevice(PIN_PH1, pin_factory=pin_factory, initial_value=False)
    motor_2_pwm = PWMOutputDevice(PIN_EN2, pin_factory=pin_factory, frequency=PWM_FREQ, initial_value=0)
    motor_2_dir = DigitalOutputDevice(PIN_PH2, pin_factory=pin_factory, initial_value=False)

    stop()

    print(
        f"Setup Complete: gpiozero initialized. "
        f"speed_scale A={MOTOR_SPEED_SCALE_1:.3f}, B={MOTOR_SPEED_SCALE_2:.3f}, "
        f"speed_offset A={MOTOR_SPEED_OFFSET_1:.1f}, B={MOTOR_SPEED_OFFSET_2:.1f}"
    )


def _ramp_pwm(pwm_dev, start_speed, target_speed, ramp_time, step_interval=0.05):
    """Ramp PWM duty in small steps to avoid sudden current draw."""
    if pwm_dev is None:
        return target_speed

    # Immediate set if ramping is disabled or step is invalid.
    if ramp_time <= 0 or step_interval <= 0:
        pwm_dev.value = max(0.0, min(1.0, target_speed / 100.0))
        return target_speed

    steps = max(1, int(ramp_time / step_interval))
    step_duration = ramp_time / steps
    for step in range(1, steps + 1):
        duty = start_speed + (target_speed - start_speed) * (step / steps)
        pwm_dev.value = max(0.0, min(1.0, duty / 100.0))
        time.sleep(step_duration)

    return target_speed


def _ramp_pwm_dual(pwm_a, start_a, target_a, pwm_b, start_b, target_b, ramp_time, step_interval=0.05):
    """Ramp two PWM devices together so both motors start/stop in sync."""
    if pwm_a is None and pwm_b is None:
        return start_a, start_b

    # Immediate set if ramping is disabled or step is invalid.
    if ramp_time <= 0 or step_interval <= 0:
        if pwm_a is not None:
            pwm_a.value = max(0.0, min(1.0, target_a / 100.0))
        if pwm_b is not None:
            pwm_b.value = max(0.0, min(1.0, target_b / 100.0))
        return target_a, target_b

    steps = max(1, int(ramp_time / step_interval))
    step_duration = ramp_time / steps
    for step in range(1, steps + 1):
        duty_a = start_a + (target_a - start_a) * (step / steps)
        duty_b = start_b + (target_b - start_b) * (step / steps)
        if pwm_a is not None:
            pwm_a.value = max(0.0, min(1.0, duty_a / 100.0))
        if pwm_b is not None:
            pwm_b.value = max(0.0, min(1.0, duty_b / 100.0))
        time.sleep(step_duration)

    return target_a, target_b


def _apply_speed_scale(speed, motor_side):
    """Apply per-motor PWM trim (scale + offset) to compensate differences."""
    scale = MOTOR_SPEED_SCALE_1 if motor_side == 'A' else MOTOR_SPEED_SCALE_2
    offset = MOTOR_SPEED_OFFSET_1 if motor_side == 'A' else MOTOR_SPEED_OFFSET_2
    adjusted = float(speed) * max(0.0, float(scale)) + float(offset)
    return max(0.0, min(100.0, adjusted))


def set_motor(motor_side, speed, direction, ramp_time=0.6, step_interval=0.05):
    """
    Control motor duty and direction with a soft-start ramp.
    :param motor_side: physical 'A' (MTR1 / left) or 'B' (MTR2 / right)
    :param speed: PWM Duty Cycle (0 - 100)
    :param direction: 1 (Forward/High) or 0 (Reverse/Low)
    :param ramp_time: Time in seconds to ramp between duty changes.
    :param step_interval: Interval between duty steps.
    """
    if motor_side == 'A':
        pwm_dev = motor_1_pwm
        dir_dev = motor_1_dir
    elif motor_side == 'B':
        pwm_dev = motor_2_pwm
        dir_dev = motor_2_dir
    else:
        return

    if pwm_dev is None or dir_dev is None:
        return

    state = motor_state[motor_side]
    current_speed = state['speed']
    current_direction = state['direction']

    # If the direction changes, ramp to zero first to reduce stress on the driver.
    if current_speed > 0 and direction != current_direction:
        current_speed = _ramp_pwm(pwm_dev, current_speed, 0, ramp_time / 2, step_interval)

    # Set direction (PH Pin)
    # Match production polarity handling through invert flags.
    motor_index = 1 if motor_side == 'A' else 2
    dir_dev.value = motor_forward_to_dir_value(motor_index, direction)

    # Set PWM duty with ramp (EN Pin - PWM 0.0-1.0)
    target_speed = _apply_speed_scale(speed, motor_side)
    current_speed = _ramp_pwm(pwm_dev, current_speed, target_speed, ramp_time, step_interval)
    state['speed'] = current_speed
    state['direction'] = direction


def _target_motor_speeds(speed_left, forward_left, speed_right, forward_right):
    """Return the same physical PWM targets for display and GPIO output."""
    speed_motor_1, _, speed_motor_2, _ = map_logical_wheels_to_physical(
        speed_left, forward_left, speed_right, forward_right,
    )
    target_motor_1 = _apply_speed_scale(speed_motor_1, 'A')
    target_motor_2 = _apply_speed_scale(speed_motor_2, 'B')
    target_motor_1, target_motor_2 = apply_drive_speed_floor(
        target_motor_1,
        target_motor_2,
        same_direction=bool(forward_left) == bool(forward_right),
    )
    return target_motor_1, target_motor_2


def set_motors(
    speed_left,
    forward_left,
    speed_right,
    forward_right,
    ramp_time=0.6,
    step_interval=0.05,
):
    """
    Control both logical wheels together with a synchronized ramp.
    :param speed_left: PWM Duty Cycle (0 - 100) for the left wheel
    :param forward_left: logical forward state for the left wheel
    :param speed_right: PWM Duty Cycle (0 - 100) for the right wheel
    :param forward_right: logical forward state for the right wheel
    :param ramp_time: Time in seconds to ramp between duty changes.
    :param step_interval: Interval between duty steps.
    """
    if motor_1_pwm is None or motor_1_dir is None or motor_2_pwm is None or motor_2_dir is None:
        return

    (
        speed_motor_1,
        forward_motor_1,
        speed_motor_2,
        forward_motor_2,
    ) = map_logical_wheels_to_physical(
        speed_left,
        forward_left,
        speed_right,
        forward_right,
    )
    state_motor_1 = motor_state['A']
    state_motor_2 = motor_state['B']
    current_motor_1 = state_motor_1['speed']
    current_motor_2 = state_motor_2['speed']

    # If either direction changes, ramp both to zero first to reduce stress and keep sync.
    if (
        current_motor_1 > 0
        and forward_motor_1 != state_motor_1['direction']
    ) or (
        current_motor_2 > 0
        and forward_motor_2 != state_motor_2['direction']
    ):
        current_motor_1, current_motor_2 = _ramp_pwm_dual(
            motor_1_pwm, current_motor_1, 0,
            motor_2_pwm, current_motor_2, 0,
            ramp_time / 2, step_interval
        )

    # Match production polarity handling through invert flags.
    motor_1_dir.value = motor_forward_to_dir_value(1, forward_motor_1)
    motor_2_dir.value = motor_forward_to_dir_value(2, forward_motor_2)

    target_motor_1, target_motor_2 = _target_motor_speeds(
        speed_left, forward_left, speed_right, forward_right,
    )
    print(f"Target PWM after trim/forward/reverse/turn floor: L={target_motor_1:.1f}% R={target_motor_2:.1f}%")
    current_motor_1, current_motor_2 = _ramp_pwm_dual(
        motor_1_pwm, current_motor_1, target_motor_1,
        motor_2_pwm, current_motor_2, target_motor_2,
        ramp_time, step_interval
    )

    state_motor_1['speed'] = current_motor_1
    state_motor_1['direction'] = forward_motor_1
    state_motor_2['speed'] = current_motor_2
    state_motor_2['direction'] = forward_motor_2


def stop():
    """Stop both motors and reset cached speed state."""
    if motor_1_pwm:
        motor_1_pwm.value = 0
    if motor_2_pwm:
        motor_2_pwm.value = 0
    if motor_1_dir:
        motor_1_dir.off()
    if motor_2_dir:
        motor_2_dir.off()
    motor_state['A']['speed'] = 0.0
    motor_state['B']['speed'] = 0.0



def _apply_manual_drive_pattern(cmd, speed=DEFAULT_SPEED):
    """Apply the production manual drive mapping through the local test motor driver."""
    pattern = get_manual_drive_pattern(cmd, speed)
    if pattern is None:
        return False
    set_motors(
        pattern["speed_a"],
        int(pattern["forward_a"]),
        pattern["speed_b"],
        int(pattern["forward_b"]),
    )
    return True


def _normalize_phase(value):
    phase = str(value or "manual").strip().lower()
    if phase in ("0", "manual", "m"):
        return "manual"
    if phase.startswith("p"):
        phase = phase[1:]
    if phase in PHASE_DRIVE_PROFILES:
        return phase
    raise ValueError("phase must be manual/0 or 1 through 7")


def _profile_index_for_name(phase, profile_name):
    profiles = PHASE_DRIVE_PROFILES.get(phase, ())
    if not profiles:
        return 0
    if profile_name is None:
        return 0
    wanted = str(profile_name).strip().lower()
    for index, profile in enumerate(profiles):
        if profile["name"].lower() == wanted:
            return index
    names = ", ".join(profile["name"] for profile in profiles)
    raise ValueError(f"unknown P{phase} profile {profile_name!r}; choose one of: {names}")


def get_phase_drive_pattern(phase, profile_index, cmd):
    """Return one exact phase-profile command in logical left/right order."""
    phase = _normalize_phase(phase)
    if phase == "manual":
        return None
    profiles = PHASE_DRIVE_PROFILES[phase]
    profile = profiles[int(profile_index) % len(profiles)]
    return profile["commands"].get((cmd or "").lower())


def _apply_phase_drive_pattern(phase, profile_index, cmd):
    pattern = get_phase_drive_pattern(phase, profile_index, cmd)
    if pattern is None:
        return False
    if pattern["speed_left"] <= 0.0 and pattern["speed_right"] <= 0.0:
        stop()
        return True
    set_motors(
        pattern["speed_left"],
        int(pattern["forward_left"]),
        pattern["speed_right"],
        int(pattern["forward_right"]),
        ramp_time=pattern["ramp_time"],
        step_interval=pattern["step_interval"],
    )
    return True


def _format_wheel(speed, forward):
    direction = "F" if forward else "R"
    return f"{float(speed):.1f}%{direction}"


def _format_output(speed_left, forward_left, speed_right, forward_right):
    targets = _target_motor_speeds(speed_left, forward_left, speed_right, forward_right)
    return (
        f"L={_format_wheel(targets[MOTOR_LEFT_MTR_INDEX - 1], forward_left)} "
        f"R={_format_wheel(targets[MOTOR_RIGHT_MTR_INDEX - 1], forward_right)}"
    )


def _format_command_output(command):
    return _format_output(
        command["speed_left"], command["forward_left"],
        command["speed_right"], command["forward_right"],
    )


def _print_phase_profile(phase, profile_index, manual_speed=DEFAULT_SPEED):
    if phase == "manual":
        print(f"Mode: manual (requested duty {manual_speed:.0f}%)")
        commands = {}
        for key in "wasd":
            pattern = get_manual_drive_pattern(key, manual_speed)
            commands[key] = _command(
                pattern["label"], pattern["speed_a"], pattern["forward_a"],
                pattern["speed_b"], pattern["forward_b"], DEFAULT_RAMP_TIME,
            )
    else:
        profiles = PHASE_DRIVE_PROFILES[phase]
        profile_index %= len(profiles)
        profile = profiles[profile_index]
        print(
            f"Mode: P{phase}/{profile['name']} "
            f"({profile_index + 1}/{len(profiles)}) - {profile['description']}"
        )
        commands = profile["commands"]
    print(f"  Target PWM after trim / {MOTOR_DRIVE_MIN_SPEED:.0f}% forward/reverse/turn floor (F=forward, R=reverse):")
    for key in ("w", "a", "s", "d"):
        command = commands[key]
        print(f"  {key.upper()}: {command['label']} {_format_command_output(command)}")


def print_profile_catalog():
    print("Available motor diagnostic profiles:")
    print(f"  Duties include motor trim and the {MOTOR_DRIVE_MIN_SPEED:.0f}% forward/reverse/turn floor.")
    print("  manual: existing variable-duty W/A/S/D behavior")
    for phase, profiles in PHASE_DRIVE_PROFILES.items():
        for index in range(len(profiles)):
            _print_phase_profile(phase, index)


def _print_controls():
    print(
        "Controls: W/A/S/D or Arrow Keys, 0=manual, 1..7=phase, "
        "M=next phase profile, +=faster, -=slower, space=stop, ?=help, q=quit"
    )


def drive_forward(speed=DEFAULT_SPEED):
    """Drive both motors forward."""
    _apply_manual_drive_pattern("w", speed)


def drive_backward(speed=DEFAULT_SPEED):
    """Drive both motors backward."""
    _apply_manual_drive_pattern("s", speed)


def turn_left(speed=DEFAULT_SPEED):
    """Steer left with differential forward speeds (no reverse)."""
    _apply_manual_drive_pattern("a", speed)


def turn_right(speed=DEFAULT_SPEED):
    """Steer right with differential forward speeds (no reverse)."""
    _apply_manual_drive_pattern("d", speed)


def _read_key():
    """
    Read a single key press and normalize to movement commands.
    Supports WASD and arrow keys on Windows and POSIX terminals.
    Returns the lowercase command character or '' if unknown.
    """
    try:
        import msvcrt  # Windows single-key capture

        key = msvcrt.getch()
        if key in (b'\x00', b'\xe0'):
            special = msvcrt.getch()
            arrow_map = {
                b'H': 'w',  # Up
                b'P': 's',  # Down
                b'K': 'a',  # Left
                b'M': 'd',  # Right
            }
            return arrow_map.get(special, '')
        return key.decode('utf-8', 'ignore').lower()
    except ImportError:
        import termios
        import tty

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch1 = sys.stdin.read(1)
            if ch1 == '\x1b':  # Escape sequence for arrows
                ch2 = sys.stdin.read(1)
                if ch2 == '[':
                    ch3 = sys.stdin.read(1)
                    arrow_map = {
                        'A': 'w',  # Up
                        'B': 's',  # Down
                        'D': 'a',  # Left
                        'C': 'd',  # Right
                    }
                    return arrow_map.get(ch3, '')
                return ''
            return ch1.lower()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def _get_command():
    """
    Wrapper to fetch a single command key.
    Falls back to input() if raw capture fails.
    """
    try:
        return _read_key()
    except Exception:
        try:
            return input("Enter command: ").strip().lower()[:1]
        except EOFError:
            return ''


def _clamp_speed(speed):
    return max(0.0, min(100.0, float(speed)))


def parse_args():
    parser = argparse.ArgumentParser(description="Interactive motor diagnostic")
    parser.add_argument("--default-speed", type=float, default=DEFAULT_SPEED)
    parser.add_argument(
        "--phase",
        default="manual",
        help="startup mode: manual/0 or P1 through P7 (default: manual)",
    )
    parser.add_argument(
        "--profile",
        default=None,
        help="startup phase profile name; use --list-profiles to show names",
    )
    parser.add_argument(
        "--list-profiles",
        action="store_true",
        help="print phase/profile names without initializing GPIO",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    current_speed = float(args.default_speed)
    if bool(getattr(args, "list_profiles", False)):
        print_profile_catalog()
        return
    active_phase = _normalize_phase(getattr(args, "phase", "manual"))
    profile_index = (
        0
        if active_phase == "manual"
        else _profile_index_for_name(active_phase, getattr(args, "profile", None))
    )
    try:
        setup()
        print("Motor Control Ready")
        _print_controls()
        print(f"Requested Duty: {current_speed:.0f}%")
        _print_phase_profile(active_phase, profile_index, current_speed)
        while True:
            cmd = _get_command()

            # Empty input -> ignore to avoid jitter.
            if not cmd:
                continue

            # Take only the first character for simplicity.
            cmd = cmd[0]

            if cmd == 'q':
                print("Quit requested.")
                break

            if cmd == ' ':
                print("Stop")
                stop()
                continue

            if cmd == '?':
                _print_controls()
                _print_phase_profile(active_phase, profile_index, current_speed)
                continue

            if cmd in "01234567":
                stop()
                active_phase = "manual" if cmd == "0" else cmd
                profile_index = 0
                _print_phase_profile(active_phase, profile_index, current_speed)
                continue

            if cmd == 'm':
                stop()
                if active_phase == "manual":
                    print("Manual mode has no fixed profiles. Select P1..P7 first.")
                else:
                    profile_index = (
                        profile_index + 1
                    ) % len(PHASE_DRIVE_PROFILES[active_phase])
                    _print_phase_profile(active_phase, profile_index, current_speed)
                continue

            if cmd in ('+', '='):
                if active_phase == "manual":
                    current_speed = _clamp_speed(current_speed + SPEED_STEP)
                    print(f"Requested Duty Up -> {current_speed:.0f}%")
                    _print_phase_profile(active_phase, profile_index, current_speed)
                else:
                    print("Phase profiles use fixed production duty. Press 0 for adjustable manual mode.")
                continue

            if cmd in ('-', '_'):
                if active_phase == "manual":
                    current_speed = _clamp_speed(current_speed - SPEED_STEP)
                    print(f"Requested Duty Down -> {current_speed:.0f}%")
                    _print_phase_profile(active_phase, profile_index, current_speed)
                else:
                    print("Phase profiles use fixed production duty. Press 0 for adjustable manual mode.")
                continue

            # Apply a short buffer before acting to reduce regen stress.
            time.sleep(COMMAND_BUFFER_SEC)

            if cmd in ("w", "a", "s", "d") and active_phase != "manual":
                pattern = get_phase_drive_pattern(active_phase, profile_index, cmd)
                print(
                    f"P{active_phase}/{PHASE_DRIVE_PROFILES[active_phase][profile_index]['name']} "
                    f"{pattern['label']}: "
                    f"{_format_command_output(pattern)}"
                )
                _apply_phase_drive_pattern(active_phase, profile_index, cmd)
            elif cmd == 'w':
                print(f"Forward (requested duty {current_speed:.0f}%)")
                drive_forward(current_speed)
            elif cmd == 's':
                print(f"Backward (requested duty {current_speed:.0f}%)")
                drive_backward(current_speed)
            elif cmd == 'a':
                print(f"Left (requested duty {current_speed:.0f}%)")
                turn_left(current_speed)
            elif cmd == 'd':
                print(f"Right (requested duty {current_speed:.0f}%)")
                turn_right(current_speed)
            else:
                print(f"Unknown command '{cmd}'. Press ? for help.")

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        stop()


if __name__ == "__main__":
    main()
