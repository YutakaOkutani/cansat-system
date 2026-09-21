from mission.const import (
    MANUAL_TURN_SPEED_RATIO,
    MOTOR_DIR_INVERT_1,
    MOTOR_DIR_INVERT_2,
    MOTOR_LEFT_MTR_INDEX,
    MOTOR_RIGHT_MTR_INDEX,
    MOTOR_DRIVE_MIN_SPEED,
)


MANUAL_DRIVE_PATTERNS = {
    "w": ("Forward", True, True),
    "s": ("Backward", False, False),
    "a": ("Left", True, True),
    "d": ("Right", True, True),
}


def apply_drive_speed_floor(speed_1, speed_2, *, same_direction):
    """Raise trimmed forward/reverse duties together; prioritize the floor over ratio at 100%.

    Stops, pivots and opposite-direction commands are not boosted.
    Preserve the motor trim ratio for straight commands as well as arcs.
    This is a steady-state target; soft-start ramps still begin at zero.
    """
    if not same_direction or min(speed_1, speed_2) <= 0:
        return speed_1, speed_2
    factor = max(1.0, MOTOR_DRIVE_MIN_SPEED / min(speed_1, speed_2))
    return min(100.0, speed_1 * factor), min(100.0, speed_2 * factor)


def motor_forward_to_dir_value(motor_index, forward):
    """Convert a logical forward command into the motor driver's PH pin value."""
    if int(motor_index) == 1:
        invert = MOTOR_DIR_INVERT_1
    elif int(motor_index) == 2:
        invert = MOTOR_DIR_INVERT_2
    else:
        raise ValueError(f"Unknown motor index: {motor_index}")
    return forward_to_dir_value(forward, invert)


def forward_to_dir_value(forward, invert):
    """Convert logical direction plus an invert flag into the PH pin value."""
    return 1 if (bool(forward) ^ bool(invert)) else 0


def map_logical_wheels_to_physical(
    left_speed,
    left_forward,
    right_speed,
    right_forward,
):
    """Map logical left/right wheel commands to physical MTR1/MTR2 order."""
    if {int(MOTOR_LEFT_MTR_INDEX), int(MOTOR_RIGHT_MTR_INDEX)} != {1, 2}:
        raise ValueError("Left/right motor mapping must contain MTR1 and MTR2 exactly once")
    commands = {
        int(MOTOR_LEFT_MTR_INDEX): (float(left_speed), bool(left_forward)),
        int(MOTOR_RIGHT_MTR_INDEX): (float(right_speed), bool(right_forward)),
    }
    motor1_speed, motor1_forward = commands[1]
    motor2_speed, motor2_forward = commands[2]
    return motor1_speed, motor1_forward, motor2_speed, motor2_forward


def get_manual_drive_pattern(cmd, speed):
    """Return a normalized two-motor command for manual WASD control.

    The returned A/B slots are logical wheel positions:
    - A: left wheel
    - B: right wheel

    Physical MTR channel routing is applied immediately before GPIO output.
    """
    pattern = MANUAL_DRIVE_PATTERNS.get((cmd or "").lower())
    if pattern is None:
        return None
    label, forward_a, forward_b = pattern
    speed_fast = float(speed)
    speed_slow = speed_fast * MANUAL_TURN_SPEED_RATIO
    speed_a = speed_fast
    speed_b = speed_fast
    cmd_key = (cmd or "").lower()
    if cmd_key == "a":
        speed_a = speed_slow
    elif cmd_key == "d":
        speed_b = speed_slow
    return {
        "label": label,
        "speed_a": speed_a,
        "forward_a": bool(forward_a),
        "speed_b": speed_b,
        "forward_b": bool(forward_b),
    }
