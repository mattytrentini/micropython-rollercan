# SPDX-License-Identifier: MIT
"""Run two short, current-limited speed tests on a secured bench motor."""

from time import sleep_ms, ticks_add, ticks_diff, ticks_ms

from rollercan import RollerCAN


_ENABLE = 0x7004
_SPEED_TARGET = 0x700A
_SPEED_LIMIT = 0x7018


def _check(feedback, state):
    if feedback.state != state or feedback.mode != RollerCAN.SPEED or feedback.faults:
        raise OSError("Unexpected motor feedback: {}".format(feedback))


def _wait_for_rest(motor):
    deadline = ticks_add(ticks_ms(), 700)
    while abs(motor.speed()) >= 0.5:
        if ticks_diff(deadline, ticks_ms()) <= 0:
            raise OSError("Motor did not settle; do not reverse it")
        sleep_ms(25)


def run(can, node_id=0xA8):
    """Use an initialized CAN bus. Secure the motor and keep the shaft clear."""
    motor = RollerCAN(can, node_id, timeout_ms=100)
    print("Motor ID: 0x{:02X}".format(motor.ping()))
    before = motor.status()
    _check(before, motor.STANDBY)
    if abs(before.speed_rpm) >= 0.5 or not 6 <= before.voltage_v <= 16:
        raise OSError("Motor must be at rest with 6–16 V CAN power: {}".format(before))
    print("Before:", before)

    # Save raw settings so the demo can put them back exactly.
    old_target = motor._read_i32(_SPEED_TARGET)
    old_limit = motor._read_i32(_SPEED_LIMIT)
    if not 0 < old_limit <= 100_000:
        raise OSError("Motor speed-mode current limit is outside 0–1000 mA")
    limit = min(old_limit, 10_000)
    try:
        _check(motor.set_current_limit(motor.SPEED, limit / 100), motor.STANDBY)
        if motor._read_i32(_SPEED_LIMIT) != limit:
            raise OSError("Current limit was not accepted")
        for target in (10, -10):
            _check(motor.set_speed(target), motor.STANDBY)
            if motor._read_i32(_SPEED_TARGET) != target * 100:
                raise OSError("Speed target was not accepted")
            try:
                _check(motor.start(), motor.RUNNING)
                sleep_ms(350)
                speed, position, current = motor.speed(), motor.position(), motor.current()
            finally:
                motor.stop()
            if motor._read_i32(_ENABLE) != 0:
                raise OSError("Output still enabled; cut motor power")
            _wait_for_rest(motor)
            print("Target {:+} rpm: {:+.2f} rpm, {:.2f} deg, {:+.2f} mA".format(
                target, speed, position, current
            ))
    finally:
        if motor._read_i32(_ENABLE) != 0:
            raise OSError("Output still enabled; cut motor power")
        motor.set_speed(old_target / 100)
        motor.set_current_limit(motor.SPEED, old_limit / 100)

    print("After:", motor.status())
