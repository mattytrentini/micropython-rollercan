"""Run from the repository root with MicroPython Unix."""

from errno import ETIMEDOUT
from struct import pack

from rollercan import RollerCAN


class Bus:
    FLAG_EXT_ID = 2
    FLAG_RTR = 1

    def __init__(self):
        self.frames = []
        self.sent = []
        self.cancelled = []
        self.replies = []
        self.full = False

    def send(self, can_id, payload, flags):
        if self.full:
            return None
        self.sent.append((can_id, bytes(payload), flags))
        self.frames.extend(self.replies)
        self.replies = []
        return 0  # Slot zero is a successful send.

    def recv(self, frame):
        if self.frames:
            can_id, data, flags = self.frames.pop(0)
            frame[:] = [can_id, memoryview(data), flags, 0]
            return frame
        return None

    def cancel_send(self, slot):
        self.cancelled.append(slot)
        return True


def raises_errno(function, number):
    try:
        function()
    except OSError as exc:
        assert exc.args[0] == number, exc
    else:
        raise AssertionError("expected OSError")


def test_motion_commands():
    bus = Bus()
    motor = RollerCAN(bus)
    zeros = bytes(8)
    feedback = pack("<hhhh", -30, 270, -100, 12)
    bus.replies = [(0x0000A8FE, zeros, 2)]
    assert motor.ping() == 0xA8
    assert bus.sent[-1] == (0x000000A8, zeros, 2)

    for action, request, payload in (
        (motor.status, 0x020000A8, zeros),
        (motor.start, 0x030000A8, zeros),
        (motor.stop, 0x040000A8, zeros),
        (lambda: motor.set_mode(motor.CURRENT), 0x120000A8, b"\x05\x70\0\0\x03\0\0\0"),
        (lambda: motor.set_speed(-30.25), 0x120000A8, b"\x0a\x70\0\0\x2f\xf4\xff\xff"),
        (lambda: motor.set_position(12.5), 0x120000A8, b"\x16\x70\0\0\xe2\x04\0\0"),
        (lambda: motor.set_current(-100), 0x120000A8, b"\x06\x70\0\0\xf0\xd8\xff\xff"),
        (lambda: motor.set_current_limit(motor.SPEED, 100), 0x120000A8, b"\x18\x70\0\0\x10\x27\0\0"),
        (lambda: motor.set_current_limit(motor.POSITION, 100), 0x120000A8, b"\x17\x70\0\0\x10\x27\0\0"),
        (lambda: motor.set_encoder_count(-100), 0x120000A8, b"\x33\x70\0\0\x9c\xff\xff\xff"),
    ):
        bus.replies = [(0x025AA800, feedback, 2)]
        state = action()
        assert bus.sent[-1] == (request, payload, 2)
        assert state.state == motor.RUNNING and state.mode == motor.CURRENT
        assert state.faults == motor.STALL
        assert (state.speed_rpm, state.position_deg, state.current_ma, state.voltage_v) == (-30, 270, -100, 12)

    count = len(bus.sent)
    try:
        motor.set_current(1000.01)
    except ValueError:
        pass
    else:
        raise AssertionError("out-of-range current was accepted")
    assert len(bus.sent) == count


def test_read_matching():
    bus = Bus()
    motor = RollerCAN(bus, 0xA1, host_id=0x34)
    reply = 0x110034A1
    stale = pack("<H2xi", 0x7030, 100)
    found = pack("<H2xi", 0x7030, -1225)
    bus.frames = [(reply, stale, 2)]
    bus.replies = [
        (reply, stale, 0),                      # standard frame
        (reply, stale, 3),                      # remote frame
        (reply, stale[:-1], 2),                 # wrong DLC
        (0x110035A1, stale, 2),                 # another host
        (0x110034A2, stale, 2),                 # another motor
        (reply, pack("<H2xi", 0x7031, 100), 2), # another register
        (reply, found, 2),
    ]
    assert motor.speed() == -12.25
    assert bus.sent[-1] == (reply, b"\x30\x70\0\0\0\0\0\0", 2)
    assert not bus.cancelled


def test_timeout():
    bus = Bus()
    motor = RollerCAN(bus, timeout_ms=5)
    raises_errno(motor.stop, ETIMEDOUT)
    assert bus.sent == [(0x040000A8, bytes(8), 2)]
    assert bus.cancelled == [0]
    bus.full = True
    raises_errno(motor.start, ETIMEDOUT)
    assert bus.cancelled == [0]
    assert len(bus.sent) == 1


def test_status_without_feedback():
    class RegisterBus(Bus):
        registers = {0x7004: 0, 0x7005: 1, 0x7030: -125, 0x7031: 27525,
                     0x7032: 24, 0x7034: 1386}

        def send(self, can_id, payload, flags):
            if can_id == 0x110000A8:
                reg = payload[0] | payload[1] << 8
                if reg in self.registers:
                    self.replies = [(can_id, pack("<H2xi", reg, self.registers[reg]), flags)]
            else:
                assert can_id == 0x020000A8, "status must not send a write or motion command"
            return super().send(can_id, payload, flags)

    bus = RegisterBus()
    motor = RollerCAN(bus, timeout_ms=10)
    assert motor.status() == (motor.STANDBY, motor.SPEED, None, -1.25, 275.25, 0.24, 13.86)
    bus.registers[0x7004] = 1
    assert motor.status().state == motor.RUNNING
    del bus.registers[0x7034]
    raises_errno(motor.status, ETIMEDOUT)


test_motion_commands()
test_read_matching()
test_timeout()
test_status_without_feedback()
print("RollerCAN motion frames, reply matching and timeout handling passed")
