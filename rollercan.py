# SPDX-License-Identifier: MIT
"""Control the M5Stack RollerCAN Lite over MicroPython machine.CAN."""

from collections import namedtuple
from errno import ETIMEDOUT
from struct import pack_into, unpack_from
from time import sleep_ms, ticks_add, ticks_diff, ticks_ms


Feedback = namedtuple(
    "Feedback", "state mode faults speed_rpm position_deg current_ma voltage_v"
)

_ZERO = b"\x00" * 8
_FEEDBACK = 0x02000000
_FEEDBACK_MASK = 0x1F00FFFF  # Command, motor ID and host ID; not status bits.
_ID_MASK = 0x1FFFFFFF


class _NoReply(OSError):
    pass


class RollerCAN:
    """One motor on an initialized Classic CAN controller.

    The caller configures the bus and receive filters. Use the controller
    sequentially: this driver consumes its receive queue and there is no
    transaction sequence number in the motor protocol.
    """

    SPEED = 1
    POSITION = 2
    CURRENT = 3
    ENCODER = 4

    STANDBY = 0
    RUNNING = 1
    ERROR = 2

    OVER_VOLTAGE = 1
    STALL = 2
    OUT_OF_RANGE = 4

    def __init__(self, can, node_id=0xA8, *, host_id=0, timeout_ms=250):
        if not isinstance(node_id, int) or not 0 <= node_id <= 255:
            raise ValueError("node_id must be an integer in 0..255")
        if not isinstance(host_id, int) or not 0 <= host_id <= 255 or host_id == node_id:
            raise ValueError("host_id must be an integer in 0..255 distinct from node_id")
        if not isinstance(timeout_ms, int) or timeout_ms <= 0:
            raise ValueError("timeout_ms must be a positive integer")
        self._can = can
        self._node_id = node_id
        self._request_address = (host_id << 8) | node_id
        self._feedback_id = _FEEDBACK | (node_id << 8) | host_id
        self._timeout_ms = timeout_ms
        self._ext = can.FLAG_EXT_ID
        self._frame_mask = self._ext | can.FLAG_RTR
        self._rx = [0, memoryview(bytearray(8)), 0, 0]
        self._tx = bytearray(8)

    def _exchange(self, command, data, reply, mask=_ID_MASK, register=None):
        can = self._can
        rx = self._rx
        deadline = ticks_add(ticks_ms(), self._timeout_ms)
        # Drop already-queued frames, including late replies to earlier requests.
        while can.recv(rx) is not None:
            if ticks_diff(deadline, ticks_ms()) <= 0:
                raise OSError(ETIMEDOUT, "CAN receive queue did not clear")
        request = (command << 24) | self._request_address
        while True:
            slot = can.send(request, data, self._ext)
            if slot is not None:
                break
            if ticks_diff(deadline, ticks_ms()) <= 0:
                raise OSError(ETIMEDOUT, "CAN transmit queue busy")
            sleep_ms(1)
        try:
            while ticks_diff(deadline, ticks_ms()) > 0:
                if can.recv(rx) is None:
                    sleep_ms(1)
                    continue
                can_id, payload, flags, _ = rx
                if flags & self._frame_mask != self._ext or len(payload) != 8:
                    continue
                if can_id & mask != reply:
                    continue
                if register is not None and (payload[0] | (payload[1] << 8)) != register:
                    continue
                return rx
            raise _NoReply(ETIMEDOUT, "no matching RollerCAN reply")
        except BaseException:
            # A queued command may still be pending if the bus is disconnected.
            try:
                can.cancel_send(slot)
            except OSError:
                pass
            raise

    def _feedback(self, command, data=_ZERO):
        can_id, payload, _, _ = self._exchange(command, data, self._feedback_id, _FEEDBACK_MASK)
        speed, position, current, voltage = unpack_from("<hhhh", payload)
        return Feedback(can_id >> 22 & 3, can_id >> 19 & 7, can_id >> 16 & 7,
                        speed, position, current, voltage)

    def _read(self, register):
        pack_into("<H2xi", self._tx, 0, register, 0)
        reply = 0x11000000 | self._request_address
        return self._exchange(0x11, self._tx, reply, register=register)[1]

    def _read_i32(self, register):
        return unpack_from("<i", self._read(register), 4)[0]

    def _write(self, register, value):
        pack_into("<H2xi", self._tx, 0, register, value)
        return self._feedback(0x12, self._tx)

    def ping(self):
        """Verify that the configured motor responds; return its node ID."""
        self._exchange(0x00, _ZERO, (self._node_id << 8) | 0xFE)
        return self._node_id

    def status(self):
        """Return motor feedback; faults are None if the firmware lacks status replies."""
        try:
            return self._feedback(0x02)
        except _NoReply:
            # Some firmware only returns feedback for writes. Read back
            # without issuing a motion command or guessing any fault bits.
            return Feedback(
                self.RUNNING if self._read_i32(0x7004) else self.STANDBY,
                self._read_i32(0x7005),
                None,
                self.speed(),
                self.position(),
                self.current(),
                self.voltage(),
            )

    def start(self):
        """Enable output in the currently configured mode and return feedback."""
        return self._feedback(0x03)

    def stop(self):
        """Disable motor output and return feedback; this is not a brake."""
        return self._feedback(0x04)

    def set_mode(self, mode):
        if not isinstance(mode, int) or mode not in (self.SPEED, self.POSITION, self.CURRENT, self.ENCODER):
            raise ValueError("invalid RollerCAN mode")
        return self._write(0x7005, mode)

    def set_speed(self, rpm):
        """Set the signed speed target in rpm without enabling the motor."""
        if not -21_000_000 <= rpm <= 21_000_000:
            raise ValueError("speed target outside the protocol range")
        return self._write(0x700A, round(rpm * 100))

    def set_position(self, degrees):
        """Set the device's signed position target in degrees."""
        if not -21_000_000 <= degrees <= 21_000_000:
            raise ValueError("position target outside the protocol range")
        return self._write(0x7016, round(degrees * 100))

    def set_current(self, milliamps):
        """Set the signed current target in mA (absolute maximum 1000 mA)."""
        if not -1000 <= milliamps <= 1000:
            raise ValueError("current target must be in -1000..1000 mA")
        return self._write(0x7006, round(milliamps * 100))

    def set_current_limit(self, mode, milliamps):
        """Set a non-negative current limit in mA for SPEED or POSITION mode."""
        if not 0 <= milliamps <= 1000:
            raise ValueError("current limit must be in 0..1000 mA")
        if mode == self.SPEED:
            register = 0x7018
        elif mode == self.POSITION:
            register = 0x7017
        else:
            raise ValueError("current limit applies to SPEED or POSITION mode")
        return self._write(register, round(milliamps * 100))

    def set_encoder_count(self, count):
        """Set the dial counter in ENCODER mode, in raw counts."""
        if not isinstance(count, int) or not -0x80000000 <= count <= 0x7FFFFFFF:
            raise ValueError("encoder count must be a signed 32-bit integer")
        return self._write(0x7033, count)

    def speed(self):
        """Read measured speed in rpm, with 0.01 rpm wire resolution."""
        return self._read_i32(0x7030) / 100

    def position(self):
        """Read measured position in degrees, with 0.01 degree wire resolution."""
        return self._read_i32(0x7031) / 100

    def current(self):
        """Read measured phase current in milliamps."""
        return self._read_i32(0x7032) / 100

    def encoder_count(self):
        """Read the dial counter in raw counts."""
        return self._read_i32(0x7033)

    def voltage(self):
        """Read input voltage in volts, with 0.01 V wire resolution."""
        return self._read_i32(0x7034) / 100

    def temperature(self):
        """Read the controller's internal temperature in degrees Celsius."""
        return self._read_i32(0x7035)
