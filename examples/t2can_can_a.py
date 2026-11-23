# SPDX-License-Identifier: MIT
"""Run the demo over CAN A on the classic LilyGo T-2CAN (MCP2515)."""

from machine import Pin
from time import sleep_ms

from mcp2515 import CAN, CAN_CLOCK, CAN_SPEED, ERROR, CANFrame, SPIESP32
from mcp2515.can import REGISTER
from rollercan_demo import run


class PortA:
    FLAG_EXT_ID = 0x80000000
    FLAG_RTR = 0x40000000

    def __init__(self):
        self.spi = SPIESP32(10)
        self.can = CAN(self.spi)

    @staticmethod
    def _check(result):
        if result != ERROR.ERROR_OK:
            raise OSError("MCP2515 error {}".format(result))

    def init(self):
        reset = Pin(9, Pin.OUT, value=0)
        sleep_ms(2)
        reset.on()
        sleep_ms(2)
        self._check(self.can.reset())
        self._check(self.can.setBitrate(CAN_SPEED.CAN_1000KBPS, CAN_CLOCK.MCP_16MHZ))
        self.can.modifyRegister(REGISTER.MCP_CANCTRL, 0x08, 0x08)  # One-shot; don't retry stale commands.
        self._check(self.can.setNormalMode())

    def send(self, can_id, data, flags):
        if self.can.readRegister(REGISTER.MCP_TXB0CTRL) & 0x08:
            return None
        try:
            self._check(self.can.sendMessage(CANFrame(can_id | flags, bytes(data)), txbn=0))
        except BaseException:
            self.cancel_send(0)
            raise
        return 0

    def recv(self, into):
        result, frame = self.can.readMessage()
        if result == ERROR.ERROR_NOMSG:
            return None
        self._check(result)
        into[0] = frame.arbitration_id
        into[1] = frame.data
        into[2] = frame.can_id & (self.FLAG_EXT_ID | self.FLAG_RTR)
        into[3] = 0
        return into

    def cancel_send(self, slot):
        self.can.modifyRegister(REGISTER.MCP_TXB0CTRL, 0x08, 0)

    def close(self):
        try:
            self.cancel_send(0)
            self._check(self.can.setConfigMode())
        finally:
            self.spi._SPI.deinit()


def main():
    bus = PortA()
    try:
        bus.init()
        run(bus)
    finally:
        bus.close()
