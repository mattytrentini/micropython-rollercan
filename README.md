# RollerCAN Lite for MicroPython

A dependency-free, MIT-licensed driver for the [M5Stack RollerCAN Lite](https://docs.m5stack.com/en/unit/Unit_RollerCAN-Lite) using [`machine.CAN`](https://docs.micropython.org/en/latest/library/machine.CAN.html). Supports speed, position, current and encoder (dial) modes, plus feedback and telemetry. It does not use `pyb.CAN`, SocketCAN, or UIFlow.

## Install

```sh
mpremote mip install github:mattytrentini/micropython-rollercan
```

On a network-connected board:

```python
import mip
mip.install("github:mattytrentini/micropython-rollercan")
```

## Connect and check status

Use a MicroPython build exposing `machine.CAN` and a CAN transceiver. The
motor uses Classic CAN with 29-bit extended IDs. The factory defaults are
1 Mbit/s and motor ID `0xA8`. Provide the motor a suitable 6–16 V supply via
the PwrCAN connector. Connect CAN-H, CAN-L and a common ground, and terminate
the two physical ends of the bus. Do not connect the motor supply to MCU I/O.

```python
from machine import CAN
from rollercan import RollerCAN

can = CAN(1, 1_000_000)  # The controller ID is board-specific.
can.set_filters(None)    # Accept all on an isolated bench bus.
motor = RollerCAN(can)
print(hex(motor.ping()))
print(motor.status())
```

`RollerCAN(can, node_id=0xA8, host_id=0, timeout_ms=250)` does not transmit,
change filters, or enable the motor during construction. Use a free host ID
that is distinct from every motor ID on the bus. The driver consumes the CAN
receive queue, discarding stale or unrelated frames. You can share the physical
bus and use multiple motor instances on one controller, but run transactions
sequentially and do not use the same controller concurrently for other I/O.
On a busy bus, configure hardware filters for the reply IDs before use.

## Run the bench demo

Fixture the motor securely and keep the shaft and attachments clear. Verify
that you have a physical way to cut motor power before commanding motion.
The manufacturer rates the Lite for 500 mA continuous without forced cooling
and up to 1 A for short periods. The driver rejects current targets or limits
above 1 A in magnitude; it does not enforce a continuous-duty thermal limit.

The [example](examples/rollercan_demo.py) pings the motor, checks that it is
idle in speed mode, and commands +10 and −10 rpm for about 350 ms each, with
a current limit no higher than 100 mA. It stops and checks that the shaft has
settled before reversing, prints speed, position and current, then restores
the prior speed target and current limit.

For the classic LilyGo T-2CAN **CAN A** port, the launcher uses the installed
`mcp2515` package with the T-2CAN `SPIESP32` pin mapping (SCK 12, MOSI 11,
MISO 13). With the motor in CAN mode at 1 Mbit/s and node `0xA8`, run this
from the driver checkout. Nothing is installed on the board:

```sh
mpremote connect /dev/ttyACM0 resume mount . exec \
  "import sys; sys.path.insert(0, '/remote'); sys.path.insert(0, '/remote/examples'); from t2can_can_a import main; main()" \
  umount
```

For a native `machine.CAN` port (including **CAN B** on the T-2CAN), mount
the checkout the same way and call `rollercan_demo.run(can)` with the bus
initialized as in [Connect and check status](#connect-and-check-status).
Do not run the demo while another application is controlling the same bus.

`stop()` disables drive output; it does not actively brake or hold position.
A timeout raises `OSError` with `ETIMEDOUT`. It does not prove that a command
was never executed or that the motor has stopped. The driver asks the CAN
controller to cancel any pending transmission on failure. If communication
is lost, cut motor power rather than relying on software stop.

## API

| Operation | Methods and units |
| --- | --- |
| Device and output | `ping()` returns the node ID; `status()`, `start()`, `stop()` return `Feedback` |
| Mode | `set_mode(RollerCAN.SPEED / POSITION / CURRENT / ENCODER)` |
| Targets | `set_speed(rpm)`, `set_position(degrees)`, `set_current(milliamps)`, `set_encoder_count(count)` |
| Current limit | `set_current_limit(mode, milliamps)` applies to `SPEED` or `POSITION` |
| Telemetry | `speed()` in rpm, `position()` in degrees, `current()` in mA, `voltage()` in V, `temperature()` in °C, `encoder_count()` in raw dial counts |

Setters return `Feedback` and do not enable output or silently switch modes.
Changing a target or mode while already running can take effect immediately.
`Feedback` contains `state`, `mode`, `faults`, `speed_rpm`, `position_deg`,
`current_ma` and `voltage_v`. Fault bits are `OVER_VOLTAGE`, `STALL` and
`OUT_OF_RANGE`. Feedback uses whole-unit values; the dedicated speed,
position, current and voltage reads have 0.01-unit resolution on the wire.
The motor acknowledges writes with feedback, not a register echo; state and
faults should be checked rather than treating a reply as proof of movement.
If the firmware doesn't answer the status command, `status()` waits for the
configured timeout, then reads the enable flag, mode and telemetry registers.
In that case `faults` is `None` (unknown), `RUNNING` means output is enabled
and does not prove rotation, and the telemetry retains 0.01-unit resolution.
The readings are taken separately, not as a single snapshot. `start()`,
`stop()` and setters still require their normal feedback replies.

This driver does not save parameters to flash or disable motor protection.

## Test and reference

Run `MICROPYPATH=. micropython tests/test_rollercan.py` from the repository root for
protocol tests against a simulated CAN controller. Bench-tested on a RollerCAN
Lite connected to a LilyGo T-2CAN CAN A port at 1 Mbit/s, using its existing
MCP2515 driver with an in-memory adapter. Verified register-based `status()`
before, during and after a current-limited 10 rpm run, and normal feedback
from the setters, `start()` and `stop()`. The native `machine.CAN` path was
not exercised in this test.

Protocol sources: [M5Stack's CAN control manual](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/779/Unit-RollerCAN-CAN-Protocol-EN.pdf)
and [RollerCAN firmware](https://github.com/m5stack/M5Unit-RollerCAN-Internal-FW/blob/5410e37b6910eb3a80c7446bdfbdd3ce93893389/code/Unit-RollerCAN/Core/User/can/can_protocol.c).
