# moose-propulsion-control
Repository for ECU source code for moose target propulsion control

## J1939 message map

All application messages use extended 29-bit identifiers, eight-byte payloads,
and J1939 little-endian multi-byte signals. The custom measurement/control
messages use project-specific PGNs:

| Message | PGN | 29-bit CAN ID | Direction |
| --- | --- | --- | --- |
| `LMSD_MCU` | `0xFF00` | `0x18FF00A2` | MCU broadcasts distance and speed |
| `MCTL_DCU` | `0xEF00` | `0x0CEFA1A0` | DCU directs commands to PCU |
| `MSTA_PCU` | `0xFF01` | `0x18FF01A1` | PCU broadcasts state, distance, and speed |

The examples use local source addresses DCU=`0xA0`, PCU=`0xA1`, and
MCU=`0xA2`. Each node sends an Address Claimed message at startup, waits for
the claim arbitration window, answers requests for its NAME, and sends Cannot
Claim if it loses a conflict at its fixed address. These are fixed addresses;
the firmware does not select a new address after a conflict.

The NAMEs currently use manufacturer code `2047` as a clearly marked test
placeholder. This value is not an assigned manufacturer code. The three NAMEs
are only suitable for testing this private network; replace the placeholder
with an SAE-assigned manufacturer code and review the other NAME fields before
connecting the ECUs to any other J1939 network. This implementation is a
minimal address-claim test profile, not a claim of complete J1939 conformance.
For current PGN signal definitions, NAME messages, and command/state values,
see `moose_propulsion.dbc` and `j1939_address.h` in each sketch directory.

`MCTL_DCU` command values are `0=HOLD`, `1=HOME`, `2=TO_MCU`, and `3=TO_PCU`.
Program 1 is the only implemented drive profile. The PCU brakes and reports an
endpoint after its lidar detects endpoint range and confirms near-zero speed
for one second. Unused bytes are transmitted as `0xFF`.

## DCU hardware defaults

The DCU sketch uses the provided keypad example's 4x4 key map and pin mapping
(rows 2–5, columns 6–9), and the provided LCD example's I2C address `0x27` with
a 20x4 display. It requires the Arduino `Keypad` and `LiquidCrystal_I2C`
libraries, plus the same `CAN` library/interface as the other nodes. Adjust
these settings near the top of `dcu/dcu.ino` if the installed hardware differs.
