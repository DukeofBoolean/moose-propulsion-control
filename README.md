# moose-propulsion-control
Repository for ECU source code for moose target propulsion control

## MCU-to-PCU CAN measurement frame

The MCU transmits measurements to the PCU using standard CAN identifier `0x12`.
The payload is exactly 4 bytes; all multi-byte values use network byte order
(most significant byte first):

| Bytes | Signal | Representation | Physical unit |
| --- | --- | --- | --- |
| 0–1 | Distance | Unsigned 16-bit integer | cm |
| 2–3 | Speed | Signed 16-bit two's-complement integer | cm/s |

Positive speed means the measured distance is decreasing (the target is
approaching that lidar); negative speed means the distance is increasing. The
PCU accepts measurement updates only for identifier `0x12` with a 4-byte
payload. The MCU does not send a frame until it has received a lidar sample.
