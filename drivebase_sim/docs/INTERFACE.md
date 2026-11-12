# DM CAN interface

Source: [DM-J6006-2EC User Manual V1.0, 2023-07-10](https://f.foxtech.com/manual/DM-J6006-2EC-V1.0.pdf),
Usage p.5; Feedback and MIT frames p.6; Velocity frame p.7 (PDF pages, cover = 1).
The task's specified frame definitions agree with those sections.

Standard 11-bit frames, 1 Mbps. Velocity: arbitration ID `0x200 + motor_id`,
exactly four bytes containing little-endian IEEE754 float32 rad/s.
Motor 1 at 10 rad/s: ID `0x201`, bytes `00 00 20 41`.

MIT arbitration ID equals motor ID. Its eight bytes are:

| Byte | Contents |
|---|---|
| 0, 1 | position bits 15:8, 7:0 |
| 2 | velocity bits 11:4 |
| 3 | velocity bits 3:0 in high nibble, Kp bits 11:8 in low nibble |
| 4 | Kp bits 7:0 |
| 5 | Kd bits 11:4 |
| 6 | Kd bits 3:0 in high nibble, torque bits 11:8 in low nibble |
| 7 | torque bits 7:0 |

The simulator's configurable symmetric bounds default to position ±12.5 rad,
velocity ±30 rad/s, torque ±12 N m. These are assumed configurations, not read
from a motor. Kp spans 0–500 and Kd 0–5. Mapping clamps to the interval and
rounds `uint = (x-low)/(high-low)*(2^bits-1)` to nearest integer.
Inverse mapping gives at most half an LSB error: position 0.000190738 rad,
velocity 0.007326007 rad/s, torque 0.002930403 N m with these defaults.
Zero in a symmetric even-level mapping cannot be represented exactly.
MIT zero targets and zero gains: `80 00 80 00 00 00 08 00`.
All positive upper bounds: `ff ff ff ff ff ff ff ff`.

Feedback uses configurable master arbitration ID (default 0):

| Byte | Contents |
|---|---|
| 0 | low nibble motor ID, high nibble error/status |
| 1, 2 | position bits 15:8, 7:0 |
| 3 | velocity bits 11:4 |
| 4 | velocity bits 3:0 in high nibble, torque bits 11:8 in low nibble |
| 5 | torque bits 7:0 |
| 6, 7 | MOS and rotor temperatures in degrees C |

Example: motor 1, enabled, zero position/speed/torque, both temperatures 25 C:
ID 0, bytes `11 80 00 80 08 00 19 19`. Tests check all examples above.
The packed ID is restricted to 1–15 in CanBus; experiments use 1–6.

Error hex codes: 8 overvoltage; 9 undervoltage; A overcurrent; B MOS temperature;
C coil temperature; D communication loss; E overload. Simulated status 1 means
enabled and 0 disabled. Unassigned status values decode as unknown.

## Simulation transport and checks

One velocity command per motor every 10 ms. Accepted commands trigger one
feedback reply; both directions have a fixed **1 ms** delay. Command loss is a
seeded independent Bernoulli process with configurable rate; feedback loss has
a separate rate, default zero. Driver target holds indefinitely until another
command arrives: this assumes a disabled/configured-out communication watchdog.
No enable sequence, arbitration, serialisation, bus congestion, bit errors,
CRC, retransmission or temperature dynamics are modeled. Temperatures are
constant 25 C. MIT is codec-only; the simulated driver runs velocity mode.

Wrong IDs are ignored, malformed lengths rejected, nonfinite velocity commands
rejected. Every 8-byte MIT/feedback payload maps to finite bounded numbers.
Tests cover custom ranges, endpoints/zero, quantisation, status codes, exact
delay, target hold, 20% dropped commands over 0.5 s and 5,000 seeded random
8-byte payloads. The loss test uses the isolated-wheel plant; it is distinct
from the zero-drop MuJoCo terrain experiments. These are software interface
tests, not a physical CAN adapter or manufacturer's firmware conformance test.
