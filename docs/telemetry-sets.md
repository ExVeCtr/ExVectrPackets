# Telemetry sets

Design 3.5.6 as adapted by the user decisions in the Revision 1 box of `docs/network-design.md`: decoded values are SI, packets are few and large grouped by rate class, USB carries everything useful, the radio only a bare minimum.

## Air (radio) set, ports < 0x80

| Packet | Port | Bytes | Air Hz | B/s incl. 7 B overhead |
|---|---|---|---|---|
| `VEHICLE_STATUS` (slow class: mode, mission, applied arm state, motor gate, control source, failure bits, sensors, last request, rc age) | 0x20 | 28 | 5 | 175 |
| `AIR_POSE` (q, position, velocity as fixed-point) | 0x21 | 24 | 10 | 310 |
| `HANDSET_TELEM` | 0x03 | 34 | 2 | 82 |
| `LINK_STATS` (from node 3) | 0x01 | 19 | 2 | 52 |
| heartbeats (nodes 1 and 3) | - | 11 | 1 | 36 |
| **Total** | | | | **about 655 B/s** (design budget for the downlink: 6.2 kB/s) |

`GNSS`, `ACTUATORS` (flaps), `MISSION` and the raw/debug packets are not on the radio. `HANDSET_TELEM` already carries position, speed and attitude for the Pocket display. A test checks that the air set stays below 1.2 kB/s.

## USB set (full)

All of the above that have a USB rate, plus ports 0x80 and up (never forwarded to air or UART).

| Class | Packet | Port | Bytes | USB Hz |
|---|---|---|---|---|
| fast | `POSE` (omega, q, velocity, position, f32) | 0x80 | 56 | 100 |
| medium | `ACTUATORS` (flaps `[TL,TR,BL,BR]` rad, 0 = extended, increasing = folded toward the back; TVC; thrust; ground distance) | 0x81 | 36 | 50 |
| slow | `VEHICLE_STATUS` | 0x20 | 28 | 20 |
| slow | `MISSION` | 0x82 | 34 | 20 |
| slow | `GNSS` (lat/lon radians, f64) | 0x83 | 50 | 5 |
| slow | `HANDSET_TELEM` | 0x03 | 34 | 2 |
| debug | `IMU_RAW` 0x90 (100 Hz), `ESTIMATOR` 0x91 (20 Hz), `TASK_TIMING` 0x92 (5 Hz) | | 48/40/37 | |

`AIR_POSE` is not sent on USB (`POSE` supersedes it); a ground station connected by radio only uses `AIR_POSE`, one connected by USB uses `POSE`. The rates are the targets for the flight software's low-priority telemetry task; they are metadata (`kUsbRateHz`, `kAirRateHz`), not enforced by the library.

## Control and request packets

`RC_STATE` (0x10, 34 B) carries the 16 channels and the flags `SOURCE_OK` (bit 0) and `MOTOR_ENABLE` (bit 1, the hard motor interlock from `docs/safety.md` section 3, sent continuously with the rest of the command state). `REQUEST` (0x11) / `REQUEST_ACK` (0x12) use the action enum `RequestAction` and the status enum `RequestStatus`.
