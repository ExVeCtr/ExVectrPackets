# Telemetry sets

Design 3.5.6 as adapted by the user decisions in the Revision 1 box of `docs/network-design.md`: decoded values are SI, packets are few and large grouped by rate class, USB carries everything useful, the radio only a bare minimum.

## Air (radio) set, ports < 0x80

| Packet | Port | Bytes | Air Hz | B/s incl. 7 B overhead |
|---|---|---|---|---|
| `VEHICLE_STATUS` (slow class: mode, mission, applied arm state, motor gate, control source, failure bits, sensors, last request, rc age) | 0x20 | 28 | 5 | 175 |
| `AIR_POSE` (q i16, position i32 at 1 mm, velocity i16 at 0.01 m/s) | 0x21 | 30 | 10 | 370 |
| `HANDSET_TELEM` | 0x03 | 34 | 2 | 82 |
| `LINK_STATS` (from node 3) | 0x01 | 19 | 2 | 52 |
| heartbeats (nodes 1 and 3) | - | 11 | 1 | 36 |
| **Total** | | | | **about 715 B/s** (design budget for the downlink: 6.2 kB/s) |

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
| debug | `CALIBRATION` 0x93 (applied sensor calibration: offsets, 3x3 matrices, sources, EKF gyro bias), every 2 s | | 164 | 0.5 |

`AIR_POSE` is not sent on USB (`POSE` supersedes it); a ground station connected by radio only uses `AIR_POSE`, one connected by USB uses `POSE`. The rates are the targets for the flight software's low-priority telemetry task; they are metadata (`kUsbRateHz`, `kAirRateHz`), not enforced by the library.

## Control and request packets

`RC_STATE` (0x10, 34 B) carries the 16 channels and the flag `SOURCE_OK` (bit 0: the handset data is valid; bits 1..7 reserved, zero). `RC_STATE` is deliberately generic: it carries only the controller's sticks and switches, so the ExVectrLink module stays independent of the Starship; the vehicle maps a switch position to the motor enable itself (workspace `docs/safety.md` section 3). `REQUEST` (0x11) / `REQUEST_ACK` (0x12) use the action enum `RequestAction` and the status enum `RequestStatus`.
