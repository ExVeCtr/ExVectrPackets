# Telemetry sets

Design 3.5.6 as adapted by the user decisions in the Revision 1 box of `docs/network-design.md`: decoded values are SI, packets are few and large grouped by rate class, USB carries everything useful, the radio only a bare minimum.

## Air (radio) set, ports < 0x80

| Packet | Port | Bytes | Air Hz | B/s incl. 7 B overhead |
|---|---|---|---|---|
| `VEHICLE_STATUS` (slow class: mode, mission, applied arm state, motor gate, control source, failure bits, sensors, last request, rc age, home epoch, calibration state) | 0x20 | 30 | 5 | 185 |
| `AIR_POSE` (q i16, position i32 at 1 mm, velocity i16 at 0.01 m/s) | 0x21 | 30 | 25 | 925 |
| `AIR_ACTUATORS` (two actuator samples per packet, 20 ms apart: TVC deflection, roll torque, thrust, flaps; i16/u16 fixed) | 0x22 | 38 | 25 | 1125 |
| `AIR_MISSION` (mission time, mode, phase, index, waypoint, position/velocity setpoint, attitude setpoint q, fixed) | 0x23 | 38 | 5 | 225 |
| `AIR_GNSS` (lat/lon i32 at 1e-7 deg, altitude i32 mm, NWU velocity i16 x3 at 0.01 m/s) | 0x24 | 22 | 10 | 290 |
| `AIR_GNSS_INFO` (fix type, satellites, h/v accuracy u16 cm) | 0x25 | 10 | 1 | 17 |
| `HANDSET_TELEM` | 0x03 | 34 | 2 | 82 |
| `LINK_STATS` (from node 3) | 0x01 | 19 | 2 | 52 |
| heartbeats (nodes 1 and 3) | - | 11 | 1 | 36 |
| **Total** | | | | **about 2.99 kB/s** (air downlink entry capacity: 6.15 kB/s, 48 %; workspace `tools/network_budget.py` counts LINK_STATS at 5 Hz: about 3.01 kB/s, 49 %; it was 2.45 kB/s, 40 %, with `AIR_POSE` at 10 Hz, and 2.14 kB/s, 35 %, before `AIR_GNSS`) |

`GNSS`, `ACTUATORS`, `MISSION` and the raw/debug packets are not on the radio; `AIR_ACTUATORS`, `AIR_MISSION` and `AIR_GNSS` (+ `AIR_GNSS_INFO`) are their air versions. `HANDSET_TELEM` already carries position, speed and attitude for the Pocket display. A test checks that the air set stays below 3000 B/s (49 % of the air downlink; the limit was 1.2 kB/s before `AIR_ACTUATORS` and `AIR_MISSION`, 2150 B/s before `AIR_GNSS`, 2500 B/s before `AIR_POSE` went to 25 Hz; the set computes to 2901 B/s, so about 100 B/s of headroom is left on purpose, any new air stream must raise the limit deliberately).

`AIR_GNSS` (2026-11-21, H1): GNSS position and the full NWU velocity at 10 Hz for verifying the barometric/estimated altitude during descent; fix type, satellites and h/v accuracy are in `AIR_GNSS_INFO` at 1 Hz (the split costs 307 B/s against 370 B/s for one 30 B packet at 10 Hz). Altitude is as `GNSS.altitude` (the driver's ellipsoid height; the datum does not matter for pad-relative use). Sentinel values decode as NaN. `HOME_POSITION` (0x50) is not a stream: it is the response object of a request channel (workspace `docs/reliable-transport-design.md` 8.1), see `docs/packet-reference.md`; `VEHICLE_STATUS.home_epoch` tells the GS when to request it again.

`AIR_ACTUATORS` sampling: the vehicle takes one actuator sample every 20 ms (50 Hz) into a ring and sends the newest two at 25 Hz, on a sampling tick. 25 Hz is the most the air link can carry for one packet type (25 downlink air packets/s, each aggregator entry type at most once per air packet). The ground station turns each packet into two timestamped samples: `t_us` and `t_us - prev_age`. Array order is newest first: `tvc = [x, y, x_prev, y_prev]`, `roll_torque = [new, prev]`, `thrust = [new, prev]`, `flaps = [TL, TR, BL, BR, TL_prev, TR_prev, BL_prev, BR_prev]`. With no previous sample, `prev_age` and the previous values are NaN.

Attitude setpoint (`MISSION.setpoint_q`, `AIR_MISSION.setpoint_q`, appended 2026-11-21): the attitude the currently active attitude controller is tracking, `[w,x,y,z]` body -> world in the NWU world frame, the same convention as `POSE.q` / `AIR_POSE.q`, so a client can draw the wanted body axes next to the real ones. All four components 0 = no attitude setpoint (no attitude controller active, for example idle, on the ground, countdown); otherwise a unit quaternion. The field is optional (append-only): from an older sender it decodes as NaN, which a client treats like 0 (no setpoint). Air encoding as `AIR_POSE.q` (i16, `limit = 1.0`, 1/32767 steps).

## USB set (full)

All of the above that have a USB rate, plus ports 0x80 and up (never forwarded to air or UART).

| Class | Packet | Port | Bytes | USB Hz |
|---|---|---|---|---|
| fast | `POSE` (omega, q, velocity, position, f32) | 0x80 | 56 | 100 |
| medium | `ACTUATORS` (flaps `[TL,TR,BL,BR]` rad, 0 = extended, increasing = folded toward the back; TVC; thrust; ground distance, roll torque) | 0x81 | 40 | 50 |
| slow | `VEHICLE_STATUS` | 0x20 | 30 | 20 |
| slow | `MISSION` (position/velocity setpoint, mission time, index, phase, attitude setpoint q, mission mode, waypoint) | 0x82 | 52 | 20 |
| slow | `GNSS` (lat/lon radians, f64) | 0x83 | 50 | 10 |
| slow | `HANDSET_TELEM` | 0x03 | 34 | 2 |
| debug | `IMU_RAW` 0x90 (100 Hz), `ESTIMATOR` 0x91 (20 Hz), `TASK_TIMING` 0x92 (5 Hz) | | 48/40/37 | |
| debug | `CALIBRATION` 0x93 (applied sensor calibration: offsets, 3x3 matrices, sources, EKF gyro bias), every 2 s | | 164 | 0.5 |

`AIR_POSE` is not sent on USB (`POSE` supersedes it); a ground station connected by radio only uses `AIR_POSE`, one connected by USB uses `POSE`. The rates are the targets for the flight software's low-priority telemetry task; they are metadata (`kUsbRateHz`, `kAirRateHz`), not enforced by the library.

## Control and request packets

`RC_STATE` (0x10, 34 B) carries the 16 channels and the flag `SOURCE_OK` (bit 0: the handset data is valid; bits 1..7 reserved, zero). `RC_STATE` is deliberately generic: it carries only the controller's sticks and switches, so the ExVectrLink module stays independent of the Starship; the vehicle maps a switch position to the motor enable itself (workspace `docs/safety.md` section 3). `REQUEST` (0x11) / `REQUEST_ACK` (0x12) use the action enum `RequestAction` and the status enum `RequestStatus`.

`AIR_POSE` 25 Hz (owner request 2026-11-22, was 10 Hz): smoother pose on the radio-only ground station. `rate_air` is part of the schema hash, so the hash changed (0x365B8308 to 0xF9A16438). The vehicle's tick is 100 Hz, so 25 Hz is every 4th tick (40 ms).

Integration fields (2026-11-22, one append-only change, hash `0xF9A16438` -> `0x414DE27A`): `ACTUATORS.roll_torque` (f32 N*m, +4 B, 36 -> 40; the same quantity as `AIR_ACTUATORS.roll_torque`, unquantised; older sender: NaN), `MISSION.mission_mode` and `MISSION.waypoint` (u8 each, +2 B, 50 -> 52; as `AIR_MISSION`, waypoint 255 = the active mission is not the waypoint mission; older sender: 0), `VEHICLE_STATUS.calib_state` (u8, +1 B, 29 -> 30). `calib_state` bits: 0 gyro calibration valid, 1 accel valid, 2 mag valid, 3 a calibration session is active (collecting or fitting), 4 ready to arm (all gates the vehicle applies for Arm and MissionBegin are satisfied), 5-7 reserved (zero); 0 from an older sender = unknown. Budget: `VEHICLE_STATUS` is the only air-set change, +1 B at 5 Hz = +5 B/s on air (180 -> 185 B/s incl. overhead; air downlink 2896 -> 2901 B/s, workspace `tools/network_budget.py` 3006 -> 3011 B/s, 48.9 % -> 49.0 % of 6150 B/s; the < 3000 B/s test still passes, about 99 B/s headroom). UART and Router USB carry the same packet: +5 B/s each (frame 41 B, far below the 234 B payload limit). USB set: `VEHICLE_STATUS` at 20 Hz +20 B/s, `ACTUATORS` +200 B/s, `MISSION` +40 B/s.
