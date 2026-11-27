#!/usr/bin/env python3
"""Writes test/vectors/packets.json. The byte strings are built here with plain `struct` and explicit raw
integers, independently of the generator, so the vectors check the generated C++ and Python against the spec.
Run: python3 tools/make_vectors.py   (output is committed; test/python checks it is up to date)."""
import json
import math
import struct
import sys
from pathlib import Path

S_RAD7 = 1.7453292519943295e-9      # lat/lon: deg*1e7 -> rad
S_CDEG = 1.7453292519943295e-4      # heading: centi-degrees -> rad
NAN = None                           # JSON null = NaN
cases, decode_only, decode_fail = [], [], []


def P(fmt, *a):
    return struct.pack("<" + fmt, *a)


def hexs(b):
    return b.hex()


def case(name, packet, values, payload, decoded=None):
    c = {"name": name, "packet": packet, "values": values, "hex": hexs(payload)}
    if decoded is not None:
        c["decoded"] = decoded
    cases.append(c)


# ------------------------------------------------------------------ common
case("heartbeat_4_8", "HEARTBEAT",
     {"role": 1, "net_version": 1, "schema_hash": 0x12345678, "uptime": 1.0, "flags": 0},
     bytes.fromhex("01 01 78 56 34 12 E8 03 00 00 00".replace(" ", "")))
case("request_ack_4_8", "REQUEST_ACK", {"req_id": 0x0102, "status": 0, "detail": 0},
     bytes.fromhex("02010000"))
case("request_ack_rejected", "REQUEST_ACK", {"req_id": 7, "status": 2, "detail": 9}, P("HBB", 7, 2, 9))
# REQUEST.session (appended, optional, offset 12): CMD_STATE session of the sender, 0 = none (Router/ship-node requests)
case("request", "REQUEST", {"req_id": 0x0102, "action": 7, "target": 2, "arg_i": -5, "arg_f": 1.5, "session": 0xDEADBEEF},
     P("HBBifI", 0x0102, 7, 2, -5, 1.5, 0xDEADBEEF))
case("request_link_upload_mode", "REQUEST", {"req_id": 5, "action": 10, "target": 3, "arg_i": 0, "arg_f": 0.0, "session": 0},
     P("HBBifI", 5, 10, 3, 0, 0.0, 0))
case("request_link_reboot", "REQUEST", {"req_id": 6, "action": 11, "target": 2, "arg_i": 0, "arg_f": 0.0, "session": 0},
     P("HBBifI", 6, 11, 2, 0, 0.0, 0))
case("request_vehicle_session", "REQUEST", {"req_id": 9, "action": 12, "target": 1, "arg_i": 0, "arg_f": 10.0, "session": 1},
     P("HBBifI", 9, 12, 1, 0, 10.0, 1))
case("request_set_stop_altitude", "REQUEST", {"req_id": 10, "action": 13, "target": 1, "arg_i": 0, "arg_f": 2.5, "session": 77},
     P("HBBifI", 10, 13, 1, 0, 2.5, 77))
case("request_calib_control", "REQUEST", {"req_id": 20, "action": 14, "target": 1, "arg_i": 21, "arg_f": 0.0, "session": 77},
     P("HBBifI", 20, 14, 1, 21, 0.0, 77))
case("request_calib_apply", "REQUEST", {"req_id": 20, "action": 7, "target": 1, "arg_i": 2, "arg_f": 0.0, "session": 77},
     P("HBBifI", 20, 7, 1, 2, 0.0, 77))
case("request_save_config", "REQUEST", {"req_id": 20, "action": 8, "target": 1, "arg_i": 0, "arg_f": 0.0, "session": 77},
     P("HBBifI", 20, 8, 1, 0, 0.0, 77))
case("request_sim_wind_speed", "REQUEST", {"req_id": 20, "action": 15, "target": 1, "arg_i": 0, "arg_f": 5.0, "session": 77},
     P("HBBifI", 20, 15, 1, 0, 5.0, 77))
case("request_sim_wind_direction", "REQUEST", {"req_id": 20, "action": 16, "target": 1, "arg_i": 0, "arg_f": 1.5, "session": 77},
     P("HBBifI", 20, 16, 1, 0, 1.5, 77))
case("request_sim_gust_strength", "REQUEST", {"req_id": 20, "action": 17, "target": 1, "arg_i": 0, "arg_f": 3.0, "session": 77},
     P("HBBifI", 20, 17, 1, 0, 3.0, 77))
case("request_sim_gust_period", "REQUEST", {"req_id": 20, "action": 18, "target": 1, "arg_i": 0, "arg_f": 4.0, "session": 77},
     P("HBBifI", 20, 18, 1, 0, 4.0, 77))
case("request_sim_gust_seed", "REQUEST", {"req_id": 20, "action": 19, "target": 1, "arg_i": 0, "arg_f": 12345.0, "session": 77},
     P("HBBifI", 20, 19, 1, 0, 12345.0, 77))
case("request_sim_flap_stuck", "REQUEST", {"req_id": 20, "action": 20, "target": 1, "arg_i": 2, "arg_f": -1.0, "session": 77},
     P("HBBifI", 20, 20, 1, 2, -1.0, 77))
case("request_sim_flap_slow", "REQUEST", {"req_id": 20, "action": 21, "target": 1, "arg_i": 3, "arg_f": 0.25, "session": 77},
     P("HBBifI", 20, 21, 1, 3, 0.25, 77))
case("request_sim_fault_clear", "REQUEST", {"req_id": 20, "action": 22, "target": 1, "arg_i": 0, "arg_f": 0.0, "session": 77},
     P("HBBifI", 20, 22, 1, 0, 0.0, 77))
case("request_get_home_position", "REQUEST", {"req_id": 20, "action": 23, "target": 1, "arg_i": 0, "arg_f": 0.0, "session": 77},
     P("HBBifI", 20, 23, 1, 0, 0.0, 77))
case("request_set_mission_param", "REQUEST", {"req_id": 21, "action": 24, "target": 1, "arg_i": 1, "arg_f": 4.5, "session": 77},
     P("HBBifI", 21, 24, 1, 1, 4.5, 77))
decode_only.append({"name": "request_older_sender_no_session", "packet": "REQUEST",
                    "hex": hexs(P("HBBif", 0x0102, 7, 2, -5, 1.5)),
                    "decoded": {"req_id": 0x0102, "action": 7, "target": 2, "arg_i": -5, "arg_f": 1.5, "session": 0}})
ch = [-1000, -500, 0, 500, 1000, -32768, 1, -1, 100, 200, 300, 400, 500, 600, 700, 800]
case("rc_state", "RC_STATE", {"flags": 1, "reserved": 0, "ch": ch}, P("BB16h", 1, 0, *ch))
case("rc_state_not_ok", "RC_STATE", {"flags": 0, "reserved": 0, "ch": [0] * 16}, P("BB16h", 0, 0, *([0] * 16)))
case("link_stats", "LINK_STATS",
     {"uptime": 123.456, "air_state": 3, "rx_lq": 97, "rssi": -80, "snr": -7, "tx_power": 14, "antenna": 0x21,
      "air_rx_ok": 1000, "air_rx_bad": 3, "frames_dropped": 2, "frames_expired": 1, "queue_peak": 55},
     P("IBBbbbBHHHHB", 123456, 3, 97, -80, -7, 14, 0x21, 1000, 3, 2, 1, 55))
case("link_config", "LINK_CONFIG", {"tx_power_dbm": 10, "flags": 0}, P("bB", 10, 0))
case("link_config_negative", "LINK_CONFIG", {"tx_power_dbm": -3, "flags": 5}, P("bB", -3, 5))
case("log_text", "LOG_TEXT", {"severity": 2, "text": "héllo"}, bytes([2]) + "héllo".encode())
case("log_text_empty", "LOG_TEXT", {"severity": 0, "text": ""}, bytes([0]))
lat_raw, lon_raw = 473977000, 85455000
hand = P("B8sii HH h B h hhh H".replace(" ", ""), 9, b"ARMED\0\0\0", lat_raw, lon_raw, 1234, 4567, 321, 12, -250,
         -15708, 1000, 31416, 12600)
hv = {"flags": 9, "mode_text": "ARMED", "latitude": lat_raw * S_RAD7, "longitude": lon_raw * S_RAD7,
      "ground_speed": 12.34, "heading": 4567 * S_CDEG, "altitude": 321.0, "satellites": 12,
      "vertical_speed": -2.5, "pitch": -1.5708, "roll": 0.1, "yaw": 3.1416, "battery": 12.6}
case("handset_telem", "HANDSET_TELEM", hv, hand,
     {**hv, "ground_speed": 1234 * 0.01, "vertical_speed": -250 * 0.01, "pitch": -15708 * 1e-4,
      "roll": 1000 * 1e-4, "yaw": 31416 * 1e-4, "battery": 12600 * 0.001})
# invalid / saturation on the wire format of HANDSET_TELEM
hand2 = P("B8sii HH h B h hhh H".replace(" ", ""), 0, b"\0" * 8, -2 ** 31, 2 ** 31 - 1, 65535, 65535, -32768, 0,
          -32768, 32767, -32767, 32767, 0)
case("handset_telem_invalid_saturated", "HANDSET_TELEM",
     {"flags": 0, "mode_text": "", "latitude": NAN, "longitude": (2 ** 31 - 1) * S_RAD7 * 2, "ground_speed": NAN,
      "heading": NAN, "altitude": NAN, "satellites": 0, "vertical_speed": NAN, "pitch": 10.0, "roll": -10.0,
      "yaw": 3.5, "battery": NAN},
     hand2,
     {"flags": 0, "mode_text": "", "latitude": NAN, "longitude": (2 ** 31 - 1) * S_RAD7, "ground_speed": NAN,
      "heading": NAN, "altitude": NAN, "satellites": 0, "vertical_speed": NAN, "pitch": 32767 * 1e-4,
      "roll": -32767 * 1e-4, "yaw": 32767 * 1e-4, "battery": NAN})

# ------------------------------------------------------------------ starship
T9B0 = {"ctrl_flags": 0, "go_refusal": 0, "degraded_bits": 0, "mission_profile": 0, "rc_rejected": 0, "cmd_session": 0, "profile_selected": 0}
vs = {"t_us": 4000000000, "t_ns": 4000000000123, "mode": 3, "mission_mode": 2, "mission_phase": 5, "ctrl_source": 1,
      "flags": 0x07, "failure_bits": 0x80000001, "sensor_state": 0x0155, "last_req_id": 0x0102,
      "last_req_status": 0, "rc_age": 0.02, "home_epoch": 2, "calib_state": 0x17,
      "ctrl_flags": 0x0481, "go_refusal": 3, "degraded_bits": 0x05, "mission_profile": 255, "rc_rejected": 200,
      "cmd_session": 0xABCD, "profile_selected": 2}
case("vehicle_status", "VEHICLE_STATUS", vs,
     P("IqBBBBBIHHBHBBHBBBBHB", 4000000000, 4000000000123, 3, 2, 5, 1, 7, 0x80000001, 0x0155, 0x0102, 0, 20, 2, 0x17, 0x0481, 3, 5, 255, 200, 0xABCD, 2),
     {**vs, "rc_age": 20 * 0.001})
case("vehicle_status_no_rc", "VEHICLE_STATUS", {**vs, "rc_age": NAN},
     P("IqBBBBBIHHBHBBHBBBBHB", 4000000000, 4000000000123, 3, 2, 5, 1, 7, 0x80000001, 0x0155, 0x0102, 0, 65535, 2, 0x17, 0x0481, 3, 5, 255, 200, 0xABCD, 2))
# older sender (before cmd_session was appended, has the T9b tail): cmd_session decodes as 0
decode_only.append({"name": "vehicle_status_older_sender_no_cmd_session", "packet": "VEHICLE_STATUS",
                    "hex": hexs(P("IqBBBBBIHHBHBBHBBBB", 4000000000, 4000000000123, 3, 2, 5, 1, 7, 0x80000001, 0x0155, 0x0102, 0, 20, 2, 0x17, 0x0481, 3, 5, 255, 200)),
                    "decoded": {**vs, "rc_age": 20 * 0.001, "cmd_session": 0, "profile_selected": 0}})
# older sender (before profile_selected was appended, has cmd_session): profile_selected decodes as 0
decode_only.append({"name": "vehicle_status_older_sender_no_profile_selected", "packet": "VEHICLE_STATUS",
                    "hex": hexs(P("IqBBBBBIHHBHBBHBBBBH", 4000000000, 4000000000123, 3, 2, 5, 1, 7, 0x80000001, 0x0155, 0x0102, 0, 20, 2, 0x17, 0x0481, 3, 5, 255, 200, 0xABCD)),
                    "decoded": {**vs, "rc_age": 20 * 0.001, "profile_selected": 0}})
# older sender (before home_epoch was appended): the optional tail decodes as 0
decode_only.append({"name": "vehicle_status_older_sender_no_home_epoch", "packet": "VEHICLE_STATUS",
                    "hex": hexs(P("IqBBBBBIHHBH", 4000000000, 4000000000123, 3, 2, 5, 1, 7, 0x80000001, 0x0155, 0x0102, 0, 20)),
                    "decoded": {**vs, "rc_age": 20 * 0.001, "home_epoch": 0, "calib_state": 0, **T9B0}})
# older sender (before calib_state was appended, has home_epoch): calib_state decodes as 0 = unknown
decode_only.append({"name": "vehicle_status_older_sender_no_calib_state", "packet": "VEHICLE_STATUS",
                    "hex": hexs(P("IqBBBBBIHHBHB", 4000000000, 4000000000123, 3, 2, 5, 1, 7, 0x80000001, 0x0155, 0x0102, 0, 20, 2)),
                    "decoded": {**vs, "rc_age": 20 * 0.001, "calib_state": 0, **T9B0}})
# older sender (before the T9b fields were appended, has calib_state): the T9b tail decodes as 0
decode_only.append({"name": "vehicle_status_older_sender_no_t9b_fields", "packet": "VEHICLE_STATUS",
                    "hex": hexs(P("IqBBBBBIHHBHBB", 4000000000, 4000000000123, 3, 2, 5, 1, 7, 0x80000001, 0x0155, 0x0102, 0, 20, 2, 0x17)),
                    "decoded": {**vs, "rc_age": 20 * 0.001, **T9B0}})
# CMD_APPLIED (USB only): applied command fields, info flags, safe-seen bits, session, ages (65535 = none), rejects
ca = {"t_us": 5000, "go": 1, "profile": 2, "vehicle_mode": 1, "motors": 1, "reset_button": 0, "failsafe": 0,
      "flags": 0x03, "safe_seen": 0x0D, "session": 0xA1B2C3D4, "age": 0.123, "usb_age": NAN, "uart_age": 0.456,
      "rejects": [1, 2, 255, 0]}
case("cmd_applied", "CMD_APPLIED", ca,
     P("I8BIHHH4B", 5000, 1, 2, 1, 1, 0, 0, 0x03, 0x0D, 0xA1B2C3D4, 123, 65535, 456, 1, 2, 255, 0))
# CMD_STATE (0x13, 14 B): flags, version, session, go, profile, vehicle_mode, motors, reset_button, failsafe, reserved
cs = {"flags": 3, "version": 1, "session": 0x12345678, "go": 1, "profile": 2, "vehicle_mode": 1, "motors": 1,
      "reset_button": 0, "failsafe": 0, "reserved": 0}
case("cmd_state", "CMD_STATE", cs, P("BBIBBBBBBH", 3, 1, 0x12345678, 1, 2, 1, 1, 0, 0, 0))
cs_safe = {"flags": 1, "version": 1, "session": 1, "go": 0, "profile": 0, "vehicle_mode": 0, "motors": 0,
           "reset_button": 0, "failsafe": 0, "reserved": 0}
case("cmd_state_safe_idle", "CMD_STATE", cs_safe, P("BBIBBBBBBH", 1, 1, 1, 0, 0, 0, 0, 0, 0, 0))
cs_fs = {"flags": 7, "version": 1, "session": 0xFFFFFFFF, "go": 0, "profile": 1, "vehicle_mode": 0, "motors": 0,
         "reset_button": 1, "failsafe": 1, "reserved": 0}
case("cmd_state_failsafe", "CMD_STATE", cs_fs, P("BBIBBBBBBH", 7, 1, 0xFFFFFFFF, 0, 1, 0, 0, 1, 1, 0))
decode_only.append({"name": "cmd_state_longer_ignored", "packet": "CMD_STATE",
                    "hex": hexs(P("BBIBBBBBBHB", 3, 1, 0x12345678, 1, 2, 1, 1, 0, 0, 0, 9)), "decoded": cs})
q = [32767, -32767, 0, 16384]
pos = [100, -100, 1234567]  # mm, beyond the old i16 range
vel = [-327, 0, 1]
S_Q = 1.0 / 32767
ap = {"t_us": 1000, "q": [r * S_Q for r in q], "pos": [r * 0.001 for r in pos], "vel": [r * 0.01 for r in vel]}
ap_nogd = {**ap, "ground_distance": NAN}
case("air_pose", "AIR_POSE", ap_nogd, P("I4h3i3hH", 1000, *q, *pos, *vel, 65535))
case("air_pose_ground_distance", "AIR_POSE", {**ap, "ground_distance": 1234 * 0.001}, P("I4h3i3hH", 1000, *q, *pos, *vel, 1234))
case("air_pose_ground_distance_max", "AIR_POSE", {**ap, "ground_distance": 65534 * 0.001}, P("I4h3i3hH", 1000, *q, *pos, *vel, 65534))
# older sender (before ground_distance was appended): decodes as NaN
decode_only.append({"name": "air_pose_older_sender_no_ground_distance", "packet": "AIR_POSE",
                    "hex": hexs(P("I4h3i3h", 1000, *q, *pos, *vel)), "decoded": ap_nogd})
# AIR_ACTUATORS: two samples, saturation (tvc beyond +-3.2767 rad, thrust beyond 65.534 N) and NaN (sentinels)
aa_tvc = [523, -523, 32767, -32767]          # 0.1 mrad
aa_rt = [1500, -32767]                       # 0.01 mN*m
aa_th = [12500, 0]                           # mN
aa_fl = [2618, 2618, 15708, 15708, 2700, 2800, 32767, 0]
aav = {"t_us": 123456, "prev_age": 0.02, "tvc": [r * 1e-4 for r in aa_tvc[:2]] + [4.0, -4.0],
       "roll_torque": [0.015, -1.0], "thrust": [12.5, 0.0], "flaps": [r * 1e-4 for r in aa_fl[:6]] + [5.0, 0.0]}
case("air_actuators", "AIR_ACTUATORS", aav, P("IH4h2h2H8h", 123456, 20000, *aa_tvc, *aa_rt, *aa_th, *aa_fl),
     {**aav, "prev_age": 20000 * 1e-6, "tvc": [r * 1e-4 for r in aa_tvc], "roll_torque": [r * 1e-5 for r in aa_rt],
      "thrust": [r * 1e-3 for r in aa_th], "flaps": [r * 1e-4 for r in aa_fl]})
aan = {"t_us": 1, "prev_age": NAN, "tvc": [0.0, 0.0, NAN, NAN], "roll_torque": [0.0, NAN], "thrust": [1.0, NAN],
       "flaps": [0.5, 0.5, 0.5, 0.5, NAN, NAN, NAN, NAN]}
case("air_actuators_no_previous", "AIR_ACTUATORS", aan,
     P("IH4h2h2H8h", 1, 65535, 0, 0, -32768, -32768, 0, -32768, 1000, 65535, *([5000] * 4), *([-32768] * 4)))
amv = {"t_us": 77, "mission_time": -9.5, "mission_mode": 1, "phase": 1, "index": 1, "waypoint": 255,
       "setpoint_pos": [0.0, -0.001, 120.0], "setpoint_vel": [0.0, 0.5, 7.0], "setpoint_q": [0.0, 0.0, 0.0, 0.0],
       "flip_start_altitude": 123.4, "stop_altitude": 20.0, "mission_elapsed": -9.5}
case("air_mission", "AIR_MISSION", amv,
     P("IiBBBB3i3h4h2hi", 77, -9500, 1, 1, 1, 255, 0, -1, 120000, 0, 50, 700, 0, 0, 0, 0, 1234, 200, -9500),
     {**amv, "mission_time": -9500 * 0.001, "mission_elapsed": -9500 * 0.001, "flip_start_altitude": 1234 * 0.1, "stop_altitude": 200 * 0.1, "setpoint_pos": [0.0, -1 * 0.001, 120000 * 0.001],
      "setpoint_vel": [0.0, 50 * 0.01, 700 * 0.01]})
# attitude setpoint present: q = [0.5, 0.5, -0.5, 0.5] (unit), i16 limit 1.0 -> raw = round(v * 32767)
amq = {**amv, "mission_elapsed": NAN, "flip_start_altitude": NAN, "stop_altitude": 60.0, "mission_mode": 2, "phase": 2, "waypoint": 3, "setpoint_q": [0.5, 0.5, -0.5, 0.5]}
case("air_mission_attitude_setpoint", "AIR_MISSION", amq,
     P("IiBBBB3i3h4h2hi", 77, -9500, 2, 2, 1, 3, 0, -1, 120000, 0, 50, 700, 16384, 16384, -16384, 16384, -32768, 600, -2**31),
     {**amq, "mission_elapsed": NAN, "flip_start_altitude": NAN, "stop_altitude": 60.0, "mission_time": -9500 * 0.001, "setpoint_pos": [0.0, -1 * 0.001, 120000 * 0.001],
      "setpoint_vel": [0.0, 50 * 0.01, 700 * 0.01], "setpoint_q": [r / 32767 for r in (16384, 16384, -16384, 16384)]})
pv = {"t_us": 5, "omega": [0.5, -0.25, 1.0], "q": [1.0, 0.0, 0.0, -0.5], "vel": [1.5, 2.5, -3.5], "pos": [10.0, -20.0, 30.5], "ground_distance": 0.875}
case("pose", "POSE", pv, P("I3f4f3f3ff", 5, *pv["omega"], *pv["q"], *pv["vel"], *pv["pos"], 0.875))
case("pose_no_ground_distance", "POSE", {**pv, "ground_distance": NAN}, P("I3f4f3f3ff", 5, *pv["omega"], *pv["q"], *pv["vel"], *pv["pos"], float("nan")))
# older sender (before ground_distance was appended): decodes as NaN
decode_only.append({"name": "pose_older_sender_no_ground_distance", "packet": "POSE",
                    "hex": hexs(P("I3f4f3f3f", 5, *pv["omega"], *pv["q"], *pv["vel"], *pv["pos"])), "decoded": {**pv, "ground_distance": NAN}})
av = {"t_us": 6, "flaps": [0.0, 0.25, 1.5, 0.75], "tvc": [0.125, -0.125], "thrust": 12.5, "roll_torque": -0.0625}
case("actuators", "ACTUATORS", av,
     P("I4f2fff", 6, *av["flaps"], *av["tvc"], 12.5, -0.0625))
# older sender (before roll_torque was appended): decodes as NaN = unknown
decode_only.append({"name": "actuators_older_sender_no_roll_torque", "packet": "ACTUATORS",
                    "hex": hexs(P("I4f2ff", 6, *av["flaps"], *av["tvc"], 12.5)),
                    "decoded": {**av, "roll_torque": NAN}})
mv = {"t_us": 7, "setpoint_pos": [1.0, 2.0, 3.0], "setpoint_vel": [0.0, 0.5, -0.5], "mission_time": 12.345, "index": 4, "phase": 2,
      "setpoint_q": [0.5, -0.5, 0.5, -0.5], "mission_mode": 3, "waypoint": 2, "flip_start_altitude": 87.5, "stop_altitude": 20.0, "mission_elapsed": 123.456}
case("mission", "MISSION", mv, P("I3f3fiBB4fBBffi", 7, 1.0, 2.0, 3.0, 0.0, 0.5, -0.5, 12345, 4, 2, 0.5, -0.5, 0.5, -0.5, 3, 2, 87.5, 20.0, 123456),
     {**mv, "mission_time": 12345 * 0.001, "mission_elapsed": 123456 * 0.001})
# older sender (before setpoint_q was appended): the optional tail decodes as NaN (= unknown, no setpoint)
decode_only.append({"name": "mission_older_sender_no_setpoint_q", "packet": "MISSION",
                    "hex": hexs(P("I3f3fiBB", 7, 1.0, 2.0, 3.0, 0.0, 0.5, -0.5, 12345, 4, 2)),
                    "decoded": {**mv, "mission_time": 12345 * 0.001, "setpoint_q": [NAN] * 4, "mission_mode": 0, "waypoint": 0, "flip_start_altitude": NAN, "stop_altitude": NAN, "mission_elapsed": NAN}})
# older sender (has setpoint_q, before mission_mode and waypoint were appended): both decode as 0
decode_only.append({"name": "mission_older_sender_no_mode_waypoint", "packet": "MISSION",
                    "hex": hexs(P("I3f3fiBB4f", 7, 1.0, 2.0, 3.0, 0.0, 0.5, -0.5, 12345, 4, 2, 0.5, -0.5, 0.5, -0.5)),
                    "decoded": {**mv, "mission_time": 12345 * 0.001, "mission_mode": 0, "waypoint": 0, "flip_start_altitude": NAN, "stop_altitude": NAN, "mission_elapsed": NAN}})
decode_only.append({"name": "mission_older_sender_no_flip_start", "packet": "MISSION",
                    "hex": hexs(P("I3f3fiBB4fBB", 7, 1.0, 2.0, 3.0, 0.0, 0.5, -0.5, 12345, 4, 2, 0.5, -0.5, 0.5, -0.5, 3, 2)),
                    "decoded": {**mv, "mission_time": 12345 * 0.001, **{"flip_start_altitude": NAN, "stop_altitude": NAN, "mission_elapsed": NAN}}})
decode_only.append({"name": "mission_older_sender_no_mission_elapsed", "packet": "MISSION",
                    "hex": hexs(P("I3f3fiBB4fBBff", 7, 1.0, 2.0, 3.0, 0.0, 0.5, -0.5, 12345, 4, 2, 0.5, -0.5, 0.5, -0.5, 3, 2, 87.5, 20.0)),
                    "decoded": {**mv, "mission_time": 12345 * 0.001, "mission_elapsed": NAN}})
case("mission_not_waypoint", "MISSION", {**mv, "mission_mode": 4, "waypoint": 255, "mission_elapsed": NAN},
     P("I3f3fiBB4fBBffi", 7, 1.0, 2.0, 3.0, 0.0, 0.5, -0.5, 12345, 4, 2, 0.5, -0.5, 0.5, -0.5, 4, 255, 87.5, 20.0, -2**31),
     {**mv, "mission_time": 12345 * 0.001, "mission_mode": 4, "waypoint": 255, "mission_elapsed": NAN})
decode_only.append({"name": "air_mission_older_sender_no_setpoint_q", "packet": "AIR_MISSION",
                    "hex": hexs(P("IiBBBB3i3h", 77, -9500, 1, 1, 1, 255, 0, -1, 120000, 0, 50, 700)),
                    "decoded": {**amv, "mission_time": -9500 * 0.001, "setpoint_pos": [0.0, -1 * 0.001, 120000 * 0.001],
                                "setpoint_vel": [0.0, 50 * 0.01, 700 * 0.01], "setpoint_q": [NAN] * 4,
                                "flip_start_altitude": NAN, "stop_altitude": NAN, "mission_elapsed": NAN}})
decode_only.append({"name": "air_mission_older_sender_no_mission_elapsed", "packet": "AIR_MISSION",
                    "hex": hexs(P("IiBBBB3i3h4h2h", 77, -9500, 1, 1, 1, 255, 0, -1, 120000, 0, 50, 700, 0, 0, 0, 0, 1234, 200)),
                    "decoded": {**amv, "mission_time": -9500 * 0.001, "setpoint_pos": [0.0, -1 * 0.001, 120000 * 0.001],
                                "setpoint_vel": [0.0, 50 * 0.01, 700 * 0.01], "flip_start_altitude": 1234 * 0.1, "stop_altitude": 200 * 0.1,
                                "mission_elapsed": NAN}})
gv = {"t_us": 8, "latitude": 0.8272, "longitude": 0.149, "altitude": 512.5, "vel": [0.5, 0.25, -1.0],
      "satellites": 14, "fix_type": 3, "h_acc": 1.5, "v_acc": 2.5, "s_acc": 0.25}
case("gnss", "GNSS", gv, P("Iddf3fBBfff", 8, 0.8272, 0.149, 512.5, 0.5, 0.25, -1.0, 14, 3, 1.5, 2.5, 0.25))
# AIR_GNSS: lat/lon i32 deg*1e7, altitude i32 mm, vel i16 x3 cm/s (NWU). AIR_GNSS_INFO: fix u8, sats u8, h_acc/v_acc u16 cm
ag_lat, ag_lon, ag_alt = 473977420, 85455940, 1234567
agv = {"t_us": 31, "latitude": ag_lat * S_RAD7, "longitude": ag_lon * S_RAD7, "altitude": 1234.567,
       "vel": [1.25, -0.5, -45.67]}
case("air_gnss", "AIR_GNSS", agv, P("Iiii3h", 31, ag_lat, ag_lon, ag_alt, 125, -50, -4567),
     {**agv, "altitude": ag_alt * 0.001, "vel": [125 * 0.01, -50 * 0.01, -4567 * 0.01]})
# no fix: every fixed field at its sentinel (i32 min, i16 min) decodes to NaN
case("air_gnss_no_fix", "AIR_GNSS",
     {"t_us": 32, "latitude": NAN, "longitude": NAN, "altitude": NAN, "vel": [NAN, NAN, NAN]},
     P("Iiii3h", 32, -2 ** 31, -2 ** 31, -2 ** 31, -32768, -32768, -32768))
aiv = {"t_us": 33, "fix_type": 3, "satellites": 14, "h_acc": 1.23, "v_acc": 2.5}
case("air_gnss_info", "AIR_GNSS_INFO", aiv, P("IBBHH", 33, 3, 14, 123, 250), {**aiv, "h_acc": 123 * 0.01, "v_acc": 250 * 0.01})
case("air_gnss_info_unknown", "AIR_GNSS_INFO", {"t_us": 34, "fix_type": 0, "satellites": 0, "h_acc": NAN, "v_acc": NAN},
     P("IBBHH", 34, 0, 0, 65535, 65535))
# HOME_POSITION (response object of request channel 0x50): flags u8, lat/lon i32 deg*1e7, altitude i32 mm, h_acc u16 cm,
# epoch u8, set_time i64 ns, reserved u8 = 25 bytes
hpv = {"flags": 0x03, "latitude": ag_lat * S_RAD7, "longitude": ag_lon * S_RAD7, "altitude": 421.5, "h_acc": 0.85,
       "epoch": 2, "set_time": 123456789012, "reserved": 0}
case("home_position", "HOME_POSITION", hpv, P("BiiiHBqB", 3, ag_lat, ag_lon, 421500, 85, 2, 123456789012, 0),
     {**hpv, "altitude": 421500 * 0.001, "h_acc": 85 * 0.01})
# nothing valid yet (flags 0): lat/lon/alt/h_acc at their sentinels
case("home_position_invalid", "HOME_POSITION",
     {"flags": 0, "latitude": NAN, "longitude": NAN, "altitude": NAN, "h_acc": NAN, "epoch": 0, "set_time": 0,
      "reserved": 0},
     P("BiiiHBqB", 0, -2 ** 31, -2 ** 31, -2 ** 31, 65535, 0, 0, 0))
iv = {"t_us": 9, "gyro": [0.5, 0.25, -0.125], "accel": [0.0, 0.0, 9.75], "mag": [2.0, -4.0, 8.0],
      "pressure": 101325.0, "temperature": 293.5}
case("imu_raw", "IMU_RAW", iv, P("I3f3f3fff", 9, 0.5, 0.25, -0.125, 0.0, 0.0, 9.75, 2.0, -4.0, 8.0, 101325.0, 293.5))
ev = {"t_us": 10, "att_std": [0.5, 0.5, 0.25], "vel_std": [0.125, 0.125, 0.125], "pos_std": [1.0, 1.0, 2.0]}
case("estimator", "ESTIMATOR", ev, P("I3f3f3f", 10, 0.5, 0.5, 0.25, 0.125, 0.125, 0.125, 1.0, 1.0, 2.0))
cm = [1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.5]
cv = {"t_us": 12, "raw_frame": 0, "gyro_source": 0x05, "accel_source": 0x03, "mag_source": 0x19,
      "gyro_offset": [0.015625, 0.03125, -0.0078125], "gyro_matrix": cm,
      "accel_offset": [0.25, -0.5, 1.5], "accel_matrix": [0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0],
      "mag_offset": [1.5e-05, -2.5e-06, 4e-06], "mag_matrix": [2.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 2.0],
      "gyro_bias_est": [0.001, NAN, -0.002]}
case("calibration", "CALIBRATION", cv,
     P("IBBBB3f9f3f9f3f9f3f", 12, 0, 0x05, 0x03, 0x19, 0.015625, 0.03125, -0.0078125, *cm, 0.25, -0.5, 1.5,
       0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 1.5e-05, -2.5e-06, 4e-06, 2.0, 0.0, 0.0, 0.0, 2.0, 0.0,
       0.0, 0.0, 2.0, 0.001, float("nan"), -0.002),
     {**cv, "mag_offset": [struct.unpack("<f", struct.pack("<f", v))[0] for v in cv["mag_offset"]],
      "gyro_bias_est": [struct.unpack("<f", struct.pack("<f", 0.001))[0], NAN,
                        struct.unpack("<f", struct.pack("<f", -0.002))[0]]})
mpv = {"ascent_speed": 4.5, "apogee_altitude": 120.0, "hover_hold_s": 30.0, "p1_hold_1_s": 10.0, "p1_hold_2_s": NAN,
       "p1_hold_3_s": 15.0, "ascent_accel": 3.0, "ascent_decel": 2.0, "apogee_x": 25.5, "apogee_y": -40.0}
case("mission_params", "MISSION_PARAMS", mpv, P("10f", 4.5, 120.0, 30.0, 10.0, float("nan"), 15.0, 3.0, 2.0, 25.5, -40.0), mpv)
mpv_old = {**mpv, "ascent_accel": NAN, "ascent_decel": NAN, "apogee_x": NAN, "apogee_y": NAN}
decode_only.append({"name": "mission_params_older_sender_24b", "packet": "MISSION_PARAMS",
                    "hex": hexs(P("6f", 4.5, 120.0, 30.0, 10.0, float("nan"), 15.0)), "decoded": mpv_old})
tavg = [100, 200, 300, 400, 500, 600, 700, 65535]
tmax = [150, 250, 350, 450, 550, 650, 750, 65534]
tv = {"t_us": 11, "count": 7, "avg": [r * 1e-6 for r in tavg[:7]] + [NAN], "max": [r * 1e-6 for r in tmax]}
case("task_timing", "TASK_TIMING", tv, P("IB8H8H", 11, 7, *tavg, *tmax),
     {**tv, "avg": [r * 1e-6 for r in tavg[:7]] + [NAN]})

# ------------------------------------------------------------------ KITCHEN_SINK: every kind + fixed-point rules
SINK_DEFAULT = dict(u8v=0, i8v=0, u16v=0, i16v=0, u32v=0, i32v=0, u64v=0, i64v=0, f32v=0.0, f64v=0.0, flag=False,
                    flags=[False] * 3, colour=0, colours=[0, 0], name="", fx=0.0, fu=0.0, fz=1.0, fi32=0.0, tail={"hex": ""})


def sink_bytes(*, base, fx_raw=0, fu_raw=0, fz_raw=1, fi32_raw=0):
    b = base
    return (P("BbHhIiQq", b["u8v"], b["i8v"], b["u16v"], b["i16v"], b["u32v"], b["i32v"], b["u64v"], b["i64v"])
            + P("fd", b["f32v"], b["f64v"]) + P("B3B", int(b["flag"]), *[int(x) for x in b["flags"]])
            + P("B2B", b["colour"], *b["colours"]) + b["name"].encode().ljust(4, b"\0")
            + P("hBHi", fx_raw, fu_raw, fz_raw, fi32_raw) + bytes.fromhex(b["tail"]["hex"]))


full = dict(u8v=255, i8v=-128, u16v=65535, i16v=-32768, u32v=4294967295, i32v=-2147483648, u64v=18446744073709551615,
            i64v=-9223372036854775808, f32v=-0.5, f64v=1e300, flag=True, flags=[True, False, True], colour=2,
            colours=[1, 0], name="AB", fx=-3.0, fu=7.0, fz=9.0, fi32=-1.5, tail={"hex": "deadbeef"})
case("sink_all_kinds", "KITCHEN_SINK", full,
     sink_bytes(base=full, fx_raw=-3, fu_raw=7, fz_raw=9, fi32_raw=-3), None)
case("sink_zero_tail_empty", "KITCHEN_SINK", {**SINK_DEFAULT, "fz": 1.0}, sink_bytes(base=SINK_DEFAULT))
case("sink_nan_floats_canonical", "KITCHEN_SINK", {**SINK_DEFAULT, "f32v": NAN, "f64v": NAN},
     P("BbHhIiQq", 0, 0, 0, 0, 0, 0, 0, 0) + bytes.fromhex("0000c07f" + "000000000000f87f")
     + sink_bytes(base=SINK_DEFAULT)[42:])
# fixed-point rules (one field per vector): value -> raw -> decoded
rules = [
    # name, field, input, raw, decoded (None = NaN)
    ("fx_round_half_up", "fx", 2.5, 3, 3.0), ("fx_round_half_down_neg", "fx", -2.5, -3, -3.0),
    ("fx_round_down", "fx", 2.25, 2, 2.0), ("fx_round_up", "fx", 2.75, 3, 3.0),
    ("fx_half_pos_small", "fx", 0.5, 1, 1.0), ("fx_half_neg_small", "fx", -0.5, -1, -1.0),
    ("fx_below_half", "fx", 0.375, 0, 0.0), ("fx_saturate_hi", "fx", 40000.0, 32767, 32767.0),
    ("fx_saturate_lo", "fx", -40000.0, -32767, -32767.0), ("fx_exact_max", "fx", 32767.0, 32767, 32767.0),
    ("fx_nan_invalid", "fx", NAN, -32768, NAN), ("fx_inf_invalid", "fx", "inf", -32768, NAN),
    ("fx_neg_inf_invalid", "fx", "-inf", -32768, NAN), ("fx_huge_saturates", "fx", 1e30, 32767, 32767.0),
    ("fu_saturate_hi", "fu", 300.0, 254, 254.0), ("fu_negative_to_zero", "fu", -3.0, 0, 0.0),
    ("fu_nan_invalid", "fu", NAN, 255, NAN),
    ("fz_zero_saturates_to_one", "fz", 0.0, 1, 1.0), ("fz_nan_is_zero", "fz", NAN, 0, NAN),
    ("fz_saturate_hi", "fz", 70000.0, 65535, 65535.0),
    ("fi32_scale_half_tie", "fi32", 0.25, 1, 0.5), ("fi32_neg_tie", "fi32", -0.25, -1, -0.5),
    ("fi32_saturate", "fi32", 1e12, 2147483647, 2147483647 * 0.5),
]
for name, fld, inp, raw, dec in rules:
    vals = {**SINK_DEFAULT, fld: inp}
    args = dict(fx_raw=0, fu_raw=0, fz_raw=1, fi32_raw=0)
    args[fld + "_raw"] = raw
    dv = {**SINK_DEFAULT, fld: dec}
    case("rule_" + name, "KITCHEN_SINK", vals, sink_bytes(base=SINK_DEFAULT, **args), dv)

# ------------------------------------------------------------------ append-only evolution (OPTIONAL_TEST)
case("optional_full", "OPTIONAL_TEST", {"a": 1000, "opt_f": 2.5, "opt_u": 9, "opt_x": -0.5}, P("HhBf", 1000, 5, 9, -0.5))
decode_only.append({"name": "optional_older_sender_2_bytes", "packet": "OPTIONAL_TEST", "hex": hexs(P("H", 1000)),
                    "decoded": {"a": 1000, "opt_f": NAN, "opt_u": 7, "opt_x": NAN}})
decode_only.append({"name": "optional_partial_5_bytes", "packet": "OPTIONAL_TEST", "hex": hexs(P("HhB", 1000, 5, 9)),
                    "decoded": {"a": 1000, "opt_f": 2.5, "opt_u": 9, "opt_x": NAN}})
decode_only.append({"name": "optional_longer_sender_ignored", "packet": "OPTIONAL_TEST",
                    "hex": hexs(P("HhBf", 1000, 5, 9, -0.5) + b"\x01\x02\x03"),
                    "decoded": {"a": 1000, "opt_f": 2.5, "opt_u": 9, "opt_x": -0.5}})
decode_only.append({"name": "rc_state_longer_ignored", "packet": "RC_STATE",
                    "hex": hexs(P("BB16h", 1, 0, *([5] * 16)) + b"\xAA\xBB"),
                    "decoded": {"flags": 1, "reserved": 0, "ch": [5] * 16}})
decode_only.append({"name": "mode_text_8bit_replaced", "packet": "HANDSET_TELEM",
                    "hex": hexs(hand[:1] + b"A\xe9B\0\0\0\0\0" + hand[9:]),
                    "decoded": {**hv, "mode_text": "A?B", "ground_speed": 1234 * 0.01, "vertical_speed": -250 * 0.01,
                                "heading": 4567 * S_CDEG, "latitude": lat_raw * S_RAD7, "longitude": lon_raw * S_RAD7,
                                "pitch": -15708 * 1e-4, "roll": 1000 * 1e-4, "yaw": 31416 * 1e-4, "battery": 12600 * 0.001}})

# decode must fail
decode_fail += [
    {"name": "rc_state_short", "packet": "RC_STATE", "hex": hexs(P("BB15h", 1, 0, *([0] * 15)))},
    {"name": "rc_state_empty", "packet": "RC_STATE", "hex": ""},
    {"name": "request_ack_short", "packet": "REQUEST_ACK", "hex": "020100"},
    {"name": "log_text_empty_payload", "packet": "LOG_TEXT", "hex": ""},
    {"name": "log_text_too_long", "packet": "LOG_TEXT", "hex": "00" + "41" * 234},
    {"name": "heartbeat_short", "packet": "HEARTBEAT", "hex": "0101785634120102030000"[:20]},
    {"name": "cmd_state_short", "packet": "CMD_STATE", "hex": hexs(P("BBIBBBBBB", 3, 1, 1, 0, 0, 0, 0, 0, 0))},
    {"name": "cmd_state_empty", "packet": "CMD_STATE", "hex": ""},
]
# CMD_STATE out-of-range field cases: each one rejects the whole frame (valid frame: cs_safe)
_ok = [1, 1, 1, 0, 0, 0, 0, 0, 0, 0]
_bad = [("flags_valid_clear", 0, 0), ("flags_valid_clear_pocket_live", 0, 2), ("flags_reserved_bit3", 0, 9),
        ("flags_reserved_bit7", 0, 0x81), ("version_0", 1, 0), ("version_2", 1, 2), ("session_0", 2, 0),
        ("go_2", 3, 2), ("go_255", 3, 255), ("profile_3", 4, 3), ("profile_255", 4, 255),
        ("vehicle_mode_2", 5, 2), ("vehicle_mode_255", 5, 255), ("motors_2", 6, 2), ("motors_255", 6, 255),
        ("reset_button_2", 7, 2), ("failsafe_2", 8, 2), ("failsafe_255", 8, 255), ("reserved_1", 9, 1),
        ("reserved_0x8000", 9, 0x8000)]
for _n, _i, _v in _bad:
    _f = list(_ok)
    _f[_i] = _v
    decode_fail.append({"name": "cmd_state_bad_" + _n, "packet": "CMD_STATE", "hex": hexs(P("BBIBBBBBBH", *_f))})
decode_fail += [
    {"name": "optional_one_byte", "packet": "OPTIONAL_TEST", "hex": "01"},
]

out = {"version": 1,
       "spec": "docs/network-design.md 3.5 and 4.7/4.8 (workspace root), schema/*.toml",
       "notes": "values: null = NaN, \"inf\"/\"-inf\" = infinity (float fields only), bytes tails as {\"hex\": ...}. "
                "Floats in `values` are assigned to the packet and encoded; `hex` is the expected payload; "
                "`decoded` (default: values) is the expected result of decoding `hex` (floats compared with a "
                "relative tolerance of 1e-6 because C++ stores float32).",
       "cases": cases, "decode_only": decode_only, "decode_fail": decode_fail}


def main():
    path = Path(__file__).resolve().parent.parent / "test" / "vectors" / "packets.json"
    text = json.dumps(out, indent=1) + "\n"
    if "--check" in sys.argv:
        return 0 if path.read_text() == text else 1
    path.write_text(text)
    print("wrote %s (%d cases)" % (path, len(cases)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
