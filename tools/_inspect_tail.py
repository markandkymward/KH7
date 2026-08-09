import sys
sys.path.insert(0, r"c:\Projects\KH7\tools")
from sdlog_analyze import parse_records, segment_flights, FIELD_NAMES, FLIGHT_MODE_NAMES

with open(r"c:\Projects\KH7\tools\sdlog_raw_20260802_181729.txt", "r") as f:
    text = f.read()

records = parse_records(text)
flights = segment_flights(records)
print("flights:", [(s, e, records[e-1]["time_ms"] - records[s]["time_ms"]) for s, e in flights])

start, end = flights[0]
sub = records[start:end]
t0 = sub[0]["time_ms"]

print("\nfull (unclipped) flight 0, {} samples\n".format(len(sub)))
header = "{:>7} {:>6} {:>7} {:>7} {:>7} {:>7} {:>7} {:>7} {:>6} {:>6} {:>6} {:>6} {:>6} {:>6} {:>5} {:>4} {:>6}".format(
    "t_ms", "dt", "sp_r", "sp_p", "sp_y", "gyr_r", "gyr_p", "gyr_y", "fl", "fr", "rr", "rl", "pid_r", "pid_p", "vbat", "armd", "mode")
print(header)
prev_t = None
for r in sub:
    t = r["time_ms"] - t0
    dt = (r["time_ms"] - prev_t) if prev_t is not None else 0
    prev_t = r["time_ms"]
    armed = r["flags"] & 0x01
    mode = FLIGHT_MODE_NAMES.get((r["flags"] >> 1) & 0x03, "?")
    print("{:7d} {:6d} {:7d} {:7d} {:7d} {:7.1f} {:7.1f} {:7.1f} {:6d} {:6d} {:6d} {:6d} {:6d} {:6d} {:5.1f} {:4d} {:>6}".format(
        t, dt, r["setpoint_roll_dps"], r["setpoint_pitch_dps"], r["setpoint_yaw_dps"],
        r["gyro_roll_dps_x10"]/10.0, r["gyro_pitch_dps_x10"]/10.0, r["gyro_yaw_dps_x10"]/10.0,
        r["motor_fl_us"], r["motor_fr_us"], r["motor_rr_us"], r["motor_rl_us"],
        r["pid_roll_us"], r["pid_pitch_us"], r["battery_decivolts"]/10.0, armed, mode))
