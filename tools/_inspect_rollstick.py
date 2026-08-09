import struct
import re
import sys

RECORD_STRUCT = struct.Struct("<I9hHHHHBBhhhhhhhh")
RECORD_SIZE = RECORD_STRUCT.size
FIELD_NAMES = (
    "time_ms",
    "setpoint_roll_dps", "setpoint_pitch_dps", "setpoint_yaw_dps",
    "gyro_roll_dps_x10", "gyro_pitch_dps_x10", "gyro_yaw_dps_x10",
    "pid_roll_us", "pid_pitch_us", "pid_yaw_us",
    "motor_fl_us", "motor_fr_us", "motor_rr_us", "motor_rl_us",
    "battery_decivolts", "flags",
    "pitch_deg_x10", "roll_deg_x10",
    "target_pitch_deg_x10", "target_roll_deg_x10",
    "baro_alt_cm", "baro_vz_cms",
    "throttle_cmd_us", "throttle_actual_us",
)

path = sys.argv[1] if len(sys.argv) > 1 else "tools/sdlog_raw_rollstick_check.txt"
line_re = re.compile(r"SDLOG\[\d+\]=([0-9a-fA-F]+)")

blocks = []
with open(path, "r") as f:
    for line in f:
        m = line_re.search(line)
        if m:
            blocks.append(bytes.fromhex(m.group(1)))

records = []
for block in blocks:
    for offset in range(0, len(block) - RECORD_SIZE + 1, RECORD_SIZE):
        chunk = block[offset:offset + RECORD_SIZE]
        if chunk == bytes(RECORD_SIZE):
            continue
        values = RECORD_STRUCT.unpack(chunk)
        records.append(dict(zip(FIELD_NAMES, values)))

records.sort(key=lambda r: r["time_ms"])
print(f"total records: {len(records)}")
if not records:
    sys.exit(0)

t0 = records[0]["time_ms"]
tail = [r for r in records if (r["time_ms"] - t0) / 1000.0 >= 11.0]
print(f"tail records (t>=11.0s): {len(tail)}")
print(f"{'t(s)':>6} {'sp_roll':>8} {'gyro_r':>8} {'tgt_roll':>9} {'act_roll':>9} {'FL':>5} {'FR':>5} {'RR':>5} {'RL':>5} {'thr_cmd':>7} {'thr_act':>7} {'mode':>4}")
for r in tail:
    t = (r["time_ms"] - t0) / 1000.0
    mode = (r["flags"] >> 1) & 0x03
    print(f"{t:6.2f} {r['setpoint_roll_dps']:8d} {r['gyro_roll_dps_x10']/10.0:8.1f} "
          f"{r['target_roll_deg_x10']/10.0:9.1f} {r['roll_deg_x10']/10.0:9.1f} "
          f"{r['motor_fl_us']:5d} {r['motor_fr_us']:5d} {r['motor_rr_us']:5d} {r['motor_rl_us']:5d} "
          f"{r['throttle_cmd_us']:7d} {r['throttle_actual_us']:7d} {mode:4d}")
