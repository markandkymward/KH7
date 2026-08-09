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

# correlation between d(roll_deg)/dt and gyro_roll_dps across the whole flight
d_roll = []
gyro = []
for i in range(1, len(records)):
    dt = (records[i]["time_ms"] - records[i-1]["time_ms"]) / 1000.0
    if dt <= 0 or dt > 0.5:
        continue
    droll = (records[i]["roll_deg_x10"] - records[i-1]["roll_deg_x10"]) / 10.0
    d_roll.append(droll / dt)
    gyro.append((records[i]["gyro_roll_dps_x10"] + records[i-1]["gyro_roll_dps_x10"]) / 20.0)

n = len(d_roll)
mean_d = sum(d_roll) / n
mean_g = sum(gyro) / n
cov = sum((d_roll[i]-mean_d)*(gyro[i]-mean_g) for i in range(n)) / n
sd_d = (sum((x-mean_d)**2 for x in d_roll) / n) ** 0.5
sd_g = (sum((x-mean_g)**2 for x in gyro) / n) ** 0.5
corr = cov / (sd_d * sd_g) if sd_d > 0 and sd_g > 0 else float("nan")
print(f"n={n} mean d(roll)/dt={mean_d:.2f} mean gyro_roll={mean_g:.2f} correlation={corr:.3f}")

t0 = records[0]["time_ms"]
print(f"{'t(s)':>6} {'tgt_roll':>9} {'act_roll':>9} {'gyro_r':>8}")
for i in range(0, len(records), 40):
    r = records[i]
    t = (r["time_ms"] - t0) / 1000.0
    print(f"{t:6.2f} {r['target_roll_deg_x10']/10.0:9.1f} {r['roll_deg_x10']/10.0:9.1f} {r['gyro_roll_dps_x10']/10.0:8.1f}")
