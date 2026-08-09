import sys

sys.path.insert(0, r"C:\Projects\KH7\tools")
from sdlog_analyze import parse_records, segment_flights  # noqa: E402
import numpy as np

path = sys.argv[1] if len(sys.argv) > 1 else "sdlog_raw_check2.txt"
text = open(path, "r", encoding="utf-8", errors="replace").read()

records = parse_records(text)
flights = segment_flights(records)

# Use the longest flight segment
start, end = max(flights, key=lambda se: se[1] - se[0])
sub = records[start:end]

t0 = sub[0]["time_ms"]
print(f"{'t_s':>6s} {'mag_hdg':>8s} {'yaw_deg':>8s} {'throttle_us':>11s} {'mfl':>5s} {'mfr':>5s} {'mrr':>5s} {'mrl':>5s}")
for i, r in enumerate(sub):
    if i % 20 != 0:
        continue
    t_s = (r["time_ms"] - t0) / 1000.0
    mag_hdg = r["mag_heading_x10"] / 10.0
    yaw_deg = r["yaw_deg_x10"] / 10.0
    print(f"{t_s:6.2f} {mag_hdg:8.1f} {yaw_deg:8.1f} {r['throttle_actual_us']:11d} "
          f"{r['motor_fl_us']:5d} {r['motor_fr_us']:5d} {r['motor_rr_us']:5d} {r['motor_rl_us']:5d}")
