import sys

sys.path.insert(0, r"C:\Projects\KH7\tools")
from sdlog_analyze import parse_records, segment_flights
import numpy as np

path = sys.argv[1] if len(sys.argv) > 1 else "sdlog_raw_last_verify.txt"
text = open(path, "r", encoding="utf-8", errors="replace").read()

records = parse_records(text)
flights = segment_flights(records)
start, end = max(flights, key=lambda se: se[1] - se[0])
sub = records[start:end]

mode_bits = np.array([(r["flags"] >> 1) & 0x03 for r in sub])
althold_mask = mode_bits == 2

t0 = sub[0]["time_ms"]
t_s = np.array([(r["time_ms"] - t0) / 1000.0 for r in sub])
cmd = np.array([r["throttle_cmd_us"] for r in sub], dtype=float)
act = np.array([r["throttle_actual_us"] for r in sub], dtype=float)
trim = act - cmd

a_t = t_s[althold_mask]
a_trim = trim[althold_mask]

d_trim = np.diff(a_trim)
print(f"ALTHOLD samples: {len(a_trim)}")
print(f"Sample-to-sample trim delta: rms={np.sqrt(np.mean(d_trim**2)):.2f}us std={d_trim.std():.2f}us "
      f"max_abs={np.max(np.abs(d_trim)):.1f}us")
print(f"Fraction of steps with |delta| > 10us: {np.mean(np.abs(d_trim) > 10) * 100:.1f}%")
print(f"Fraction of steps with |delta| > 20us: {np.mean(np.abs(d_trim) > 20) * 100:.1f}%")

# Steady hover window (t=33-42s: cmd flat, holding)
mask = (a_t >= 33.0) & (a_t <= 42.0)
if np.any(mask):
    steady_trim = a_trim[mask]
    d_steady = np.diff(steady_trim)
    print(f"\nSteady window (33-42s), {len(steady_trim)} samples:")
    print(f"  trim: mean={steady_trim.mean():.2f} std={steady_trim.std():.2f}")
    print(f"  delta: rms={np.sqrt(np.mean(d_steady**2)):.2f}us max_abs={np.max(np.abs(d_steady)):.1f}us")
