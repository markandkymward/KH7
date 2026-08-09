import sys

sys.path.insert(0, r"C:\Projects\KH7\tools")
from sdlog_analyze import parse_records, segment_flights, FLIGHT_MODE_NAMES  # noqa: E402
import numpy as np

path = sys.argv[1] if len(sys.argv) > 1 else "sdlog_raw_relocated_check.txt"
text = open(path, "r", encoding="utf-8", errors="replace").read()

records = parse_records(text)
flights = segment_flights(records)

start, end = max(flights, key=lambda se: se[1] - se[0])
sub = records[start:end]

t0 = sub[0]["time_ms"]
mode_bits = np.array([(r["flags"] >> 1) & 0x03 for r in sub])
althold_mask = mode_bits == 2

print(f"Flight: {len(sub)} samples, {(sub[-1]['time_ms']-sub[0]['time_ms'])/1000.0:.1f}s total")
print(f"ALTHOLD samples: {int(althold_mask.sum())}/{len(sub)}")

if not np.any(althold_mask):
    print("No ALTHOLD-mode samples found in this flight.")
    sys.exit(0)

t_s = np.array([(r["time_ms"] - t0) / 1000.0 for r in sub])
baro_alt_m = np.array([r["baro_alt_cm"] for r in sub], dtype=float) / 100.0
baro_vz_mps = np.array([r["baro_vz_cms"] for r in sub], dtype=float) / 100.0
throttle_cmd_us = np.array([r["throttle_cmd_us"] for r in sub], dtype=float)
throttle_actual_us = np.array([r["throttle_actual_us"] for r in sub], dtype=float)

a_alt = baro_alt_m[althold_mask]
a_vz = baro_vz_mps[althold_mask]
a_cmd = throttle_cmd_us[althold_mask]
a_act = throttle_actual_us[althold_mask]
a_t = t_s[althold_mask]

print(f"\nALTHOLD window: t={a_t[0]:.2f}s to t={a_t[-1]:.2f}s ({a_t[-1]-a_t[0]:.2f}s duration)")
print(f"baro_alt_m: mean={a_alt.mean():.2f} std={a_alt.std():.2f} min={a_alt.min():.2f} max={a_alt.max():.2f} (range={a_alt.max()-a_alt.min():.2f}m)")
print(f"baro_vz_mps: mean={a_vz.mean():.2f} std={a_vz.std():.2f} min={a_vz.min():.2f} max={a_vz.max():.2f}")
print(f"throttle_cmd_us (stick): mean={a_cmd.mean():.0f} std={a_cmd.std():.1f} min={a_cmd.min():.0f} max={a_cmd.max():.0f}")
print(f"throttle_actual_us (mixer output): mean={a_act.mean():.0f} std={a_act.std():.1f} min={a_act.min():.0f} max={a_act.max():.0f}")
trim = a_act - a_cmd
print(f"ALTHOLD trim (actual - cmd): mean={trim.mean():.1f} std={trim.std():.1f} min={trim.min():.1f} max={trim.max():.1f}")

# Per-second breakdown to see where the variability actually is (e.g. ground effect at start vs throughout)
print(f"\n{'t_s':>6s} {'alt_m':>7s} {'vz_mps':>7s} {'cmd_us':>7s} {'act_us':>7s} {'trim_us':>8s}")
bucket_s = 1.0
t_end = a_t[-1]
tb = a_t[0]
while tb < t_end:
    mask = (a_t >= tb) & (a_t < tb + bucket_s)
    if np.any(mask):
        print(f"{tb:6.1f} {a_alt[mask].mean():7.2f} {a_vz[mask].std():7.2f} "
              f"{a_cmd[mask].mean():7.0f} {a_act[mask].mean():7.0f} {trim[mask].mean():8.1f}")
    tb += bucket_s
