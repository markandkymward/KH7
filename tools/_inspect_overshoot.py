import sys

sys.path.insert(0, r"C:\Projects\KH7\tools")
from sdlog_analyze import parse_records, segment_flights, _flight_arrays
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

path = sys.argv[1] if len(sys.argv) > 1 else "sdlog_raw_last_verify.txt"
text = open(path, "r", encoding="utf-8", errors="replace").read()

records = parse_records(text)
flights = segment_flights(records)
start, end = max(flights, key=lambda se: se[1] - se[0])
sub = records[start:end]
a = _flight_arrays(sub)

t = a["t_s"]

fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
axes[0].plot(t, a["target_roll_deg"], label="commanded", linewidth=1.0)
axes[0].plot(t, a["roll_deg"], label="actual", linewidth=1.0)
axes[0].set_ylabel("roll deg")
axes[0].legend()
axes[0].grid(True, alpha=0.3)

axes[1].plot(t, a["target_pitch_deg"], label="commanded", linewidth=1.0)
axes[1].plot(t, a["pitch_deg"], label="actual", linewidth=1.0)
axes[1].set_ylabel("pitch deg")
axes[1].set_xlabel("time (s)")
axes[1].legend()
axes[1].grid(True, alpha=0.3)

plt.tight_layout()
out = r"C:\Projects\KH7\tools\_zoom_angle_full.png"
plt.savefig(out, dpi=110)
print("saved", out)

# Compute overshoot metric: after each commanded step change, does actual cross beyond
# the NEW commanded value and keep going past it (classic overshoot), vs just lag behind?
err = a["target_roll_deg"] - a["roll_deg"]
print(f"roll error: rms={np.sqrt(np.mean(err**2)):.2f} peak={np.max(np.abs(err)):.2f} "
      f"peak_idx_t={t[np.argmax(np.abs(err))]:.2f}s")
err_p = a["target_pitch_deg"] - a["pitch_deg"]
print(f"pitch error: rms={np.sqrt(np.mean(err_p**2)):.2f} peak={np.max(np.abs(err_p)):.2f} "
      f"peak_idx_t={t[np.argmax(np.abs(err_p))]:.2f}s")

# zoom windows around peak error times
for label, tc in (("roll", t[np.argmax(np.abs(err))]), ("pitch", t[np.argmax(np.abs(err_p))])):
    mask = (t >= tc - 3) & (t <= tc + 3)
    fig2, ax = plt.subplots(figsize=(10, 4))
    key_cmd = "target_roll_deg" if label == "roll" else "target_pitch_deg"
    key_act = "roll_deg" if label == "roll" else "pitch_deg"
    ax.plot(t[mask], a[key_cmd][mask], label="commanded", marker='.', linewidth=1.2)
    ax.plot(t[mask], a[key_act][mask], label="actual", marker='.', linewidth=1.2)
    ax.set_title(f"{label} zoom around peak err t={tc:.2f}s")
    ax.legend()
    ax.grid(True, alpha=0.3)
    outz = rf"C:\Projects\KH7\tools\_zoom_{label}_peak.png"
    plt.tight_layout()
    plt.savefig(outz, dpi=110)
    print("saved", outz)
