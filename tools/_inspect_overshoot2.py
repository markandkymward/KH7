import sys

sys.path.insert(0, r"C:\Projects\KH7\tools")
from sdlog_analyze import parse_records, segment_flights, _flight_arrays
import numpy as np

path = sys.argv[1] if len(sys.argv) > 1 else "sdlog_raw_last_verify.txt"
text = open(path, "r", encoding="utf-8", errors="replace").read()

records = parse_records(text)
flights = segment_flights(records)
start, end = max(flights, key=lambda se: se[1] - se[0])
sub = records[start:end]
a = _flight_arrays(sub)

for label, cmd_key, act_key in (("roll", "target_roll_deg", "roll_deg"),
                                 ("pitch", "target_pitch_deg", "pitch_deg")):
    cmd = a[cmd_key]
    act = a[act_key]
    err = act - cmd  # positive = actual ahead of / overshooting commanded

    # drop single-sample dropout glitches (act snaps toward 0 for exactly one sample)
    glitch = (np.abs(act) < 1.0) & (np.abs(cmd) > 5.0)
    clean_err = err[~glitch]

    # "overshoot" = actual moving PAST commanded in the direction of travel, i.e. the
    # rate of change of cmd and the sign of err agree with err growing beyond a margin
    # right after cmd changes direction. Simplify: fraction of samples where |err|>3deg
    # AND actual is farther from 0 than commanded in the same direction (true overshoot,
    # not lag).
    same_dir_overshoot = (np.sign(act) == np.sign(cmd)) & (np.abs(act) > np.abs(cmd) + 2.0)
    frac_overshoot = np.mean(same_dir_overshoot[~glitch]) * 100

    print(f"{label}: err rms={np.sqrt(np.mean(clean_err**2)):.2f} "
          f"peak(+)={clean_err.max():.2f} peak(-)={clean_err.min():.2f} "
          f"mean={clean_err.mean():.2f}")
    print(f"  overshoot (actual beyond commanded by >2deg, same direction): {frac_overshoot:.1f}% of samples")
