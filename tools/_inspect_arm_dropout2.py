import sys

sys.path.insert(0, r"C:\Projects\KH7\tools")
from sdlog_analyze import parse_records, segment_flights, FLIGHT_MODE_NAMES  # noqa: E402

path = sys.argv[1] if len(sys.argv) > 1 else "sdlog_raw_arm_dropout_v2.txt"
text = open(path, "r", encoding="utf-8", errors="replace").read()

records = parse_records(text)
flights = segment_flights(records)

print(f"\n{len(records)} total records, {len(flights)} time-gap-separated flight segment(s)\n")

for fi, (start, end) in enumerate(flights):
    sub = records[start:end]
    duration_s = (sub[-1]["time_ms"] - sub[0]["time_ms"]) / 1000.0
    print(f"=== Segment {fi}: {len(sub)} samples, {duration_s:.2f}s, "
          f"t={sub[0]['time_ms']}..{sub[-1]['time_ms']}ms ===")

    prev_armed = None
    for r in sub:
        armed = r["flags"] & 0x01
        if prev_armed is None or armed != prev_armed:
            mode = FLIGHT_MODE_NAMES.get((r["flags"] >> 1) & 0x03, "?")
            link = 1 if (r["flags"] & 0x08) else 0
            print(f"    t={r['time_ms']}ms  armed={armed}  mode={mode}  link_active={link}  "
                  f"arm_us={r['arm_us']}  "
                  f"motors=[{r['motor_fl_us']} {r['motor_fr_us']} {r['motor_rr_us']} {r['motor_rl_us']}]")
        prev_armed = armed

    print(f"    last 5 records before segment end:")
    for r in sub[-5:]:
        link = 1 if (r["flags"] & 0x08) else 0
        print(f"      t={r['time_ms']}ms  arm_us={r['arm_us']}  link_active={link}")

# Also report time gaps BETWEEN segments (these are the real disarm/rearm gaps)
print("\nGaps between segments (these represent real disarm time or dump/logging pauses):")
for fi in range(1, len(flights)):
    prev_end = records[flights[fi - 1][1] - 1]["time_ms"]
    this_start = records[flights[fi][0]]["time_ms"]
    print(f"  segment {fi-1} end t={prev_end}ms -> segment {fi} start t={this_start}ms  "
          f"(gap={this_start - prev_end}ms)")
