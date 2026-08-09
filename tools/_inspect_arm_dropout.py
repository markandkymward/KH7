import re
import struct
import sys

RECORD_STRUCT = struct.Struct("<I9hHHHHBBhhhhhhhh")
RECORD_SIZE = RECORD_STRUCT.size

path = sys.argv[1] if len(sys.argv) > 1 else "sdlog_raw_arm_dropout_v2.txt"
text = open(path, "r", encoding="utf-8", errors="replace").read()

block_re = re.compile(r"SDLOG\[(\d+)\]=([0-9A-Fa-f]+)")
blocks = {}
for m in block_re.finditer(text):
    idx = int(m.group(1))
    hexstr = m.group(2)
    try:
        raw = bytes.fromhex(hexstr)
    except ValueError:
        continue
    blocks[idx] = raw

records = []
for idx in sorted(blocks.keys()):
    raw = blocks[idx]
    n = len(raw) // RECORD_SIZE
    for i in range(n):
        chunk = raw[i * RECORD_SIZE:(i + 1) * RECORD_SIZE]
        if len(chunk) < RECORD_SIZE:
            continue
        values = RECORD_STRUCT.unpack(chunk)
        records.append(values)

print(f"Total records parsed: {len(records)}")

# time_ms is field 0, flags is field 15 (0-indexed in the tuple)
prev_armed = None
prev_time = None
transitions = []
for r in records:
    time_ms = r[0]
    flags = r[15]
    armed = flags & 0x01
    if prev_armed is None or armed != prev_armed:
        transitions.append((time_ms, armed))
    prev_armed = armed
    prev_time = time_ms

print(f"\nArmed-flag transitions ({len(transitions)}):")
for i, (t, a) in enumerate(transitions):
    gap = ""
    if i > 0:
        gap = f"  (+{t - transitions[i-1][0]}ms since prev transition)"
    print(f"  t={t}ms  armed={a}{gap}")

print(f"\nLast record time_ms={records[-1][0] if records else 'N/A'}")
print(f"First record time_ms={records[0][0] if records else 'N/A'}")
