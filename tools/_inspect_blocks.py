import struct, re

RECORD_STRUCT = struct.Struct('<I9hHHHHBBhhhhhh')
BLOCK_RE = re.compile(r'SDLOG\[(\d+)\]=([0-9A-Fa-f]+)')

with open('sdlog_raw_takeoff_investigate.txt', 'r') as f:
    text = f.read()

blocks = {}
for m in BLOCK_RE.finditer(text):
    idx = int(m.group(1))
    blocks[idx] = bytes.fromhex(m.group(2))

idxs = sorted(blocks.keys())
print('num blocks', len(idxs), 'range', idxs[0], '-', idxs[-1])
missing = [i for i in range(idxs[0], idxs[-1] + 1) if i not in blocks]
print('missing block indices:', missing)
for i in idxs:
    print(i, len(blocks[i]))
