import struct, re, sys

RECORD_STRUCT = struct.Struct('<I9hHHHHBBhhhhhh')
BLOCK_RE = re.compile(r'SDLOG\[(\d+)\]=([0-9A-Fa-f]+)')

fname = sys.argv[1] if len(sys.argv) > 1 else 'sdlog_raw_posfix_check.txt'

with open(fname, 'r') as f:
    text = f.read()

blocks = {}
bad = []
for m in BLOCK_RE.finditer(text):
    idx = int(m.group(1))
    try:
        b = bytes.fromhex(m.group(2))
    except ValueError:
        bad.append(idx)
        continue
    if len(b) != 512:
        bad.append(idx)
        continue
    blocks[idx] = b
print('bad hex blocks skipped:', bad)

idxs = sorted(blocks.keys())
print('num blocks', len(idxs), 'range', idxs[0], '-', idxs[-1])
missing = [i for i in range(idxs[0], idxs[-1] + 1) if i not in blocks]
print('missing block indices:', missing)

# NOTE: records do NOT pack contiguously across 512-byte block boundaries -
# each block independently holds floor(512/44)=11 records + 28 leftover/padding
# bytes that are discarded (matches sdlog_analyze.py's parse_records()).
records = []
for idx in idxs:
    data = blocks[idx]
    for off in range(0, len(data) - RECORD_STRUCT.size + 1, RECORD_STRUCT.size):
        chunk = data[off:off + RECORD_STRUCT.size]
        if chunk == bytes(RECORD_STRUCT.size):
            continue
        records.append(RECORD_STRUCT.unpack(chunk))

t0 = records[0][0]
print('total records', len(records), 'first time_ms', t0)

print('\n--- gaps > 100ms between consecutive records (sane t only) ---')
prev_t = records[0][0]
for rec in records[1:]:
    t = rec[0]
    dt = t - prev_t
    t_s = (prev_t - t0) / 1000.0
    t2_s = (t - t0) / 1000.0
    if dt > 100 and -5 <= t_s <= 60 and -5 <= t2_s <= 60:
        print(f'gap {dt}ms  at t={t_s:.3f}s -> {t2_s:.3f}s')
    prev_t = t

print('\n--- records t=4.0..9.0s ---')
for rec in records:
    t_s = (rec[0] - t0) / 1000.0
    if 4.0 <= t_s <= 9.0:
        (time_ms, sp_roll, sp_pitch, sp_yaw, gr, gp, gy, pid_r, pid_p, pid_y,
         m_fl, m_fr, m_rr, m_rl, batt_dv, flags, pitch_x10, roll_x10,
         tgt_pitch_x10, tgt_roll_x10, baro_alt_cm, baro_vz_cms) = rec
        print(f't={t_s:6.3f}s sp_r={sp_roll:4d} sp_p={sp_pitch:4d} sp_y={sp_yaw:4d} '
              f'gyro_r={gr/10:6.1f} gyro_p={gp/10:6.1f} gyro_y={gy/10:6.1f} '
              f'FL={m_fl} FR={m_fr} RR={m_rr} RL={m_rl} '
              f'roll_deg={roll_x10/10:5.1f} pitch_deg={pitch_x10/10:5.1f} '
              f'tgt_r={tgt_roll_x10/10:5.1f} tgt_p={tgt_pitch_x10/10:5.1f} '
              f'alt_cm={baro_alt_cm} vz_cms={baro_vz_cms} flags={flags}')
