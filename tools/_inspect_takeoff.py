import struct, re

RECORD_STRUCT = struct.Struct('<I9hHHHHBBhhhhhh')
BLOCK_RE = re.compile(r'SDLOG\[(\d+)\]=([0-9A-Fa-f]+)')

with open('sdlog_raw_takeoff_investigate.txt', 'r') as f:
    text = f.read()

blocks = {}
for m in BLOCK_RE.finditer(text):
    idx = int(m.group(1))
    blocks[idx] = bytes.fromhex(m.group(2))

data = b''.join(blocks[i] for i in sorted(blocks.keys()))
records = []
for off in range(0, len(data) - RECORD_STRUCT.size + 1, RECORD_STRUCT.size):
    rec = RECORD_STRUCT.unpack_from(data, off)
    records.append(rec)

t0 = records[0][0]
print('total records', len(records))
print('first time_ms', t0)

for rec in records:
    t_s = (rec[0] - t0) / 1000.0
    if 6.5 <= t_s <= 12.0:
        (time_ms, sp_roll, sp_pitch, sp_yaw, gr, gp, gy, pid_r, pid_p, pid_y,
         m_fl, m_fr, m_rr, m_rl, batt_dv, flags, pitch_x10, roll_x10,
         tgt_pitch_x10, tgt_roll_x10, baro_alt_cm, baro_vz_cms) = rec
        print(f't={t_s:6.3f}s sp_r={sp_roll:4d} sp_p={sp_pitch:4d} sp_y={sp_yaw:4d} '
              f'gyro_r={gr/10:6.1f} gyro_p={gp/10:6.1f} gyro_y={gy/10:6.1f} '
              f'FL={m_fl} FR={m_fr} RR={m_rr} RL={m_rl} '
              f'roll_deg={roll_x10/10:5.1f} pitch_deg={pitch_x10/10:5.1f} '
              f'tgt_r={tgt_roll_x10/10:5.1f} tgt_p={tgt_pitch_x10/10:5.1f} '
              f'alt_cm={baro_alt_cm} vz_cms={baro_vz_cms}')
