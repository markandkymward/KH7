import socket, time, re

s = socket.create_connection(('10.0.0.39', 3333), timeout=5.0)
s.settimeout(0.5)
buf = b''
end = time.time() + 6.0
rx16_re = re.compile(r'RX16\[link=(\d+) frames=(\d+) us\]=\[(.*?)\]')
imu_re = re.compile(r'IMU\[x100/x10\]=\[(.*?)\]')

last_rx = None
rows = []
start = time.time()
while time.time() < end:
    try:
        chunk = s.recv(4096)
    except socket.timeout:
        continue
    if not chunk:
        break
    buf += chunk
    while b'\r\n' in buf:
        line, buf = buf.split(b'\r\n', 1)
        line = line.decode(errors='replace')
        m = rx16_re.match(line)
        if m:
            vals = [int(x) for x in m.group(3).split()]
            last_rx = vals
            continue
        m = imu_re.match(line)
        if m and last_rx is not None:
            vals = [int(x) for x in m.group(1).split()]
            ax, ay, az, gx, gy, gz, p, r, y = vals
            roll_us, pitch_us, throttle_us, yaw_us = last_rx[0], last_rx[1], last_rx[2], last_rx[3]
            t = time.time() - start
            rows.append((t, roll_us, pitch_us, throttle_us, ax/100.0, ay/100.0, az/100.0, gx/10.0, gy/10.0, gz/10.0, p/10.0, r/10.0, y/10.0))

s.close()
header = "{:>6} {:>7} {:>8} {:>7} {:>6} {:>6} {:>6} {:>6} {:>6} {:>6} {:>7} {:>7} {:>8}".format(
    "t", "roll_us", "pitch_us", "thr_us", "ax_g", "ay_g", "az_g", "gx", "gy", "gz", "pitch", "roll", "yaw")
print(header)
for row in rows:
    t, roll_us, pitch_us, throttle_us, ax, ay, az, gx, gy, gz, p, r, y = row
    print("{:6.2f} {:7d} {:8d} {:7d} {:6.2f} {:6.2f} {:6.2f} {:6.1f} {:6.1f} {:6.1f} {:7.1f} {:7.1f} {:8.1f}".format(
        t, roll_us, pitch_us, throttle_us, ax, ay, az, gx, gy, gz, p, r, y))
print("total rows:", len(rows))
