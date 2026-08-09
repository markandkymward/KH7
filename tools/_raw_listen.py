import socket
import sys
import time

host = sys.argv[1] if len(sys.argv) > 1 else "10.0.0.39"
port = int(sys.argv[2]) if len(sys.argv) > 2 else 3333
duration = float(sys.argv[3]) if len(sys.argv) > 3 else 6.0

s = socket.create_connection((host, port), timeout=5)
s.settimeout(0.5)
end = time.time() + duration
buf = b""
while time.time() < end:
    try:
        chunk = s.recv(4096)
        if not chunk:
            break
        buf += chunk
    except socket.timeout:
        continue
s.close()
print("TOTAL BYTES:", len(buf))
print(buf[:2000].decode(errors="replace"))
