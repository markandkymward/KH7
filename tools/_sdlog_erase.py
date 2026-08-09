import socket
import sys
import time

host = sys.argv[1] if len(sys.argv) > 1 else "10.0.0.39"
port = int(sys.argv[2]) if len(sys.argv) > 2 else 3333
cmd = sys.argv[3] if len(sys.argv) > 3 else "SDLOG ERASE"

s = socket.create_connection((host, port), timeout=5)
s.settimeout(3.0)
s.sendall((cmd + "\r\n").encode("ascii"))
time.sleep(1.0)
buf = b""
try:
    while True:
        chunk = s.recv(4096)
        if not chunk:
            break
        buf += chunk
except socket.timeout:
    pass
s.close()
print(buf.decode(errors="replace"))
