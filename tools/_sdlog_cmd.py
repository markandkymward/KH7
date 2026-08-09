import socket
import sys
import time

host = sys.argv[1] if len(sys.argv) > 1 else "10.0.0.39"
port = int(sys.argv[2]) if len(sys.argv) > 2 else 3333
cmd = sys.argv[3] if len(sys.argv) > 3 else "SDLOG STATUS"
duration = float(sys.argv[4]) if len(sys.argv) > 4 else 8.0

print("connecting...")
s = socket.create_connection((host, port), timeout=5)
print("connected, sending command:", cmd)
s.sendall((cmd + "\r\n").encode("ascii"))
s.settimeout(0.5)
end = time.time() + duration
buf = b""
while time.time() < end:
    try:
        chunk = s.recv(4096)
        if not chunk:
            print("connection closed by peer")
            break
        buf += chunk
    except socket.timeout:
        continue
s.close()
print("TOTAL BYTES:", len(buf))
print(buf.decode(errors="replace"))
