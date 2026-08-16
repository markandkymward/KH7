import argparse
import re
import socket
import time


RX_RE = re.compile(
    rb"RX16\[link=(\d+) frames=(\d+) age=(\d+) crc=(\d+) sync=(\d+) "
    rb"ore=(\d+) fe=(\d+) ne=(\d+) us\]=\[([^]]+)\]"
)
ARM_RE = re.compile(
    rb"ARM\[a=(\d+) sw=(\d+) lowSeen=(\d+) thr=(\d+) m\]=\[([^]]+)\]"
)
ARM_EVENT_RE = re.compile(rb"ARM_EVENT\[([^]]+)\]")
BRIDGE_STAT_RE = re.compile(rb"\[BRIDGE\] STAT[^\r\n]*")


def capture(host: str, port: int, duration_s: float) -> None:
    rows = []
    arm_samples = []
    arm_events = []
    bridge_stats = []
    malformed_rx16_count = 0
    buffer = bytearray()
    started = time.monotonic()

    with socket.create_connection((host, port), timeout=3.0) as connection:
        connection.settimeout(0.5)
        while time.monotonic() - started < duration_s:
            try:
                chunk = connection.recv(4096)
            except socket.timeout:
                continue
            if not chunk:
                break

            buffer.extend(chunk)
            while True:
                newline_index = buffer.find(b"\n")
                if newline_index < 0:
                    break
                line = bytes(buffer[:newline_index])
                del buffer[:newline_index + 1]
                elapsed_s = time.monotonic() - started

                match = RX_RE.search(line)
                if match is not None:
                    fields = tuple(map(int, match.groups()[:8]))
                    try:
                        channels = tuple(map(int, match.group(9).split()))
                    except ValueError:
                        malformed_rx16_count += 1
                    else:
                        if len(channels) == 16:
                            rows.append((elapsed_s, *fields, channels))
                        else:
                            malformed_rx16_count += 1

                match = ARM_RE.search(line)
                if match is not None:
                    arm_samples.append((elapsed_s, *map(int, match.groups()[:4])))

                match = ARM_EVENT_RE.search(line)
                if match is not None:
                    arm_events.append((elapsed_s, match.group(1).decode(errors="replace")))

                match = BRIDGE_STAT_RE.search(line)
                if match is not None:
                    bridge_stats.append((elapsed_s, match.group(0).decode(errors="replace")))

    print(f"RX16 samples: {len(rows)}")
    print(f"Malformed/interleaved RX16 lines: {malformed_rx16_count}")
    print(f"ARM samples: {len(arm_samples)}")
    print(f"ARM events: {arm_events}")
    if bridge_stats:
        print(f"Bridge first: {bridge_stats[0][1]}")
        print(f"Bridge last:  {bridge_stats[-1][1]}")
    if not rows:
        print("No RX16 telemetry received")
        return

    zero_rows = [row for row in rows if row[1] == 0]
    print(f"Capture duration: {rows[-1][0]:.1f} s")
    print(f"Link-zero samples: {len(zero_rows)}")
    print(f"Maximum frame age: {max(row[3] for row in rows)} ms")
    print(f"First link/frame/age/counters: {rows[0][1:9]}")
    print(f"Last link/frame/age/counters:  {rows[-1][1:9]}")
    print(
        "Counter deltas CRC/sync/ORE/FE/NE: "
        f"{tuple(rows[-1][index] - rows[0][index] for index in range(4, 9))}"
    )
    print(f"ARM states: {sorted(set(sample[1:] for sample in arm_samples))}")
    for row in zero_rows[:20]:
        print(
            f"LINK_ZERO t={row[0]:.3f}s frame={row[2]} "
            f"age={row[3]}ms arm_ch={row[9][4]}us"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="Passively summarize KH7 receiver telemetry.")
    parser.add_argument("--host", default="kh7bridge.local")
    parser.add_argument("--port", type=int, default=3333)
    parser.add_argument("--duration", type=float, default=60.0)
    args = parser.parse_args()
    capture(args.host, args.port, args.duration)


if __name__ == "__main__":
    main()