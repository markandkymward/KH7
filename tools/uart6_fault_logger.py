#!/usr/bin/env python3
"""Continuously capture every UART6 telemetry line to a timestamped log file.

Intended for real-flight testing: leave this running (over the WiFi bridge or
USB) so that if the board ever hangs/reboots in the air, the last lines
transmitted before the reset - including the new FAULT[...]/RESET_CAUSE[...]
diagnostics added to stm32h7xx_it.c/main.c - are captured to disk even if
nobody is watching the screen at that exact moment. Auto-reconnects on
disconnect (a dropped link right as a reboot happens is itself useful
evidence, and is logged as a marker line).

Usage:
    python tools/uart6_fault_logger.py                       # WiFi bridge, default host
    python tools/uart6_fault_logger.py --host 10.0.0.39       # WiFi bridge, explicit IP
    python tools/uart6_fault_logger.py --usb --port COM6      # USB serial instead

Every line is written to the log file (flushed immediately) and echoed to
stdout, each prefixed with a wall-clock timestamp.
"""
import argparse
import datetime
import socket
import sys
import time

DEFAULT_HOST = "kh7bridge.local"
DEFAULT_TCP_PORT = 3333
DEFAULT_BAUD = 115200
RECONNECT_DELAY_S = 1.0


def _timestamp() -> str:
    return datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def _open_logfile(path: str):
    f = open(path, "a", encoding="utf-8", newline="")
    f.write(f"[{_timestamp()}] ==== logger started, writing to {path} ====\n")
    f.flush()
    return f


def _emit(logfile, line: str) -> None:
    stamped = f"[{_timestamp()}] {line}"
    logfile.write(stamped + "\n")
    logfile.flush()
    print(stamped)


def _run_wifi(host: str, tcp_port: int, logfile) -> None:
    buf = bytearray()
    while True:
        try:
            _emit(logfile, f"=== connecting to {host}:{tcp_port} ===")
            sock = socket.create_connection((host, tcp_port), timeout=5.0)
            sock.settimeout(1.0)
            _emit(logfile, "=== connected ===")
            buf.clear()
            while True:
                try:
                    chunk = sock.recv(4096)
                except socket.timeout:
                    continue
                if not chunk:
                    _emit(logfile, "=== remote closed connection ===")
                    break
                buf.extend(chunk)
                while True:
                    idx = buf.find(b"\n")
                    if idx < 0:
                        break
                    raw_line = bytes(buf[:idx + 1])
                    del buf[:idx + 1]
                    text = raw_line.decode("ascii", errors="replace").rstrip("\r\n")
                    if text:
                        _emit(logfile, text)
        except OSError as exc:
            _emit(logfile, f"=== connection error: {exc} ===")
        finally:
            try:
                sock.close()
            except Exception:
                pass
        time.sleep(RECONNECT_DELAY_S)


def _run_usb(port: str, baud: int, logfile) -> None:
    import serial
    from serial import SerialException

    while True:
        try:
            _emit(logfile, f"=== opening {port} @ {baud} ===")
            ser = serial.Serial(port, baud, timeout=1.0)
            _emit(logfile, "=== connected ===")
            while True:
                raw = ser.readline()
                if not raw:
                    continue
                text = raw.decode("ascii", errors="replace").rstrip("\r\n")
                if text:
                    _emit(logfile, text)
        except (SerialException, OSError) as exc:
            _emit(logfile, f"=== connection error: {exc} ===")
        finally:
            try:
                ser.close()
            except Exception:
                pass
        time.sleep(RECONNECT_DELAY_S)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default=DEFAULT_HOST, help="WiFi bridge hostname/IP (default: %(default)s)")
    parser.add_argument("--tcp-port", type=int, default=DEFAULT_TCP_PORT, help="WiFi bridge TCP port (default: %(default)s)")
    parser.add_argument("--usb", action="store_true", help="Use USB serial instead of the WiFi bridge")
    parser.add_argument("--port", default="COM6", help="Serial port when --usb is used (default: %(default)s)")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Serial baud rate when --usb is used (default: %(default)s)")
    parser.add_argument("--logfile", default=None, help="Log file path (default: uart6_log_<timestamp>.txt in cwd)")
    args = parser.parse_args()

    logfile_path = args.logfile or f"uart6_log_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
    logfile = _open_logfile(logfile_path)

    try:
        if args.usb:
            _run_usb(args.port, args.baud, logfile)
        else:
            _run_wifi(args.host, args.tcp_port, logfile)
    except KeyboardInterrupt:
        _emit(logfile, "=== stopped by user ===")
    finally:
        logfile.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
