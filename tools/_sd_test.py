import re
import sys
import time

import serial

ser = serial.Serial('COM6', 115200, timeout=0.2, write_timeout=1.0)
ser.reset_input_buffer()


def cmd(c, window_s=1.5):
    # Bounded-duration read: the board may stream telemetry continuously, so
    # "read until idle" can loop forever. Just collect for a fixed window.
    print(f'--- {c} (writing) ---', flush=True)
    ser.write((c + '\r\n').encode())
    print(f'--- {c} (write done, reading) ---', flush=True)
    end = time.time() + window_s
    buf = b''
    while time.time() < end:
        n = ser.in_waiting
        if n:
            buf += ser.read(n)
        else:
            time.sleep(0.02)
    text = buf.decode(errors='replace')
    lines = [ln for ln in re.split(r'\r\n', text) if ln.startswith('SD_')]
    print(f'--- {c} result ---', flush=True)
    print('\n'.join(lines) if lines else '(no SD_* response found; raw bytes: %d)' % len(buf), flush=True)


cmd('PID GET', window_s=1.0)
cmd('SD INIT', window_s=2.0)
cmd('SD STATUS', window_s=1.0)
cmd('SD WBLOCK 100', window_s=1.5)
cmd('SD RBLOCK 100', window_s=1.5)
ser.close()
