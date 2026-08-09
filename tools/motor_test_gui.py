import json
import queue
import re
import socket
import struct
import threading
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import serial
import serial.tools.list_ports
from serial import SerialException


ARM_LINE_RE = re.compile(
    r"ARM\[a=(\d+) sw=(\d+) lowSeen=(\d+) thr=(\d+) m\]=\[(\d+) (\d+) (\d+) (\d+)\]"
)
RX_LINE_RE = re.compile(r"RX\[link=(\d+) frames=(\d+) us\]=\[(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\]")
RX16_LINE_RE = re.compile(
    r"RX16\[link=(\d+) frames=(\d+) us\]=\["
    r"(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+"
    r"(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\]"
)
IMU_LINE_RE = re.compile(
    r"IMU\[x100/x10\]=\[\s*(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s*\]"
)
VBAT_LINE_RE = re.compile(r"VBAT\[mV raw\]=\[\s*(\d+)\s+(\d+)\s*\]")
BARO_LINE_RE = re.compile(r"BARO\[healthy cm cm_s\]=\[\s*(\d+)\s+(-?\d+)\s+(-?\d+)\s*\]")
BARO_INIT_LINE_RE = re.compile(r"BARO_INIT\[(OK) addr=0x([0-9A-Fa-f]{2})\]|BARO_INIT\[(FAIL) chip_id=0x([0-9A-Fa-f]{2})\]")
GPS_LINE_RE = re.compile(
    r"GPS\[cfg healthy fix sats\]=\[\s*(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s*\]\s+"
    r"lla=\[\s*(-?\d*\.?\d+)\s+(-?\d*\.?\d+)\s+(-?\d*\.?\d+)\s*\]"
)
GPS_INIT_LINE_RE = re.compile(r"GPS_INIT\[(OK)\]|GPS_INIT\[(FAIL) prt_ack=(\d) msg_ack=(\d)\]")
MAG_LINE_RE = re.compile(
    r"MAG\[healthy heading_x10 mg\]=\[\s*(\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s*\]"
)
MAG_INIT_LINE_RE = re.compile(r"MAG_INIT\[(OK|FAIL) chip_id=0x([0-9A-Fa-f]{2})\]")
MAG_CAL_STARTED_RE = re.compile(r"MAG_CAL\[STARTED\]")
MAG_CAL_OK_RE = re.compile(
    r"MAG_CAL\[OK ox=(-?\d*\.?\d+) oy=(-?\d*\.?\d+) sx=(-?\d*\.?\d+) sy=(-?\d*\.?\d+)\]"
)
MAG_CAL_FAIL_RE = re.compile(r"MAG_CAL\[FAIL([^\]]*)\]")
MODE_LINE_RE = re.compile(r"MODE\[name=([^\s\]]+) ch6=(\d+)\]")
ATT_LINE_RE = re.compile(r"ATT\[src=([^\]]+)\]=\[ROLL_KP\s+([-+]?\d*\.?\d+)\s+PITCH_KP\s+([-+]?\d*\.?\d+)\s+MAX_ANG\s+([-+]?\d*\.?\d+)\]")
PID_LINE_RE = re.compile(
    r"PID\[src=([^\]]+)\]=\[R\s+([-+]?\d*\.?\d+)\s+([-+]?\d*\.?\d+)\s+([-+]?\d*\.?\d+)\s+([-+]?\d*\.?\d+)\s+"
    r"P\s+([-+]?\d*\.?\d+)\s+([-+]?\d*\.?\d+)\s+([-+]?\d*\.?\d+)\s+([-+]?\d*\.?\d+)\s+"
    r"Y\s+([-+]?\d*\.?\d+)\s+([-+]?\d*\.?\d+)\s+([-+]?\d*\.?\d+)\s+([-+]?\d*\.?\d+)\]"
)
PID_STATUS_RE = re.compile(r"PID_(SET|SAVE|LOAD)\[([^\]]+)\]")
PID_DEBUG_RE = re.compile(r"PID_DEBUG\[q=(\d+) h=(\d+) p=(\d+)\]")
PID_FLASH_RE = re.compile(
    r"PID_FLASH\[addr=(0x[0-9A-Fa-f]+) magic=(0x[0-9A-Fa-f]+) ver=(\d+) crc=(0x[0-9A-Fa-f]+) calc=(0x[0-9A-Fa-f]+) header=(\d+) crc_ok=(\d+) gains_ok=(\d+)\]"
)

# Blackbox (SD log) playback: parses raw "SDLOG DUMP"/"SDLOG DUMP LAST" text dumps saved by
# tools/sdlog_analyze.py (--save-raw), mirroring its RECORD_STRUCT/FIELD_NAMES/flight
# segmentation without pulling in numpy/matplotlib.
BB_BLOCK_RE = re.compile(r"SDLOG\[(\d+)\]=([0-9A-Fa-f]+)")
BB_RECORD_STRUCT = struct.Struct("<I9hHHHHBB")
BB_RECORD_SIZE = BB_RECORD_STRUCT.size  # 32 bytes, matches App_SdLogRecord_t in Core/Src/app.c
BB_FIELD_NAMES = (
    "time_ms",
    "setpoint_roll_dps", "setpoint_pitch_dps", "setpoint_yaw_dps",
    "gyro_roll_dps_x10", "gyro_pitch_dps_x10", "gyro_yaw_dps_x10",
    "pid_roll_us", "pid_pitch_us", "pid_yaw_us",
    "motor_fl_us", "motor_fr_us", "motor_rr_us", "motor_rl_us",
    "battery_decivolts", "flags",
)
BB_FLIGHT_GAP_MS = 1500  # a stored-sample time gap bigger than this means a new flight
BB_FLIGHT_MODE_NAMES = {0: "RATE", 1: "ATTITUDE", 2: "ALTHOLD"}


def _bb_parse_records(text: str) -> list:
    blocks = {}
    for line in text.splitlines():
        m = BB_BLOCK_RE.match(line.strip())
        if m is not None:
            blocks[int(m.group(1))] = bytes.fromhex(m.group(2))

    records = []
    for block_idx in sorted(blocks):
        data = blocks[block_idx]
        for offset in range(0, len(data) - BB_RECORD_SIZE + 1, BB_RECORD_SIZE):
            chunk = data[offset:offset + BB_RECORD_SIZE]
            if chunk == bytes(BB_RECORD_SIZE):
                continue  # zero-padded tail of a flight's last (partial) block
            values = BB_RECORD_STRUCT.unpack(chunk)
            records.append(dict(zip(BB_FIELD_NAMES, values)))
    return records


def _bb_segment_flights(records: list) -> list:
    flights = []
    start = 0
    for i in range(1, len(records)):
        dt = records[i]["time_ms"] - records[i - 1]["time_ms"]
        if dt < 0 or dt > BB_FLIGHT_GAP_MS:
            flights.append((start, i))
            start = i
    if records:
        flights.append((start, len(records)))
    return flights

PID_LIMITS = {
    "roll_kp": (0.0, 4.0),
    "pitch_kp": (0.0, 4.0),
    "yaw_kp": (0.0, 4.0),
    "roll_ki": (0.0, 2.0),
    "pitch_ki": (0.0, 2.0),
    "yaw_ki": (0.0, 2.0),
    "roll_kd": (0.0, 0.2),
    "pitch_kd": (0.0, 0.2),
    "yaw_kd": (0.0, 0.2),
    "roll_kff": (0.0, 0.5),
    "pitch_kff": (0.0, 0.5),
    "yaw_kff": (0.0, 0.5),
}

GUI_STATE_PATH = Path.home() / ".kh7_ground_gui_state.json"
GUI_STATE_VERSION = 2


class Kh7GroundGui:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("KH7 USB Ground Station")
        self.root.geometry("1080x780")
        self.root.resizable(True, True)

        self.serial_port = None
        self.tcp_socket = None
        self.tcp_rx_buffer = bytearray()
        self.wifi_connecting = False
        self.wifi_connect_queue = queue.Queue()
        self.motor_canvas = None
        self.pose_canvas = None
        self.rc_canvas = None
        self.motor_shapes = {}
        self.chart_canvases = {}
        self.chart_configs = {}
        self.chart_history_len = 180
        self.chart_data = {
            "accel_x": [],
            "accel_y": [],
            "accel_z": [],
            "gyro_x": [],
            "gyro_y": [],
            "gyro_z": [],
            "motor_1": [],
            "motor_2": [],
            "motor_3": [],
            "motor_4": [],
            "baro_alt": [],
            "baro_vz": [],
            "mag_heading": [],
        }

        self.transport_var = tk.StringVar(value="USB")
        self.port_var = tk.StringVar(value="COM6")
        self.host_var = tk.StringVar(value="kh7bridge.local")
        self.tcp_port_var = tk.StringVar(value="3333")
        self.connected_var = tk.StringVar(value="Disconnected")
        self.event_var = tk.StringVar(value="Ready")
        self.bridge_ip_var = tk.StringVar(value="Bridge IP: -")
        self.bridge_status_var = tk.StringVar(value="Bridge status: -")
        self.bridge_status_lines = []
        self.battery_var = tk.StringVar(value="Battery: -")
        self.mode_var = tk.StringVar(value="Mode: -")
        self.pid_status_var = tk.StringVar(value="PID: idle")
        self.att_status_var = tk.StringVar(value="ATT: idle")
        self.pid_debug_var = tk.StringVar(value="PID debug: -")
        self.pid_flash_var = tk.StringVar(value="PID flash: -")
        self.pid_health_var = tk.StringVar(value="PID link: waiting for checks")
        self.escpt_status_var = tk.StringVar(value="ESC pass-through: OFF")
        self.pid_health_label = None
        self.pid_debug_ok = None
        self.pid_flash_ok = None

        self.ax_var = tk.StringVar(value="0.00")
        self.ay_var = tk.StringVar(value="0.00")
        self.az_var = tk.StringVar(value="0.00")
        self.gx_var = tk.StringVar(value="0.0")
        self.gy_var = tk.StringVar(value="0.0")
        self.gz_var = tk.StringVar(value="0.0")
        self.pitch_var = tk.StringVar(value="0.0")
        self.roll_var = tk.StringVar(value="0.0")
        self.yaw_var = tk.StringVar(value="0.0")
        self.baro_alt_var = tk.StringVar(value="-")
        self.baro_vz_var = tk.StringVar(value="-")
        self.baro_health_var = tk.StringVar(value="-")
        self.baro_init_var = tk.StringVar(value="Baro init: waiting for boot message")
        self.gps_health_var = tk.StringVar(value="-")
        self.gps_fix_var = tk.StringVar(value="-")
        self.gps_sats_var = tk.StringVar(value="-")
        self.gps_pos_var = tk.StringVar(value="GPS: waiting for data")
        self.gps_init_var = tk.StringVar(value="GPS init: waiting for boot message")
        self.mag_health_var = tk.StringVar(value="-")
        self.mag_heading_var = tk.StringVar(value="-")
        self.mag_init_var = tk.StringVar(value="Mag init: waiting for boot message")
        self.mag_cal_status_var = tk.StringVar(value="Compass cal: not calibrated")
        self.mag_cal_active = False

        self.rc_channels_us = [1500 for _ in range(16)]
        self.rc_channels_us[2] = 988
        self.rc_link_active = 0
        self.rc_frame_count = 0
        self.rc_arm_switch = 0
        self.rc_armed = 0

        self.bb_records = []
        self.bb_flights = []
        self.bb_selected_flight_records = []
        self.bb_play_index = 0
        self.bb_playing = False
        self.bb_after_id = None
        self.bb_ignore_scrub_event = False
        self.bb_flight_combo = None
        self.bb_scrub_scale = None
        self.bb_speed_var = tk.StringVar(value="1x")
        self.bb_progress_var = tk.StringVar(value="No log loaded")
        self.bb_setpoint_roll_var = tk.StringVar(value="0.0")
        self.bb_setpoint_pitch_var = tk.StringVar(value="0.0")
        self.bb_setpoint_yaw_var = tk.StringVar(value="0.0")
        self.bb_pid_roll_var = tk.StringVar(value="0")
        self.bb_pid_pitch_var = tk.StringVar(value="0")
        self.bb_pid_yaw_var = tk.StringVar(value="0")

        self.pid_vars = {
            "roll_kp": tk.StringVar(value="0.9000"),
            "roll_ki": tk.StringVar(value="0.0000"),
            "roll_kd": tk.StringVar(value="0.0000"),
            "roll_kff": tk.StringVar(value="0.0000"),
            "pitch_kp": tk.StringVar(value="0.9000"),
            "pitch_ki": tk.StringVar(value="0.0000"),
            "pitch_kd": tk.StringVar(value="0.0000"),
            "pitch_kff": tk.StringVar(value="0.0000"),
            "yaw_kp": tk.StringVar(value="0.8000"),
            "yaw_ki": tk.StringVar(value="0.0000"),
            "yaw_kd": tk.StringVar(value="0.0000"),
            "yaw_kff": tk.StringVar(value="0.0000"),
        }
        self.att_vars = {
            "roll_kp": tk.StringVar(value="5.0000"),
            "pitch_kp": tk.StringVar(value="5.0000"),
            "max_angle": tk.StringVar(value="35.0"),
        }
        self.current_tuning_mode = "RATE"

        self.motor_var = tk.IntVar(value=1)
        self.pulse_var = tk.IntVar(value=1100)
        self.motor_outputs = {1: 988, 2: 988, 3: 988, 4: 988}
        self.motor_test_visible = True
        self.pid_panel_visible = True
        self.pid_received_once = False
        self.pid_sync_retries_left = 0
        self.pid_sync_after_id = None
        self.att_verify_target = None
        self.att_verify_timeout_after_id = None
        self.toggle_motor_test_button = None
        self.toggle_pid_button = None
        self.right_panel = None
        self.motor_test_box = None
        self.pid_box = None
        self.att_pid_box = None
        self.log_text = None
        self.log_line_count = 0
        self.log_max_lines = 800

        self._ui_state = self._load_ui_state()
        self.motor_test_visible = bool(self._ui_state.get("motor_test_visible", self.motor_test_visible))
        self.pid_panel_visible = bool(self._ui_state.get("pid_panel_visible", self.pid_panel_visible))
        self.transport_var.set(self._ui_state.get("transport", self.transport_var.get()))
        self.port_var.set(self._ui_state.get("port", self.port_var.get()))
        self.host_var.set(self._ui_state.get("host", self.host_var.get()))
        self.tcp_port_var.set(self._ui_state.get("tcp_port", self.tcp_port_var.get()))

        self._build_ui()
        self._apply_loaded_window_state()
        self.refresh_ports()
        self.root.after(300, self._auto_connect_tick)
        self.root.after(50, self._poll_io)

    def _load_ui_state(self) -> dict:
        try:
            with GUI_STATE_PATH.open("r", encoding="utf-8") as handle:
                state = json.load(handle)
                if not isinstance(state, dict):
                    return {}
                if state.get("version") != GUI_STATE_VERSION:
                    return {}
                return state
        except FileNotFoundError:
            return {}
        except Exception:
            return {}

    def _apply_loaded_window_state(self) -> None:
        geometry = self._ui_state.get("geometry")
        if isinstance(geometry, str) and geometry:
            try:
                match = re.match(r"^(\d+)x(\d+)([+-]\d+)?([+-]\d+)?$", geometry)
                if match is None:
                    self.root.geometry(geometry)
                    return

                self.root.update_idletasks()
                screen_width = max(self.root.winfo_screenwidth(), 1)
                screen_height = max(self.root.winfo_screenheight(), 1)
                width = max(640, min(int(match.group(1)), screen_width - 40))
                height = max(480, min(int(match.group(2)), screen_height - 80))
                x = int(match.group(3) or 20)
                y = int(match.group(4) or 20)
                x = max(0, min(x, screen_width - width))
                y = max(0, min(y, screen_height - height))
                self.root.geometry(f"{width}x{height}+{x}+{y}")
            except tk.TclError:
                pass

    def _save_ui_state(self) -> None:
        state = {
            "version": GUI_STATE_VERSION,
            "geometry": self.root.geometry(),
            "motor_test_visible": self.motor_test_visible,
            "pid_panel_visible": self.pid_panel_visible,
            "transport": self.transport_var.get(),
            "port": self.port_var.get(),
            "host": self.host_var.get(),
            "tcp_port": self.tcp_port_var.get(),
        }
        try:
            GUI_STATE_PATH.parent.mkdir(parents=True, exist_ok=True)
            with GUI_STATE_PATH.open("w", encoding="utf-8") as handle:
                json.dump(state, handle, indent=2, sort_keys=True)
        except Exception:
            pass

    def _build_ui(self) -> None:
        top = ttk.Frame(self.root, padding=10)
        top.pack(fill=tk.X)

        ttk.Label(top, text="Transport:").pack(side=tk.LEFT)
        self.transport_combo = ttk.Combobox(top,
                            textvariable=self.transport_var,
                            values=["USB", "Wi-Fi"],
                            state="readonly",
                            width=8)
        self.transport_combo.pack(side=tk.LEFT, padx=(6, 10))
        self.transport_combo.bind("<<ComboboxSelected>>", self._on_transport_changed)

        ttk.Label(top, text="USB CDC Port:").pack(side=tk.LEFT)
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, state="readonly", width=14)
        self.port_combo.pack(side=tk.LEFT, padx=(6, 10))
        self.refresh_button = ttk.Button(top, text="Refresh", command=self.refresh_ports)
        self.refresh_button.pack(side=tk.LEFT)

        ttk.Label(top, text="Host:").pack(side=tk.LEFT, padx=(10, 0))
        self.host_entry = ttk.Entry(top, textvariable=self.host_var, width=18)
        self.host_entry.pack(side=tk.LEFT, padx=(6, 10))

        ttk.Label(top, text="TCP:").pack(side=tk.LEFT)
        self.tcp_port_entry = ttk.Entry(top, textvariable=self.tcp_port_var, width=6)
        self.tcp_port_entry.pack(side=tk.LEFT, padx=(6, 10))

        ttk.Button(top, text="Connect", command=self.connect).pack(side=tk.LEFT, padx=6)
        ttk.Button(top, text="Disconnect", command=self.disconnect).pack(side=tk.LEFT)
        ttk.Button(top, text="ESC PT ON", command=self.escpt_on).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(top, text="ESC PT OFF", command=self.escpt_off).pack(side=tk.LEFT, padx=(6, 0))
        self.toggle_motor_test_button = ttk.Button(top, text="Hide Motor Test", command=self.toggle_motor_test_panel)
        self.toggle_motor_test_button.pack(side=tk.LEFT, padx=(8, 0))
        self.toggle_pid_button = ttk.Button(top, text="Hide PID", command=self.toggle_pid_panel)
        self.toggle_pid_button.pack(side=tk.LEFT, padx=(6, 0))

        ttk.Label(top, textvariable=self.connected_var).pack(side=tk.LEFT, padx=(12, 0))
        ttk.Label(top, textvariable=self.bridge_ip_var).pack(side=tk.LEFT, padx=(10, 0))
        ttk.Label(top, textvariable=self.event_var).pack(side=tk.RIGHT)

        # Body is scrollable so every section (charts, playback, log) stays reachable
        # even when the window is shorter than the total content height.
        body_container = ttk.Frame(self.root)
        body_container.pack(fill=tk.BOTH, expand=True)

        body_canvas = tk.Canvas(body_container, highlightthickness=0)
        body_scroll = ttk.Scrollbar(body_container, orient=tk.VERTICAL, command=body_canvas.yview)
        body_canvas.configure(yscrollcommand=body_scroll.set)
        body_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        body_scroll.pack(side=tk.RIGHT, fill=tk.Y)

        body = ttk.Frame(body_canvas, padding=(10, 0, 10, 10))
        body_window = body_canvas.create_window((0, 0), window=body, anchor="nw")

        def _on_body_configure(_event=None) -> None:
            body_canvas.configure(scrollregion=body_canvas.bbox("all"))

        def _on_canvas_configure(event) -> None:
            body_canvas.itemconfigure(body_window, width=event.width)

        body.bind("<Configure>", _on_body_configure)
        body_canvas.bind("<Configure>", _on_canvas_configure)

        def _on_mousewheel(event) -> None:
            body_canvas.yview_scroll(-1 * int(event.delta / 120), "units")

        def _bind_mousewheel(_event=None) -> None:
            body_canvas.bind_all("<MouseWheel>", _on_mousewheel)

        def _unbind_mousewheel(_event=None) -> None:
            body_canvas.unbind_all("<MouseWheel>")

        body_canvas.bind("<Enter>", _bind_mousewheel)
        body_canvas.bind("<Leave>", _unbind_mousewheel)

        telemetry_row = ttk.Frame(body)
        telemetry_row.pack(fill=tk.X, pady=(0, 10))

        telemetry = ttk.LabelFrame(telemetry_row, text="Board Telemetry")
        telemetry.pack(side=tk.LEFT, fill=tk.BOTH, padx=(0, 8))

        self._metric_row(telemetry, 0, "Accel X (g)", self.ax_var, "Rate X (dps)", self.gx_var, "Pitch (deg)", self.pitch_var)
        self._metric_row(telemetry, 1, "Accel Y (g)", self.ay_var, "Rate Y (dps)", self.gy_var, "Roll (deg)", self.roll_var)
        self._metric_row(telemetry, 2, "Accel Z (g)", self.az_var, "Rate Z (dps)", self.gz_var, "Yaw (deg)", self.yaw_var)

        telemetry.grid_columnconfigure(6, weight=1)
        ttk.Label(
            telemetry,
            textvariable=self.battery_var,
            font=("Segoe UI", 24, "bold"),
            anchor="e",
            justify=tk.RIGHT,
        ).grid(row=0, column=6, rowspan=2, sticky="e", padx=(12, 14), pady=(4, 6))
        ttk.Label(
            telemetry,
            textvariable=self.mode_var,
            font=("Segoe UI", 16, "bold"),
            anchor="e",
            justify=tk.RIGHT,
        ).grid(row=2, column=6, sticky="e", padx=(12, 14), pady=(0, 8))

        # Baro/GPS/Compass health placed beside (not below) Board Telemetry so this
        # row stays as short as the 3-line IMU block instead of growing to 10 rows.
        sensor_box = ttk.LabelFrame(telemetry_row, text="Sensor Health")
        sensor_box.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        ttk.Label(sensor_box, text="Baro", font=("Segoe UI", 9, "bold")).grid(row=0, column=0, columnspan=2, sticky="w", padx=(10, 4), pady=(4, 0))
        ttk.Label(sensor_box, text="GPS", font=("Segoe UI", 9, "bold")).grid(row=0, column=2, columnspan=2, sticky="w", padx=(10, 4), pady=(4, 0))
        ttk.Label(sensor_box, text="Compass", font=("Segoe UI", 9, "bold")).grid(row=0, column=4, columnspan=2, sticky="w", padx=(10, 4), pady=(4, 0))

        self._sensor_metric(sensor_box, 1, 0, "Health", self.baro_health_var)
        self._sensor_metric(sensor_box, 1, 2, "Health", self.gps_health_var)
        self._sensor_metric(sensor_box, 1, 4, "Health", self.mag_health_var)

        self._sensor_metric(sensor_box, 2, 0, "Alt (m)", self.baro_alt_var)
        self._sensor_metric(sensor_box, 2, 2, "Fix", self.gps_fix_var)
        self._sensor_metric(sensor_box, 2, 4, "Heading", self.mag_heading_var)

        self._sensor_metric(sensor_box, 3, 0, "VZ (m/s)", self.baro_vz_var)
        self._sensor_metric(sensor_box, 3, 2, "Sats", self.gps_sats_var)

        ttk.Label(sensor_box, textvariable=self.baro_init_var, foreground="#9aa6b2").grid(
            row=4, column=0, columnspan=2, sticky="w", padx=(10, 4), pady=(2, 4)
        )
        ttk.Label(sensor_box, textvariable=self.gps_pos_var, foreground="#9aa6b2").grid(
            row=4, column=2, columnspan=2, sticky="w", padx=(10, 4), pady=(2, 4)
        )
        ttk.Label(sensor_box, textvariable=self.gps_init_var, foreground="#9aa6b2").grid(
            row=5, column=2, columnspan=2, sticky="w", padx=(10, 4), pady=(0, 4)
        )
        ttk.Label(sensor_box, textvariable=self.mag_init_var, foreground="#9aa6b2").grid(
            row=4, column=4, columnspan=2, sticky="w", padx=(10, 4), pady=(2, 4)
        )
        self.mag_cal_button = ttk.Button(sensor_box, text="Calibrate", command=self.mag_cal_toggle)
        self.mag_cal_button.grid(row=5, column=4, columnspan=2, sticky="we", padx=(10, 4), pady=(2, 2))
        ttk.Label(sensor_box, textvariable=self.mag_cal_status_var, foreground="#9aa6b2", wraplength=220).grid(
            row=6, column=4, columnspan=2, sticky="w", padx=(10, 4), pady=(0, 4)
        )

        mid = ttk.Frame(body)
        mid.pack(fill=tk.BOTH, expand=True, pady=(0, 10))

        map_box = ttk.LabelFrame(mid, text="Logical Motor Map and Commanded Output")
        map_box.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 8))

        self.motor_canvas = tk.Canvas(map_box, width=320, height=213, bg="#101214", highlightthickness=0)
        self.motor_canvas.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)
        self.motor_canvas.bind("<Configure>", self._on_motor_canvas_resize)
        self._init_motor_map()

        pose_box = ttk.LabelFrame(mid, text="Air Vehicle Pose")
        pose_box.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 8))

        self.pose_canvas = tk.Canvas(pose_box, width=320, height=213, bg="#0b1015", highlightthickness=0)
        self.pose_canvas.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)
        self.pose_canvas.bind("<Configure>", self._on_pose_canvas_resize)
        self._draw_pose_canvas(0.0, 0.0, 0.0)

        rc_box = ttk.LabelFrame(mid, text="RC Channel Status")
        rc_box.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 8))

        self.rc_canvas = tk.Canvas(rc_box, width=320, height=213, bg="#0b1015", highlightthickness=0)
        self.rc_canvas.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)
        self.rc_canvas.bind("<Configure>", self._on_rc_canvas_resize)
        self._draw_rc_canvas()

        self.right_panel = ttk.Frame(mid)
        self.right_panel.pack(side=tk.RIGHT, fill=tk.Y)

        self.motor_test_box = ttk.LabelFrame(self.right_panel, text="Motor Test")

        for idx in range(1, 5):
            ttk.Radiobutton(self.motor_test_box, text=f"S{idx}", variable=self.motor_var, value=idx).pack(anchor=tk.W, padx=10, pady=2)

        self.scale = ttk.Scale(
            self.motor_test_box,
            from_=988,
            to=2012,
            orient=tk.HORIZONTAL,
            command=self._on_scale,
            length=210,
        )
        self.scale.pack(padx=10, pady=(12, 6))

        pulse_row = ttk.Frame(self.motor_test_box)
        pulse_row.pack(fill=tk.X, padx=10, pady=(0, 8))
        self.pulse_entry = ttk.Entry(pulse_row, width=8)
        self.pulse_entry.pack(side=tk.LEFT)
        self.pulse_entry.insert(0, str(self.pulse_var.get()))
        ttk.Button(pulse_row, text="Set", command=self.apply_entry_pulse).pack(side=tk.LEFT, padx=6)

        ttk.Button(self.motor_test_box, text="Run Selected", command=self.run_selected).pack(fill=tk.X, padx=10, pady=(2, 4))
        ttk.Button(self.motor_test_box, text="Stop", command=self.stop).pack(fill=tk.X, padx=10, pady=(0, 10))

        hint = (
            "Logical map:\n"
            "S1 LF, S2 RF, S3 RA, S4 LA\n\n"
            "USB commands:\n"
            "MTEST <1..4> <pulse_us>\n"
            "MTEST OFF"
        )
        ttk.Label(self.motor_test_box, text=hint, justify=tk.LEFT).pack(anchor=tk.W, padx=10, pady=(0, 10))

        self.pid_box = ttk.LabelFrame(self.right_panel, text="Rate PID Tuning")

        ttk.Label(self.pid_box, text="Axis").grid(row=0, column=0, sticky="w", padx=(6, 4), pady=(6, 4))
        ttk.Label(self.pid_box, text="Kp").grid(row=0, column=1, sticky="w", padx=4, pady=(6, 4))
        ttk.Label(self.pid_box, text="Ki").grid(row=0, column=2, sticky="w", padx=4, pady=(6, 4))
        ttk.Label(self.pid_box, text="Kd").grid(row=0, column=3, sticky="w", padx=4, pady=(6, 4))
        ttk.Label(self.pid_box, text="Kff").grid(row=0, column=4, sticky="w", padx=4, pady=(6, 4))

        self._add_pid_row(self.pid_box, 1, "Roll", "roll")
        self._add_pid_row(self.pid_box, 2, "Pitch", "pitch")
        self._add_pid_row(self.pid_box, 3, "Yaw", "yaw")

        pid_btns = ttk.Frame(self.pid_box)
        pid_btns.grid(row=4, column=0, columnspan=5, sticky="ew", padx=6, pady=(6, 6))

        ttk.Button(pid_btns, text="Read", command=self.pid_read).pack(side=tk.LEFT, padx=(0, 4))
        ttk.Button(pid_btns, text="Apply", command=self.pid_apply).pack(side=tk.LEFT, padx=4)
        ttk.Button(pid_btns, text="Save", command=self.pid_save).pack(side=tk.LEFT, padx=4)
        ttk.Button(pid_btns, text="Load", command=self.pid_load).pack(side=tk.LEFT, padx=4)
        ttk.Button(pid_btns, text="Defaults", command=self.pid_defaults).pack(side=tk.LEFT, padx=4)
        ttk.Button(pid_btns, text="Debug", command=self.pid_debug).pack(side=tk.LEFT, padx=4)

        ttk.Label(
            self.pid_box,
            text="Max allowed: Kp <= 4.0, Ki <= 2.0, Kd <= 0.2, Kff <= 0.5",
            foreground="#9aa6b2",
            justify=tk.LEFT,
        ).grid(row=5, column=0, columnspan=5, sticky="w", padx=6, pady=(0, 6))

        ttk.Label(self.pid_box, textvariable=self.pid_status_var).grid(
            row=6, column=0, columnspan=5, sticky="w", padx=6, pady=(0, 6)
        )
        ttk.Label(self.pid_box, textvariable=self.pid_debug_var, justify=tk.LEFT).grid(
            row=7, column=0, columnspan=5, sticky="w", padx=6, pady=(0, 2)
        )
        ttk.Label(self.pid_box, textvariable=self.pid_flash_var, justify=tk.LEFT).grid(
            row=8, column=0, columnspan=5, sticky="w", padx=6, pady=(0, 6)
        )
        self.pid_health_label = tk.Label(
            self.pid_box,
            textvariable=self.pid_health_var,
            fg="#9aa6b2",
            anchor="w",
            justify=tk.LEFT,
            font=("Segoe UI", 9, "bold"),
        )
        self.pid_health_label.grid(row=9, column=0, columnspan=4, sticky="w", padx=6, pady=(0, 8))

        self.att_pid_box = ttk.LabelFrame(self.right_panel, text="Attitude Tuning")
        ttk.Label(self.att_pid_box, text="Roll Kp").grid(row=0, column=0, sticky="w", padx=(6, 4), pady=(8, 4))
        ttk.Entry(self.att_pid_box, width=8, textvariable=self.att_vars["roll_kp"]).grid(row=0, column=1, sticky="w", padx=(0, 8), pady=(8, 4))
        ttk.Label(self.att_pid_box, text="Pitch Kp").grid(row=1, column=0, sticky="w", padx=(6, 4), pady=4)
        ttk.Entry(self.att_pid_box, width=8, textvariable=self.att_vars["pitch_kp"]).grid(row=1, column=1, sticky="w", padx=(0, 8), pady=4)
        ttk.Label(self.att_pid_box, text="Max Angle").grid(row=2, column=0, sticky="w", padx=(6, 4), pady=4)
        ttk.Entry(self.att_pid_box, width=8, textvariable=self.att_vars["max_angle"]).grid(row=2, column=1, sticky="w", padx=(0, 8), pady=4)

        att_btns = ttk.Frame(self.att_pid_box)
        att_btns.grid(row=3, column=0, columnspan=2, sticky="ew", padx=6, pady=(6, 6))
        ttk.Button(att_btns, text="Read", command=self.att_read).pack(side=tk.LEFT, padx=(0, 4))
        ttk.Button(att_btns, text="Apply", command=self.att_apply).pack(side=tk.LEFT, padx=4)
        ttk.Button(att_btns, text="Defaults", command=self.att_defaults).pack(side=tk.LEFT, padx=4)

        ttk.Label(
            self.att_pid_box,
            text="Max allowed: Roll/Pitch Kp <= 25.0, Max Angle <= 70 deg",
            foreground="#9aa6b2",
            justify=tk.LEFT,
        ).grid(row=4, column=0, columnspan=2, sticky="w", padx=6, pady=(0, 6))

        ttk.Label(self.att_pid_box, textvariable=self.att_status_var).grid(
            row=5, column=0, columnspan=2, sticky="w", padx=6, pady=(0, 8)
        )

        self._apply_right_panel_visibility()

        charts_box = ttk.LabelFrame(body, text="Strip Charts")
        charts_box.pack(fill=tk.BOTH, expand=True)

        self._create_strip_chart(
            charts_box,
            chart_key="accels",
            title="Accels (g)",
            series=[("accel_x", "X", "#45d1ff"), ("accel_y", "Y", "#69f38a"), ("accel_z", "Z", "#ffd166")],
            y_min=-4.0,
            y_max=4.0,
        )
        self._create_strip_chart(
            charts_box,
            chart_key="gyro",
            title="Gyro (dps)",
            series=[("gyro_x", "X", "#45d1ff"), ("gyro_y", "Y", "#69f38a"), ("gyro_z", "Z", "#ffd166")],
            y_min=-500.0,
            y_max=500.0,
        )
        self._create_strip_chart(
            charts_box,
            chart_key="motors",
            title="Motor Commands (us)",
            series=[("motor_1", "S1", "#4caf50"), ("motor_2", "S2", "#66bb6a"), ("motor_3", "S3", "#81c784"), ("motor_4", "S4", "#a5d6a7")],
            y_min=988.0,
            y_max=2012.0,
        )

        self._create_strip_chart(
            charts_box,
            chart_key="baro",
            title="Baro Altitude (m) / Climb Rate (m/s)",
            series=[("baro_alt", "Alt", "#45d1ff"), ("baro_vz", "VZ", "#ffd166")],
            y_min=-5.0,
            y_max=5.0,
        )

        self._create_strip_chart(
            charts_box,
            chart_key="mag",
            title="Compass Heading (deg, uncompensated)",
            series=[("mag_heading", "Heading", "#c586ff")],
            y_min=0.0,
            y_max=360.0,
        )

        bb_box = ttk.LabelFrame(body, text="Blackbox Playback (SD log)")
        bb_box.pack(fill=tk.X, pady=(0, 10))

        bb_top = ttk.Frame(bb_box)
        bb_top.pack(fill=tk.X, padx=8, pady=(8, 4))

        ttk.Button(bb_top, text="Load Log...", command=self.bb_load_log).pack(side=tk.LEFT)

        ttk.Label(bb_top, text="Flight:").pack(side=tk.LEFT, padx=(10, 4))
        self.bb_flight_combo = ttk.Combobox(bb_top, state="readonly", width=28)
        self.bb_flight_combo.pack(side=tk.LEFT)
        self.bb_flight_combo.bind("<<ComboboxSelected>>", self.bb_on_flight_selected)

        ttk.Label(bb_top, text="Speed:").pack(side=tk.LEFT, padx=(10, 4))
        ttk.Combobox(
            bb_top,
            textvariable=self.bb_speed_var,
            state="readonly",
            width=5,
            values=["0.25x", "0.5x", "1x", "2x", "4x", "8x"],
        ).pack(side=tk.LEFT)

        ttk.Button(bb_top, text="Play", command=self.bb_play).pack(side=tk.LEFT, padx=(10, 2))
        ttk.Button(bb_top, text="Pause", command=self.bb_pause).pack(side=tk.LEFT, padx=2)
        ttk.Button(bb_top, text="Stop", command=self.bb_stop).pack(side=tk.LEFT, padx=2)

        bb_scrub_row = ttk.Frame(bb_box)
        bb_scrub_row.pack(fill=tk.X, padx=8, pady=(0, 4))
        self.bb_scrub_scale = ttk.Scale(bb_scrub_row, from_=0, to=0, orient=tk.HORIZONTAL, command=self._on_bb_scrub)
        self.bb_scrub_scale.pack(fill=tk.X, expand=True, side=tk.LEFT)
        ttk.Label(bb_scrub_row, textvariable=self.bb_progress_var, width=34, anchor="e").pack(side=tk.LEFT, padx=(8, 0))

        bb_readout = ttk.Frame(bb_box)
        bb_readout.pack(fill=tk.X, padx=8, pady=(0, 8))
        ttk.Label(bb_readout, text="Setpoint R/P/Y (dps):").pack(side=tk.LEFT)
        ttk.Label(bb_readout, textvariable=self.bb_setpoint_roll_var, width=7).pack(side=tk.LEFT, padx=(4, 2))
        ttk.Label(bb_readout, textvariable=self.bb_setpoint_pitch_var, width=7).pack(side=tk.LEFT, padx=2)
        ttk.Label(bb_readout, textvariable=self.bb_setpoint_yaw_var, width=7).pack(side=tk.LEFT, padx=(2, 16))
        ttk.Label(bb_readout, text="PID out R/P/Y (us):").pack(side=tk.LEFT)
        ttk.Label(bb_readout, textvariable=self.bb_pid_roll_var, width=6).pack(side=tk.LEFT, padx=(4, 2))
        ttk.Label(bb_readout, textvariable=self.bb_pid_pitch_var, width=6).pack(side=tk.LEFT, padx=2)
        ttk.Label(bb_readout, textvariable=self.bb_pid_yaw_var, width=6).pack(side=tk.LEFT, padx=2)
        ttk.Label(
            bb_readout,
            text="(reuses the Gyro/Motor strip charts above; no accel/angle data in the SD log)",
            foreground="#9aa6b2",
        ).pack(side=tk.LEFT, padx=(16, 0))

        log_box = ttk.LabelFrame(body, text="Raw Link Log")
        log_box.pack(fill=tk.BOTH, expand=True, pady=(0, 4))

        log_actions = ttk.Frame(log_box)
        log_actions.pack(fill=tk.X, padx=8, pady=(6, 2))
        ttk.Button(log_actions, text="Clear", command=self.clear_log).pack(side=tk.RIGHT)

        log_frame = ttk.Frame(log_box)
        log_frame.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))

        self.log_text = tk.Text(
            log_frame,
            height=10,
            wrap=tk.NONE,
            bg="#0d1117",
            fg="#d8dde3",
            insertbackground="#d8dde3",
            font=("Consolas", 9),
        )
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.log_text.configure(state=tk.DISABLED)

        log_scroll = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.log_text.yview)
        log_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.configure(yscrollcommand=log_scroll.set)

        self.scale.set(self.pulse_var.get())
        self._on_transport_changed()

    def bb_load_log(self) -> None:
        path = filedialog.askopenfilename(
            title="Load blackbox raw dump (tools/sdlog_analyze.py --save-raw output)",
            filetypes=[("Raw SDLOG dump", "*.txt"), ("All files", "*.*")],
        )
        if not path:
            return

        try:
            with open(path, "r", encoding="ascii", errors="replace") as f:
                text = f.read()
            records = _bb_parse_records(text)
            flights = _bb_segment_flights(records)
        except Exception as exc:
            messagebox.showerror("Load failed", str(exc))
            return

        if not records or not flights:
            messagebox.showerror("Load failed", "No SDLOG[...] records found in that file.")
            return

        self.bb_stop()
        self.bb_records = records
        self.bb_flights = flights

        labels = []
        for i, (start, end) in enumerate(flights):
            sub = records[start:end]
            duration_s = (sub[-1]["time_ms"] - sub[0]["time_ms"]) / 1000.0
            labels.append(f"Flight {i} ({len(sub)} samples, {duration_s:.1f}s)")
        self.bb_flight_combo["values"] = labels
        self.bb_flight_combo.current(len(flights) - 1)
        self.bb_select_flight(len(flights) - 1)
        self.event_var.set(f"Loaded {len(flights)} flight(s) from {Path(path).name}")

    def bb_on_flight_selected(self, _event=None) -> None:
        idx = self.bb_flight_combo.current()
        if idx >= 0:
            self.bb_select_flight(idx)

    def bb_select_flight(self, idx: int) -> None:
        self.bb_stop()
        start, end = self.bb_flights[idx]
        self.bb_selected_flight_records = self.bb_records[start:end]
        self.bb_play_index = 0
        n = max(len(self.bb_selected_flight_records) - 1, 0)
        self.bb_scrub_scale.configure(to=n)
        self._bb_render_frame(0)

    def _bb_set_scrub(self, index: int) -> None:
        self.bb_ignore_scrub_event = True
        try:
            self.bb_scrub_scale.set(index)
        finally:
            self.bb_ignore_scrub_event = False

    def bb_play(self) -> None:
        if not self.bb_selected_flight_records:
            messagebox.showerror("No flight", "Load a blackbox log and select a flight first.")
            return

        if self.bb_play_index >= len(self.bb_selected_flight_records) - 1:
            self.bb_play_index = 0

        self.bb_playing = True
        self._bb_schedule_next()

    def bb_pause(self) -> None:
        self.bb_playing = False
        if self.bb_after_id is not None:
            self.root.after_cancel(self.bb_after_id)
            self.bb_after_id = None

    def bb_stop(self) -> None:
        self.bb_pause()
        self.bb_play_index = 0
        if self.bb_selected_flight_records:
            self._bb_render_frame(0)

    def _bb_speed_multiplier(self) -> float:
        try:
            return max(float(self.bb_speed_var.get().rstrip("xX")), 0.05)
        except ValueError:
            return 1.0

    def _bb_schedule_next(self) -> None:
        if not self.bb_playing:
            return

        records = self.bb_selected_flight_records
        self._bb_render_frame(self.bb_play_index)

        if self.bb_play_index >= len(records) - 1:
            self.bb_playing = False
            self.event_var.set("Blackbox playback finished")
            return

        dt_ms = max(records[self.bb_play_index + 1]["time_ms"] - records[self.bb_play_index]["time_ms"], 0)
        delay_ms = int(max(5, min(250, dt_ms / self._bb_speed_multiplier())))
        self.bb_play_index += 1
        self.bb_after_id = self.root.after(delay_ms, self._bb_schedule_next)

    def _bb_render_frame(self, index: int) -> None:
        records = self.bb_selected_flight_records
        if not records or index >= len(records):
            return
        r = records[index]

        self._bb_set_scrub(index)

        t_s = (r["time_ms"] - records[0]["time_ms"]) / 1000.0
        total_s = (records[-1]["time_ms"] - records[0]["time_ms"]) / 1000.0
        self.bb_progress_var.set(f"{t_s:6.2f}s / {total_s:6.2f}s (sample {index + 1}/{len(records)})")

        gx = r["gyro_roll_dps_x10"] / 10.0
        gy = r["gyro_pitch_dps_x10"] / 10.0
        gz = r["gyro_yaw_dps_x10"] / 10.0
        self.gx_var.set(f"{gx:.1f}")
        self.gy_var.set(f"{gy:.1f}")
        self.gz_var.set(f"{gz:.1f}")
        self._append_samples([("gyro_x", gx), ("gyro_y", gy), ("gyro_z", gz)])
        self._redraw_strip_chart("gyro")

        self.bb_setpoint_roll_var.set(f"{r['setpoint_roll_dps']:.1f}")
        self.bb_setpoint_pitch_var.set(f"{r['setpoint_pitch_dps']:.1f}")
        self.bb_setpoint_yaw_var.set(f"{r['setpoint_yaw_dps']:.1f}")
        self.bb_pid_roll_var.set(str(r["pid_roll_us"]))
        self.bb_pid_pitch_var.set(str(r["pid_pitch_us"]))
        self.bb_pid_yaw_var.set(str(r["pid_yaw_us"]))

        self.motor_outputs[1] = r["motor_fl_us"]
        self.motor_outputs[2] = r["motor_fr_us"]
        self.motor_outputs[3] = r["motor_rr_us"]
        self.motor_outputs[4] = r["motor_rl_us"]
        self._append_samples([
            ("motor_1", float(self.motor_outputs[1])),
            ("motor_2", float(self.motor_outputs[2])),
            ("motor_3", float(self.motor_outputs[3])),
            ("motor_4", float(self.motor_outputs[4])),
        ])
        self._refresh_motor_map()
        self._redraw_strip_chart("motors")

        battery_v = r["battery_decivolts"] / 10.0
        armed = r["flags"] & 0x01
        mode_name = BB_FLIGHT_MODE_NAMES.get((r["flags"] >> 1) & 0x03, "?")
        self.battery_var.set(f"Battery: {battery_v:.2f} V (BB)")
        self.mode_var.set(f"Mode: {mode_name} (BB {'ARMED' if armed else 'disarmed'})")

    def _on_bb_scrub(self, value: str) -> None:
        if self.bb_ignore_scrub_event or not self.bb_selected_flight_records:
            return

        self.bb_pause()
        index = max(0, min(int(float(value)), len(self.bb_selected_flight_records) - 1))
        self.bb_play_index = index
        self._bb_render_frame(index)

    def clear_log(self) -> None:
        if self.log_text is None:
            return

        self.log_text.configure(state=tk.NORMAL)
        self.log_text.delete("1.0", tk.END)
        self.log_text.configure(state=tk.DISABLED)
        self.log_line_count = 0

    def _append_log_line(self, line: str) -> None:
        if self.log_text is None:
            return

        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, line + "\n")
        self.log_line_count += 1

        if self.log_line_count > self.log_max_lines:
            extra = self.log_line_count - self.log_max_lines
            self.log_text.delete("1.0", f"{extra + 1}.0")
            self.log_line_count = self.log_max_lines

        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def _add_pid_row(self, parent: ttk.LabelFrame, row: int, label: str, prefix: str) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", padx=(6, 4), pady=2)
        ttk.Entry(parent, width=7, textvariable=self.pid_vars[f"{prefix}_kp"]).grid(row=row, column=1, sticky="w", padx=4, pady=2)
        ttk.Entry(parent, width=7, textvariable=self.pid_vars[f"{prefix}_ki"]).grid(row=row, column=2, sticky="w", padx=4, pady=2)
        ttk.Entry(parent, width=7, textvariable=self.pid_vars[f"{prefix}_kd"]).grid(row=row, column=3, sticky="w", padx=4, pady=2)
        ttk.Entry(parent, width=7, textvariable=self.pid_vars[f"{prefix}_kff"]).grid(row=row, column=4, sticky="w", padx=4, pady=2)

    def _apply_right_panel_visibility(self) -> None:
        if self.motor_test_box is None or self.pid_box is None or self.att_pid_box is None:
            return

        self.motor_test_box.pack_forget()
        self.pid_box.pack_forget()
        self.att_pid_box.pack_forget()

        if self.motor_test_visible:
            self.motor_test_box.pack(side=tk.TOP, fill=tk.X, pady=(0, 8))

        if self.pid_panel_visible:
            if self.current_tuning_mode == "ATTITUDE":
                self.att_pid_box.pack(side=tk.TOP, fill=tk.X)
            else:
                self.pid_box.pack(side=tk.TOP, fill=tk.X)

    def toggle_motor_test_panel(self) -> None:
        if self.motor_test_box is None:
            return

        self.motor_test_visible = not self.motor_test_visible
        self._apply_right_panel_visibility()

        if self.toggle_motor_test_button is not None:
            self.toggle_motor_test_button.configure(text="Hide Motor Test" if self.motor_test_visible else "Show Motor Test")

    def toggle_pid_panel(self) -> None:
        if self.pid_box is None:
            return

        self.pid_panel_visible = not self.pid_panel_visible
        self._apply_right_panel_visibility()

        if self.toggle_pid_button is not None:
            self.toggle_pid_button.configure(text="Hide PID" if self.pid_panel_visible else "Show PID")

    def _on_transport_changed(self, _event=None) -> None:
        is_usb = self.transport_var.get() == "USB"

        self.port_combo.configure(state="readonly" if is_usb else "disabled")
        self.refresh_button.configure(state="normal" if is_usb else "disabled")
        self.host_entry.configure(state="disabled" if is_usb else "normal")
        self.tcp_port_entry.configure(state="disabled" if is_usb else "normal")

        if not is_usb:
            self.refresh_ports()

    def _create_strip_chart(self,
                            parent: ttk.LabelFrame,
                            chart_key: str,
                            title: str,
                            series,
                            y_min: float,
                            y_max: float) -> None:
        row = ttk.Frame(parent)
        row.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)

        hdr = ttk.Frame(row)
        hdr.pack(fill=tk.X)
        ttk.Label(hdr, text=title).pack(side=tk.LEFT)
        legend = "  ".join(f"{name}:{label}" for name, label, _ in series)
        ttk.Label(hdr, text=legend).pack(side=tk.RIGHT)

        canvas = tk.Canvas(row, height=110, bg="#0d1117", highlightthickness=1, highlightbackground="#2c3640")
        canvas.pack(fill=tk.BOTH, expand=True, pady=(2, 0))
        canvas.bind("<Configure>", lambda _e, key=chart_key: self._redraw_strip_chart(key))

        self.chart_canvases[chart_key] = canvas
        self.chart_configs[chart_key] = {
            "series": series,
            "y_min": y_min,
            "y_max": y_max,
        }
        self._redraw_strip_chart(chart_key)

    def _metric_row(
        self,
        parent: ttk.LabelFrame,
        row: int,
        l1: str,
        v1: tk.StringVar,
        l2: str,
        v2: tk.StringVar,
        l3: str,
        v3: tk.StringVar,
    ) -> None:
        ttk.Label(parent, text=l1).grid(row=row, column=0, sticky="w", padx=(10, 4), pady=4)
        ttk.Label(parent, textvariable=v1, width=10).grid(row=row, column=1, sticky="w", padx=(0, 12), pady=4)
        ttk.Label(parent, text=l2).grid(row=row, column=2, sticky="w", padx=(0, 4), pady=4)
        ttk.Label(parent, textvariable=v2, width=10).grid(row=row, column=3, sticky="w", padx=(0, 12), pady=4)
        ttk.Label(parent, text=l3).grid(row=row, column=4, sticky="w", padx=(0, 4), pady=4)
        ttk.Label(parent, textvariable=v3, width=10).grid(row=row, column=5, sticky="w", padx=(0, 10), pady=4)

    def _sensor_metric(self, parent: ttk.LabelFrame, row: int, col: int, label: str, var: tk.StringVar) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=col, sticky="w", padx=(10, 4), pady=2)
        ttk.Label(parent, textvariable=var, width=8).grid(row=row, column=col + 1, sticky="w", padx=(0, 8), pady=2)

    def _init_motor_map(self) -> None:
        self.motor_canvas.delete("all")
        width = max(int(self.motor_canvas.winfo_width()), 10)
        height = max(int(self.motor_canvas.winfo_height()), 10)
        side = max(min(width, height) - 24, 140)
        left = (width - side) / 2.0
        top = (height - side) / 2.0
        right = left + side
        bottom = top + side

        self.motor_canvas.create_rectangle(left, top, right, bottom, outline="#39586b", width=2)
        self.motor_canvas.create_text((left + right) / 2.0, top + 14, text="Board", fill="#89b6d1", font=("Segoe UI", 12, "bold"))

        x_offset = side * 0.27
        y_offset = side * 0.26
        positions = {
            1: ((left + (side / 2.0) - x_offset), (top + (side / 2.0) - y_offset)),    # LF
            2: ((left + (side / 2.0) + x_offset), (top + (side / 2.0) - y_offset)),    # RF
            3: ((left + (side / 2.0) + x_offset), (top + (side / 2.0) + y_offset)),    # RA
            4: ((left + (side / 2.0) - x_offset), (top + (side / 2.0) + y_offset)),    # LA
        }
        labels = {
            1: "S1 LF",
            2: "S2 RF",
            3: "S3 RA",
            4: "S4 LA",
        }

        for idx in range(1, 5):
            x, y = positions[idx]
            radius = max(min(side * 0.12, 38), 26)
            bg = self.motor_canvas.create_oval(
                x - radius,
                y - radius,
                x + radius,
                y + radius,
                fill="#1a1f23",
                outline="#6b7580",
                width=2,
            )
            arc = self.motor_canvas.create_arc(
                x - (radius - 2),
                y - (radius - 2),
                x + (radius - 2),
                y + (radius - 2),
                start=90,
                extent=0,
                style=tk.PIESLICE,
                fill="#2a6a2a",
                outline="",
            )
            name = self.motor_canvas.create_text(x, y - 8, text=labels[idx], fill="#d8dde3", font=("Segoe UI", 10, "bold"))
            cmd = self.motor_canvas.create_text(x, y + 14, text="cmd 988 us", fill="#9eb5c8", font=("Consolas", 9))
            self.motor_shapes[idx] = {"bg": bg, "arc": arc, "name": name, "cmd": cmd}

    def _motor_green_color(self, pulse_us: int) -> str:
        min_us = 988
        max_us = 2012
        value = max(min_us, min(max_us, pulse_us))
        ratio = (value - min_us) / float(max_us - min_us)

        if ratio >= 1.0:
            return "#d61f1f"

        r = int(20 + ratio * 25)
        g = int(95 + ratio * 145)
        b = int(20 + ratio * 25)
        return f"#{r:02x}{g:02x}{b:02x}"

    @staticmethod
    def _motor_ratio(pulse_us: int) -> float:
        min_us = 988
        max_us = 2012
        value = max(min_us, min(max_us, pulse_us))
        return (value - min_us) / float(max_us - min_us)

    def _refresh_motor_map(self) -> None:
        if self.motor_canvas is None or not self.motor_shapes:
            return

        for idx in range(1, 5):
            pulse = self.motor_outputs.get(idx, 988)
            shape = self.motor_shapes[idx]
            ratio = self._motor_ratio(pulse)
            extent = -360.0 * ratio
            self.motor_canvas.itemconfigure(shape["arc"], extent=extent, fill=self._motor_green_color(pulse))
            self.motor_canvas.itemconfigure(shape["cmd"], text=f"cmd {pulse} us")

    def _on_motor_canvas_resize(self, _event) -> None:
        if self.motor_canvas is None:
            return

        self._init_motor_map()
        self._refresh_motor_map()

    @staticmethod
    def _vehicle_rotate_point(x: float, y: float, z: float, yaw_deg: float, pitch_deg: float, roll_deg: float):
        import math

        yaw = math.radians(yaw_deg)
        pitch = math.radians(pitch_deg)
        roll = math.radians(roll_deg)

        # Vehicle frame:
        # X = right, Y = forward, Z = up.
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cr = math.cos(roll)
        sr = math.sin(roll)

        # Yaw around Z.
        x1 = cy * x - sy * y
        y1 = sy * x + cy * y
        z1 = z

        # Pitch around X.
        x2 = x1
        y2 = cp * y1 - sp * z1
        z2 = sp * y1 + cp * z1

        # Roll around Y.
        x3 = cr * x2 + sr * z2
        y3 = y2
        z3 = -sr * x2 + cr * z2
        return x3, y3, z3

    @staticmethod
    def _vector_length(x: float, y: float, z: float) -> float:
        import math

        return math.sqrt((x * x) + (y * y) + (z * z))

    @staticmethod
    def _vector_normalize(x: float, y: float, z: float):
        length = Kh7GroundGui._vector_length(x, y, z)
        if length <= 1e-9:
            return 0.0, 0.0, 0.0
        return x / length, y / length, z / length

    @staticmethod
    def _vector_cross(ax: float, ay: float, az: float, bx: float, by: float, bz: float):
        return (ay * bz) - (az * by), (az * bx) - (ax * bz), (ax * by) - (ay * bx)

    @staticmethod
    def _vector_dot(ax: float, ay: float, az: float, bx: float, by: float, bz: float) -> float:
        return (ax * bx) + (ay * by) + (az * bz)

    def _project_pose_point(
        self,
        x: float,
        y: float,
        z: float,
        yaw_deg: float,
        pitch_deg: float,
        roll_deg: float,
        width: int,
        height: int,
        camera_distance: float,
        camera_height: float,
        focal_length: float,
    ):
        rotated = self._vehicle_rotate_point(x, y, z, yaw_deg, pitch_deg, roll_deg)

        import math

        yaw = math.radians(yaw_deg)
        camera_x = -math.sin(yaw) * camera_distance
        camera_y = -math.cos(yaw) * camera_distance
        camera_z = camera_height

        forward_x, forward_y, forward_z = self._vector_normalize(-camera_x, -camera_y, -camera_z)
        right_x, right_y, right_z = self._vector_cross(forward_x, forward_y, forward_z, 0.0, 0.0, 1.0)
        right_x, right_y, right_z = self._vector_normalize(right_x, right_y, right_z)

        if self._vector_length(right_x, right_y, right_z) <= 1e-9:
            right_x, right_y, right_z = 1.0, 0.0, 0.0

        up_x, up_y, up_z = self._vector_cross(right_x, right_y, right_z, forward_x, forward_y, forward_z)
        up_x, up_y, up_z = self._vector_normalize(up_x, up_y, up_z)

        rel_x = rotated[0] - camera_x
        rel_y = rotated[1] - camera_y
        rel_z = rotated[2] - camera_z

        depth = self._vector_dot(rel_x, rel_y, rel_z, forward_x, forward_y, forward_z)
        if depth <= 0.08:
            return None

        cx = width / 2.0
        cy = height / 2.0
        scale = focal_length / depth
        screen_x = cx + (self._vector_dot(rel_x, rel_y, rel_z, right_x, right_y, right_z) * scale)
        screen_y = cy - (self._vector_dot(rel_x, rel_y, rel_z, up_x, up_y, up_z) * scale)
        return screen_x, screen_y, depth

    def _draw_pose_canvas(self, yaw_deg: float, pitch_deg: float, roll_deg: float) -> None:
        if self.pose_canvas is None:
            return

        canvas = self.pose_canvas
        canvas.delete("all")

        width = max(int(canvas.winfo_width()), 10)
        height = max(int(canvas.winfo_height()), 10)
        side = max(min(width, height) - 24, 140)
        left = (width - side) / 2.0
        top = (height - side) / 2.0
        right = left + side
        bottom = top + side
        cx = (left + right) / 2.0
        cy = (top + bottom) / 2.0
        canvas.create_rectangle(left, top, right, bottom, outline="#31404d", width=2)
        canvas.create_text(cx, top + 16, text=f"Pose  yaw={yaw_deg:.1f}  pitch={pitch_deg:.1f}  roll={roll_deg:.1f}", fill="#d8dde3", font=("Consolas", 9, "bold"))

        # Rear-view axes: X horizontal, Z vertical, Y is depth into the screen.
        axis_len = side * 0.26
        depth_len = side * 0.16
        canvas.create_line(cx, cy, cx + axis_len, cy, fill="#ef5350", width=3, arrow=tk.LAST)
        canvas.create_text(cx + axis_len + 14, cy, text="X", fill="#ef5350", font=("Segoe UI", 10, "bold"))
        canvas.create_line(cx, cy, cx, cy - axis_len, fill="#42a5f5", width=3, arrow=tk.LAST)
        canvas.create_text(cx + 10, cy - axis_len - 12, text="Z", fill="#42a5f5", font=("Segoe UI", 10, "bold"))
        canvas.create_line(cx, cy, cx - depth_len, cy + depth_len * 0.55, fill="#66bb6a", width=2, dash=(4, 3), arrow=tk.LAST)
        canvas.create_text(cx - depth_len - 2, cy + depth_len * 0.55 + 4, text="Y depth", fill="#66bb6a", font=("Segoe UI", 9, "bold"))

        # Quadcopter-style air vehicle geometry.
        body = {
            "left": -0.28,
            "right": 0.28,
            "front": 0.46,
            "rear": -0.38,
            "top": 0.10,
            "bottom": -0.08,
        }
        motors = [
            (-0.58, 0.55, 0.02),
            (0.58, 0.55, 0.02),
            (0.58, -0.50, 0.02),
            (-0.58, -0.50, 0.02),
        ]

        center = (0.0, 0.0, 0.0)
        camera_distance = 4.4
        camera_height = 1.7
        focal_length = side * 0.85

        def project_point(pt):
            return self._project_pose_point(
                pt[0],
                pt[1],
                pt[2],
                yaw_deg,
                pitch_deg,
                roll_deg,
                width,
                height,
                camera_distance,
                camera_height,
                focal_length,
            )

        def draw_poly(points, fill, outline, width=1):
            if any(point is None for point in points):
                return None
            depth = sum(point[2] for point in points) / float(len(points))
            coords = []
            for point in points:
                coords.extend((point[0], point[1]))
            canvas.create_polygon(*coords, fill=fill, outline=outline, width=width)
            return depth

        def draw_line(a, b, color, width=2, dash=None):
            if (a is None) or (b is None):
                return
            canvas.create_line(a[0], a[1], b[0], b[1], fill=color, width=width, dash=dash)

        body_points = {
            "lfb": project_point((body["left"], body["front"], body["bottom"])),
            "rfb": project_point((body["right"], body["front"], body["bottom"])),
            "rrb": project_point((body["right"], body["rear"], body["bottom"])),
            "lrb": project_point((body["left"], body["rear"], body["bottom"])),
            "lft": project_point((body["left"], body["front"], body["top"])),
            "rft": project_point((body["right"], body["front"], body["top"])),
            "rrt": project_point((body["right"], body["rear"], body["top"])),
            "lrt": project_point((body["left"], body["rear"], body["top"])),
        }

        face_defs = [
            ([body_points["lft"], body_points["rft"], body_points["rrt"], body_points["lrt"]], "#4d8aa0", "#c6eef7", 2),
            ([body_points["lfb"], body_points["rfb"], body_points["rft"], body_points["lft"]], "#2c5a6c", "#5d8fa4", 1),
            ([body_points["lrb"], body_points["rrb"], body_points["rrt"], body_points["lrt"]], "#214757", "#4b7386", 1),
            ([body_points["lfb"], body_points["lrb"], body_points["lrt"], body_points["lft"]], "#263f4e", "#4c6475", 1),
            ([body_points["rfb"], body_points["rrb"], body_points["rrt"], body_points["rft"]], "#263f4e", "#4c6475", 1),
            ([body_points["lfb"], body_points["rfb"], body_points["rrb"], body_points["lrb"]], "#17242c", "#33414f", 1),
        ]
        face_defs.sort(key=lambda item: sum(point[2] for point in item[0]) / float(len(item[0])), reverse=True)

        # Arms to the motors.
        arm_color = "#6b7580"
        motor_fill = "#20262c"
        motor_outline = "#8997a3"
        motor_glow = "#4fd0f0"

        top_center = project_point((0.0, 0.0, body["top"]))
        rear_center = project_point((0.0, body["rear"], body["top"]))
        front_center = project_point((0.0, body["front"], body["top"]))
        draw_line(rear_center, front_center, "#8fb3c6", width=2, dash=(5, 4))

        for face_points, fill, outline, line_width in face_defs:
            draw_poly(face_points, fill, outline, line_width)

        projected_motors = [project_point(pt) for pt in motors]
        body_mounts = [
            project_point((-0.22, 0.32, body["top"])),
            project_point((0.22, 0.32, body["top"])),
            project_point((0.22, -0.26, body["top"])),
            project_point((-0.22, -0.26, body["top"])),
        ]

        for mount, motor in zip(body_mounts, projected_motors):
            draw_line(mount, motor, arm_color, width=4)

        for index, motor in enumerate(projected_motors):
            if motor is None:
                continue
            radius = 12 if motor[2] < 3.0 else 14
            canvas.create_oval(
                motor[0] - radius,
                motor[1] - radius,
                motor[0] + radius,
                motor[1] + radius,
                fill=motor_fill,
                outline=motor_outline,
                width=2,
            )
            canvas.create_oval(
                motor[0] - (radius - 4),
                motor[1] - (radius - 4),
                motor[0] + (radius - 4),
                motor[1] + (radius - 4),
                outline=motor_glow,
                width=1,
            )
            canvas.create_text(
                motor[0],
                motor[1] + 18,
                text=f"M{index + 1}",
                fill="#d8dde3",
                font=("Segoe UI", 8, "bold"),
            )

        # Flight direction / rear-view cue.
        nose = project_point((0.0, 0.82, 0.06))
        tail = project_point((0.0, -0.82, 0.06))
        draw_line(tail, nose, "#ffd54f", width=4)
        if nose is not None:
            canvas.create_text(nose[0] + 14, nose[1] - 10, text="FRONT", fill="#ffd54f", font=("Segoe UI", 9, "bold"))
        if tail is not None:
            canvas.create_text(tail[0] - 18, tail[1] + 10, text="REAR", fill="#ffb74d", font=("Segoe UI", 9, "bold"))

        # Top marker and center reference.
        top_marker = project_point((0.0, 0.0, 0.20))
        if top_marker is not None:
            canvas.create_text(top_marker[0], top_marker[1] - 18, text="TOP", fill="#eaffff", font=("Segoe UI", 11, "bold"))
        if center is not None:
            center_pt = project_point(center)
            if center_pt is not None:
                canvas.create_oval(center_pt[0] - 4, center_pt[1] - 4, center_pt[0] + 4, center_pt[1] + 4, fill="#ffffff", outline="")

    def _on_pose_canvas_resize(self, _event) -> None:
        if self.pose_canvas is None:
            return

        try:
            self._draw_pose_canvas(float(self.yaw_var.get()), float(self.pitch_var.get()), float(self.roll_var.get()))
        except Exception:
            self._draw_pose_canvas(0.0, 0.0, 0.0)

    @staticmethod
    def _rc_ratio(channel_us: int) -> float:
        min_us = 988
        max_us = 2012
        value = max(min_us, min(max_us, int(channel_us)))
        return (value - min_us) / float(max_us - min_us)

    def _draw_rc_canvas(self) -> None:
        if self.rc_canvas is None:
            return

        canvas = self.rc_canvas
        canvas.delete("all")

        width = max(int(canvas.winfo_width()), 10)
        height = max(int(canvas.winfo_height()), 10)
        side = max(min(width, height) - 24, 140)
        left = (width - side) / 2.0
        top = (height - side) / 2.0
        right = left + side
        bottom = top + side

        canvas.create_rectangle(left, top, right, bottom, outline="#31404d", width=2)
        canvas.create_text(
            (left + right) / 2.0,
            top + 16,
            text=f"RC link={self.rc_link_active} frames={self.rc_frame_count}",
            fill="#d8dde3",
            font=("Consolas", 9, "bold"),
        )
        canvas.create_text(
            (left + right) / 2.0,
            top + 34,
            text=f"armed={self.rc_armed} arm_sw={self.rc_arm_switch}",
            fill="#8fb3c6",
            font=("Consolas", 9),
        )

        special_labels = {0: "Roll", 1: "Pitch", 2: "Thr", 3: "Yaw"}
        start_y = top + 52
        rows = 8
        row_h = max((bottom - start_y - 10) / float(rows), 16.0)
        gutter = 8.0
        col_w = ((right - left) - (2 * 12.0) - gutter) / 2.0

        for idx in range(16):
            col = 0 if idx < 8 else 1
            row = idx if idx < 8 else idx - 8
            section_left = left + 12.0 + (col * (col_w + gutter))
            section_right = section_left + col_w
            y = start_y + (row * row_h)
            y_mid = y + (row_h * 0.5)

            us = int(self.rc_channels_us[idx])
            ratio = self._rc_ratio(us)
            label = special_labels.get(idx, f"Ch{idx + 1}")

            label_w = 34.0
            value_w = 42.0
            bar_left = section_left + label_w
            bar_right = section_right - value_w
            bar_width = max(12.0, bar_right - bar_left)
            center_x = bar_left + (bar_width / 2.0)

            label_color = "#e0edf7" if idx < 4 else "#b9c7d3"
            canvas.create_text(section_left, y_mid, text=label, anchor="w", fill=label_color, font=("Segoe UI", 8, "bold"))
            canvas.create_text(section_right, y_mid, text=f"{us}", anchor="e", fill="#9eb5c8", font=("Consolas", 8))
            canvas.create_rectangle(bar_left, y + 4, bar_right, y + row_h - 4, outline="#33414f", width=1)

            if idx == 2:
                x2 = bar_left + (bar_width * ratio)
                canvas.create_rectangle(bar_left + 1, y + 5, x2, y + row_h - 5, fill="#2c7fb8", outline="")
            else:
                canvas.create_line(center_x, y + 4, center_x, y + row_h - 4, fill="#405563", dash=(2, 2))
                x_val = bar_left + (bar_width * ratio)
                fill = "#4fd0f0" if abs(us - 1500) < 25 else "#7bc96f"
                if x_val >= center_x:
                    canvas.create_rectangle(center_x, y + 5, x_val, y + row_h - 5, fill=fill, outline="")
                else:
                    canvas.create_rectangle(x_val, y + 5, center_x, y + row_h - 5, fill=fill, outline="")

    def _on_rc_canvas_resize(self, _event) -> None:
        self._draw_rc_canvas()

    def _append_history(self, key: str, value: float) -> None:
        data = self.chart_data[key]
        data.append(value)
        if len(data) > self.chart_history_len:
            del data[0:len(data) - self.chart_history_len]

    def _append_samples(self, samples) -> None:
        for key, value in samples:
            self._append_history(key, value)

    def _redraw_strip_chart(self, chart_key: str) -> None:
        canvas = self.chart_canvases.get(chart_key)
        config = self.chart_configs.get(chart_key)
        if (canvas is None) or (config is None):
            return

        width = max(int(canvas.winfo_width()), 60)
        height = max(int(canvas.winfo_height()), 40)

        pad_x = 8
        pad_y = 8
        plot_w = max(width - (2 * pad_x), 10)
        plot_h = max(height - (2 * pad_y), 10)

        y_min = config["y_min"]
        y_max = config["y_max"]
        y_span = max(y_max - y_min, 1e-6)

        canvas.delete("all")
        canvas.create_rectangle(pad_x, pad_y, pad_x + plot_w, pad_y + plot_h, outline="#33414f")
        canvas.create_line(pad_x, pad_y + (plot_h / 2), pad_x + plot_w, pad_y + (plot_h / 2), fill="#1c2731", dash=(3, 3))

        for key, _label, color in config["series"]:
            values = self.chart_data[key]
            if len(values) < 2:
                continue

            points = []
            count = len(values)
            for idx, value in enumerate(values):
                x = pad_x + (idx * plot_w / float(max(count - 1, 1)))
                clipped = min(y_max, max(y_min, value))
                ratio = (clipped - y_min) / y_span
                y = pad_y + ((1.0 - ratio) * plot_h)
                points.extend((x, y))

            canvas.create_line(points, fill=color, width=2, smooth=True)

    def _refresh_all_charts(self) -> None:
        self._redraw_strip_chart("accels")
        self._redraw_strip_chart("gyro")
        self._redraw_strip_chart("motors")
        self._redraw_strip_chart("baro")
        self._redraw_strip_chart("mag")
        try:
            self._draw_pose_canvas(float(self.yaw_var.get()), float(self.pitch_var.get()), float(self.roll_var.get()))
        except Exception:
            self._draw_pose_canvas(0.0, 0.0, 0.0)
        self._draw_rc_canvas()

    @staticmethod
    def _bytes_to_ascii(raw: bytes) -> str:
        chars = []
        for value in raw:
            if 32 <= value <= 126:
                chars.append(chr(value))
            elif value in (9, 10, 13):
                chars.append(" ")
            else:
                chars.append(f"\\x{value:02X}")
        return "".join(chars).strip()

    @staticmethod
    def _extract_printable_ascii(text: str) -> str:
        return "".join(ch for ch in text if 32 <= ord(ch) <= 126)

    @staticmethod
    def _decode_for_parser(raw: bytes) -> str:
        decoded = raw.decode("ascii", errors="ignore")
        decoded = decoded.replace("\r", " ").replace("\n", " ")
        return " ".join(decoded.split())

    def _on_scale(self, value: str) -> None:
        pulse = int(float(value))
        self.pulse_var.set(pulse)
        if hasattr(self, "pulse_entry"):
            self.pulse_entry.delete(0, tk.END)
            self.pulse_entry.insert(0, str(pulse))

    def apply_entry_pulse(self) -> bool:
        try:
            pulse = int(self.pulse_entry.get())
        except ValueError:
            messagebox.showerror("Invalid pulse", "Pulse must be an integer in 988..2012.")
            return False

        pulse = max(988, min(2012, pulse))
        self.pulse_var.set(pulse)
        self.scale.set(pulse)
        return True

    def refresh_ports(self) -> None:
        if self.transport_var.get() != "USB":
            return

        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports

        if self.port_var.get() == "COM6":
            return

        if "COM6" in ports:
            self.port_var.set("COM6")
        elif not self.port_var.get() and ports:
            self.port_var.set(ports[0])

    def _connect_usb(self, show_errors: bool = True) -> bool:
        if self.serial_port is not None and self.serial_port.is_open:
            return True

        port = self.port_var.get().strip() or "COM6"
        self.port_var.set(port)

        try:
            self.serial_port = serial.Serial(port, 115200, timeout=0.02, write_timeout=0.2)
            self.connected_var.set(f"Connected USB: {port}")
            self.bridge_ip_var.set("Bridge IP: USB")
            self.bridge_status_lines = []
            self.bridge_status_var.set("Bridge status: USB direct link")
            self.event_var.set(f"Connected to {port}")
            self._reset_pid_health()
            self.pid_received_once = False
            return True
        except Exception as exc:
            self.serial_port = None
            self.connected_var.set(f"Waiting for {port}...")
            if show_errors:
                messagebox.showerror("Connect failed", str(exc))
            return False

    def _connect_wifi(self, show_errors: bool = True) -> bool:
        host = self.host_var.get().strip()
        if not host:
            if show_errors:
                messagebox.showerror("No host", "Enter ESP32 host or IP address.")
            return False

        try:
            port = int(self.tcp_port_var.get().strip())
        except ValueError:
            if show_errors:
                messagebox.showerror("Bad port", "TCP port must be an integer.")
            return False

        if self.tcp_socket is not None:
            return True

        if self.wifi_connecting:
            return False

        # DNS resolution for hostnames like kh7bridge.local can take seconds
        # (Windows mDNS lookups especially) and socket timeout= only bounds the
        # connect() step, not getaddrinfo() - so this always runs off the main
        # thread to avoid freezing the whole GUI (charts included) while waiting.
        self.wifi_connecting = True
        self.connected_var.set(f"Connecting to {host}:{port}...")

        def _do_connect() -> None:
            try:
                sock = socket.create_connection((host, port), timeout=3.0)
                self.wifi_connect_queue.put(("ok", sock, host, port))
            except OSError as exc:
                self.wifi_connect_queue.put(("error", exc, host, port))

        threading.Thread(target=_do_connect, daemon=True).start()
        self._poll_wifi_connect_result(show_errors)
        return False

    def _poll_wifi_connect_result(self, show_errors: bool) -> None:
        try:
            status, payload, host, port = self.wifi_connect_queue.get_nowait()
        except queue.Empty:
            self.root.after(100, lambda: self._poll_wifi_connect_result(show_errors))
            return

        self.wifi_connecting = False

        if status == "error":
            exc = payload
            self.connected_var.set(f"Waiting for {host}:{port}...")
            if show_errors:
                messagebox.showerror("Wi-Fi connect failed", str(exc))
            return

        sock = payload
        # A stale/duplicate result: already connected another way, or the user
        # switched transport away from Wi-Fi while this connect was in flight.
        if self.tcp_socket is not None or self.transport_var.get() != "Wi-Fi":
            try:
                sock.close()
            except Exception:
                pass
            return

        sock.setblocking(False)
        self.tcp_socket = sock
        self.tcp_rx_buffer = bytearray()
        peer_ip, peer_port = sock.getpeername()
        self.connected_var.set(f"Connected Wi-Fi: {host}:{port}")
        self.bridge_ip_var.set(f"Bridge IP: {peer_ip}:{peer_port}")
        self.bridge_status_lines = []
        self.bridge_status_var.set("Bridge status: waiting for bridge lines...")
        self.event_var.set(f"Connected to {host}:{port}")
        self._reset_pid_health()
        self.pid_received_once = False
        self.start_pid_sync()
        self.att_read()

    def connect(self, show_errors: bool = True) -> bool:
        if self.transport_var.get() == "USB":
            connected = self._connect_usb(show_errors)
            if connected:
                self.start_pid_sync()
                self.att_read()
            return connected

        if self.tcp_socket is not None:
            return True

        self._connect_wifi(show_errors)
        return False

    def start_pid_sync(self) -> None:
        self.pid_received_once = False
        self.pid_sync_retries_left = 8
        self.pid_status_var.set("PID: syncing from board...")

        if self.pid_sync_after_id is not None:
            try:
                self.root.after_cancel(self.pid_sync_after_id)
            except Exception:
                pass
            self.pid_sync_after_id = None

        self._pid_sync_tick()

    def _pid_sync_tick(self) -> None:
        self.pid_sync_after_id = None

        if self.pid_received_once:
            self.pid_status_var.set("PID: synced")
            return

        if self.transport_var.get() == "USB":
            connected = self.serial_port is not None and self.serial_port.is_open
        else:
            connected = self.tcp_socket is not None

        if not connected:
            return

        if self.pid_sync_retries_left <= 0:
            self.pid_status_var.set("PID: sync timeout (click Read)")
            return

        self.send_command("PID GET")
        self.pid_sync_retries_left -= 1
        self.pid_sync_after_id = self.root.after(350, self._pid_sync_tick)

    def _cancel_att_verify(self) -> None:
        self.att_verify_target = None
        if self.att_verify_timeout_after_id is not None:
            try:
                self.root.after_cancel(self.att_verify_timeout_after_id)
            except Exception:
                pass
            self.att_verify_timeout_after_id = None

    def _att_verify_timeout(self) -> None:
        self.att_verify_timeout_after_id = None
        if self.att_verify_target is None:
            return
        self.att_verify_target = None
        self.att_status_var.set("ATT verify timeout: no readback from FC")

    def disconnect(self) -> None:
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except Exception:
                pass

        if self.tcp_socket is not None:
            try:
                self.tcp_socket.close()
            except Exception:
                pass

        self.serial_port = None
        self.tcp_socket = None
        self.tcp_rx_buffer = bytearray()
        self.wifi_connecting = False
        self.pid_received_once = False
        self.pid_sync_retries_left = 0
        self._cancel_att_verify()
        if self.pid_sync_after_id is not None:
            try:
                self.root.after_cancel(self.pid_sync_after_id)
            except Exception:
                pass
            self.pid_sync_after_id = None
        self.connected_var.set("Disconnected")
        self.bridge_ip_var.set("Bridge IP: -")
        self.bridge_status_lines = []
        self.bridge_status_var.set("Bridge status: -")
        self._reset_pid_health()

    def _handle_link_loss(self, reason: str) -> None:
        self.event_var.set(f"Link lost: {reason}")
        self.disconnect()

    def _parse_line(self, line: str) -> None:
        if line.startswith("ESCPT["):
            if "[ON" in line:
                self.escpt_status_var.set("ESC pass-through: ON")
            elif "[OFF" in line:
                self.escpt_status_var.set("ESC pass-through: OFF")
            self.event_var.set(line)
            return

        if line.startswith("[BRIDGE]") or line.startswith("[WIFI]") or line.startswith("[TCP]") or line.startswith("[STAT]") or line.startswith("[BOOT]") or line.startswith("[UART2]") or line.startswith("[MDNS]"):
            self._update_bridge_status(line)
            self.event_var.set(line)
            return

        if line.startswith("CMD_RX[") or line.startswith("CMD_RX_AUTO["):
            self.event_var.set(line)
            return

        if line.startswith("CMD_UNKNOWN["):
            self.pid_status_var.set(line)
            self.event_var.set(line)
            return

        m_pid_debug = PID_DEBUG_RE.search(line)
        if m_pid_debug is not None:
            queued = int(m_pid_debug.group(1))
            handled = int(m_pid_debug.group(2))
            pending = int(m_pid_debug.group(3))
            self.pid_debug_var.set(
                f"PID debug: queued={queued} handled={handled} pending={pending}"
            )
            self.pid_status_var.set(
                f"PID DEBUG q={queued} h={handled} p={pending}"
            )
            self.pid_debug_ok = (pending == 0) and (handled >= queued)
            self._update_pid_health()
            self.event_var.set(line)
            return

        m_pid_flash = PID_FLASH_RE.search(line)
        if m_pid_flash is not None:
            header_ok = int(m_pid_flash.group(6))
            crc_ok = int(m_pid_flash.group(7))
            gains_ok = int(m_pid_flash.group(8))
            self.pid_flash_var.set(
                f"PID flash: hdr={header_ok} crc={crc_ok} gains={gains_ok} magic={m_pid_flash.group(2)}"
            )
            self.pid_status_var.set(
                f"PID flash hdr={header_ok} crc={crc_ok} gains={gains_ok}"
            )
            self.pid_flash_ok = (header_ok == 1) and (crc_ok == 1) and (gains_ok == 1)
            self._update_pid_health()
            self.event_var.set(line)
            return

        m_pid = PID_LINE_RE.search(line)
        if m_pid is not None:
            self.pid_received_once = True
            self.pid_vars["roll_kp"].set(f"{float(m_pid.group(2)):.4f}")
            self.pid_vars["roll_ki"].set(f"{float(m_pid.group(3)):.4f}")
            self.pid_vars["roll_kd"].set(f"{float(m_pid.group(4)):.4f}")
            self.pid_vars["roll_kff"].set(f"{float(m_pid.group(5)):.4f}")
            self.pid_vars["pitch_kp"].set(f"{float(m_pid.group(6)):.4f}")
            self.pid_vars["pitch_ki"].set(f"{float(m_pid.group(7)):.4f}")
            self.pid_vars["pitch_kd"].set(f"{float(m_pid.group(8)):.4f}")
            self.pid_vars["pitch_kff"].set(f"{float(m_pid.group(9)):.4f}")
            self.pid_vars["yaw_kp"].set(f"{float(m_pid.group(10)):.4f}")
            self.pid_vars["yaw_ki"].set(f"{float(m_pid.group(11)):.4f}")
            self.pid_vars["yaw_kd"].set(f"{float(m_pid.group(12)):.4f}")
            self.pid_vars["yaw_kff"].set(f"{float(m_pid.group(13)):.4f}")
            self.pid_status_var.set(f"PID source: {m_pid.group(1)}")
            self._update_pid_health()
            return

        m_pid_status = PID_STATUS_RE.search(line)
        if m_pid_status is not None:
            self.pid_status_var.set(f"{m_pid_status.group(1)}: {m_pid_status.group(2)}")
            return

        m_mode = MODE_LINE_RE.search(line)
        if m_mode is not None:
            mode_name = m_mode.group(1)
            mode_ch6 = int(m_mode.group(2))
            self.mode_var.set(f"Mode: {mode_name} (ch6 {mode_ch6})")
            if mode_name in ("RATE", "ATTITUDE", "ALTHOLD"):
                # ALTHOLD reuses the same self-level angle gains as ATTITUDE, so
                # it shares that tuning panel while still showing its own name above.
                tuning_mode = "ATTITUDE" if mode_name == "ALTHOLD" else mode_name
                if tuning_mode != self.current_tuning_mode:
                    self.current_tuning_mode = tuning_mode
                    self._apply_right_panel_visibility()
                    if tuning_mode == "ATTITUDE":
                        self.att_read()
                    else:
                        self.pid_read()
            self.event_var.set(line)
            return

        m_att = ATT_LINE_RE.search(line)
        if m_att is not None:
            roll_kp = float(m_att.group(2))
            pitch_kp = float(m_att.group(3))
            max_angle = float(m_att.group(4))

            self.att_vars["roll_kp"].set(f"{roll_kp:.4f}")
            self.att_vars["pitch_kp"].set(f"{pitch_kp:.4f}")
            self.att_vars["max_angle"].set(f"{max_angle:.2f}")

            if self.att_verify_target is not None:
                exp_roll_kp, exp_pitch_kp, exp_max_angle = self.att_verify_target
                roll_ok = abs(roll_kp - exp_roll_kp) <= 0.0005
                pitch_ok = abs(pitch_kp - exp_pitch_kp) <= 0.0005
                angle_ok = abs(max_angle - exp_max_angle) <= 0.05
                if roll_ok and pitch_ok and angle_ok:
                    self.att_status_var.set(f"ATT saved on FC (verified, src={m_att.group(1)})")
                else:
                    self.att_status_var.set(
                        "ATT verify mismatch: FC values differ from requested"
                    )
                self._cancel_att_verify()
            else:
                self.att_status_var.set(f"ATT source: {m_att.group(1)}")
            return

        m_arm = ARM_LINE_RE.search(line)
        if m_arm is not None:
            armed = int(m_arm.group(1))
            arm_sw = int(m_arm.group(2))
            _low_seen = int(m_arm.group(3))
            thr = int(m_arm.group(4))

            self.rc_armed = armed
            self.rc_arm_switch = arm_sw
            self.rc_channels_us[2] = thr

            self.motor_outputs[1] = int(m_arm.group(5))
            self.motor_outputs[2] = int(m_arm.group(6))
            self.motor_outputs[3] = int(m_arm.group(7))
            self.motor_outputs[4] = int(m_arm.group(8))
            self._append_samples([
                ("motor_1", float(self.motor_outputs[1])),
                ("motor_2", float(self.motor_outputs[2])),
                ("motor_3", float(self.motor_outputs[3])),
                ("motor_4", float(self.motor_outputs[4])),
            ])
            self._refresh_motor_map()
            self._redraw_strip_chart("motors")
            self._draw_rc_canvas()
            return

        m_rx = RX_LINE_RE.search(line)
        if m_rx is not None:
            self.rc_link_active = int(m_rx.group(1))
            self.rc_frame_count = int(m_rx.group(2))
            self.rc_channels_us[0] = int(m_rx.group(3))
            self.rc_channels_us[1] = int(m_rx.group(4))
            self.rc_channels_us[2] = int(m_rx.group(5))
            self.rc_channels_us[3] = int(m_rx.group(6))
            self._draw_rc_canvas()
            return

        m_rx16 = RX16_LINE_RE.search(line)
        if m_rx16 is not None:
            self.rc_link_active = int(m_rx16.group(1))
            self.rc_frame_count = int(m_rx16.group(2))
            for idx in range(16):
                self.rc_channels_us[idx] = int(m_rx16.group(3 + idx))
            self._draw_rc_canvas()
            return

        m_vbat = VBAT_LINE_RE.search(line)
        if m_vbat is not None:
            battery_mv = int(m_vbat.group(1))
            self.battery_var.set(f"Battery: {battery_mv / 1000.0:.2f} V")
            return

        m_baro = BARO_LINE_RE.search(line)
        if m_baro is not None:
            healthy = int(m_baro.group(1))
            alt_m = int(m_baro.group(2)) / 100.0
            vz_mps = int(m_baro.group(3)) / 100.0
            self.baro_alt_var.set(f"{alt_m:.2f}")
            self.baro_vz_var.set(f"{vz_mps:.2f}")
            self.baro_health_var.set("OK" if healthy != 0 else "NOT READY")
            self._append_samples([
                ("baro_alt", alt_m),
                ("baro_vz", vz_mps),
            ])
            self._redraw_strip_chart("baro")
            return

        m_baro_init = BARO_INIT_LINE_RE.search(line)
        if m_baro_init is not None:
            if m_baro_init.group(1) == "OK":
                self.baro_init_var.set(f"Baro init: OK, detected at I2C addr 0x{m_baro_init.group(2)}")
            else:
                self.baro_init_var.set(f"Baro init: FAIL, no ACK/wrong chip (last chip_id read=0x{m_baro_init.group(4)})")
            return

        m_gps = GPS_LINE_RE.search(line)
        if m_gps is not None:
            configured = int(m_gps.group(1))
            healthy = int(m_gps.group(2))
            fix_type = int(m_gps.group(3))
            num_sv = int(m_gps.group(4))
            lat_deg = float(m_gps.group(5))
            lon_deg = float(m_gps.group(6))
            alt_m = float(m_gps.group(7))

            fix_names = {0: "No fix", 1: "Dead reckoning", 2: "2D", 3: "3D", 4: "GNSS+DR", 5: "Time only"}
            self.gps_fix_var.set(fix_names.get(fix_type, str(fix_type)))
            self.gps_sats_var.set(str(num_sv))
            if healthy != 0:
                self.gps_health_var.set("OK")
            elif configured != 0:
                self.gps_health_var.set("NO COMMS")
            else:
                self.gps_health_var.set("NOT READY")
            self.gps_pos_var.set(f"GPS: lat={lat_deg:.7f} lon={lon_deg:.7f} alt={alt_m:.2f} m")
            # This periodic line's "configured" field is the LIVE config state, unlike
            # the one-time GPS_INIT[...] boot message below - refresh it here so a
            # later successful retry (or a retry the GUI simply missed) isn't left
            # showing a stale FAIL from the very first boot-time attempt.
            if configured != 0:
                self.gps_init_var.set("GPS init: OK (UBX config ACKed)")
            return

        m_gps_init = GPS_INIT_LINE_RE.search(line)
        if m_gps_init is not None:
            if m_gps_init.group(1) == "OK":
                self.gps_init_var.set("GPS init: OK (UBX config ACKed)")
            else:
                prt_ack = m_gps_init.group(3)
                msg_ack = m_gps_init.group(4)
                self.gps_init_var.set(
                    f"GPS init: FAIL (prt_ack={prt_ack} msg_ack={msg_ack})"
                )
            return

        m_mag = MAG_LINE_RE.search(line)
        if m_mag is not None:
            healthy = int(m_mag.group(1))
            heading_deg = int(m_mag.group(2)) / 10.0
            self.mag_health_var.set("OK" if healthy != 0 else "NOT READY")
            self.mag_heading_var.set(f"{heading_deg:.1f}")
            self._append_samples([("mag_heading", heading_deg)])
            self._redraw_strip_chart("mag")
            return

        m_mag_init = MAG_INIT_LINE_RE.search(line)
        if m_mag_init is not None:
            if m_mag_init.group(1) == "OK":
                self.mag_init_var.set(f"Mag init: OK, chip_id=0x{m_mag_init.group(2)}")
            else:
                self.mag_init_var.set(f"Mag init: FAIL, no ACK/wrong chip (last chip_id read=0x{m_mag_init.group(2)})")
            return

        if MAG_CAL_STARTED_RE.search(line) is not None:
            self.mag_cal_status_var.set("Rotating... slowly spin 360 deg flat, then click Stop & Save")
            return

        m_mag_cal_ok = MAG_CAL_OK_RE.search(line)
        if m_mag_cal_ok is not None:
            ox, oy, sx, sy = m_mag_cal_ok.groups()
            self.mag_cal_status_var.set(f"Calibrated OK (offset={ox},{oy} scale={sx},{sy})")
            self.mag_cal_active = False
            self.mag_cal_button.configure(text="Calibrate")
            return

        m_mag_cal_fail = MAG_CAL_FAIL_RE.search(line)
        if m_mag_cal_fail is not None:
            self.mag_cal_status_var.set(f"Calibration FAILED{m_mag_cal_fail.group(1)}")
            self.mag_cal_active = False
            self.mag_cal_button.configure(text="Calibrate")
            return

        m_imu = IMU_LINE_RE.search(line)
        if m_imu is not None:
            ax = int(m_imu.group(1)) / 100.0
            ay = int(m_imu.group(2)) / 100.0
            az = int(m_imu.group(3)) / 100.0
            gx = int(m_imu.group(4)) / 10.0
            gy = int(m_imu.group(5)) / 10.0
            gz = int(m_imu.group(6)) / 10.0
            pitch = int(m_imu.group(7)) / 10.0
            roll = int(m_imu.group(8)) / 10.0
            yaw = int(m_imu.group(9)) / 10.0

            self.ax_var.set(f"{ax:.2f}")
            self.ay_var.set(f"{ay:.2f}")
            self.az_var.set(f"{az:.2f}")
            self.gx_var.set(f"{gx:.1f}")
            self.gy_var.set(f"{gy:.1f}")
            self.gz_var.set(f"{gz:.1f}")
            self.pitch_var.set(f"{pitch:.1f}")
            self.roll_var.set(f"{roll:.1f}")
            self.yaw_var.set(f"{yaw:.1f}")
            self._draw_pose_canvas(yaw, pitch, roll)
            self._append_samples([
                ("accel_x", ax),
                ("accel_y", ay),
                ("accel_z", az),
                ("gyro_x", gx),
                ("gyro_y", gy),
                ("gyro_z", gz),
            ])
            self._redraw_strip_chart("accels")
            self._redraw_strip_chart("gyro")
            return

    def _consume_raw_line(self, raw: bytes) -> None:
        line = self._bytes_to_ascii(raw)
        if line:
            self._append_log_line(line)
            parsed_line = self._decode_for_parser(raw)
            self._parse_line(parsed_line)

    def _update_bridge_status(self, line: str) -> None:
        self.bridge_status_lines.append(line)
        if len(self.bridge_status_lines) > 3:
            del self.bridge_status_lines[0:len(self.bridge_status_lines) - 3]
        self.bridge_status_var.set("Bridge status:\n" + "\n".join(self.bridge_status_lines))

    def _reset_pid_health(self) -> None:
        self.pid_received_once = False
        self.pid_debug_ok = None
        self.pid_flash_ok = None
        self.pid_health_var.set("PID link: waiting for checks")
        if self.pid_health_label is not None:
            self.pid_health_label.configure(fg="#9aa6b2")

    def _update_pid_health(self) -> None:
        if not self.pid_received_once:
            self.pid_health_var.set("PID link: waiting for PID read")
            if self.pid_health_label is not None:
                self.pid_health_label.configure(fg="#c78f2b")
            return

        if (self.pid_debug_ok is True) and (self.pid_flash_ok is True):
            self.pid_health_var.set("PID link: healthy")
            if self.pid_health_label is not None:
                self.pid_health_label.configure(fg="#1f9d43")
            return

        if (self.pid_debug_ok is False) or (self.pid_flash_ok is False):
            self.pid_health_var.set("PID link: check failed")
            if self.pid_health_label is not None:
                self.pid_health_label.configure(fg="#c4362b")
            return

        self.pid_health_var.set("PID link: partial checks")
        if self.pid_health_label is not None:
            self.pid_health_label.configure(fg="#c78f2b")

    def _poll_usb(self) -> None:
        if self.serial_port is not None and self.serial_port.is_open:
            try:
                for _ in range(40):
                    raw = self.serial_port.readline()
                    if not raw:
                        break
                    self._consume_raw_line(raw)
            except (SerialException, OSError) as exc:
                self._handle_link_loss(str(exc))

    def _poll_wifi(self) -> None:
        if self.tcp_socket is None:
            return

        try:
            for _ in range(40):
                chunk = self.tcp_socket.recv(1024)
                if not chunk:
                    self._handle_link_loss("Remote closed")
                    return

                self.tcp_rx_buffer.extend(chunk)

                while True:
                    newline_index = self.tcp_rx_buffer.find(b"\n")
                    if newline_index < 0:
                        break

                    raw_line = bytes(self.tcp_rx_buffer[:newline_index + 1])
                    del self.tcp_rx_buffer[:newline_index + 1]
                    self._consume_raw_line(raw_line)

                if len(chunk) < 1024:
                    break
        except BlockingIOError:
            return
        except OSError as exc:
            self._handle_link_loss(str(exc))

    def _poll_io(self) -> None:
        if self.transport_var.get() == "USB":
            self._poll_usb()
        else:
            self._poll_wifi()

        self.root.after(50, self._poll_io)

    def _auto_connect_tick(self) -> None:
        if self.transport_var.get() == "USB":
            if self.serial_port is None or not self.serial_port.is_open:
                self.refresh_ports()
                self.connect(show_errors=False)
        else:
            if self.tcp_socket is None:
                self.connect(show_errors=False)

        self.root.after(1500, self._auto_connect_tick)

    def send_command(self, cmd: str) -> None:
        data = (cmd + "\n").encode("ascii", errors="ignore")

        if self.transport_var.get() == "USB":
            if self.serial_port is None or not self.serial_port.is_open:
                messagebox.showerror("Not connected", "Waiting for USB CDC device. Current default is COM6.")
                return

            try:
                self.serial_port.write(data)
                self.serial_port.flush()
                self._append_log_line(f"TX[USB] {cmd}")
                self.event_var.set(f"Sent: {cmd}")
            except (SerialException, OSError) as exc:
                self._handle_link_loss(str(exc))
                messagebox.showerror("Serial write failed", str(exc))
            return

        if self.tcp_socket is None:
            messagebox.showerror("Not connected", "Connect to ESP32 Wi-Fi bridge first.")
            return

        try:
            self.tcp_socket.sendall(data)
            self._append_log_line(f"TX[Wi-Fi] {cmd}")
            self.event_var.set(f"Sent: {cmd}")
        except OSError as exc:
            self._handle_link_loss(str(exc))
            messagebox.showerror("TCP send failed", str(exc))

    def escpt_on(self) -> None:
        if self.transport_var.get() != "USB":
            messagebox.showerror("USB required", "ESC passthrough mode requires USB transport.")
            return
        self.send_command("ESCPT ON")
        self.escpt_status_var.set("ESC pass-through: ON (requested)")

    def escpt_off(self) -> None:
        if self.transport_var.get() == "USB" and self.serial_port is not None and self.serial_port.is_open:
            try:
                # Passthrough mode bypasses command parsing, so use the raw escape sequence.
                self.serial_port.write(b"+++ESCPTOFF+++")
                self.serial_port.flush()
                self._append_log_line("TX[USB RAW] +++ESCPTOFF+++")
                self.event_var.set("Sent ESC passthrough OFF escape")
            except (SerialException, OSError) as exc:
                self._handle_link_loss(str(exc))
                messagebox.showerror("Serial write failed", str(exc))
                return
        else:
            self.send_command("ESCPT OFF")

        self.escpt_status_var.set("ESC pass-through: OFF (requested)")

    def _pid_values_from_ui(self):
        try:
            values = {
                "roll_kp": float(self.pid_vars["roll_kp"].get()),
                "roll_ki": float(self.pid_vars["roll_ki"].get()),
                "roll_kd": float(self.pid_vars["roll_kd"].get()),
                "roll_kff": float(self.pid_vars["roll_kff"].get()),
                "pitch_kp": float(self.pid_vars["pitch_kp"].get()),
                "pitch_ki": float(self.pid_vars["pitch_ki"].get()),
                "pitch_kd": float(self.pid_vars["pitch_kd"].get()),
                "pitch_kff": float(self.pid_vars["pitch_kff"].get()),
                "yaw_kp": float(self.pid_vars["yaw_kp"].get()),
                "yaw_ki": float(self.pid_vars["yaw_ki"].get()),
                "yaw_kd": float(self.pid_vars["yaw_kd"].get()),
                "yaw_kff": float(self.pid_vars["yaw_kff"].get()),
            }
        except ValueError:
            messagebox.showerror("Invalid PID", "All PID fields must be numeric values.")
            return None

        for key, value in values.items():
            min_value, max_value = PID_LIMITS[key]
            if not (min_value <= value <= max_value):
                messagebox.showerror(
                    "PID Out of Range",
                    f"{key} must be in range [{min_value}, {max_value}].",
                )
                return None

        return [
            values["roll_kp"],
            values["roll_ki"],
            values["roll_kd"],
            values["roll_kff"],
            values["pitch_kp"],
            values["pitch_ki"],
            values["pitch_kd"],
            values["pitch_kff"],
            values["yaw_kp"],
            values["yaw_ki"],
            values["yaw_kd"],
            values["yaw_kff"],
        ]

    def pid_read(self) -> None:
        self.send_command("PID GET")

    def att_read(self) -> None:
        self.send_command("ATT GET")

    def mag_cal_toggle(self) -> None:
        if not self.mag_cal_active:
            self.send_command("MAG CAL START")
            self.mag_cal_active = True
            self.mag_cal_button.configure(text="Stop & Save")
            self.mag_cal_status_var.set("Cal starting...")
        else:
            self.send_command("MAG CAL STOP")
            self.mag_cal_active = False
            self.mag_cal_button.configure(text="Calibrate")
            self.mag_cal_status_var.set("Stopping, computing...")

    def pid_apply(self) -> None:
        values = self._pid_values_from_ui()
        if values is None:
            return

        cmd = "PID SET " + " ".join(f"{v:.6f}" for v in values)
        self.send_command(cmd)

    def pid_save(self) -> None:
        values = self._pid_values_from_ui()
        if values is None:
            return

        # Firmware PID SET path applies and saves in one operation.
        cmd = "PID SET " + " ".join(f"{v:.6f}" for v in values)
        self.send_command(cmd)

    def pid_load(self) -> None:
        self.send_command("PID LOAD")

    def pid_defaults(self) -> None:
        self.send_command("PID DEFAULT")

    def att_defaults(self) -> None:
        self.send_command("ATT DEFAULT")

    def att_apply(self) -> None:
        try:
            roll_kp = float(self.att_vars["roll_kp"].get())
            pitch_kp = float(self.att_vars["pitch_kp"].get())
            max_angle = float(self.att_vars["max_angle"].get())
        except ValueError:
            messagebox.showerror("Invalid Attitude", "Attitude fields must be numeric values.")
            return

        if not (0.2 <= roll_kp <= 25.0 and 0.2 <= pitch_kp <= 25.0 and 5.0 <= max_angle <= 70.0):
            messagebox.showerror(
                "Attitude Out of Range",
                "Roll/Pitch Kp must be 0.2..25.0 and Max Angle must be 5..70 deg.",
            )
            return

        self._cancel_att_verify()
        self.att_verify_target = (roll_kp, pitch_kp, max_angle)
        self.att_status_var.set("ATT apply sent, verifying on FC...")
        self.send_command(f"ATT SET {roll_kp:.4f} {pitch_kp:.4f} {max_angle:.2f}")
        self.send_command("ATT GET")
        self.att_verify_timeout_after_id = self.root.after(1200, self._att_verify_timeout)

    def pid_debug(self) -> None:
        self.send_command("PID DEBUG")

    def run_selected(self) -> None:
        if not self.apply_entry_pulse():
            return
        motor = self.motor_var.get()
        pulse = self.pulse_var.get()
        self.send_command(f"MTEST {motor} {pulse}")

    def stop(self) -> None:
        self.send_command("MTEST OFF")

    def shutdown(self) -> None:
        self._save_ui_state()
        self.bb_pause()
        self.disconnect()


def main() -> None:
    root = tk.Tk()
    app = Kh7GroundGui(root)

    def on_close() -> None:
        app.shutdown()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
