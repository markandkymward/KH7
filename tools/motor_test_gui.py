import re
import socket
import tkinter as tk
from tkinter import messagebox, ttk

import serial
import serial.tools.list_ports
from serial import SerialException


ARM_LINE_RE = re.compile(
    r"ARM\[a=(\d+) sw=(\d+) lowSeen=(\d+) thr=(\d+) m\]=\[(\d+) (\d+) (\d+) (\d+)\]"
)
IMU_LINE_RE = re.compile(
    r"IMU\[x100/x10\]=\[\s*(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s*\]"
)
PID_LINE_RE = re.compile(
    r"PID\[src=([^\]]+)\]=\[R\s+([-+]?\d*\.?\d+)\s+([-+]?\d*\.?\d+)\s+([-+]?\d*\.?\d+)\s+P\s+([-+]?\d*\.?\d+)\s+([-+]?\d*\.?\d+)\s+([-+]?\d*\.?\d+)\s+Y\s+([-+]?\d*\.?\d+)\s+([-+]?\d*\.?\d+)\s+([-+]?\d*\.?\d+)\]"
)
PID_STATUS_RE = re.compile(r"PID_(SET|SAVE|LOAD)\[([^\]]+)\]")


class Kh7GroundGui:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("KH7 USB Ground Station")
        self.root.geometry("1080x780")

        self.serial_port = None
        self.tcp_socket = None
        self.tcp_rx_buffer = bytearray()
        self.motor_canvas = None
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
        }

        self.transport_var = tk.StringVar(value="USB")
        self.port_var = tk.StringVar(value="COM6")
        self.host_var = tk.StringVar(value="kh7bridge.local")
        self.tcp_port_var = tk.StringVar(value="3333")
        self.connected_var = tk.StringVar(value="Disconnected")
        self.event_var = tk.StringVar(value="Ready")
        self.bridge_ip_var = tk.StringVar(value="Bridge IP: -")
        self.arm_state_var = tk.StringVar(value="Armed: 0  Arm switch: 0  Low seen: 0")
        self.throttle_var = tk.StringVar(value="Throttle us: 988")
        self.pid_status_var = tk.StringVar(value="PID: idle")

        self.ax_var = tk.StringVar(value="0.00")
        self.ay_var = tk.StringVar(value="0.00")
        self.az_var = tk.StringVar(value="0.00")
        self.gx_var = tk.StringVar(value="0.0")
        self.gy_var = tk.StringVar(value="0.0")
        self.gz_var = tk.StringVar(value="0.0")
        self.pitch_var = tk.StringVar(value="0.0")
        self.roll_var = tk.StringVar(value="0.0")
        self.yaw_var = tk.StringVar(value="0.0")

        self.pid_vars = {
            "roll_kp": tk.StringVar(value="0.9000"),
            "roll_ki": tk.StringVar(value="0.0000"),
            "roll_kd": tk.StringVar(value="0.0000"),
            "pitch_kp": tk.StringVar(value="0.9000"),
            "pitch_ki": tk.StringVar(value="0.0000"),
            "pitch_kd": tk.StringVar(value="0.0000"),
            "yaw_kp": tk.StringVar(value="0.8000"),
            "yaw_ki": tk.StringVar(value="0.0000"),
            "yaw_kd": tk.StringVar(value="0.0000"),
        }

        self.motor_var = tk.IntVar(value=1)
        self.pulse_var = tk.IntVar(value=1100)
        self.motor_outputs = {1: 988, 2: 988, 3: 988, 4: 988}
        self.motor_test_visible = True
        self.pid_panel_visible = True
        self.toggle_motor_test_button = None
        self.toggle_pid_button = None
        self.right_panel = None
        self.motor_test_box = None
        self.pid_box = None

        self._build_ui()
        self.refresh_ports()
        self.root.after(300, self._auto_connect_tick)
        self.root.after(50, self._poll_io)

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
        self.toggle_motor_test_button = ttk.Button(top, text="Hide Motor Test", command=self.toggle_motor_test_panel)
        self.toggle_motor_test_button.pack(side=tk.LEFT, padx=(8, 0))
        self.toggle_pid_button = ttk.Button(top, text="Hide PID", command=self.toggle_pid_panel)
        self.toggle_pid_button.pack(side=tk.LEFT, padx=(6, 0))

        ttk.Label(top, textvariable=self.connected_var).pack(side=tk.LEFT, padx=(12, 0))
        ttk.Label(top, textvariable=self.bridge_ip_var).pack(side=tk.LEFT, padx=(10, 0))
        ttk.Label(top, textvariable=self.event_var).pack(side=tk.RIGHT)

        body = ttk.Frame(self.root, padding=(10, 0, 10, 10))
        body.pack(fill=tk.BOTH, expand=True)

        telemetry = ttk.LabelFrame(body, text="Board Telemetry")
        telemetry.pack(fill=tk.X, pady=(0, 10))

        self._metric_row(telemetry, 0, "Accel X (g)", self.ax_var, "Rate X (dps)", self.gx_var, "Pitch (deg)", self.pitch_var)
        self._metric_row(telemetry, 1, "Accel Y (g)", self.ay_var, "Rate Y (dps)", self.gy_var, "Roll (deg)", self.roll_var)
        self._metric_row(telemetry, 2, "Accel Z (g)", self.az_var, "Rate Z (dps)", self.gz_var, "Yaw (deg)", self.yaw_var)

        ttk.Label(telemetry, textvariable=self.arm_state_var).grid(row=3, column=0, columnspan=6, sticky="w", padx=10, pady=(6, 0))
        ttk.Label(telemetry, textvariable=self.throttle_var).grid(row=4, column=0, columnspan=6, sticky="w", padx=10, pady=(0, 8))

        mid = ttk.Frame(body)
        mid.pack(fill=tk.X, pady=(0, 10))

        map_box = ttk.LabelFrame(mid, text="Logical Motor Map and Commanded Output")
        map_box.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 8))

        self.motor_canvas = tk.Canvas(map_box, width=460, height=300, bg="#101214", highlightthickness=0)
        self.motor_canvas.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)
        self._init_motor_map()

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

        self._add_pid_row(self.pid_box, 1, "Roll", "roll")
        self._add_pid_row(self.pid_box, 2, "Pitch", "pitch")
        self._add_pid_row(self.pid_box, 3, "Yaw", "yaw")

        pid_btns = ttk.Frame(self.pid_box)
        pid_btns.grid(row=4, column=0, columnspan=4, sticky="ew", padx=6, pady=(6, 6))

        ttk.Button(pid_btns, text="Read", command=self.pid_read).pack(side=tk.LEFT, padx=(0, 4))
        ttk.Button(pid_btns, text="Apply", command=self.pid_apply).pack(side=tk.LEFT, padx=4)
        ttk.Button(pid_btns, text="Save", command=self.pid_save).pack(side=tk.LEFT, padx=4)
        ttk.Button(pid_btns, text="Load", command=self.pid_load).pack(side=tk.LEFT, padx=4)
        ttk.Button(pid_btns, text="Defaults", command=self.pid_defaults).pack(side=tk.LEFT, padx=4)

        ttk.Label(self.pid_box, textvariable=self.pid_status_var).grid(
            row=5, column=0, columnspan=4, sticky="w", padx=6, pady=(0, 6)
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

        self.scale.set(self.pulse_var.get())
        self._on_transport_changed()

    def _add_pid_row(self, parent: ttk.LabelFrame, row: int, label: str, prefix: str) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", padx=(6, 4), pady=2)
        ttk.Entry(parent, width=7, textvariable=self.pid_vars[f"{prefix}_kp"]).grid(row=row, column=1, sticky="w", padx=4, pady=2)
        ttk.Entry(parent, width=7, textvariable=self.pid_vars[f"{prefix}_ki"]).grid(row=row, column=2, sticky="w", padx=4, pady=2)
        ttk.Entry(parent, width=7, textvariable=self.pid_vars[f"{prefix}_kd"]).grid(row=row, column=3, sticky="w", padx=4, pady=2)

    def _apply_right_panel_visibility(self) -> None:
        if self.motor_test_box is None or self.pid_box is None:
            return

        self.motor_test_box.pack_forget()
        self.pid_box.pack_forget()

        if self.motor_test_visible:
            self.motor_test_box.pack(side=tk.TOP, fill=tk.X, pady=(0, 8))

        if self.pid_panel_visible:
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

    def _init_motor_map(self) -> None:
        self.motor_canvas.create_rectangle(140, 95, 320, 205, outline="#39586b", width=2)
        self.motor_canvas.create_text(230, 150, text="Board", fill="#89b6d1", font=("Segoe UI", 12, "bold"))

        positions = {
            1: (100, 70),   # LF
            2: (360, 70),   # RF
            3: (360, 235),  # RA
            4: (100, 235),  # LA
        }
        labels = {
            1: "S1 LF",
            2: "S2 RF",
            3: "S3 RA",
            4: "S4 LA",
        }

        for idx in range(1, 5):
            x, y = positions[idx]
            bg = self.motor_canvas.create_oval(
                x - 36,
                y - 36,
                x + 36,
                y + 36,
                fill="#1a1f23",
                outline="#6b7580",
                width=2,
            )
            arc = self.motor_canvas.create_arc(
                x - 34,
                y - 34,
                x + 34,
                y + 34,
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
        for idx in range(1, 5):
            pulse = self.motor_outputs.get(idx, 988)
            shape = self.motor_shapes[idx]
            ratio = self._motor_ratio(pulse)
            extent = -360.0 * ratio
            self.motor_canvas.itemconfigure(shape["arc"], extent=extent, fill=self._motor_green_color(pulse))
            self.motor_canvas.itemconfigure(shape["cmd"], text=f"cmd {pulse} us")

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
            self.event_var.set(f"Connected to {port}")
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

        try:
            sock = socket.create_connection((host, port), timeout=1.0)
            sock.setblocking(False)
            self.tcp_socket = sock
            self.tcp_rx_buffer = bytearray()
            peer_ip, peer_port = sock.getpeername()
            self.connected_var.set(f"Connected Wi-Fi: {host}:{port}")
            self.bridge_ip_var.set(f"Bridge IP: {peer_ip}:{peer_port}")
            self.event_var.set(f"Connected to {host}:{port}")
            return True
        except OSError as exc:
            self.connected_var.set(f"Waiting for {host}:{port}...")
            if show_errors:
                messagebox.showerror("Wi-Fi connect failed", str(exc))
            return False

    def connect(self, show_errors: bool = True) -> bool:
        connected = False
        if self.transport_var.get() == "USB":
            connected = self._connect_usb(show_errors)
        else:
            connected = self._connect_wifi(show_errors)

        if connected:
            self.pid_read()

        return connected

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
        self.connected_var.set("Disconnected")
        self.bridge_ip_var.set("Bridge IP: -")

    def _handle_link_loss(self, reason: str) -> None:
        self.event_var.set(f"Link lost: {reason}")
        self.disconnect()

    def _parse_line(self, line: str) -> None:
        m_pid = PID_LINE_RE.search(line)
        if m_pid is not None:
            self.pid_vars["roll_kp"].set(f"{float(m_pid.group(2)):.4f}")
            self.pid_vars["roll_ki"].set(f"{float(m_pid.group(3)):.4f}")
            self.pid_vars["roll_kd"].set(f"{float(m_pid.group(4)):.4f}")
            self.pid_vars["pitch_kp"].set(f"{float(m_pid.group(5)):.4f}")
            self.pid_vars["pitch_ki"].set(f"{float(m_pid.group(6)):.4f}")
            self.pid_vars["pitch_kd"].set(f"{float(m_pid.group(7)):.4f}")
            self.pid_vars["yaw_kp"].set(f"{float(m_pid.group(8)):.4f}")
            self.pid_vars["yaw_ki"].set(f"{float(m_pid.group(9)):.4f}")
            self.pid_vars["yaw_kd"].set(f"{float(m_pid.group(10)):.4f}")
            self.pid_status_var.set(f"PID source: {m_pid.group(1)}")
            return

        m_pid_status = PID_STATUS_RE.search(line)
        if m_pid_status is not None:
            self.pid_status_var.set(f"{m_pid_status.group(1)}: {m_pid_status.group(2)}")
            return

        m_arm = ARM_LINE_RE.search(line)
        if m_arm is not None:
            armed = int(m_arm.group(1))
            arm_sw = int(m_arm.group(2))
            low_seen = int(m_arm.group(3))
            thr = int(m_arm.group(4))

            self.arm_state_var.set(f"Armed: {armed}  Arm switch: {arm_sw}  Low seen: {low_seen}")
            self.throttle_var.set(f"Throttle us: {thr}")

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
            parsed_line = self._extract_printable_ascii(line)
            self._parse_line(parsed_line)

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
            self.event_var.set(f"Sent: {cmd}")
        except OSError as exc:
            self._handle_link_loss(str(exc))
            messagebox.showerror("TCP send failed", str(exc))

    def _pid_values_from_ui(self):
        try:
            return [
                float(self.pid_vars["roll_kp"].get()),
                float(self.pid_vars["roll_ki"].get()),
                float(self.pid_vars["roll_kd"].get()),
                float(self.pid_vars["pitch_kp"].get()),
                float(self.pid_vars["pitch_ki"].get()),
                float(self.pid_vars["pitch_kd"].get()),
                float(self.pid_vars["yaw_kp"].get()),
                float(self.pid_vars["yaw_ki"].get()),
                float(self.pid_vars["yaw_kd"].get()),
            ]
        except ValueError:
            messagebox.showerror("Invalid PID", "All PID fields must be numeric values.")
            return None

    def pid_read(self) -> None:
        self.send_command("PID GET")

    def pid_apply(self) -> None:
        values = self._pid_values_from_ui()
        if values is None:
            return

        cmd = "PID SET " + " ".join(f"{v:.6f}" for v in values)
        self.send_command(cmd)

    def pid_save(self) -> None:
        self.send_command("PID SAVE")

    def pid_load(self) -> None:
        self.send_command("PID LOAD")

    def pid_defaults(self) -> None:
        self.send_command("PID DEFAULT")

    def run_selected(self) -> None:
        if not self.apply_entry_pulse():
            return
        motor = self.motor_var.get()
        pulse = self.pulse_var.get()
        self.send_command(f"MTEST {motor} {pulse}")

    def stop(self) -> None:
        self.send_command("MTEST OFF")


def main() -> None:
    root = tk.Tk()
    app = Kh7GroundGui(root)

    def on_close() -> None:
        app.disconnect()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
