import queue
import re
import threading
import tkinter as tk
from tkinter import ttk

import numpy as np
import serial
import serial.tools.list_ports
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


ANGLE_LINE_REGEX = re.compile(
    r"ANGLES\[p r y\]=\[\s*([-+]?\d+(?:\.\d+)?)\s+([-+]?\d+(?:\.\d+)?)\s+([-+]?\d+(?:\.\d+)?)\s*\]"
)

FRAME_CONVENTION_TEXT = "Convention: Body FRD (X=Forward, Y=Right, Z=Down) / World NED (X=North, Y=East, Z=Down)"


class SerialReader(threading.Thread):
    def __init__(self, port, baudrate, output_queue, stop_event):
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.output_queue = output_queue
        self.stop_event = stop_event
        self.serial_conn = None

    def run(self):
        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.output_queue.put(("status", f"Connected to {self.port} @ {self.baudrate}"))

            while not self.stop_event.is_set():
                raw = self.serial_conn.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="ignore").strip()
                match = ANGLE_LINE_REGEX.search(line)
                if not match:
                    continue

                pitch = float(match.group(1))
                roll = float(match.group(2))
                yaw = float(match.group(3))
                self.output_queue.put(("angles", (pitch, roll, yaw)))
        except Exception as exc:
            self.output_queue.put(("status", f"Serial error: {exc}"))
        finally:
            if self.serial_conn is not None and self.serial_conn.is_open:
                self.serial_conn.close()
                self.output_queue.put(("status", "Serial connection closed"))


class BoardViewerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("KH7 IMU 3D Board Viewer")
        self.root.geometry("1100x760")

        self.data_queue = queue.Queue()
        self.stop_event = threading.Event()
        self.reader_thread = None

        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0

        self._build_ui()
        self._build_plot()
        self._refresh_ports()
        self.root.after(250, self._auto_connect_startup)
        self._process_queue()

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self):
        controls = ttk.Frame(self.root, padding=8)
        controls.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(controls, text="COM Port:").grid(row=0, column=0, sticky="w")
        self.port_var = tk.StringVar(value="COM6")
        self.port_combo = ttk.Combobox(controls, textvariable=self.port_var, width=18, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=(6, 12), sticky="w")

        ttk.Label(controls, text="Baud:").grid(row=0, column=2, sticky="w")
        self.baud_var = tk.StringVar(value="115200")
        self.baud_combo = ttk.Combobox(
            controls,
            textvariable=self.baud_var,
            width=10,
            values=["115200", "230400", "460800", "921600"],
            state="readonly",
        )
        self.baud_combo.grid(row=0, column=3, padx=(6, 12), sticky="w")

        ttk.Button(controls, text="Refresh Ports", command=self._refresh_ports).grid(row=0, column=4, padx=4)
        self.connect_btn = ttk.Button(controls, text="Connect", command=self._toggle_connection)
        self.connect_btn.grid(row=0, column=5, padx=4)

        self.status_var = tk.StringVar(value="Disconnected")
        self.angles_var = tk.StringVar(value="Pitch: 0.0   Roll: 0.0   Yaw: 0.0")
        self.convention_var = tk.StringVar(value=FRAME_CONVENTION_TEXT)

        ttk.Label(self.root, textvariable=self.status_var, padding=(8, 4)).pack(anchor="w")
        ttk.Label(self.root, textvariable=self.angles_var, padding=(8, 0)).pack(anchor="w")
        ttk.Label(self.root, textvariable=self.convention_var, padding=(8, 0)).pack(anchor="w")

    def _build_plot(self):
        self.figure = Figure(figsize=(10, 6), dpi=100)
        self.ax = self.figure.add_subplot(111, projection="3d")
        self.canvas = FigureCanvasTkAgg(self.figure, master=self.root)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.board_length = 2.2
        self.board_width = 1.4
        self.board_height = 0.2
        self.sphere_radius = 1.6

        self._init_scene()
        self._draw_board(0.0, 0.0, 0.0)

    def _init_scene(self):
        self.ax.clear()
        x_color = "#d62728"
        y_color = "#2ca02c"
        z_color = "#1f77b4"

        self.ax.set_box_aspect((2.2, 1.6, 1.2))
        self.ax.set_xlim(-2.0, 2.0)
        self.ax.set_ylim(-2.0, 2.0)
        self.ax.set_zlim(-1.5, 1.5)
        self.ax.set_xlabel("Plot X (East)")
        self.ax.set_ylabel("Plot Y (North)")
        self.ax.set_zlabel("Plot Z (Up)")
        self.ax.set_title("KH7 Body Orientation (FRD in NED, displayed as ENU view)")
        self.ax.view_init(elev=22.0, azim=-58.0)

        self.ax.xaxis.label.set_color(x_color)
        self.ax.yaxis.label.set_color(y_color)
        self.ax.zaxis.label.set_color(z_color)
        self.ax.tick_params(axis="x", colors=x_color)
        self.ax.tick_params(axis="y", colors=y_color)
        self.ax.tick_params(axis="z", colors=z_color)
        self.ax.xaxis.line.set_color(x_color)
        self.ax.yaxis.line.set_color(y_color)
        self.ax.zaxis.line.set_color(z_color)
        self.ax.xaxis.line.set_linewidth(2.0)
        self.ax.yaxis.line.set_linewidth(2.0)
        self.ax.zaxis.line.set_linewidth(2.0)

        self._draw_spherical_reference()

    @staticmethod
    def _ned_to_plot(points):
        # NED -> plot conversion: [North, East, Down] -> [East, North, Up]
        arr = np.asarray(points)
        if arr.ndim == 1:
            return np.array([arr[1], arr[0], -arr[2]])
        return np.column_stack((arr[:, 1], arr[:, 0], -arr[:, 2]))

    def _draw_spherical_reference(self):
        u = np.linspace(0.0, 2.0 * np.pi, 40)
        v = np.linspace(0.0, np.pi, 20)
        x = self.sphere_radius * np.outer(np.cos(u), np.sin(v))
        y = self.sphere_radius * np.outer(np.sin(u), np.sin(v))
        z = self.sphere_radius * np.outer(np.ones_like(u), np.cos(v))

        self.ax.plot_wireframe(x, y, z, rstride=3, cstride=3, color="#8a8a8a", linewidth=0.35, alpha=0.35)

        circle = np.linspace(0.0, 2.0 * np.pi, 200)
        xe = self.sphere_radius * np.cos(circle)
        ye = self.sphere_radius * np.sin(circle)
        ze = np.zeros_like(circle)
        self.ax.plot(xe, ye, ze, color="#666666", linewidth=1.0, linestyle="--", alpha=0.8)

        xp = self.sphere_radius * np.cos(circle)
        yp = np.zeros_like(circle)
        zp = self.sphere_radius * np.sin(circle)
        self.ax.plot(xp, yp, zp, color="#666666", linewidth=1.0, linestyle="--", alpha=0.8)

        self.ax.text(self.sphere_radius + 0.12, 0.0, 0.0, "phi = 0", color="#444444")
        self.ax.text(0.0, 0.0, self.sphere_radius + 0.12, "theta = 0", color="#444444")
        self.ax.text(0.0, 0.0, 0.0, "r", color="#444444")

    @staticmethod
    def _rotation_matrix(roll_deg, pitch_deg, yaw_deg):
        # Incoming firmware angles are interpreted as FRD body attitude in NED frame.
        # This is a direct display mapping (no angle remapping): R = Rz(yaw) * Ry(pitch) * Rx(roll).
        roll = np.radians(roll_deg)
        pitch = np.radians(pitch_deg)
        yaw = np.radians(yaw_deg)

        cx, sx = np.cos(roll), np.sin(roll)
        cy, sy = np.cos(pitch), np.sin(pitch)
        cz, sz = np.cos(yaw), np.sin(yaw)

        rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
        ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
        rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])

        return rz @ ry @ rx

    def _draw_board(self, pitch_deg, roll_deg, yaw_deg):
        self._init_scene()

        l = self.board_length / 2.0
        w = self.board_width / 2.0
        h = self.board_height / 2.0

        vertices = np.array(
            [
                [-l, -w, -h],
                [l, -w, -h],
                [l, w, -h],
                [-l, w, -h],
                [-l, -w, h],
                [l, -w, h],
                [l, w, h],
                [-l, w, h],
            ]
        )

        rotation = self._rotation_matrix(roll_deg, pitch_deg, yaw_deg)
        rotated_ned = vertices @ rotation.T
        rotated = self._ned_to_plot(rotated_ned)

        faces = [
            [rotated[0], rotated[1], rotated[2], rotated[3]],
            [rotated[4], rotated[5], rotated[6], rotated[7]],
            [rotated[0], rotated[1], rotated[5], rotated[4]],
            [rotated[1], rotated[2], rotated[6], rotated[5]],
            [rotated[2], rotated[3], rotated[7], rotated[6]],
            [rotated[3], rotated[0], rotated[4], rotated[7]],
        ]

        board = Poly3DCollection(faces, alpha=0.72, edgecolor="black", linewidths=0.8)
        board.set_facecolor("#2b8cbe")
        self.ax.add_collection3d(board)

        label_points_body = {
            "Front": np.array([l + 0.18, 0.0, 0.0]),
            "Left": np.array([0.0, -w - 0.18, 0.0]),
            "Right": np.array([0.0, w + 0.18, 0.0]),
            "Top": np.array([0.0, 0.0, -h - 0.18]),
        }

        for label, point_body in label_points_body.items():
            point_world = self._ned_to_plot(point_body @ rotation.T)
            self.ax.text(
                point_world[0],
                point_world[1],
                point_world[2],
                label,
                color="#111111",
                fontsize=10,
                ha="center",
                va="center",
            )

        # Body axes attached to the board.
        body_axis = np.array(
            [
                [1.1, 0.0, 0.0],
                [0.0, 1.1, 0.0],
                [0.0, 0.0, 1.1],
            ]
        )
        body_axis_rot = self._ned_to_plot(body_axis @ rotation.T)

        self.ax.plot([0, body_axis_rot[0, 0]], [0, body_axis_rot[0, 1]], [0, body_axis_rot[0, 2]], color="#d62728", linewidth=3, linestyle=":")
        self.ax.plot([0, body_axis_rot[1, 0]], [0, body_axis_rot[1, 1]], [0, body_axis_rot[1, 2]], color="#2ca02c", linewidth=3, linestyle=":")
        self.ax.plot([0, body_axis_rot[2, 0]], [0, body_axis_rot[2, 1]], [0, body_axis_rot[2, 2]], color="#1f77b4", linewidth=3, linestyle=":")

        self.canvas.draw_idle()

    def _auto_connect_startup(self):
        if self.reader_thread and self.reader_thread.is_alive():
            return
        if not self.port_var.get().strip():
            self.port_var.set("COM6")
        self._toggle_connection()

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports

        if ports:
            if self.port_var.get() == "":
                if "COM6" in ports:
                    self.port_var.set("COM6")
                else:
                    self.port_var.set(ports[0])
            self.status_var.set(f"Found {len(ports)} serial port(s)")
        else:
            self.status_var.set("No serial ports found")

    def _toggle_connection(self):
        if self.reader_thread and self.reader_thread.is_alive():
            self.stop_event.set()
            self.connect_btn.configure(text="Connect")
            self.status_var.set("Disconnecting...")
            return

        port = self.port_var.get().strip()
        baud = self.baud_var.get().strip()
        if not port:
            self.status_var.set("Select a COM port first")
            return

        try:
            baudrate = int(baud)
        except ValueError:
            self.status_var.set("Invalid baudrate")
            return

        self.stop_event.clear()
        self.reader_thread = SerialReader(port, baudrate, self.data_queue, self.stop_event)
        self.reader_thread.start()
        self.connect_btn.configure(text="Disconnect")

    def _process_queue(self):
        while True:
            try:
                event_type, payload = self.data_queue.get_nowait()
            except queue.Empty:
                break

            if event_type == "status":
                self.status_var.set(payload)
                if "error" in payload.lower() or "closed" in payload.lower():
                    self.connect_btn.configure(text="Connect")
            elif event_type == "angles":
                self.pitch, self.roll, self.yaw = payload
                self.angles_var.set(
                    f"Pitch: {self.pitch:+07.2f}   Roll: {self.roll:+07.2f}   Yaw: {self.yaw:+07.2f}"
                )
                self._draw_board(self.pitch, self.roll, self.yaw)

        self.root.after(25, self._process_queue)

    def _on_close(self):
        self.stop_event.set()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = BoardViewerApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
