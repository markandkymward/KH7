import threading
import tkinter as tk
from tkinter import ttk, messagebox

import serial
import serial.tools.list_ports
from serial import SerialException


class MotorTestGui:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("KH7 USB-C Motor Test")
        self.root.geometry("440x330")

        self.serial_port = None
        self.serial_lock = threading.Lock()

        self.port_var = tk.StringVar()
        self.connected_var = tk.StringVar(value="Disconnected")
        self.motor_var = tk.IntVar(value=1)
        self.pulse_var = tk.IntVar(value=1100)

        self._build_ui()
        self.refresh_ports()

    def _build_ui(self) -> None:
        frame = ttk.Frame(self.root, padding=12)
        frame.pack(fill=tk.BOTH, expand=True)

        top = ttk.Frame(frame)
        top.pack(fill=tk.X)

        ttk.Label(top, text="USB CDC Port:").pack(side=tk.LEFT)
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, state="readonly", width=24)
        self.port_combo.pack(side=tk.LEFT, padx=6)

        ttk.Button(top, text="Refresh", command=self.refresh_ports).pack(side=tk.LEFT)
        ttk.Button(top, text="Connect", command=self.connect).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="Disconnect", command=self.disconnect).pack(side=tk.LEFT)

        ttk.Label(frame, textvariable=self.connected_var, foreground="#1f6f43").pack(anchor=tk.W, pady=(8, 10))

        motor_box = ttk.LabelFrame(frame, text="Motor")
        motor_box.pack(fill=tk.X, pady=(0, 10))

        for idx in range(1, 5):
            ttk.Radiobutton(motor_box, text=f"S{idx}", variable=self.motor_var, value=idx).pack(side=tk.LEFT, padx=10, pady=8)

        pulse_box = ttk.LabelFrame(frame, text="Pulse (us)")
        pulse_box.pack(fill=tk.X, pady=(0, 10))

        self.scale = ttk.Scale(
            pulse_box,
            from_=988,
            to=2012,
            orient=tk.HORIZONTAL,
            command=self._on_scale,
        )
        self.scale.pack(fill=tk.X, padx=10, pady=8)

        pulse_row = ttk.Frame(pulse_box)
        pulse_row.pack(fill=tk.X, padx=10, pady=(0, 8))
        self.pulse_entry = ttk.Entry(pulse_row, width=8)
        self.pulse_entry.insert(0, str(self.pulse_var.get()))
        self.pulse_entry.pack(side=tk.LEFT)
        ttk.Button(pulse_row, text="Set", command=self.apply_entry_pulse).pack(side=tk.LEFT, padx=6)
        self.scale.set(self.pulse_var.get())

        actions = ttk.Frame(frame)
        actions.pack(fill=tk.X, pady=(0, 8))
        ttk.Button(actions, text="Run Selected", command=self.run_selected).pack(side=tk.LEFT)
        ttk.Button(actions, text="Stop", command=self.stop).pack(side=tk.LEFT, padx=8)

        hint = (
            "Protocol:\n"
            "  MTEST <motor 1..4> <pulse_us>\n"
            "  MTEST OFF\n"
            "Use low pulse values first and keep props removed for bench tests."
        )
        ttk.Label(frame, text=hint, justify=tk.LEFT).pack(anchor=tk.W)

    def _on_scale(self, value: str) -> None:
        pulse = int(float(value))
        self.pulse_var.set(pulse)
        if not hasattr(self, "pulse_entry"):
            return
        self.pulse_entry.delete(0, tk.END)
        self.pulse_entry.insert(0, str(pulse))

    def apply_entry_pulse(self) -> None:
        try:
            pulse = int(self.pulse_entry.get())
        except ValueError:
            messagebox.showerror("Invalid pulse", "Pulse must be an integer in 988..2012.")
            return

        pulse = max(988, min(2012, pulse))
        self.pulse_var.set(pulse)
        self.scale.set(pulse)

    def refresh_ports(self) -> None:
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def connect(self) -> None:
        if self.serial_port is not None and self.serial_port.is_open:
            return

        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("No port", "Select a USB CDC COM port first.")
            return

        try:
            self.serial_port = serial.Serial(port, 115200, timeout=0.2)
            self.connected_var.set(f"Connected: {port}")
        except Exception as exc:
            self.serial_port = None
            messagebox.showerror("Connect failed", str(exc))

    def disconnect(self) -> None:
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except Exception:
                pass
        self.serial_port = None
        self.connected_var.set("Disconnected")

    def send_command(self, cmd: str) -> None:
        if self.serial_port is None or not self.serial_port.is_open:
            messagebox.showerror("Not connected", "Connect to the USB CDC COM port first.")
            return

        data = (cmd + "\n").encode("ascii", errors="ignore")
        try:
            with self.serial_lock:
                self.serial_port.write(data)
        except (SerialException, OSError) as exc:
            self.disconnect()
            messagebox.showerror(
                "Serial write failed",
                f"Failed to send command to device. Reconnect the COM port and try again.\n\n{exc}",
            )

    def run_selected(self) -> None:
        self.apply_entry_pulse()
        motor = self.motor_var.get()
        pulse = self.pulse_var.get()
        self.send_command(f"MTEST {motor} {pulse}")

    def stop(self) -> None:
        self.send_command("MTEST OFF")


def main() -> None:
    root = tk.Tk()
    app = MotorTestGui(root)

    def on_close() -> None:
        app.disconnect()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
