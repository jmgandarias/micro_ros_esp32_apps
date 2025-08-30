import sys
import time
import threading
import queue
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
import serial.tools.list_ports
import csv
from datetime import datetime
import os

DEFAULT_BAUD = 115200         # Match Serial.begin() on ESP32
READ_THREAD_SLEEP_S = 0.01    # Serial poll interval


class ESP32StepGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ESP32 Step Experiment (CSV)")
        self.geometry("640x380")

        # Serial state
        self.ser = None
        self.read_thread = None
        self.stop_event = threading.Event()
        self.collecting = False
        self.lines_queue = queue.Queue()

        # Data buffer (list of rows)
        # Each row: [PWM(int), pos(float), vel(float), time_ms(float)]
        self.data_rows = []
        self.data_lock = threading.Lock()

        self._build_ui()
        self.after(100, self._process_lines_queue)

    # -------------------- UI --------------------
    def _build_ui(self):
        pad = {'padx': 8, 'pady': 6}

        frm = ttk.LabelFrame(self, text="Connection")
        frm.pack(fill='x', **pad)

        ttk.Label(frm, text="Port:").grid(row=0, column=0, sticky='w', **pad)
        self.port_cmb = ttk.ComboBox(frm=self, base=frm)

    def _build_ui(self):
        pad = {'padx': 8, 'pady': 6}

        conn = ttk.LabelFrame(self, text="Connection")
        conn.pack(fill='x', **pad)

        ttk.Label(conn, text="Port:").grid(row=0, column=0, sticky='w', **pad)
        self.port_cmb = ttk.Combobox(conn, width=36, state='readonly', values=self._list_ports())
        self.port_cmb.grid(row=0, column=1, sticky='w', **pad)

        self.refresh_btn = ttk.Button(conn, text="Refresh", command=self._refresh_ports)
        self.refresh_btn.grid(row=0, column=2, sticky='w', **pad)

        ttk.Label(conn, text="Baud:").grid(row=1, column=0, sticky='w', **pad)
        self.baud_cmb = ttk.Combobox(conn, width=12, state='readonly',
                                     values=[9600, 19200, 38400, 57600, 115200, 230400, 460800])
        self.baud_cmb.set(DEFAULT_BAUD)
        self.baud_cmb.grid(row=1, column=1, sticky='w', **pad)

        # Output
        out = ttk.LabelFrame(self, text="Output")
        out.pack(fill='x', **pad)

        ttk.Label(out, text="Save to (.csv):").grid(row=0, column=0, sticky='w', **pad)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_name = f"esp32_step_{timestamp}.csv"
        self.file_entry = ttk.Entry(out, width=45)
        self.file_entry.insert(0, os.path.join(os.getcwd(), default_name))
        self.file_entry.grid(row=0, column=1, sticky='we', **pad)

        self.browse_btn = ttk.Button(out, text="Browse…", command=self._browse_outfile)
        self.browse_btn.grid(row=0, column=2, sticky='w', **pad)
        out.columnconfigure(1, weight=1)

        # Controls
        ctrl = ttk.LabelFrame(self, text="Controls")
        ctrl.pack(fill='x', **pad)

        self.start_btn = ttk.Button(ctrl, text="Start Experiment", command=self._on_start)
        self.start_btn.grid(row=0, column=0, **pad)

        self.stop_btn = ttk.Button(ctrl, text="Stop & Save", command=self._on_stop_and_save, state='disabled')
        self.stop_btn.grid(row=0, column=1, **pad)

        self.exit_btn = ttk.Button(ctrl, text="Exit", command=self._on_exit)
        self.exit_btn.grid(row=0, column=2, **pad)

        # Status + log
        self.status_var = tk.StringVar(value="Idle")
        ttk.Label(self, textvariable=self.status_var).pack(anchor='w', **pad)

        self.log = tk.Text(self, height=9, wrap='none', state='disabled')
        self.log.pack(fill='both', expand=True, **pad)

    # -------------------- Serial helpers --------------------
    def _list_ports(self):
        ports = serial.tools.list_ports.comports()
        return [f"{p.device} - {p.description}" for p in ports]

    def _refresh_ports(self):
        self.port_cmb['values'] = self._list_ports()
        self._log("Ports refreshed.")

    def _open_serial(self):
        if self.ser and self.ser.is_open:
            return True
        sel = self.port_cmb.get()
        if not sel:
            messagebox.showwarning("Select port", "Please select a serial port first.")
            return False
        port = sel.split(" - ")[0]
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=int(self.baud_cmb.get()),
                timeout=0.05
            )
            time.sleep(0.2)  # allow port to settle
            self._log(f"Opened {port} @ {self.ser.baudrate} baud.")
            return True
        except Exception as e:
            messagebox.showerror("Serial error", f"Could not open port:\n{e}")
            return False

    def _close_serial(self):
        if self.ser:
            try:
                self.ser.close()
                self._log("Serial port closed.")
            except Exception:
                pass
            self.ser = None

    # -------------------- Actions --------------------
    def _on_start(self):
        if not self._open_serial():
            return

        # Prepare a fresh run
        with self.data_lock:
            self.data_rows = []

        self.collecting = True
        self.start_btn['state'] = 'disabled'
        self.stop_btn['state'] = 'normal'
        self.status_var.set("Experiment running…")

        # Start reader thread if needed
        if self.read_thread is None or not self.read_thread.is_alive():
            self.stop_event.clear()
            self.read_thread = threading.Thread(target=self._reader_loop, daemon=True)
            self.read_thread.start()

        # Clear buffers then send start byte '1'
        try:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.ser.write(b'1')  # Single byte '1'
            self.ser.flush()
            self._log("Sent start trigger: '1'")
        except Exception as e:
            messagebox.showerror("Serial error", f"Failed to write start byte:\n{e}")
            self._on_stop_and_save(force_save=False)

    def _on_stop_and_save(self, force_save=True):
        self.collecting = False
        self.stop_btn['state'] = 'disabled'
        self.start_btn['state'] = 'normal'
        saved = False
        if force_save:
            saved = self._save_csv()
        self.status_var.set("Saved and ready." if saved else "Ready.")

    def _on_exit(self):
        self.stop_event.set()
        self.collecting = False
        try:
            if self.read_thread and self.read_thread.is_alive():
                self.read_thread.join(timeout=1.0)
        except Exception:
            pass
        self._close_serial()
        self.destroy()

    # -------------------- Reader & parsing --------------------
    def _reader_loop(self):
        buffer = b""
        while not self.stop_event.is_set():
            try:
                if self.ser and self.ser.is_open:
                    chunk = self.ser.read(1024)
                    if chunk:
                        buffer += chunk
                        while b'\n' in buffer:
                            line, buffer = buffer.split(b'\n', 1)
                            line = line.strip().decode(errors='ignore')  # handles CRLF
                            if line:
                                self._handle_line(line)
                    else:
                        time.sleep(READ_THREAD_SLEEP_S)
                else:
                    time.sleep(0.2)
            except Exception as e:
                self.lines_queue.put(("log", f"[Serial read error] {e}"))
                time.sleep(0.2)

    def _handle_line(self, line: str):
        # Show every line in the log
        self.lines_queue.put(("log", line))

        # Stop on explicit END marker from ESP32
        if line == "END":
            # Let main thread stop & save
            self.after(0, lambda: self._on_stop_and_save(force_save=True))
            return

        if not self.collecting:
            return

        parts = line.split(';')
        if len(parts) != 4:
            return  # ignore malformed lines

        try:
            pwm = int(float(parts[0]))  # robust to "1" or "1.0"
            pos = float(parts[1])
            vel = float(parts[2])
            tms = float(parts[3])       # milliseconds from ESP32 timer
        except ValueError:
            return  # ignore parse errors

        with self.data_lock:
            self.data_rows.append([pwm, pos, vel, tms])

    # -------------------- Save CSV --------------------
    def _save_csv(self):
        with self.data_lock:
            rows = list(self.data_rows)

        if not rows:
            self._log("No data to save.")
            return False

        out_path = self.file_entry.get().strip()
        if not out_path:
            messagebox.showwarning("Missing path", "Please specify an output .csv filename.")
            return False
        if not out_path.lower().endswith(".csv"):
            out_path += ".csv"

        out_dir = os.path.dirname(out_path) or "."
        os.makedirs(out_dir, exist_ok=True)

        try:
            with open(out_path, mode='w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(["PWM", "pos_rad", "vel_rad_per_s", "time_ms"])
                writer.writerows(rows)
            self._log(f"Saved {len(rows)} rows to:\n{out_path}")
            return True
        except Exception as e:
            messagebox.showerror("Save error", f"Failed to save CSV:\n{e}")
            return False

    # -------------------- UI helpers --------------------
    def _browse_outfile(self):
        initial = self.file_entry.get()
        if not initial.lower().endswith(".csv"):
            initial += ".csv"
        file = filedialog.asksaveasfilename(
            title="Choose output .csv file",
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv")],
            initialfile=os.path.basename(initial),
            initialdir=os.path.dirname(initial) if os.path.dirname(initial) else os.getcwd()
        )
        if file:
            self.file_entry.delete(0, tk.END)
            self.file_entry.insert(0, file)

    def _process_lines_queue(self):
        try:
            while True:
                kind, payload = self.lines_queue.get_nowait()
                if kind == "log":
                    self._log(payload)
        except queue.Empty:
            pass
        self.after(100, self._process_lines_queue)

    def _log(self, message: str):
        self.log.configure(state='normal')
        self.log.insert('end', message + "\n")
        self.log.see('end')
        self.log.configure(state='disabled')


if __name__ == "__main__":
    app = ESP32StepGUI()
    app.mainloop()