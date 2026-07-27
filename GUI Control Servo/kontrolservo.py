#!/usr/bin/env python3
"""
Robot Arm 4-DOF GUI Controller
Hardware: ESP32 + PCA9685 + 3x MG996R + 1x MG90S
Komunikasi: Serial USB (atau ganti ke TCP/IP jika pakai WiFi)

Wiring:
  ESP32 GPIO21 -> PCA9685 SDA
  ESP32 GPIO22 -> PCA9685 SCL
  CH0: Base (MG996R)
  CH1: Shoulder (MG996R)
  CH2: Elbow (MG996R)
  CH3: Gripper (MG90S)

Cara pakai:
  1. Upload robot_arm_esp32.ino ke ESP32
  2. Jalankan: python3 robot_arm_gui.py
  3. Pilih port COM/ttyUSB, klik Connect
"""

import tkinter as tk
from tkinter import ttk, messagebox, font
import serial
import serial.tools.list_ports
import threading
import time
import math
import json

# ─── Konfigurasi Default ──────────────────────────────────────────────────────
BAUD_RATE    = 115200
UPDATE_DELAY = 50       # ms antar kiriman serial (throttle)

SERVO_CONFIG = [
    {"name": "Base",     "channel": 0, "type": "MG996R", "min": 0,   "max": 180, "default": 90,  "color": "#185FA5"},
    {"name": "Shoulder", "channel": 1, "type": "MG996R", "min": 0,   "max": 180, "default": 90,  "color": "#0F6E56"},
    {"name": "Elbow",    "channel": 2, "type": "MG996R", "min": 0,   "max": 180, "default": 90,  "color": "#993C1D"},
    {"name": "Gripper",  "channel": 3, "type": "MG90S",  "min": 0,   "max": 90,  "default": 45,  "color": "#533AB7"},
]

PRESET_POSES = [
    {"name": "Home",           "angles": [90, 90, 90, 45]},
    {"name": "Jangkau Bawah",  "angles": [90, 45, 135, 10]},
    {"name": "Jangkau Atas",   "angles": [90, 135, 45, 10]},
    {"name": "Putar Kiri",     "angles": [0,  90, 90, 45]},
    {"name": "Putar Kanan",    "angles": [180,90, 90, 45]},
    {"name": "Genggam",        "angles": [90, 90, 90, 0]},
    {"name": "Lepas",          "angles": [90, 90, 90, 90]},
    {"name": "Parkir",         "angles": [90, 0,  0,  0]},
]


class RobotArmGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Arm 4-DOF Controller  |  ESP32 + PCA9685")
        self.root.geometry("950x720")
        self.root.resizable(True, True)
        self.root.configure(bg="#F5F5F5")

        # State
        self.serial_conn  = None
        self.connected    = False
        self.angles       = [cfg["default"] for cfg in SERVO_CONFIG]
        self.last_sent    = [None] * 4
        self.send_pending = False
        self.log_lines    = []

        self._build_ui()
        self._refresh_ports()
        self._draw_arm()
        self._schedule_send()

    # ─── Build UI ─────────────────────────────────────────────────────────────

    def _build_ui(self):
        # ── Top bar ──
        top = tk.Frame(self.root, bg="#1A1A2E", height=52)
        top.pack(fill="x")
        top.pack_propagate(False)

        tk.Label(top, text="🤖  Robot Arm Controller", bg="#1A1A2E", fg="#E0E0FF",
                 font=("Helvetica", 15, "bold")).pack(side="left", padx=18, pady=12)
        tk.Label(top, text="ESP32 + PCA9685  |  4 DOF", bg="#1A1A2E", fg="#8888AA",
                 font=("Helvetica", 10)).pack(side="left", pady=12)

        # ── Main layout ──
        body = tk.Frame(self.root, bg="#F5F5F5")
        body.pack(fill="both", expand=True, padx=12, pady=10)

        left  = tk.Frame(body, bg="#F5F5F5")
        right = tk.Frame(body, bg="#F5F5F5")
        left.pack(side="left", fill="both", expand=True)
        right.pack(side="right", fill="both", expand=False, padx=(10,0))

        self._build_connection(left)
        self._build_servo_panel(left)
        self._build_preset_panel(left)
        self._build_canvas(right)
        self._build_log(right)

    def _card(self, parent, title):
        """Buat frame card dengan judul."""
        frame = tk.LabelFrame(parent, text=f"  {title}  ", bg="white",
                              font=("Helvetica", 10, "bold"), fg="#333",
                              relief="flat", bd=1, padx=12, pady=8)
        frame.pack(fill="x", pady=(0, 10))
        frame.config(highlightbackground="#DDD", highlightthickness=1)
        return frame

    def _build_connection(self, parent):
        card = self._card(parent, "🔌  Koneksi Serial")
        row  = tk.Frame(card, bg="white")
        row.pack(fill="x")

        tk.Label(row, text="Port:", bg="white", font=("Helvetica", 10)).pack(side="left")
        self.port_var = tk.StringVar()
        self.port_cb  = ttk.Combobox(row, textvariable=self.port_var, width=18, state="readonly")
        self.port_cb.pack(side="left", padx=(4, 8))

        tk.Label(row, text="Baud:", bg="white", font=("Helvetica", 10)).pack(side="left")
        self.baud_var = tk.StringVar(value=str(BAUD_RATE))
        ttk.Combobox(row, textvariable=self.baud_var, width=9,
                     values=["9600","19200","57600","115200","230400"],
                     state="readonly").pack(side="left", padx=(4, 10))

        self.btn_refresh = tk.Button(row, text="⟳ Refresh", command=self._refresh_ports,
                                     bg="#EEEEEE", relief="flat", padx=8, cursor="hand2")
        self.btn_refresh.pack(side="left", padx=(0,6))

        self.btn_connect = tk.Button(row, text="Connect", command=self._toggle_connect,
                                      bg="#185FA5", fg="white", relief="flat",
                                      padx=12, font=("Helvetica", 10, "bold"), cursor="hand2")
        self.btn_connect.pack(side="left")

        self.lbl_status = tk.Label(card, text="● Terputus", fg="#AA3300",
                                    bg="white", font=("Helvetica", 10))
        self.lbl_status.pack(anchor="w", pady=(6,0))

    def _build_servo_panel(self, parent):
        card = self._card(parent, "🎮  Kontrol Servo")
        self.sliders      = []
        self.angle_labels = []
        self.value_labels = []

        for i, cfg in enumerate(SERVO_CONFIG):
            row = tk.Frame(card, bg="white")
            row.pack(fill="x", pady=4)

            # Warna dot + label
            dot = tk.Label(row, text="●", fg=cfg["color"], bg="white", font=("Helvetica", 12))
            dot.grid(row=0, column=0, sticky="w")

            lbl = tk.Label(row, text=f"{cfg['name']}  [{cfg['type']} · CH{cfg['channel']}]",
                           bg="white", width=26, anchor="w", font=("Helvetica", 10, "bold"))
            lbl.grid(row=0, column=1, sticky="w")

            val_lbl = tk.Label(row, text=f"{cfg['default']}°", bg="white",
                               font=("Helvetica", 11, "bold"), fg=cfg["color"], width=5, anchor="e")
            val_lbl.grid(row=0, column=5, sticky="e")
            self.value_labels.append(val_lbl)

            # Slider
            slider_var = tk.IntVar(value=cfg["default"])
            slider = ttk.Scale(row, from_=cfg["min"], to=cfg["max"],
                               orient="horizontal", length=320,
                               variable=slider_var,
                               command=lambda v, idx=i: self._on_slider(idx, v))
            slider.grid(row=0, column=2, padx=8)

            tk.Label(row, text=f"{cfg['min']}°", bg="white", fg="#888",
                     font=("Helvetica", 9)).grid(row=0, column=3)
            tk.Label(row, text=f"{cfg['max']}°", bg="white", fg="#888",
                     font=("Helvetica", 9)).grid(row=0, column=4)

            self.sliders.append((slider, slider_var))
            self.angle_labels.append(val_lbl)

            row.columnconfigure(2, weight=1)

        # Tombol cepat gripper
        grip_row = tk.Frame(card, bg="white")
        grip_row.pack(fill="x", pady=(4,0))
        tk.Label(grip_row, text="Gripper cepat:", bg="white",
                 font=("Helvetica", 10)).pack(side="left")
        tk.Button(grip_row, text="  Buka (90°)  ", command=lambda: self._set_servo(3, 90),
                  bg="#E8F4FF", relief="flat", cursor="hand2", padx=8).pack(side="left", padx=6)
        tk.Button(grip_row, text="  Tutup (0°)  ", command=lambda: self._set_servo(3, 0),
                  bg="#FFF0E8", relief="flat", cursor="hand2", padx=8).pack(side="left")

        # Tombol utama
        btn_row = tk.Frame(card, bg="white")
        btn_row.pack(fill="x", pady=(10,2))

        btns = [
            ("🏠 Home",     "#185FA5", "white", self._go_home),
            ("⬇ Min All",  "#444",    "white", lambda: self._set_all(0)),
            ("⬆ Max All",  "#444",    "white", lambda: self._set_all(180)),
            ("⛔ STOP",    "#CC2200", "white", self._emergency_stop),
        ]
        for txt, bg, fg, cmd in btns:
            tk.Button(btn_row, text=txt, command=cmd, bg=bg, fg=fg,
                      relief="flat", padx=12, pady=5, cursor="hand2",
                      font=("Helvetica", 10, "bold")).pack(side="left", padx=(0,6))

    def _build_preset_panel(self, parent):
        card = self._card(parent, "📌  Preset Pose")
        grid = tk.Frame(card, bg="white")
        grid.pack(fill="x")
        for i, pose in enumerate(PRESET_POSES):
            col, row = i % 4, i // 4
            tk.Button(grid, text=pose["name"],
                      command=lambda p=pose: self._apply_pose(p["angles"]),
                      bg="#F0F4FF", relief="flat", padx=6, pady=5,
                      cursor="hand2", font=("Helvetica", 10),
                      width=14).grid(row=row, column=col, padx=4, pady=3)

    def _build_canvas(self, parent):
        card = tk.LabelFrame(parent, text="  📐  Visualisasi Arm  ", bg="white",
                             font=("Helvetica", 10, "bold"), fg="#333",
                             relief="flat", bd=1, padx=8, pady=8)
        card.pack(fill="x", pady=(0,10))
        card.config(highlightbackground="#DDD", highlightthickness=1)
        self.canvas = tk.Canvas(card, width=280, height=300, bg="#FAFAFA",
                                highlightthickness=0)
        self.canvas.pack()

    def _build_log(self, parent):
        card = tk.LabelFrame(parent, text="  📋  Log Serial  ", bg="white",
                             font=("Helvetica", 10, "bold"), fg="#333",
                             relief="flat", bd=1, padx=8, pady=8)
        card.pack(fill="both", expand=True)
        card.config(highlightbackground="#DDD", highlightthickness=1)
        self.log_text = tk.Text(card, height=10, width=34, font=("Courier", 9),
                                bg="#1A1A2E", fg="#00FF88", state="disabled",
                                relief="flat", wrap="word")
        self.log_text.pack(fill="both", expand=True)
        sb = ttk.Scrollbar(card, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=sb.set)
        tk.Button(card, text="Clear Log", command=self._clear_log,
                  bg="#333", fg="white", relief="flat", padx=8,
                  cursor="hand2").pack(anchor="e", pady=(4,0))

    # ─── Serial ───────────────────────────────────────────────────────────────

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb["values"] = ports
        if ports:
            self.port_var.set(ports[0])

    def _toggle_connect(self):
        if self.connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_var.get()
        baud = int(self.baud_var.get())
        if not port:
            messagebox.showerror("Error", "Pilih port serial terlebih dahulu.")
            return
        try:
            self.serial_conn = serial.Serial(port, baud, timeout=1)
            time.sleep(2)  # tunggu ESP32 reset
            self.connected = True
            self.btn_connect.config(text="Disconnect", bg="#AA2200")
            self.lbl_status.config(text=f"● Terhubung ke {port} @ {baud}", fg="#007700")
            self._log(f"[CONNECTED] {port} @ {baud} baud")
            threading.Thread(target=self._read_serial, daemon=True).start()
        except Exception as e:
            messagebox.showerror("Koneksi Gagal", str(e))

    def _disconnect(self):
        self.connected = False
        if self.serial_conn:
            self.serial_conn.close()
            self.serial_conn = None
        self.btn_connect.config(text="Connect", bg="#185FA5")
        self.lbl_status.config(text="● Terputus", fg="#AA3300")
        self._log("[DISCONNECTED]")

    def _send_angles(self, angles=None):
        """Kirim sudut ke ESP32. Format JSON: {"s":[a0,a1,a2,a3]}"""
        if not self.connected or not self.serial_conn:
            return
        a = angles if angles else self.angles
        cmd = json.dumps({"s": [int(x) for x in a]}) + "\n"
        try:
            self.serial_conn.write(cmd.encode())
            self._log(f"[TX] {cmd.strip()}")
        except Exception as e:
            self._log(f"[ERROR] {e}")
            self._disconnect()

    def _read_serial(self):
        """Thread: baca respons dari ESP32."""
        while self.connected and self.serial_conn:
            try:
                line = self.serial_conn.readline().decode(errors="ignore").strip()
                if line:
                    self._log(f"[RX] {line}")
            except:
                break

    def _schedule_send(self):
        """Throttle pengiriman serial supaya tidak banjir."""
        if self.send_pending:
            self._send_angles()
            self.send_pending = False
        self.root.after(UPDATE_DELAY, self._schedule_send)

    # ─── Kontrol ──────────────────────────────────────────────────────────────

    def _on_slider(self, idx, val):
        v = int(float(val))
        self.angles[idx] = v
        self.value_labels[idx].config(text=f"{v}°")
        self.send_pending = True
        self._draw_arm()

    def _set_servo(self, idx, val):
        cfg = SERVO_CONFIG[idx]
        v = max(cfg["min"], min(cfg["max"], val))
        self.angles[idx] = v
        self.sliders[idx][1].set(v)
        self.value_labels[idx].config(text=f"{v}°")
        self._send_angles()
        self._draw_arm()

    def _set_all(self, val):
        for i in range(3):  # hanya 3 servo badan, gripper tidak di-max
            self._set_servo(i, val)

    def _go_home(self):
        self._apply_pose([90, 90, 90, 45])

    def _apply_pose(self, pose_angles):
        for i, a in enumerate(pose_angles):
            cfg = SERVO_CONFIG[i]
            v = max(cfg["min"], min(cfg["max"], a))
            self.angles[i] = v
            self.sliders[i][1].set(v)
            self.value_labels[i].config(text=f"{v}°")
        self._send_angles()
        self._draw_arm()

    def _emergency_stop(self):
        """Kirim perintah stop, set semua ke home."""
        if self.connected and self.serial_conn:
            try:
                self.serial_conn.write(b'{"cmd":"stop"}\n')
                self._log("[TX] EMERGENCY STOP")
            except:
                pass
        self._apply_pose([90, 90, 90, 45])

    # ─── Visualisasi Arm ──────────────────────────────────────────────────────

    def _draw_arm(self):
        c = self.canvas
        c.delete("all")
        W, H = 280, 300

        # Grid background
        for x in range(0, W, 20):
            c.create_line(x, 0, x, H, fill="#EEEEEE", width=1)
        for y in range(0, H, 20):
            c.create_line(0, y, W, y, fill="#EEEEEE", width=1)

        BX, BY = W // 2, H - 30
        L1, L2, L3 = 70, 60, 45

        # Sudut dalam radian (tampilan 2D side view)
        a0 = math.radians(self.angles[0])   # base: rotasi horizontal (ditampilkan sbg offset)
        a1 = math.radians(self.angles[1])   # shoulder
        a2 = math.radians(self.angles[2])   # elbow
        a3 = self.angles[3]                 # gripper: buka/tutup

        # Base platform
        c.create_oval(BX-18, BY-18, BX+18, BY+18, fill="#185FA5", outline="white", width=2)
        c.create_text(BX, BY, text="B", fill="white", font=("Helvetica", 9, "bold"))

        # Shoulder joint
        s_angle = math.radians(-(a1 - 90) * 0.9 + math.radians(a0 - 90) * 0.3)
        s_angle = math.radians(-(self.angles[1] - 90))
        SX = BX + L1 * math.sin(s_angle)
        SY = BY - L1 * math.cos(s_angle)

        c.create_line(BX, BY, SX, SY, fill="#0F6E56", width=9, capstyle="round")
        c.create_oval(SX-11, SY-11, SX+11, SY+11, fill="#0F6E56", outline="white", width=2)
        c.create_text(SX, SY, text="S", fill="white", font=("Helvetica", 8, "bold"))

        # Elbow joint
        e_base = s_angle + math.radians(-(self.angles[2] - 90) * 0.8)
        EX = SX + L2 * math.sin(e_base)
        EY = SY - L2 * math.cos(e_base)

        c.create_line(SX, SY, EX, EY, fill="#993C1D", width=7, capstyle="round")
        c.create_oval(EX-9, EY-9, EX+9, EY+9, fill="#993C1D", outline="white", width=2)
        c.create_text(EX, EY, text="E", fill="white", font=("Helvetica", 8, "bold"))

        # Wrist / Gripper
        g_angle = e_base + math.radians(-20)
        GX = EX + L3 * math.sin(g_angle)
        GY = EY - L3 * math.cos(g_angle)

        c.create_line(EX, EY, GX, GY, fill="#533AB7", width=5, capstyle="round")
        c.create_oval(GX-7, GY-7, GX+7, GY+7, fill="#533AB7", outline="white", width=2)

        # Gripper fingers
        grip_open = (a3 / 90.0) * 12 + 3
        perp_x = math.cos(g_angle)
        perp_y = math.sin(g_angle)
        for sign in (+1, -1):
            fx = GX + sign * perp_x * grip_open
            fy = GY + sign * perp_y * grip_open
            tip_x = fx + math.sin(g_angle) * 14
            tip_y = fy - math.cos(g_angle) * 14
            c.create_line(GX, GY, fx, fy, fill="#533AB7", width=3, capstyle="round")
            c.create_line(fx, fy, tip_x, tip_y, fill="#533AB7", width=3, capstyle="round")

        # Label sudut di samping
        c.create_text(5, 10, anchor="nw", text=f"Base:     {self.angles[0]}°",
                      fill="#185FA5", font=("Courier", 9, "bold"))
        c.create_text(5, 26, anchor="nw", text=f"Shoulder: {self.angles[1]}°",
                      fill="#0F6E56", font=("Courier", 9, "bold"))
        c.create_text(5, 42, anchor="nw", text=f"Elbow:    {self.angles[2]}°",
                      fill="#993C1D", font=("Courier", 9, "bold"))
        c.create_text(5, 58, anchor="nw", text=f"Gripper:  {self.angles[3]}°",
                      fill="#533AB7", font=("Courier", 9, "bold"))

        # Ground line
        c.create_line(20, BY+20, W-20, BY+20, fill="#CCCCCC", width=2, dash=(4,4))
        c.create_text(W-5, BY+20, anchor="e", text="ground",
                      fill="#AAAAAA", font=("Helvetica", 8))

    # ─── Log ──────────────────────────────────────────────────────────────────

    def _log(self, msg):
        ts  = time.strftime("%H:%M:%S")
        txt = f"[{ts}] {msg}\n"
        self.root.after(0, self._append_log, txt)

    def _append_log(self, txt):
        self.log_text.config(state="normal")
        self.log_text.insert("end", txt)
        self.log_text.see("end")
        if int(self.log_text.index("end-1c").split(".")[0]) > 200:
            self.log_text.delete("1.0", "50.0")
        self.log_text.config(state="disabled")

    def _clear_log(self):
        self.log_text.config(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.config(state="disabled")

    # ─── Cleanup ──────────────────────────────────────────────────────────────

    def on_close(self):
        if self.connected:
            self._disconnect()
        self.root.destroy()


# ─── Main ─────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    root = tk.Tk()
    app  = RobotArmGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()