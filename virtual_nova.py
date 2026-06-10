import math
import threading
import time

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, TextBox
import serial
import spatialmath as sm

import nova_config as cfg

N_JOINTS = len(cfg.URDF_JOINTS)
q_deg = [0.0] * N_JOINTS

# ==========================================
# 1. SETUP SERIAL & THREAD FEEDBACK
# ==========================================
stm_serial = None
try:
    stm_serial = serial.Serial(cfg.COM_PORT, cfg.BAUD_RATE, timeout=0.1)
    print(f"Berhasil terhubung ke STM32 di {cfg.COM_PORT}!")
except serial.SerialException:
    print(f"Peringatan: Tidak dapat terhubung ke {cfg.COM_PORT}. Mode SIMULASI OFFLINE.")

# Lock ditambahkan untuk mencegah tabrakan antara UI Thread dan Background Thread
serial_lock = threading.Lock()

def _read_serial_loop():
    """Berjalan di background untuk mencetak balasan dari STM32."""
    while stm_serial and stm_serial.is_open:
        try:
            if stm_serial.in_waiting > 0:
                line = stm_serial.readline().decode("utf-8", errors='ignore').strip()
                if line:
                    print(f"[STM32] {line}")
        except Exception:
            pass

if stm_serial:
    threading.Thread(target=_read_serial_loop, daemon=True).start()

def _send(payload: bytes):
    """Mengirim data ke serial dengan aman (Thread-Safe)."""
    if stm_serial is not None and stm_serial.is_open:
        # Gunakan lock agar Thread UI dan Thread Background tidak saling timpa
        with serial_lock: 
            try:
                stm_serial.write(payload)
            except serial.SerialTimeoutException:
                print("[Python] Warning: Serial write timeout (diabaikan, antrean dilewati)")

# ==========================================
# 2. PROTOKOL PERINTAH STM32
# ==========================================
def send_step(idx, delta_deg):
    """Kirim step diskrit untuk input nilai manual (bukan jog)."""
    if delta_deg == 0:
        return
        
    steps = int(round((delta_deg / 360.0) * cfg.PULSE_PER_REV[idx] * cfg.GEAR_RATIO[idx]))
    
    # Pencegahan pengiriman beban kosong akibat focus loss di UI TextBox
    if steps == 0: 
        return
        
    _send(f"M:{idx}:{steps}\n".encode('utf-8'))

def send_jog_char(idx, sign):
    """Kirim 1 byte karakter kontrol untuk Streaming Velocity."""
    neg, pos = cfg.JOG_KEYS[idx]
    _send((pos if sign > 0 else neg).encode('ascii'))

def send_stop_all():
    """Rem darurat untuk semua motor."""
    _send(cfg.STOP_ALL_CHAR.encode('ascii'))

def send_home():
    """Perintah kembali ke posisi 0."""
    _send(b"H\n")

def send_set_speed():
    """Setel kecepatan per motor dari konfigurasi statis saat boot."""
    for idx in range(N_JOINTS):
        _send(f"V:{idx}:{cfg.JOG_HALFPERIOD_US_ARRAY[idx]}\n".encode('utf-8'))
        time.sleep(0.05) # Jeda aman saat inisialisasi awal

def jog_speed_deg_per_sec(idx):
    """Hitung kecepatan rotasi visual teoretis untuk UI Matplotlib."""
    motor_steps_per_sec = 1e6 / (2 * cfg.JOG_HALFPERIOD_US_ARRAY[idx])
    return motor_steps_per_sec / (cfg.PULSE_PER_REV[idx] * cfg.GEAR_RATIO[idx]) * 360.0

# ==========================================
# 3. KINEMATIKA (FORWARD KINEMATICS)
# ==========================================
_joint_offsets = [sm.SE3(*j.xyz) * sm.SE3.RPY(*j.rpy) for j in cfg.URDF_JOINTS]

def forward_kinematics(q_rad):
    pts = [sm.SE3().t]
    T = sm.SE3()
    for i in range(N_JOINTS):
        T = T * _joint_offsets[i] * sm.SE3.Rz(q_rad[i])
        pts.append(T.t)
    tip = T * sm.SE3.Tz(cfg.NEEDLE_LENGTH_MM)
    return np.asarray(pts), tip.t

# ==========================================
# 4. SETUP GRAFIK 3D MATPLOTLIB
# ==========================================
fig = plt.figure(figsize=(10, 9))
ax = fig.add_subplot(111, projection='3d')
plt.subplots_adjust(left=0.05, right=0.95, bottom=0.42, top=0.97)

ax.set_xlim(*cfg.WORKSPACE['xlim'])
ax.set_ylim(*cfg.WORKSPACE['ylim'])
ax.set_zlim(*cfg.WORKSPACE['zlim'])
ax.set_xlabel('X (mm)'); ax.set_ylabel('Y (mm)'); ax.set_zlabel('Z (mm)')
ax.set_title("Virtual Twin - DOBOT NOVA 5 + STM32 Control")

_segment_lines = [
    ax.plot([], [], [], color=cfg.LINK_COLORS[i], linewidth=6, marker='o', markersize=8)[0]
    for i in range(N_JOINTS)
]
_needle_line, = ax.plot([], [], [], color='yellow', linewidth=4, linestyle='--')

def redraw():
    pts, tip = forward_kinematics([math.radians(v) for v in q_deg])
    for i in range(N_JOINTS):
        _segment_lines[i].set_data_3d(*pts[i:i+2].T)
    _needle_line.set_data_3d(*np.array([pts[-1], tip]).T)
    fig.canvas.draw_idle()

# ==========================================
# 5. UI PANEL & INPUT HANDLING
# ==========================================
value_boxes = []

def _apply_q(idx, value):
    """Update array derajat visual tanpa mengirim ke STM32."""
    lo, hi = cfg.URDF_JOINTS[idx].limit_deg
    value = max(lo, min(hi, value))
    q_deg[idx] = value
    value_boxes[idx].set_val(f"{value:.2f}")
    return value

def set_joint(idx, value):
    """Handler saat pengguna mengetik angka manual di Textbox."""
    old = q_deg[idx]
    new = _apply_q(idx, value)
    redraw()
    send_step(idx, new - old)

def _make_submit(idx):
    def _submit(text):
        try:    
            set_joint(idx, float(text))
        except ValueError: 
            _apply_q(idx, q_deg[idx]) 
    return _submit

# --- Render Tombol & Textbox ---
ROW_H, TOP_Y = 0.045, 0.36
_button_action = {}

def _text_panel(rect, text, ha='center', **kwargs):
    a = plt.axes(rect); a.axis('off')
    a.text(0.5 if ha == 'center' else 0, 0.5, text, ha=ha, va='center', **kwargs)

for i, joint in enumerate(cfg.URDF_JOINTS):
    y = TOP_Y - i * ROW_H
    _text_panel((0.04, y, 0.07, 0.035), joint.name, fontsize=11, fontweight='bold')
    
    minus_btn = Button(plt.axes((0.13, y, 0.06, 0.035)), '−', color='#f5c6c6', hovercolor='#ef6a6a')
    plus_btn = Button(plt.axes((0.35, y, 0.06, 0.035)), '+', color='#c6e8c6', hovercolor='#5fbf5f')
    
    tb = TextBox(plt.axes((0.21, y, 0.12, 0.035)), '', initial='0.00')
    tb.on_submit(_make_submit(i))
    
    lo, hi = joint.limit_deg
    _text_panel((0.42, y, 0.14, 0.035), f'[{lo}°, {hi}°]', ha='left', fontsize=9, color='gray')
    
    value_boxes.append(tb)
    _button_action[minus_btn.ax] = (i, -1)
    _button_action[plus_btn.ax]  = (i, +1)

# --- Tombol Home ---
a_home = plt.axes((0.83, TOP_Y - ROW_H, 0.13, 0.06))
b_home = Button(a_home, 'HOME', color='#c6dcef', hovercolor='#3aa1ef')

def _home(*_):
    for i in range(N_JOINTS): 
        _apply_q(i, 0.0)
    redraw()
    send_home()
b_home.on_clicked(_home)

# ==========================================
# 6. PURE VELOCITY JOG LOGIC (ANTI-JITTER)
# ==========================================
_jog_active_idx  = None
_jog_active_sign = 0
_is_streaming    = False

def _usb_jog_streamer_loop():
    """Background thread murni untuk menembakkan karakter jog tanpa lag UI."""
    sleep_delay = getattr(cfg, 'JOG_STREAM_MS', 40) / 1000.0 
    while True:
        if _jog_active_idx is not None and _is_streaming:
            send_jog_char(_jog_active_idx, _jog_active_sign)
        time.sleep(sleep_delay)

threading.Thread(target=_usb_jog_streamer_loop, daemon=True).start()

_hold_timer  = None
_visual_skip = 0

def _on_gui_tick():
    """Tick khusus Matplotlib. Hanya melayani update visual UI."""
    global _visual_skip
    if _jog_active_idx is None: return

    delta_deg = _jog_active_sign * jog_speed_deg_per_sec(_jog_active_idx) * (getattr(cfg, 'JOG_STREAM_MS', 40) / 1000.0)
    _apply_q(_jog_active_idx, q_deg[_jog_active_idx] + delta_deg)
    
    _visual_skip += 1
    if _visual_skip >= getattr(cfg, 'JOG_REDRAW_EVERY_N', 2):
        _visual_skip = 0
        redraw()

def _on_press(event):
    """Mendeteksi tombol ditekan. Langsung aktifkan mode streaming."""
    global _jog_active_idx, _jog_active_sign, _is_streaming, _hold_timer, _visual_skip
    
    target = _button_action.get(event.inaxes)
    if target is None: return

    _jog_active_idx, _jog_active_sign = target
    _is_streaming = True
    _visual_skip = 0

    if _hold_timer is None:
        _hold_timer = fig.canvas.new_timer(interval=getattr(cfg, 'JOG_STREAM_MS', 40))
        _hold_timer.add_callback(_on_gui_tick)
    _hold_timer.start()

def _on_release(*_):
    """Mendeteksi tombol dilepas. Kirim sinyal rem."""
    global _jog_active_idx, _is_streaming
    
    was_streaming = _is_streaming
    _jog_active_idx = None
    _is_streaming   = False
    
    if _hold_timer is not None:
        _hold_timer.stop()
        
    if was_streaming:
        send_stop_all() 
        redraw()        

fig.canvas.mpl_connect('button_press_event', _on_press)
fig.canvas.mpl_connect('button_release_event', _on_release)

# ==========================================
# 7. MULAI APLIKASI
# ==========================================
send_set_speed()
redraw()
plt.show()

if stm_serial:
    stm_serial.close()