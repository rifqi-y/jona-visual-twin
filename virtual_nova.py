"""Dobot Nova 5 virtual twin + STM32 jog control.

Konfigurasi (port serial, limit sendi, gear ratio, geometri URDF) ada di
nova_config.py — file ini hanya logika aplikasi dan UI.
"""
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

stm_serial = None
try:
    stm_serial = serial.Serial(cfg.COM_PORT, cfg.BAUD_RATE, timeout=0.1)
    print(f"Berhasil terhubung ke STM32 di {cfg.COM_PORT}!")
except serial.SerialException:
    print(f"Peringatan: Tidak dapat terhubung ke {cfg.COM_PORT}. Mode SIMULASI OFFLINE.")

def _read_serial_loop():
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
    if stm_serial is not None:
        stm_serial.write(payload)

# Protokol serial ke firmware STM32:
#   M:idx:steps -> step diskrit (mode COMMAND)
#   JOG_KEYS    -> jog real-time 1 karakter/tick; firmware auto-stop ~200 ms
#   spasi       -> emergency stop semua motor (target = current)
#   V:idx:us    -> set halfPeriodMicros (kecepatan jog)
#   H           -> homing
def send_step(idx, delta_deg):
    if delta_deg == 0:
        return
    steps = int(round((delta_deg / 360.0) * cfg.PULSE_PER_REV[idx] * cfg.GEAR_RATIO[idx]))
    _send(f"M:{idx}:{steps}\n".encode('utf-8'))

def send_jog_char(idx, sign):
    neg, pos = cfg.JOG_KEYS[idx]
    _send((pos if sign > 0 else neg).encode('ascii'))

def send_stop_all():
    _send(cfg.STOP_ALL_CHAR.encode('ascii'))

def send_home():
    _send(b"H\n")

def send_set_speed(halfperiod_us):
    for idx in range(N_JOINTS):
        _send(f"V:{idx}:{halfperiod_us}\n".encode('utf-8'))

def jog_speed_deg_per_sec(idx):
    """Kecepatan sendi teoretis saat jog kontinu — dipakai untuk animasi visual."""
    motor_steps_per_sec = 1e6 / (2 * cfg.JOG_HALFPERIOD_US)
    return motor_steps_per_sec / (cfg.PULSE_PER_REV[idx] * cfg.GEAR_RATIO[idx]) * 360.0

q_deg = [0.0] * N_JOINTS

# Transform tetap per sendi (xyz + rpy) dihitung sekali, lalu di-reuse tiap FK.
_joint_offsets = [sm.SE3(*j.xyz) * sm.SE3.RPY(*j.rpy) for j in cfg.URDF_JOINTS]

def forward_kinematics(q_rad):
    pts = []
    T = sm.SE3()
    pts.append(T.t)
    for i in range(N_JOINTS):
        T = T * _joint_offsets[i] * sm.SE3.Rz(q_rad[i])
        pts.append(T.t)
    tip = T * sm.SE3.Tz(cfg.NEEDLE_LENGTH_MM)
    return np.asarray(pts), tip.t   # pts[-1] = ujung sendi 6, tip = ujung jarum

fig = plt.figure(figsize=(10, 9))
ax = fig.add_subplot(111, projection='3d')
plt.subplots_adjust(left=0.05, right=0.95, bottom=0.42, top=0.97)

ax.set_xlim(*cfg.WORKSPACE['xlim'])
ax.set_ylim(*cfg.WORKSPACE['ylim'])
ax.set_zlim(*cfg.WORKSPACE['zlim'])
ax.set_xlabel('X (mm)'); ax.set_ylabel('Y (mm)'); ax.set_zlabel('Z (mm)')
ax.set_title("Virtual Twin - DOBOT NOVA 5 + STM32 Control")

# Artist dibuat sekali; redraw() hanya update data lewat set_data_3d.
_segment_lines = [
    ax.plot([], [], [], color=cfg.LINK_COLORS[i],
            linewidth=6, marker='o', markersize=8)[0]
    for i in range(N_JOINTS)
]
_needle_line, = ax.plot([], [], [], color='yellow', linewidth=4, linestyle='--')

def redraw():
    pts, tip = forward_kinematics([math.radians(v) for v in q_deg])
    for i in range(N_JOINTS):
        _segment_lines[i].set_data_3d(*pts[i:i+2].T)
    _needle_line.set_data_3d(*np.array([pts[-1], tip]).T)
    fig.canvas.draw_idle()

value_boxes = []

def _apply_q(idx, value):
    """Clamp ke limit lalu update q_deg + textbox. TIDAK redraw, TIDAK kirim serial."""
    lo, hi = cfg.URDF_JOINTS[idx].limit_deg
    value = max(lo, min(hi, value))
    q_deg[idx] = value
    value_boxes[idx].set_val(f"{value:.2f}")
    return value

def set_joint(idx, value):
    """Pindah sendi ke nilai absolut: update visual + kirim selisih step ke STM32."""
    old = q_deg[idx]
    new = _apply_q(idx, value)
    redraw()
    send_step(idx, new - old)

def nudge_joint(idx, sign):
    set_joint(idx, q_deg[idx] + sign * cfg.STEP_SIZE_DEG)

def _make_submit(idx):
    def _submit(text):
        try:    set_joint(idx, float(text))
        except ValueError: _apply_q(idx, q_deg[idx])  # revert textbox ke nilai sah
    return _submit

# --- Panel jog (− / value / +) ala Dobot Studio ---
ROW_H = 0.045
TOP_Y = 0.36

def _text_panel(rect, text, x=0.5, ha='center', **kwargs):
    a = plt.axes(rect); a.axis('off')
    a.text(x, 0.5, text, ha=ha, va='center', **kwargs)

def _build_joint_row(i, joint, y):
    _text_panel((0.04, y, 0.07, 0.035), joint.name, fontsize=11, fontweight='bold')

    minus_btn = Button(plt.axes((0.13, y, 0.06, 0.035)),
                       '−', color='#f5c6c6', hovercolor='#ef6a6a')

    tb = TextBox(plt.axes((0.21, y, 0.12, 0.035)), '', initial='0.00')
    tb.on_submit(_make_submit(i))

    plus_btn = Button(plt.axes((0.35, y, 0.06, 0.035)),
                      '+', color='#c6e8c6', hovercolor='#5fbf5f')

    lo, hi = joint.limit_deg
    _text_panel((0.42, y, 0.14, 0.035), f'[{lo}°, {hi}°]',
                x=0, ha='left', fontsize=9, color='gray')

    return minus_btn, plus_btn, tb

_jog_buttons   = []   # retainer agar Button tidak di-GC oleh matplotlib
_button_action = {}   # event.inaxes -> (joint_idx, sign)
for i, joint in enumerate(cfg.URDF_JOINTS):
    minus, plus, tb = _build_joint_row(i, joint, TOP_Y - i * ROW_H)
    value_boxes.append(tb)
    _jog_buttons.extend([minus, plus])
    _button_action[minus.ax] = (i, -1)
    _button_action[plus.ax]  = (i, +1)

a = plt.axes((0.83, TOP_Y - ROW_H, 0.13, 0.06))
b_home = Button(a, 'HOME', color='#c6dcef', hovercolor='#3aa1ef')
def _home(*_):
    for i in range(N_JOINTS):
        _apply_q(i, 0.0)
    redraw()
    send_home()
b_home.on_clicked(_home)

# ===================================================================
# --- NEW: BACKGROUND THREAD KHUSUS JOGGING (ANTI JITTER) ---
# ===================================================================
# Variabel global untuk komunikasi antar thread
_jog_active_idx  = None
_jog_active_sign = 0
_is_streaming    = False

def _usb_jog_streamer_loop():
    """Berjalan di background: Hanya bertugas menembakkan karakter ke STM32 tanpa lag."""
    # Ambil delay dari config, fallback ke 40ms jika tidak ada
    sleep_delay = getattr(cfg, 'JOG_STREAM_MS', 40) / 1000.0 
    
    while True:
        if _jog_active_idx is not None and _is_streaming:
            send_jog_char(_jog_active_idx, _jog_active_sign)
        time.sleep(sleep_delay)

# Mulai thread streamer
threading.Thread(target=_usb_jog_streamer_loop, daemon=True).start()

# --- Klik = step, tahan = jog kontinu (Visual Only) ---
_hold_timer   = None
_press_time   = 0.0
_visual_skip  = 0

def _on_gui_tick():
    """Fungsi ini dipanggil oleh GUI Matplotlib. Boleh lag, motor fisik tetap aman."""
    global _is_streaming, _visual_skip
    
    if _jog_active_idx is None:
        return
        
    # Cek apakah ditahan cukup lama untuk masuk mode streaming
    if not _is_streaming:
        if (time.monotonic() - _press_time) * 1000.0 >= getattr(cfg, 'JOG_HOLD_THRESHOLD_MS', 300):
            _is_streaming = True
        return # Masih tahap 'klik', belum streaming

    # 1. Update variabel derajat untuk visualisasi saja
    delta_deg = _jog_active_sign * jog_speed_deg_per_sec(_jog_active_idx) * (getattr(cfg, 'JOG_STREAM_MS', 40) / 1000.0)
    _apply_q(_jog_active_idx, q_deg[_jog_active_idx] + delta_deg)
    
    # 2. Render grafis (Matplotlib mungkin nge-lag di sini, tapi thread USB tetap aman!)
    _visual_skip += 1
    if _visual_skip >= getattr(cfg, 'JOG_REDRAW_EVERY_N', 2):
        _visual_skip = 0
        redraw()

def _on_press(event):
    global _jog_active_idx, _jog_active_sign, _press_time, _is_streaming, _hold_timer, _visual_skip
    
    target = _button_action.get(event.inaxes)
    if target is None:
        return

    _jog_active_idx, _jog_active_sign = target
    _press_time  = time.monotonic()
    _is_streaming = False
    _visual_skip = 0

    # Kirim 1 step saat ditekan pertama kali
    nudge_joint(*target)  

    if _hold_timer is None:
        _hold_timer = fig.canvas.new_timer(interval=getattr(cfg, 'JOG_STREAM_MS', 40))
        _hold_timer.add_callback(_on_gui_tick)
    _hold_timer.start()

def _on_release(*_):
    global _jog_active_idx, _is_streaming
    
    was_streaming = _is_streaming
    
    # Matikan stream USB dan GUI
    _jog_active_idx = None
    _is_streaming   = False
    
    if _hold_timer is not None:
        _hold_timer.stop()
        
    if was_streaming:
        send_stop_all() # Tembak spasi untuk emergency brake di STM32
        redraw()        # Sinkronisasi visual terakhir

fig.canvas.mpl_connect('button_press_event', _on_press)
fig.canvas.mpl_connect('button_release_event', _on_release)

# Sinkronkan kecepatan firmware ke config (firmware boot pakai 250 us).
send_set_speed(cfg.JOG_HALFPERIOD_US)

redraw()
plt.show()

if stm_serial:
    stm_serial.close()