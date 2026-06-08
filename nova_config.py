"""Konfigurasi Dobot Nova 5 virtual twin + STM32 bridge.

Edit file ini (bukan virtual_nova.py) untuk mengubah port serial, limit
sendi, spesifikasi motor, atau geometri URDF.
"""
from collections import namedtuple

# ---------- Serial link ke STM32 ----------
COM_PORT  = 'COM5'
BAUD_RATE = 115200

# ---------- Visual ----------
LINK_COLORS      = ['black', 'red', 'orange', 'green', 'blue', 'purple']
NEEDLE_LENGTH_MM = 80.0     # panjang penunjuk end-effector

# ---------- Rantai kinematik (dari URDF resmi Dobot Nova 5) ----------
# Tiap sendi: transform tetap (xyz dalam mm + rpy dalam radian, order ZYX)
# yang diterapkan SEBELUM rotasi revolute Rz(q). limit_deg = batas software.
Joint = namedtuple('Joint', ['name', 'xyz', 'rpy', 'limit_deg'])

URDF_JOINTS = [
    Joint('Joint 1', (   0.0,    0.0, 240.0), ( 0.0,         0.0,         0.0),        (-360, 360)),
    Joint('Joint 2', (   0.0,    0.0,   0.0), (-1.57080287,  1.53586622,  3.14159265), (-180, 180)),
    Joint('Joint 3', (-399.756, -13.969, 0.0), ( 0.0,         0.0,         0.0),        (-160, 160)),
    Joint('Joint 4', (-329.798, -11.524, 135.0), ( 0.0,       0.0,        -1.53586622), (-360, 360)),
    Joint('Joint 5', (   0.0, -120.0,   0.0), ( 1.5708,      0.0,         0.0),        (-360, 360)),
    Joint('Joint 6', (   0.0,   88.328, 0.0), (-1.5708,      0.0,         0.0),        (-360, 360)),
]

# ---------- Spesifikasi stepper per sendi ----------
PULSE_PER_REV = [2000, 2000, 2000, 1600, 1600, 1600]
GEAR_RATIO    = [ 100,   50,   40,   30,   15,   10]

# ---------- Batas ruang plot 3D (mm) ----------
WORKSPACE = {
    'xlim': (-900, 900),
    'ylim': (-900, 900),
    'zlim': (0,   1200),
}

# ---------- UI jog (klik = step, tahan = jog kontinu ala Dobot Studio) ----------
STEP_SIZE_DEG         = 1.0   # besar 1x klik tombol +/- (derajat)
JOG_HALFPERIOD_US     = 250   # kecepatan jog firmware: us / setengah pulsa (kecil = cepat)
JOG_HOLD_THRESHOLD_MS = 350   # tahan >= ini ms dianggap "hold" → mulai stream jog
JOG_STREAM_MS         = 40    # interval kirim karakter jog ke STM32 saat hold
JOG_REDRAW_EVERY_N    = 2     # redraw visual tiap N tick (hemat CPU, serial tetap di JOG_STREAM_MS)

# Karakter real-time yang dipahami firmware STM32: (kunci_negatif, kunci_positif)
# untuk tiap sendi. Firmware menambah/mengurangi targetPosition tiap karakter
# (besar increment ditentukan oleh JOG_STEP_INCREMENT di firmware), dan akan
# auto-stop bila stream berhenti > JOG_TIMEOUT_MS.
JOG_KEYS = [
    ('s', 'w'),  # M0
    ('a', 'd'),  # M1
    ('e', 'q'),  # M2
    ('f', 'r'),  # M3  -- butuh firmware patch (lihat README/perintah)
    ('g', 't'),  # M4  -- butuh firmware patch
    ('h', 'y'),  # M5  -- butuh firmware patch
]
STOP_ALL_CHAR = ' '   # firmware: spasi = emergency stop semua motor
