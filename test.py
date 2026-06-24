import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import os

PORT = "COM7"
BAUD = 115200

HEAD_FILE = "heading_zero.txt"

ser = serial.Serial(PORT, BAUD, timeout=0.1)

yaw_raw = 0.0
pitch_raw = 0.0
roll_raw = 0.0

yaw0 = 0.0
pitch = 0.0
roll = 0.0

cal = [0, 0, 0, 0]
heading_zero = 0.0


def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def load_heading_zero():
    global heading_zero
    if os.path.exists(HEAD_FILE):
        try:
            with open(HEAD_FILE, "r") as f:
                heading_zero = float(f.read().strip())
            print("LOAD heading_zero =", heading_zero)
        except:
            heading_zero = 0.0
    else:
        heading_zero = 0.0


def save_heading_zero(value):
    global heading_zero
    heading_zero = value
    with open(HEAD_FILE, "w") as f:
        f.write(str(heading_zero))
    print("SAVE heading_zero =", heading_zero)


load_heading_zero()


def rot_x(a):
    a = np.radians(a)
    return np.array([
        [1, 0, 0],
        [0, np.cos(a), -np.sin(a)],
        [0, np.sin(a), np.cos(a)]
    ])


def rot_y(a):
    a = np.radians(a)
    return np.array([
        [np.cos(a), 0, np.sin(a)],
        [0, 1, 0],
        [-np.sin(a), 0, np.cos(a)]
    ])


def rot_z(a):
    a = np.radians(a)
    return np.array([
        [np.cos(a), -np.sin(a), 0],
        [np.sin(a), np.cos(a), 0],
        [0, 0, 1]
    ])


vertices = np.array([
    [-1, -0.6, -0.2],
    [ 1, -0.6, -0.2],
    [ 1,  0.6, -0.2],
    [-1,  0.6, -0.2],
    [-1, -0.6,  0.2],
    [ 1, -0.6,  0.2],
    [ 1,  0.6,  0.2],
    [-1,  0.6,  0.2],
])

faces_idx = [
    [0, 1, 2, 3],
    [4, 5, 6, 7],
    [0, 1, 5, 4],
    [2, 3, 7, 6],
    [1, 2, 6, 5],
    [0, 3, 7, 4],
]

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")


def on_key(event):
    global heading_zero

    if event.key == "h":
        save_heading_zero(yaw_raw)
        print("HEAD SET: current yaw is 0")

    elif event.key == "r":
        save_heading_zero(0.0)
        print("HEAD RESET")


fig.canvas.mpl_connect("key_press_event", on_key)


def update(frame):
    global yaw_raw, pitch_raw, roll_raw
    global yaw0, pitch, roll, cal

    try:
        while ser.in_waiting:
            line = ser.readline().decode(errors="ignore").strip()

            if line.startswith("IMU,"):
                parts = line.split(",")

                if len(parts) >= 8:
                    yaw_raw = float(parts[1])
                    pitch_raw = float(parts[2])
                    roll_raw = float(parts[3])

                    cal = [
                        int(parts[4]),
                        int(parts[5]),
                        int(parts[6]),
                        int(parts[7])
                    ]

                    # คำนวณหัวใน Python
                    yaw0 = normalize_angle(heading_zero - yaw_raw + 180)

                    # แก้เอียงกลับด้านตรงนี้
                    pitch = pitch_raw
                    roll = roll_raw

    except Exception as e:
        print("ERROR:", e)

    ax.clear()

    R = rot_z(yaw0) @ rot_y(pitch) @ rot_x(roll)
    v = vertices @ R.T

    faces = [[v[i] for i in face] for face in faces_idx]
    poly = Poly3DCollection(faces, alpha=0.65)
    ax.add_collection3d(poly)

    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_zlim(-2, 2)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    ax.set_title(
        f"Yaw={yaw0:.1f} Pitch={pitch:.1f} Roll={roll:.1f}\n"
        f"Raw Yaw={yaw_raw:.1f} PitchRaw={pitch_raw:.1f} RollRaw={roll_raw:.1f}\n"
        f"CAL SYS={cal[0]} G={cal[1]} A={cal[2]} M={cal[3]}\n"
        f"Press h = set/save head | r = reset head"
    )


ani = FuncAnimation(fig, update, interval=50)
plt.show()