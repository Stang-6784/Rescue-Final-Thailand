#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lidar_sim.py — จำลองยิง SLAM map (OccupancyGrid) ทาง UDP เข้า rescue.py
ใช้ทดสอบ flow lidar โดยไม่ต้องมี Pi จริง

วิธีใช้ (รัน rescue.py ไว้ก่อน แล้วเปิดอีก terminal):
    python lidar_sim.py                 # ยิงเข้า 127.0.0.1:8766
    python lidar_sim.py 192.168.1.50    # ยิงเข้าเครื่องอื่น
"""
import json
import math
import socket
import sys
import time

try: sys.stdout.reconfigure(encoding="utf-8")
except Exception: pass

HOST = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
PORT = 8766

W, H, RES = 120, 120, 0.05          # 120x120 cell, 5cm/cell = 6x6 m
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
print(f"ยิง map จำลองไป {HOST}:{PORT} (Ctrl+C เพื่อหยุด)")

frame = 0
try:
    while True:
        frame += 1
        # -1 = unknown, 0 = free, 100 = obstacle
        data = [-1] * (W * H)
        # พื้นที่ว่างตรงกลาง
        for y in range(20, H - 20):
            for x in range(20, W - 20):
                data[y * W + x] = 0
        # กำแพงสี่ด้าน
        for x in range(20, W - 20):
            data[20 * W + x] = 100
            data[(H - 21) * W + x] = 100
        for y in range(20, H - 20):
            data[y * W + 20] = 100
            data[y * W + (W - 21)] = 100
        # สิ่งกีดขวางวิ่งวนเป็นวงกลม (ให้เห็นว่า map อัปเดต)
        cx = W // 2 + int(25 * math.cos(frame * 0.1))
        cy = H // 2 + int(25 * math.sin(frame * 0.1))
        for dy in range(-2, 3):
            for dx in range(-2, 3):
                xx, yy = cx + dx, cy + dy
                if 0 <= xx < W and 0 <= yy < H:
                    data[yy * W + xx] = 100

        msg = {"type": "map", "w": W, "h": H, "res": RES, "data": data}
        sock.sendto(json.dumps(msg).encode("utf-8"), (HOST, PORT))
        print(f"frame {frame:5d}  ส่ง {W}x{H} cell")
        time.sleep(0.2)   # 5 Hz
except KeyboardInterrupt:
    print("\nหยุดแล้ว")
finally:
    sock.close()
