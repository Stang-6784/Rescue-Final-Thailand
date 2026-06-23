#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
windows_scan_receiver.py

รันบนเครื่อง Windows เพื่อรับข้อมูล lidar (UDP JSON) จาก Raspberry Pi
ต้องมีแค่ Python 3 (ไม่ต้องลง ROS2)

วิธีรันบน Windows (PowerShell / cmd):
    python windows_scan_receiver.py
    python windows_scan_receiver.py --port 9000

หมายเหตุ: เปิด Windows Firewall ให้ Python รับ inbound UDP พอร์ต 9000
"""

import argparse
import json
import math
import socket


def main():
    parser = argparse.ArgumentParser(description="รับข้อมูล lidar จาก ROS2 ผ่าน UDP")
    parser.add_argument("--host", default="0.0.0.0",
                        help="IP ที่จะ bind (ค่าเริ่มต้นรับทุก interface)")
    parser.add_argument("--port", type=int, default=9000, help="พอร์ต UDP")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.host, args.port))
    print(f"กำลังฟัง UDP {args.host}:{args.port} ... (Ctrl+C เพื่อหยุด)")

    frame = 0
    try:
        while True:
            # 65535 = ขนาดสูงสุดของ UDP datagram
            data, addr = sock.recvfrom(65535)
            try:
                scan = json.loads(data.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                print(f"  decode ไม่สำเร็จ: {exc}")
                continue

            frame += 1
            ranges = scan.get("ranges", [])
            valid = [r for r in ranges if r is not None]

            # ตัวอย่างการใช้งาน: หาจุดที่ใกล้ที่สุด + มุมของมัน
            nearest_txt = "-"
            if valid:
                idx = min(range(len(ranges)),
                          key=lambda i: ranges[i] if ranges[i] is not None else math.inf)
                ang_deg = math.degrees(scan["angle_min"] + idx * scan["angle_increment"])
                nearest_txt = f"{ranges[idx]:.3f} m @ {ang_deg:6.1f} deg"

            print(f"frame {frame:5d} | จาก {addr[0]} | "
                  f"จุด {len(ranges):3d} (valid {len(valid):3d}) | "
                  f"ใกล้สุด {nearest_txt}")
    except KeyboardInterrupt:
        print("\nหยุดแล้ว")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
