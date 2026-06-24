#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
map_marker_pi.py  (ROS2 humble) — ฝั่ง Pi

ทำงานหลักของระบบ "save map + mark จุด scan" ไว้ที่ Pi เพราะ ROS มีของพร้อม:
  - /map (OccupancyGrid จาก Cartographer) มีอยู่แล้ว
  - tf2 หา pose หุ่น (map → base_link) ได้ตรงๆ ไม่ต้องไล่ TF chain เอง
  - เขียน .pgm/.yaml มาตรฐาน map_server ได้เลย

เปิด WebSocket server :8767 ให้ฝั่ง Windows (test2.py / rescue.py / browser) ต่อมา
สั่งงาน:
  รับเข้า:
    {"type":"mark","kind":"qr"|"ai","text":"..."}  → snapshot pose ปัจจุบัน เก็บเป็น mark
    {"type":"save_map"}                            → เขียนไฟล์บน Pi แล้วส่ง bytes กลับ
    {"type":"clear_marks"}
    {"type":"get"}                                 → ขอ marks ปัจจุบัน
  ส่งออก:
    {"type":"marks","marks":[...]}                 → broadcast ทุกครั้งที่ marks เปลี่ยน
    {"type":"save_result","ok":true,
       "files":[{"name":..,"b64":..}, ...]}        → ให้ Windows เขียนลงดิสก์เอง

วิธีรัน (หลังรัน SLAM + rosbridge ตามปกติ):
    source ~/Documents/ros2_ws/install/setup.bash
    python3 map_marker_pi.py
"""

import asyncio
import base64
import datetime
import json
import math
import os
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                       QoSReliabilityPolicy, QoSHistoryPolicy)
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener

import websockets

HOST = "0.0.0.0"
PORT = 8767

MAP_FRAME  = "map"
BASE_FRAME = "base_link"        # เปลี่ยนเป็น base_footprint ถ้า TF tree ใช้ชื่อนั้น

SAVE_DIR = os.path.expanduser("~/saved_maps")   # ที่เก็บชั่วคราวบน Pi
OCC_THRESH  = 65
FREE_THRESH = 25


# ═══════════════════════════════════════════════════════════════
#  ROS2 worker node
# ═══════════════════════════════════════════════════════════════
class MarkerNode(Node):
    def __init__(self):
        super().__init__("map_marker_pi")
        self.latest_map = None
        self.marks = []
        self._mark_id = 0

        # /map ของ Cartographer = latched (RELIABLE + TRANSIENT_LOCAL)
        qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/map", self._on_map, qos)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info(f"map_marker_pi → WebSocket {HOST}:{PORT}")

    def _on_map(self, msg: OccupancyGrid):
        self.latest_map = msg

    # ── pose หุ่นในเฟรม map (x,y,yaw) จาก tf2 ──
    def get_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                MAP_FRAME, BASE_FRAME, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"lookup {MAP_FRAME}->{BASE_FRAME} fail: {e}")
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (t.transform.translation.x, t.transform.translation.y, yaw)

    # ── ปัก mark ที่ตำแหน่งหุ่นปัจจุบัน ──
    def add_mark(self, kind, text):
        pose = self.get_pose()
        if pose is None:
            x = y = yaw = None
            pose_ok = False
        else:
            x, y, yaw = pose
            pose_ok = True
        self._mark_id += 1
        mark = {
            "id": self._mark_id,
            "kind": kind,                 # 'qr' | 'ai'
            "text": text or "",
            "x": x, "y": y, "yaw": yaw,   # เมตร ในเฟรม map
            "pose_ok": pose_ok,
            "t": datetime.datetime.now().isoformat(timespec="seconds"),
        }
        self.marks.append(mark)
        self.get_logger().info(
            f"mark +{kind} #{mark['id']} '{text}' "
            f"pose={'(%.2f,%.2f)' % (x, y) if pose_ok else 'unknown'}")
        return mark

    # ── เขียน .pgm/.yaml/_marks.json บน Pi → คืน path list ──
    def save_files(self):
        msg = self.latest_map
        if msg is None:
            return None, "ยังไม่มีข้อมูล /map"
        w = msg.info.width
        h = msg.info.height
        res = round(msg.info.resolution, 5)
        ox = round(msg.info.origin.position.x, 4)
        oy = round(msg.info.origin.position.y, 4)
        data = msg.data

        os.makedirs(SAVE_DIR, exist_ok=True)
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        base = f"map_{ts}"
        pgm_path   = os.path.join(SAVE_DIR, base + ".pgm")
        yaml_path  = os.path.join(SAVE_DIR, base + ".yaml")
        marks_path = os.path.join(SAVE_DIR, base + "_marks.json")

        # .pgm (P5) — ROS: row 0 = บนสุด ; grid เริ่มมุมล่างซ้าย → พลิกแนวตั้ง
        buf = bytearray()
        row = bytearray(w)
        for i in range(h):
            g = (h - 1 - i) * w
            for x in range(w):
                v = data[g + x]
                if v < 0:              px = 205
                elif v >= OCC_THRESH:  px = 0
                elif v <= FREE_THRESH: px = 254
                else:                  px = 205
                row[x] = px
            buf += row
        with open(pgm_path, "wb") as f:
            f.write(b"P5\n")
            f.write(f"# saved by map_marker_pi @ {ts}\n".encode())
            f.write(f"{w} {h}\n255\n".encode())
            f.write(buf)

        with open(yaml_path, "w", encoding="utf-8") as f:
            f.write(f"image: {base}.pgm\n")
            f.write(f"resolution: {res}\n")
            f.write(f"origin: [{ox}, {oy}, 0.0]\n")
            f.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")

        with open(marks_path, "w", encoding="utf-8") as f:
            json.dump({
                "saved": ts,
                "map": {"w": w, "h": h, "res": res, "ox": ox, "oy": oy},
                "marks": self.marks,
            }, f, ensure_ascii=False, indent=2)

        self.get_logger().info(f"saved → {SAVE_DIR}/{base}.*")
        return [pgm_path, yaml_path, marks_path], None


# ═══════════════════════════════════════════════════════════════
#  WebSocket hub
# ═══════════════════════════════════════════════════════════════
class WsHub:
    def __init__(self, node: MarkerNode):
        self.node = node
        self.clients = set()

    async def _send(self, ws, payload):
        try:
            await ws.send(json.dumps(payload, ensure_ascii=False))
        except Exception:
            self.clients.discard(ws)

    async def broadcast(self, payload):
        for ws in list(self.clients):
            await self._send(ws, payload)

    async def push_marks(self):
        await self.broadcast({"type": "marks", "marks": self.node.marks})

    async def handler(self, ws):
        self.clients.add(ws)
        peer = ws.remote_address[0] if ws.remote_address else "?"
        print(f"[ws] connected {peer} (รวม {len(self.clients)})")
        try:
            await self._send(ws, {"type": "marks", "marks": self.node.marks})
            async for raw in ws:
                try:
                    msg = json.loads(raw)
                except Exception:
                    continue
                t = msg.get("type", "")

                if t == "mark":
                    self.node.add_mark(msg.get("kind", "qr"), msg.get("text", ""))
                    await self.push_marks()

                elif t == "clear_marks":
                    self.node.marks.clear()
                    await self.push_marks()

                elif t == "get":
                    await self._send(ws, {"type": "marks", "marks": self.node.marks})

                elif t == "save_map":
                    files, err = self.node.save_files()
                    if err:
                        await self._send(ws, {"type": "save_result", "ok": False, "error": err})
                    else:
                        payload = []
                        for p in files:
                            with open(p, "rb") as f:
                                payload.append({
                                    "name": os.path.basename(p),
                                    "b64": base64.b64encode(f.read()).decode("ascii"),
                                })
                        await self._send(ws, {"type": "save_result", "ok": True,
                                              "files": payload})
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.clients.discard(ws)
            print(f"[ws] disconnected {peer} (เหลือ {len(self.clients)})")


# ═══════════════════════════════════════════════════════════════
#  main
# ═══════════════════════════════════════════════════════════════
async def amain():
    rclpy.init()
    node = MarkerNode()
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()

    hub = WsHub(node)
    async with websockets.serve(hub.handler, HOST, PORT, max_size=None):
        print(f"map_marker_pi ฟังที่ ws://{HOST}:{PORT}")
        try:
            await asyncio.Future()       # รันค้าง
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    try:
        asyncio.run(amain())
    except KeyboardInterrupt:
        pass
