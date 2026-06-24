#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
map_ws_server.py  (ROS2 humble)

subscribe /map (nav_msgs/OccupancyGrid จาก Cartographer) แล้ว broadcast เป็น
WebSocket JSON {type:'map', ...} ที่พอร์ต 8766 ให้ browser ฝั่ง Windows ต่อตรงมาวาด

ออกแบบให้ทำงานคู่กับหน้า index.html เดิม (เปิด WS เส้นที่ 2 แยกจากระบบควบคุม 8765)
JSON 1 ข้อความ = 1 แผนที่:
  { "type":"map", "w":int, "h":int, "res":float,
    "ox":float, "oy":float,          # origin (เมตร) มุมล่างซ้าย
    "data":[int8...] }               # -1=unknown, 0=ว่าง, 100=มีสิ่งกีดขวาง (row-major, ล่างซ้ายขึ้นบน)

วิธีรัน (หลังรัน SLAM แล้ว):
  source ~/Documents/ros2_ws/install/setup.bash
  python3 map_ws_server.py            # ฟังที่ 0.0.0.0:8766
"""

import asyncio
import json
import signal
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import OccupancyGrid

import websockets

HOST = "0.0.0.0"
PORT = 8766


class MapBridge(Node):
    """ROS2 node: subscribe /map แล้วเก็บ JSON ล่าสุด + แจ้ง asyncio ให้ broadcast."""

    def __init__(self, loop, broadcast_cb):
        super().__init__("map_ws_server")
        self._loop = loop
        self._broadcast_cb = broadcast_cb
        self.latest_json = None

        # /map ของ Cartographer เป็น latched: RELIABLE + TRANSIENT_LOCAL
        # ต้องตั้ง QoS ให้ตรง ไม่งั้นจะ subscribe ไม่ติด (ไม่ได้ข้อมูล)
        qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/map", self.on_map, qos)
        self.get_logger().info(f"subscribe /map -> WebSocket {HOST}:{PORT}")

    def on_map(self, msg: OccupancyGrid):
        payload = {
            "type": "map",
            "w": msg.info.width,
            "h": msg.info.height,
            "res": round(msg.info.resolution, 5),
            "ox": round(msg.info.origin.position.x, 4),
            "oy": round(msg.info.origin.position.y, 4),
            "data": list(msg.data),   # int8[] : -1 / 0..100
        }
        self.latest_json = json.dumps(payload)
        occupied = sum(1 for v in msg.data if v >= 50)
        self.get_logger().info(
            f"map {msg.info.width}x{msg.info.height} res={msg.info.resolution:.3f} "
            f"occupied={occupied}"
        )
        # ส่งข้าม thread (rclpy -> asyncio) อย่างปลอดภัย
        self._loop.call_soon_threadsafe(self._broadcast_cb, self.latest_json)


class WsHub:
    """จัดการ WebSocket clients + broadcast แผนที่ล่าสุด."""

    def __init__(self):
        self.clients = set()
        self.last_map = None

    async def handler(self, ws):
        self.clients.add(ws)
        peer = ws.remote_address[0] if ws.remote_address else "?"
        print(f"[ws] client connected: {peer} (รวม {len(self.clients)})")
        try:
            # ส่งแผนที่ล่าสุดให้ client ที่เพิ่งต่อทันที (ไม่ต้องรอ frame ถัดไป)
            if self.last_map is not None:
                await ws.send(self.last_map)
            await ws.wait_closed()
        finally:
            self.clients.discard(ws)
            print(f"[ws] client disconnected: {peer} (เหลือ {len(self.clients)})")

    def broadcast(self, data: str):
        """เรียกจาก asyncio loop (ผ่าน call_soon_threadsafe)."""
        self.last_map = data
        for ws in list(self.clients):
            # ส่งแบบ fire-and-forget; ถ้า client ตายจะถูกเอาออกใน handler
            asyncio.create_task(self._safe_send(ws, data))

    async def _safe_send(self, ws, data):
        try:
            await ws.send(data)
        except Exception:
            self.clients.discard(ws)


async def amain():
    loop = asyncio.get_running_loop()
    hub = WsHub()

    rclpy.init()
    node = MapBridge(loop, hub.broadcast)

    # spin rclpy ใน thread แยก (asyncio อยู่ thread หลัก)
    spin_thread = threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True
    )
    spin_thread.start()

    # Event ที่จะถูกตั้งเมื่อรับ SIGINT/SIGTERM — สำคัญตอนรันผ่าน ros2 launch
    # (ExecuteProcess) เพราะ Ctrl+C ที่ launch ส่ง signal มาแต่ไม่ raise
    # KeyboardInterrupt เข้า main thread ได้เสมอ -> ถ้าไม่ดักไว้ WS จะค้าง port
    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, stop.set)
        except NotImplementedError:
            pass   # บางแพลตฟอร์มไม่รองรับ; fallback เป็น KeyboardInterrupt ด้านล่าง

    async with websockets.serve(hub.handler, HOST, PORT, max_size=None):
        print(f"WebSocket map server ฟังที่ ws://{HOST}:{PORT}")
        try:
            await stop.wait()        # รันค้างจนกว่าจะได้ signal
        finally:
            print(f"map_ws_server ปิด port {PORT} แล้ว")
            node.destroy_node()
            rclpy.shutdown()
    # ออกจาก async with -> websockets.serve ปิด listening socket ปล่อย port ทันที


if __name__ == "__main__":
    try:
        asyncio.run(amain())
    except KeyboardInterrupt:
        pass
