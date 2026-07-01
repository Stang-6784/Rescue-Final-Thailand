#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
============================================================================
 tcp_bridge_node.py  —  TCP(Windows rescue.py)  ⇄  ROS2 topics
============================================================================
 รันบน Pi/Ubuntu เป็น ROS2 node. หน้าที่:
   1) เป็น TCP SERVER (port 9000) ให้ rescue.py ฝั่ง Windows มาต่อ
   2) "แปลโปรโตคอล" rescue.py -> คำสั่ง Teensy แล้ว publish ลง ROS2 topic:
        - JSON สำหรับมอเตอร์  -> /motor/command   (std_msgs/String)
        - text SERVO/POSTURE  -> /arm/command     (std_msgs/String)
      (ตรงกับ contract ที่ control_motor_servo.py / teensy_serial_node รับ)
   3) subscribe /teensy/feedback (telemetry จาก Teensy ที่ serial node อ่านมา)
      แล้วส่งกลับขึ้น Windows ผ่าน TCP

 ตาราง map (rescue.py -> ROS2 topic):
   move_start(key,lin,ang) -> /motor/command  {"type":"drive","linear","angular"}
   move_stop               -> /motor/command  {"type":"drive","linear":0,"angular":0}
   lock                    -> /motor/command  drive 0 + {"type":"motor_all","action":"stop"}
   motor/motor_sync/motor_all/ping -> /motor/command  (JSON ผ่านตรงๆ)
   servo_set(index,angle)  -> /arm/command    "SERVO <i> <deg>"
   motor_grip/reverse/stop -> /arm/command    "SERVO 5 <deg>"  (gripper = servo 5)
   posture(name)           -> /arm/command    "POSTURE <name>"
   arm_raw(cmd)           -> /arm/command    "<cmd>"
============================================================================
"""

import socket
import json
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# ═══════════════════════════════════════════════════════════════
#  ค่าคงที่สำหรับการแปลโปรโตคอล (แก้ในไฟล์ได้, ส่วน max_lin/max_ang เป็น ROS param)
# ═══════════════════════════════════════════════════════════════
#  ทิศของแต่ละปุ่ม: (fwd, turn)  fwd:+1 = หน้า / turn:+1 = เลี้ยวซ้าย
#  ถ้าหน้า/หลัง หรือ ซ้าย/ขวากลับด้าน -> สลับเครื่องหมายตรงนี้ (หรือแก้ MOTOR_DIR_* ใน .ino)
KEY_VECTORS = {
    'w': (+1.0,  0.0),
    's': (-1.0,  0.0),
    'a': ( 0.0, +1.0),
    'd': ( 0.0, -1.0),
    'q': (+1.0, +1.0),
    'e': (+1.0, -1.0),
    'z': (-1.0, +1.0),
    'c': (-1.0, -1.0),
}

# Gripper = servo index 5 ใน Teensy build นี้ (clamp [45,90] ใน .ino)
GRIP_SERVO_IDX = 5
GRIP_CLOSE_DEG = 45
GRIP_OPEN_DEG  = 70
GRIP_MID_DEG   = 58

TEENSY_POSTURES = {"home", "guard", "giraff", "stair"}

HEARTBEAT_SEC = 1.0


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class TcpBridgeNode(Node):
    def __init__(self):
        super().__init__("tcp_bridge")

        # ── ROS2 parameters ──
        self.declare_parameter("tcp_host", "0.0.0.0")
        self.declare_parameter("tcp_port", 9000)
        self.declare_parameter("max_lin", 0.60)   # m/s  ที่ map เป็น linear=100
        self.declare_parameter("max_ang", 2.00)   # rad/s ที่ map เป็น angular=100

        self.tcp_host = self.get_parameter("tcp_host").value
        self.tcp_port = int(self.get_parameter("tcp_port").value)
        self.max_lin  = float(self.get_parameter("max_lin").value)
        self.max_ang  = float(self.get_parameter("max_ang").value)

        # ── Publishers (ออกไปหา serial node / Teensy) ──
        self.pub_motor = self.create_publisher(String, "/motor/command", 10)
        self.pub_arm   = self.create_publisher(String, "/arm/command", 10)
        # ── คำสั่งปักหมุด hazmat/QR → map_marker_pi (subscribe /mark/request) ──
        self.pub_mark  = self.create_publisher(String, "/mark/request", 10)

        # ── Subscriber (telemetry กลับจาก Teensy) ──
        self.create_subscription(String, "/teensy/feedback",
                                 self._on_feedback, 10)
        # imu_node parse บรรทัด "IMU,..." เป็น JSON สะอาด แล้ว publish มาที่นี่
        # -> forward ตรงๆ ขึ้น Windows (rescue.py รับ {"type":"imu",...})
        self.create_subscription(String, "/imu/win", self._on_imu_win, 10)
        # สถานะ serial จริงจาก serial node (true/false) — ไม่เดาเอง
        self._serial_ok = False
        self.create_subscription(String, "/teensy/status",
                                 self._on_status, 10)

        # ── TCP client state ──
        self._client      = None
        self._client_lock = threading.Lock()
        self._srv_sock    = None     # server socket (ตั้งใน _serve)
        self._closing     = False    # True ตอน node กำลังปิด -> ออก accept loop

        # heartbeat ส่ง state ขึ้น Windows ให้ UI รู้ว่าระบบยังมีชีวิต
        self.create_timer(HEARTBEAT_SEC, self._heartbeat)

        # TCP server thread
        threading.Thread(target=self._serve, daemon=True).start()

        self.get_logger().info(
            f"tcp_bridge listening on {self.tcp_host}:{self.tcp_port} "
            f"(max_lin={self.max_lin}, max_ang={self.max_ang})")

    def destroy_node(self):
        # ปิด server socket ให้ accept() หลุด + ปล่อย port ทันที (กัน Address already in use)
        self._closing = True
        if self._srv_sock is not None:
            try: self._srv_sock.close()
            except Exception: pass
        with self._client_lock:
            sock = self._client
            self._client = None
        if sock is not None:
            try: sock.close()
            except Exception: pass
        super().destroy_node()

    # ───────────────────── ส่งขึ้น Windows ─────────────────────
    def _send_win(self, payload):
        with self._client_lock:
            sock = self._client
        if sock is None:
            return
        try:
            sock.sendall((json.dumps(payload) + "\n").encode("utf-8"))
        except Exception as e:
            self.get_logger().warn(f"tcp send error: {e}")
            self._drop_client(sock)

    def _on_status(self, msg: String):
        # serial node บอกสถานะจริง -> เก็บไว้ + ส่งขึ้น Windows ทันทีเมื่อเปลี่ยน
        ok = (msg.data.strip().lower() == "true")
        if ok != self._serial_ok:
            self._serial_ok = ok
            self.get_logger().info(f"serial status from node -> {ok}")
            self._send_win({"type": "state", "serial": ok, "from_serial": True})

    def _heartbeat(self):
        # ส่งสถานะ serial "จริง" ที่ได้จาก serial node (ไม่ optimistic อีกต่อไป)
        self._send_win({"type": "state",
                        "serial": self._serial_ok,
                        "from_serial": True})

    # ───────────── /imu/win -> Windows ─────────────
    def _on_imu_win(self, msg: String):
        line = msg.data.strip()
        if not line:
            return
        try:
            self._send_win(json.loads(line))   # {"type":"imu",...} จาก imu_node
        except Exception:
            pass

    # ───────────── /teensy/feedback -> Windows ─────────────
    def _on_feedback(self, msg: String):
        line = msg.data.strip()
        if not line:
            return
        # บรรทัด IMU ดิบส่งผ่านช่อง /imu/win (parse แล้ว) -> ไม่ต้องดันเป็น arm_log ซ้ำ
        if line.startswith("IMU,"):
            return
        if line.startswith("{"):
            try:
                obj = json.loads(line)
            except Exception:
                self._send_win({"type": "arm_log", "text": line})
                return
            t = obj.get("type", "")
            if t == "pong":
                self._send_win({"type": "motor_ack", "ack": "pong"})
            else:
                self._send_win(obj)          # motor_feedback ฯลฯ ส่งต่อตรงๆ
        else:
            self._send_win({"type": "arm_log", "text": line})

    # ───────────── Windows msg -> publish ROS2 ─────────────
    def _publish_motor(self, payload_dict):
        m = String(); m.data = json.dumps(payload_dict)
        self.pub_motor.publish(m)

    def _publish_arm(self, text):
        m = String(); m.data = text
        self.pub_arm.publish(m)

    def _on_win_msg(self, msg):
        t = msg.get("type", "")

        # ── ขับเคลื่อน -> /motor/command (JSON) ──
        if t == "move_start":
            key = msg.get("key", "")
            if key not in KEY_VECTORS:
                return
            lin = float(msg.get("lin", 0.0))
            ang = float(msg.get("ang", 0.0))
            fwd, turn = KEY_VECTORS[key]
            linear  = clamp(fwd  * (lin / self.max_lin) * 100.0, -100, 100)
            angular = clamp(turn * (ang / self.max_ang) * 100.0, -100, 100)
            self._publish_motor({"type": "drive",
                                 "linear": round(linear, 1),
                                 "angular": round(angular, 1)})
            return

        if t == "move_stop":
            self._publish_motor({"type": "drive", "linear": 0, "angular": 0})
            return

        if t == "lock":
            self._publish_motor({"type": "drive", "linear": 0, "angular": 0})
            self._publish_motor({"type": "motor_all", "action": "stop"})
            return

        if t == "unlock":
            return  # มอเตอร์ปลุกเองตอน drive ครั้งถัดไป

        if t == "set_speed":
            return  # ใช้ lin/ang ที่แนบมากับ move_start อยู่แล้ว

        if t in ("motor", "motor_sync", "motor_all", "ping"):
            self._publish_motor(msg)         # JSON ผ่านตรงๆ
            return

        # ── ปักหมุด hazmat/QR ลงแผนที่ -> /mark/request (map_marker_pi รับไปหา pose) ──
        if t == "mark":
            kind = str(msg.get("kind", "qr"))
            text = str(msg.get("text", ""))
            m = String(); m.data = json.dumps({"kind": kind, "text": text})
            self.pub_mark.publish(m)
            self.get_logger().info(f"mark -> {kind}: {text!r}")
            return

        # ── Servo / Gripper / Posture / arm_raw -> /arm/command (text) ──
        if t == "servo_set":
            idx = int(msg.get("index", 0))
            deg = int(round(float(msg.get("angle", 90))))
            self._publish_arm(f"SERVO {idx} {deg}")
            return

        if t == "servo_set_multi":
            for idx, angle in msg.get("joints", {}).items():
                self._publish_arm(f"SERVO {int(idx)} {int(round(float(angle)))}")
            return

        if t == "motor_grip":
            self._publish_arm(f"SERVO {GRIP_SERVO_IDX} {GRIP_CLOSE_DEG}"); return
        if t == "motor_reverse":
            self._publish_arm(f"SERVO {GRIP_SERVO_IDX} {GRIP_OPEN_DEG}"); return
        if t == "motor_stop":
            self._publish_arm(f"SERVO {GRIP_SERVO_IDX} {GRIP_MID_DEG}"); return

        if t == "posture":
            name = str(msg.get("name", "")).lower()
            if name in TEENSY_POSTURES:
                self._publish_arm(f"POSTURE {name}")
            else:
                self.get_logger().info(f"posture '{name}' ไม่รองรับ -> ข้าม")
            return

        if t == "arm_raw":
            cmd = str(msg.get("cmd", "")).strip()
            if cmd:
                self._publish_arm(cmd)
            return

        # ── LED -> /motor/command (JSON) : Teensy รับใน handleJsonCommand ──
        if t == "led":
            state = 1 if msg.get("state", 0) in (1, True, "1") else 0
            self._publish_motor({"type": "led", "state": state})
            self.get_logger().info(f"led -> {state}")
            return

        # ── LASER -> /motor/command (JSON) : Teensy รับใน handleJsonCommand ──
        if t == "laser":
            state = 1 if msg.get("state", 0) in (1, True, "1") else 0
            self._publish_motor({"type": "laser", "state": state})
            self.get_logger().info(f"laser -> {state}")
            return

        if t in ("leds", "snapshot"):
            self.get_logger().info(f"'{t}' ยังไม่มีใน Teensy build นี้ -> ข้าม")
            return

    # ───────────────────── TCP server ─────────────────────
    def _drop_client(self, sock):
        with self._client_lock:
            if self._client is sock:
                self._client = None
        try: sock.shutdown(socket.SHUT_RDWR)
        except Exception: pass
        try: sock.close()
        except Exception: pass

    def _handle_client(self, conn, addr):
        self.get_logger().info(f"Windows connected: {addr}")
        with self._client_lock:
            old = self._client
            self._client = conn
        if old is not None and old is not conn:
            try: old.close()
            except Exception: pass

        # ส่งสถานะ serial จริง ณ ตอน client ต่อเข้ามา (ไม่หลอกว่า True)
        self._send_win({"type": "state",
                        "serial": self._serial_ok,
                        "from_serial": True})

        buf = ""
        try:
            conn.settimeout(None)
            while True:
                try:
                    data = conn.recv(4096)
                except (ConnectionResetError, ConnectionError, OSError) as e:
                    # Windows ส่ง RST (เช่นตอน Ctrl+C) -> ปิด client ตัวนี้ แล้วให้ _serve
                    # วน accept() รอตัวใหม่ต่อ ไม่ทำให้ทั้งโปรแกรมล้ม
                    self.get_logger().info(f"client {addr} reset: {e}")
                    break
                if not data:
                    # empty recv() = peer ปิดสาย (FIN) อย่างสุภาพ
                    break
                buf += data.decode("utf-8", errors="ignore")
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        msg = json.loads(line)
                    except Exception:
                        continue
                    self._on_win_msg(msg)
        except Exception as e:
            # error ที่ไม่คาดคิด (bug จริง) -> log ดังๆ แต่ยังไม่ทำให้ accept loop ล้ม
            self.get_logger().error(f"client handler unexpected error: {e}")
        finally:
            self.get_logger().info(f"Windows disconnected: {addr}")
            self._drop_client(conn)

    def _serve(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.tcp_host, self.tcp_port))
        srv.listen(1)
        self._srv_sock = srv
        while rclpy.ok():
            try:
                conn, addr = srv.accept()
            except OSError as e:
                # ถ้า node กำลังปิด (socket ถูกปิด) -> ออก loop จริง
                # ถ้าเป็น error ชั่วคราว -> วนกลับไป accept() ใหม่ ไม่ crash
                if getattr(self, "_closing", False):
                    break
                self.get_logger().warn(f"accept() error, retry: {e}")
                continue
            try:
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                # keepalive: ตรวจเจอ Windows ที่ตายไป (Ctrl+C) แทนที่จะค้าง recv ตลอด
                conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                if hasattr(socket, "TCP_KEEPIDLE"):
                    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 3)
                    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 2)
                    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 2)
            except Exception:
                pass
            threading.Thread(target=self._handle_client,
                             args=(conn, addr), daemon=True).start()
        srv.close()


def main(args=None):
    rclpy.init(args=args)
    node = TcpBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()