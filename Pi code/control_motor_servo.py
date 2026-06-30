#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
============================================================================
 teensy_serial_node.py  —  ROS2 topics  ⇄  Teensy USB serial
============================================================================
 รันบน Pi/Ubuntu เป็น ROS2 node. หน้าที่ตรงไปตรงมา:
   - subscribe /motor/command (std_msgs/String)  -> เขียนลง serial Teensy (ทุกบรรทัด)
   - subscribe /arm/command   (std_msgs/String)  -> เขียนลง serial Teensy
   - อ่าน serial Teensy ทีละบรรทัด -> publish /teensy/feedback (std_msgs/String)

 Teensy เป็นคน dispatch เองตาม content ของบรรทัด:
   "{...}"        -> JSON (drive/motor/motor_sync/motor_all/ping)
   "SERVO i deg"  -> servo
   "POSTURE name" -> posture

 *ถ้าคุณมี control_motor_servo.py อยู่แล้ว* จะใช้ตัวนั้นแทน node นี้ก็ได้
  ขอแค่ให้มัน (ก) subscribe /motor/command + /arm/command และ
  (ข) publish telemetry ออก /teensy/feedback เพื่อให้ tcp_bridge ส่งกลับ Windows ได้

 params:
   serial_port (string) : '' = auto-detect Teensy, หรือกำหนด เช่น '/dev/ttyACM0'
   baud (int)           : 115200 (ต้องตรงกับ .ino)
============================================================================
"""

import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

# ───────────────────── ตั้งค่าพอร์ตตรงนี้ ─────────────────────
# เปลี่ยนค่านี้เพื่อกำหนด serial port ที่จะใช้ (เผื่อมีอุปกรณ์อื่นเสียบหลายตัว)
#   ""            -> auto-detect Teensy เอง
#   "/dev/ttyACM0" / "/dev/ttyACM1" / "/dev/ttyUSB0" -> ใช้พอร์ตที่ระบุ
# หมายเหตุ: ใช้ udev symlink คงที่ /dev/teensy (ดู /etc/udev/rules.d/99-robot-serial.rules)
DEFAULT_SERIAL_PORT = "/dev/teensy"
DEFAULT_BAUD        = 115200
# ──────────────────────────────────────────────────────────────

SERIAL_RECONNECT_SEC = 2.0


class TeensySerialNode(Node):
    def __init__(self):
        super().__init__("teensy_serial")

        if serial is None:
            self.get_logger().error("ต้องติดตั้ง pyserial ก่อน:  pip3 install pyserial")
            raise SystemExit(1)

        self.declare_parameter("serial_port", DEFAULT_SERIAL_PORT)   # '' = auto
        self.declare_parameter("baud", DEFAULT_BAUD)
        self._port_cfg = self.get_parameter("serial_port").value or None
        self._baud     = int(self.get_parameter("baud").value)

        self._ser   = None
        self._wlock = threading.Lock()

        self.pub_fb = self.create_publisher(String, "/teensy/feedback", 10)
        # สถานะ serial จริง (true/false) ให้ tcp_bridge รู้ ไม่ต้องเดา
        self.pub_status = self.create_publisher(String, "/teensy/status", 10)

        self.create_subscription(String, "/motor/command", self._on_cmd, 10)
        self.create_subscription(String, "/arm/command",   self._on_cmd, 10)

        # serial reader + auto-reconnect thread
        threading.Thread(target=self._serial_loop, daemon=True).start()

        # publish สถานะ serial จริงเป็นจังหวะ (latched-ish) ให้ bridge/UI ตามทัน
        self._last_status = None
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f"teensy_serial up (port={self._port_cfg or 'auto'} @ {self._baud})")

    # ───────────── หา Teensy port อัตโนมัติ ─────────────
    @staticmethod
    def _find_teensy():
        if list_ports is None:
            return None
        for p in list_ports.comports():
            if getattr(p, "vid", None) == 0x16C0:        # PJRC/Teensy VID
                return p.device
        for p in list_ports.comports():
            dev, desc = (p.device or ""), (p.description or "")
            if "ttyACM" in dev or "Teensy" in desc:
                return p.device
        return None

    def _is_open(self):
        return self._ser is not None and self._ser.is_open

    def _publish_status(self):
        # ส่งสถานะ serial จริงทุกวินาที (เปลี่ยนค่าก็รีบ log ให้เห็น)
        ok = self._is_open()
        m = String(); m.data = "true" if ok else "false"
        self.pub_status.publish(m)
        if ok != self._last_status:
            self._last_status = ok
            self.get_logger().info(f"serial status -> {'OPEN' if ok else 'CLOSED'}")

    # ───────────── /motor/command + /arm/command -> serial ─────────────
    def _on_cmd(self, msg: String):
        line = msg.data.rstrip("\r\n")
        if not line:
            return
        # รับคำสั่งจาก Window (ผ่าน topic) เข้ามาที่ node
        self.get_logger().info(f"[RX  Window] {line}")
        if not self._is_open():
            self.get_logger().warn(f"serial ไม่พร้อม, ทิ้ง: {line}")
            # แจ้งสถานะจริงทันทีให้ bridge/UI รู้ว่าคำสั่งถูกทิ้งเพราะ serial ปิด
            sm = String(); sm.data = "false"
            self.pub_status.publish(sm)
            return
        with self._wlock:
            try:
                self._ser.write((line + "\n").encode("utf-8"))
                # ส่งคำสั่งออกไปยัง Teensy ทาง serial
                self.get_logger().info(f"[TX Teensy] {line}")
            except Exception as e:
                self.get_logger().warn(f"serial write error: {e}")
                self._drop()

    # ───────────── serial reader + reconnect ─────────────
    def _drop(self):
        if self._ser is not None:
            try: self._ser.close()
            except Exception: pass
        self._ser = None

    def _try_open(self):
        port = self._port_cfg or self._find_teensy()
        if not port:
            return False
        try:
            self._ser = serial.Serial(port, self._baud, timeout=0.2)
            time.sleep(0.3)
            try: self._ser.reset_input_buffer()
            except Exception: pass
            self.get_logger().info(f"serial connected: {port} @ {self._baud}")
            return True
        except Exception as e:
            self.get_logger().warn(f"serial open failed ({port}): {e}")
            self._ser = None
            return False

    def _serial_loop(self):
        buf = ""
        while rclpy.ok():
            if not self._is_open():
                if not self._try_open():
                    time.sleep(SERIAL_RECONNECT_SEC)
                    continue
                buf = ""
            try:
                chunk = self._ser.read(256)
                if chunk:
                    buf += chunk.decode("utf-8", errors="ignore")
                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        line = line.strip("\r ").rstrip()
                        if line:
                            # รับ feedback จาก Teensy ทาง serial
                            self.get_logger().info(f"[RX Teensy] {line}")
                            m = String(); m.data = line
                            self.pub_fb.publish(m)
                            # ส่ง feedback กลับไปยัง Window (ผ่าน topic)
                            self.get_logger().info(f"[TX  Window] {line}")
            except Exception as e:
                self.get_logger().warn(f"serial read error: {e}")
                self._drop()
                time.sleep(SERIAL_RECONNECT_SEC)


def main(args=None):
    rclpy.init(args=args)
    node = TeensySerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()