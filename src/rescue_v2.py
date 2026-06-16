#!/usr/bin/env python3
"""
motor_client_windows.py  –  ฝั่ง Windows (ใช้กับ rescue.py)
=============================================================
เชื่อมต่อ Raspberry Pi ผ่าน TCP แล้วส่ง/รับคำสั่ง JSON
ไปยัง Teensy + AK45 motor โดยไม่ต้องต่อ USB โดยตรง

การใช้งานใน rescue.py:
    from motor_client_windows import MotorClient
    mc = MotorClient("192.168.x.x", 9000)
    mc.start()
    mc.drive(linear=60, angular=0)   # เดินหน้า 60%
    mc.drive(linear=0,  angular=30)  # เลี้ยวขวา
    mc.stop_all()

Events / callback:
    mc.on_feedback = lambda d: ...   # รับ motor_feedback ทุก 100ms
    mc.on_ack      = lambda d: ...   # รับ motor_ack / motor_status
    mc.on_connect  = lambda: ...
    mc.on_disconnect = lambda: ...
"""

import json
import socket
import threading
import time
import logging

log = logging.getLogger("motor_client")
log.setLevel(logging.INFO)
if not log.handlers:
    h = logging.StreamHandler()
    h.setFormatter(logging.Formatter("[%(asctime)s][%(levelname)s] %(message)s", "%H:%M:%S"))
    log.addHandler(h)


class MotorClient:
    """
    Thread-safe TCP client สำหรับ AK45 Motor Bridge.
    ทำงานเป็น background thread – ปลอดภัยใช้ใน rescue.py หลัก
    """

    def __init__(self, host: str, port: int = 9000, auto_reconnect: bool = True):
        self.host           = host
        self.port           = port
        self.auto_reconnect = auto_reconnect
        self._sock          = None
        self._lock          = threading.Lock()
        self._buf           = b""
        self._connected     = False

        # callbacks
        self.on_feedback    = None   # fn(dict)
        self.on_ack         = None   # fn(dict)
        self.on_connect     = None   # fn()
        self.on_disconnect  = None   # fn()

    # ── public API ────────────────────────────────────────────────

    def start(self):
        """เริ่ม background thread – เรียกครั้งเดียว"""
        t = threading.Thread(target=self._loop, daemon=True, name="MotorClient")
        t.start()

    @property
    def connected(self) -> bool:
        return self._connected

    def send(self, obj: dict) -> bool:
        """ส่ง JSON command ไปยัง Pi (thread-safe)"""
        with self._lock:
            if not self._sock or not self._connected:
                log.warning("Not connected – command dropped")
                return False
            try:
                self._sock.sendall((json.dumps(obj) + "\n").encode())
                return True
            except Exception as e:
                log.error(f"Send error: {e}")
                self._connected = False
                return False

    # ── convenience wrappers ──────────────────────────────────────

    def drive(self, linear: float, angular: float):
        """
        สั่งขับเคลื่อน
        linear  ∈ [-100, 100]  (+ = เดินหน้า, − = ถอยหลัง)
        angular ∈ [-100, 100]  (+ = เลี้ยวขวา, − = เลี้ยวซ้าย)
        """
        self.send({"type": "drive", "linear": linear, "angular": angular})

    def stop_all(self):
        self.send({"type": "motor_all", "action": "stop"})

    def enable_all(self):
        self.send({"type": "motor", "id": 1, "action": "enable"})
        time.sleep(0.05)
        self.send({"type": "motor", "id": 2, "action": "enable"})

    def disable_all(self):
        self.send({"type": "motor", "id": 1, "action": "disable"})
        time.sleep(0.05)
        self.send({"type": "motor", "id": 2, "action": "disable"})

    def set_velocity(self, motor_id: int, pct: float):
        """pct ∈ [-100, 100]"""
        self.send({"type": "motor", "id": motor_id, "action": "setvel", "value": pct})

    def sync_vel(self, v1: float, v2: float):
        """v1/v2 ∈ [-100, 100]"""
        self.send({"type": "motor_sync", "action": "syncvel", "v1": v1, "v2": v2})

    def ping(self):
        self.send({"type": "ping"})

    def get_status(self):
        self.send({"type": "motor_all", "action": "status"})

    # ── internal ──────────────────────────────────────────────────

    def _connect(self):
        while True:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(5.0)
                s.connect((self.host, self.port))
                s.settimeout(None)
                with self._lock:
                    self._sock = s
                    self._buf  = b""
                self._connected = True
                log.info(f"Connected to Pi motor bridge {self.host}:{self.port}")
                if self.on_connect:
                    try: self.on_connect()
                    except Exception: pass
                return
            except Exception as e:
                log.warning(f"Connect failed ({e}), retry in 3 s ...")
                time.sleep(3)

    def _loop(self):
        while True:
            self._connect()
            try:
                while True:
                    chunk = self._sock.recv(1024)
                    if not chunk:
                        raise ConnectionResetError("Pi closed connection")
                    self._buf += chunk
                    while b"\n" in self._buf:
                        line, self._buf = self._buf.split(b"\n", 1)
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            d = json.loads(line)
                        except json.JSONDecodeError:
                            log.warning(f"Bad JSON from Pi: {line}")
                            continue
                        t = d.get("type", "")
                        if t == "motor_feedback":
                            if self.on_feedback:
                                try: self.on_feedback(d)
                                except Exception: pass
                        elif t == "heartbeat":
                            pass  # ignore
                        else:
                            if self.on_ack:
                                try: self.on_ack(d)
                                except Exception: pass
                            else:
                                log.debug(f"Pi → {d}")
            except Exception as e:
                self._connected = False
                log.warning(f"Disconnected from Pi: {e}")
                if self.on_disconnect:
                    try: self.on_disconnect()
                    except Exception: pass
                if not self.auto_reconnect:
                    break
                log.info("Reconnecting in 3 s ...")
                time.sleep(3)


# ── ตัวอย่างการใช้งานร่วมกับ rescue.py ──────────────────────────
if __name__ == "__main__":
    import sys

    PI_IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.1.111"

    mc = MotorClient(PI_IP, 9000)

    def on_fb(d):
        motors = d.get("motors", [])
        for m in motors:
            print(f"  Motor {m['id']}: vel={m['vel_pct']:+.1f}%  pos={m['pos']:.3f}  cur={m['cur']:.2f}A")

    def on_ack(d):
        print(f"  ACK: {d}")

    mc.on_feedback   = on_fb
    mc.on_ack        = on_ack
    mc.on_connect    = lambda: print(">>> Pi connected")
    mc.on_disconnect = lambda: print(">>> Pi disconnected")

    mc.start()
    time.sleep(1.5)

    print("=== Enable motors ===")
    mc.enable_all()
    time.sleep(0.5)

    print("=== Drive forward 50% ===")
    mc.drive(linear=50, angular=0)
    time.sleep(2)

    print("=== Turn right ===")
    mc.drive(linear=30, angular=20)
    time.sleep(1.5)

    print("=== Stop ===")
    mc.stop_all()
    time.sleep(1)

    print("=== Disable ===")
    mc.disable_all()
    time.sleep(0.5)