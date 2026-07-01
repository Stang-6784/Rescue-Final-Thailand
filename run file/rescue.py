import socket
import json
import time
import threading
import sys
import os
import signal
import atexit
import asyncio
import websockets
import webbrowser
import urllib.request
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

# ═══════════════════════════════════════════════════════════════
#  Configuration
# ═══════════════════════════════════════════════════════════════
PI_IP       = "192.168.1.67"
PI_TCP_PORT = 9000
WS_PORT     = 8765
HTTP_PORT   = 8088      # ย้ายจาก 8766 หนี VS Code auto port-forward ที่ยึด 8766

LINEAR_SPEED  = 0.30 # เริ่มที่ gear 3 (ตรงกับ LIN_GEARS[2] ใน UI)
ANGULAR_SPEED = 1.00 # เริ่มที่ gear 3 (ตรงกับ ANG_GEARS[2] ใน UI)
BASE_RPM      = 150 # was 80. AK45-10 rated output = 150 RPM @24V

STATE_PUSH_HZ    = 20
WS_SEND_TIMEOUT  = 0.08
EVENT_QUEUE_MAX  = 200

# Camera / QR
RTSP_URL = "rtsp://admin:admin@192.168.0.106:554/live"
QR_ENABLE = True
QR_JPEG_QUALITY = 70
QR_DETECT_WIDTH = 960
QR_SEND_FPS = 8

# Flask QR server (python qr_server.py รันแยก)
FLASK_BASE = "http://127.0.0.1:5000"   # ← แก้ IP ถ้า Flask อยู่เครื่องอื่น

# ── Map marking: เมื่อ scan เจอ hazmat/QR → ส่ง mark ผ่าน Pi TCP ไป ROS2 ──
#   rescue.py → tcp_bridge (/mark/request) → map_marker_pi → /scan_marks → lidar.html
MARK_ON_SCAN     = True   # ปิดได้ถ้าไม่อยากปักหมุดอัตโนมัติ
AI_MARK_DEBOUNCE = 15.0   # วินาที: คลาส hazmat เดิมจะปักหมุดซ้ำได้หลังพ้นช่วงนี้ (กันสแปม)

# ═══════════════════════════════════════════════════════════════
#  Servo config
# ═══════════════════════════════════════════════════════════════
NUM_SERVOS = 9
SERVO_NAMES = [
    "Joint1","Joint2","Joint3","Joint 4","Joint 5","Gripper", "Gripper2", "Flip-F","Flip-R"
]
SERVO_DEFAULTS = [98, 150, 140,  80,  100, 100,  0,   85,   90]
SERVO_MINS     = [50,  10,   0,  0,  0, 0, 0, 0, 0]
SERVO_MAXS     = [150, 170, 157, 180, 170, 180, 180, 180, 180]

GRIP_OPEN  = 70
GRIP_CLOSE = 10
GRIP_MID   = (GRIP_OPEN + GRIP_CLOSE) // 2

GRIP2_OPEN  = 180
GRIP2_CLOSE = 0
GRIP2_MID   = (GRIP2_OPEN + GRIP2_CLOSE) // 2

HOME_Y, HOME_Z, HOME_P = 30.0, 18.0, 30.0

# ── Servo slew (ให้ servo "ค่อยๆ หมุน" ขณะเปลี่ยนท่า แทนกระชากไปค่าปลายทาง) ──
# Teensy (.ino) จำกัดความเร็วจริงของ servo อยู่แล้ว; ฝั่งนี้ส่งองศา "ทีละขั้น"
# ด้วยอัตราเดียวกัน เพื่อให้โมเดล 3D + UI ขยับตามไปพร้อมกับ servo จริง.
#   SERVO_SLEW_DPS  = องศา/วินาที (ต่ำ = ช้าลง) — ตั้งให้ตรงกับ .ino
#   SERVO_SLEW_HZ   = ความถี่ส่งคำสั่งระหว่างไล่ (เฟรม/วินาที)
#   SERVO_SLEW_ON   = False → ปิด, ส่งค่าปลายทางทีเดียวแบบเดิม
SERVO_SLEW_DPS = 90.0
SERVO_SLEW_HZ  = 50.0
SERVO_SLEW_ON  = True

POSTURE_ANGLES = {
    "home":   [98, 150, 140,  80,  100, 100,  0,   85,   90],
    "guard":  [98, 150, 150,  80,  100, 100,  0, 154, 154],
    "giraff": [98, 150, 150,  80,  100, 100,  0, 27,  38],
    "stair":  [98, 150, 138, 80,  100, 100,  0, 118, 52],
}

# Custom postures F4-F9 — กำหนดจาก Browser Settings UI
# format: [J1, J2, J3, J4, J5, J6, Flip-F, Flip-R]
# giraff/stair มีค่า default แต่ override ได้จาก UI
CUSTOM_POSTURES = {
    "giraff":   [98, 150, 157,  100, 100, 70, 90,  27,  38],   # F4 default
    "stair":    [98, 150, 138, 80,  100, 100,  0, 118, 52],   # F5 default
    "K-Rail": [98, 150, 157, 100, 100, 70, 90, 78, 154],   # F6 ด่าน K-Rails
    "custom_2": [106, 103, 0, 69, 100, 70, 90, 140, 45],   # F7 QR SCAN + Flipper down
    "custom_3": [60, 130,  0,  90, 100, 70, 90, 100, 100],   # F8 default = HOME
    "custom_4": [60, 130,  0,  90, 100, 70, 90, 126, 113],   # F9 default = HOME
}

MOVE_KEYS = {'w','a','s','d','q','e','z','c'}


# ═══════════════════════════════════════════════════════════════
#  TCP → Pi
# ═══════════════════════════════════════════════════════════════
_pi_sock = None
_pi_lock = threading.Lock()
_ws_loop = None
_ws_ctrl = None
_ws_clients = set()
_ws_queue = None
_qr_worker = None

def _sv_clamp(idx, v):
    return int(max(SERVO_MINS[idx], min(SERVO_MAXS[idx], v)))


def _close_pi_sock(hard=False):
    """ปิด socket ไป Pi.
    hard=True → ส่ง TCP RST (SO_LINGER timeout 0) แทน FIN
    เพื่อบังคับให้ Pi ที่ block อยู่ที่ recv() ได้ ECONNRESET ทันที
    ใช้ตอน Ctrl+C เพื่อให้ Pi ปล่อย connection เก่าก่อน run ใหม่
    """
    global _pi_sock
    with _pi_lock:
        if _pi_sock is not None:
            if hard:
                try:
                    import struct
                    _pi_sock.setsockopt(
                        socket.SOL_SOCKET, socket.SO_LINGER,
                        struct.pack("ii", 1, 0))  # linger on, timeout 0 → RST
                except: pass
            else:
                try: _pi_sock.shutdown(socket.SHUT_RDWR)
                except: pass
            try: _pi_sock.close()
            except: pass
            _pi_sock = None


def _send_pi(payload):
    global _pi_sock
    with _pi_lock:
        if _pi_sock is None: return
        try:
            _pi_sock.sendall((json.dumps(payload)+'\n').encode('utf-8'))
        except Exception as e:
            print(f"[rescue_win] Pi send error: {e}")
            try: _pi_sock.shutdown(socket.SHUT_RDWR)
            except: pass
            try: _pi_sock.close()
            except: pass
            _pi_sock = None


def _pi_recv_thread(ctrl):
    global _pi_sock
    buf = ""
    while True:
        with _pi_lock:
            sock = _pi_sock
        if sock is None:
            time.sleep(0.3); continue
        try:
            data = sock.recv(4096)
            if not data:
                ctrl._log("Pi disconnected")
                _close_pi_sock(); time.sleep(0.5); continue
            buf += data.decode("utf-8", errors="ignore")
            while '\n' in buf:
                line, buf = buf.split('\n', 1)
                line = line.strip()
                if not line: continue
                try: ctrl._apply_from_pi(json.loads(line))
                except: pass
        except Exception as e:
            ctrl._log(f"Pi recv error: {e}")
            _close_pi_sock(); time.sleep(1)


def _pi_connect_thread(ctrl):
    global _pi_sock
    while True:
        with _pi_lock:
            connected = _pi_sock is not None
        if connected:
            time.sleep(2); continue
        s = None
        try:
            ctrl._log(f"Connecting {PI_IP}:{PI_TCP_PORT}...")
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(2)   # was 5 — IP ผิด/Pi ดับ จะรู้เร็วขึ้น ไม่ค้างนาน
            s.connect((PI_IP, PI_TCP_PORT))
            s.settimeout(None)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            # TCP keepalive: ตรวจจับ half-open connection เร็วขึ้น
            # (กรณี Ctrl+C แล้ว Pi ยังถือ connection เก่าค้าง)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            try:
                # Windows: idle 2s, interval 1s ก่อนตัดสินว่า dead
                s.ioctl(socket.SIO_KEEPALIVE_VALS, (1, 2000, 1000))
            except (AttributeError, OSError):
                pass
            with _pi_lock:
                if _pi_sock is not None:
                    try: _pi_sock.close()
                    except: pass
                _pi_sock = s
            ctrl._log("Pi connected!")
        except Exception as e:
            ctrl._log(f"Pi connect failed: {e}")
            try:
                if s: s.close()
            except: pass
            time.sleep(1)   # was 3 — retry ถี่ขึ้น เชื่อมติดไวขึ้นหลัง Pi พร้อม



# ═══════════════════════════════════════════════════════════════
#  QR Worker
#  Poll Flask qr_server.py /status ทุก 0.5 วิ
#  แล้ว push ผล QR ผ่าน WebSocket ไปยัง Browser
#  Flask รันแยกต่างหาก: python qr_server.py
# ═══════════════════════════════════════════════════════════════
class QRWorker:
    def __init__(self, rtsp_url, push_callback, log_callback,
                 jpeg_quality=70, detect_width=960, send_fps=8):
        self._push     = push_callback
        self._log      = log_callback
        self._stop_evt = threading.Event()
        self._thread   = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop_evt.set()

    def _run(self):
        self._log(f"[QRWorker] polling {FLASK_BASE}/status")
        last_qr = ""
        ai_marked = {}   # class name -> last mark time (debounce hazmat)
        while not self._stop_evt.is_set():
            try:
                with urllib.request.urlopen(
                    f"{FLASK_BASE}/status", timeout=2
                ) as resp:
                    data = json.loads(resp.read().decode())

                qr_text = data.get("qr", "")
                all_qr  = data.get("all_qr", [])
                found   = bool(data.get("qr_found", False))

                # push ทุกครั้งที่ QR เปลี่ยน
                if found and qr_text != last_qr:
                    last_qr = qr_text
                    self._push({
                        "type":   "qr_found",
                        "qr":     qr_text,
                        "all_qr": all_qr,
                    })
                    self._log(f"[QR] {qr_text}")
                    # ปักหมุด QR ลงแผนที่ (ผ่าน Pi → ROS2)
                    if MARK_ON_SCAN and qr_text:
                        _send_pi({"type": "mark", "kind": "qr", "text": qr_text})
                        self._log(f"[MARK] QR → map: {qr_text}")

                # ปักหมุด hazmat ที่ AI เจอ (debounce ต่อคลาส กันสแปม)
                if MARK_ON_SCAN:
                    now = time.time()
                    for cls in data.get("ai_classes", []):
                        if now - ai_marked.get(cls, 0.0) >= AI_MARK_DEBOUNCE:
                            ai_marked[cls] = now
                            _send_pi({"type": "mark", "kind": "ai", "text": cls})
                            self._log(f"[MARK] hazmat → map: {cls}")

            except Exception:
                # Flask ยังไม่รัน หรือ network error → รอต่อ
                pass

            self._stop_evt.wait(0.5)   # poll ทุก 0.5 วิ


# ═══════════════════════════════════════════════════════════════
#  IMU Reader (BNO055 ผ่าน Teensy/Arduino)  — เตรียมไว้ ยังไม่เปิดใช้
# ═══════════════════════════════════════════════════════════════
#  ตอนนี้ค่า IMU มาจาก Pi (forward ผ่าน _apply_from_pi → t == "imu").
#  บล็อกด้านล่างนี้ "คอมเมนต์ไว้ทั้งหมด" เผื่อกรณีต้องการต่อ BNO055
#  (ผ่านบอร์ด Teensy/Arduino ที่รัน Arduino Code/imu/imu.ino) เข้าเครื่อง
#  Windows นี้โดยตรงทาง USB → COM port แล้วให้ rescue.py อ่านเอง
#
#  imu.ino ส่งเป็น ASCII บรรทัดละแพ็กเก็ต @ 115200 (สตรีมเองโดยอัตโนมัติ):
#      IMU,<yaw>,<pitch>,<roll>,<sys>,<gyro>,<accel>,<mag>
#  (Euler จาก BNO055: x=yaw 0..360, y=pitch, z=roll หน่วยองศา;
#   sys/gyro/accel/mag = สถานะ calibration 0..3)
#  มี INFO,* / ERROR,* / CAL,* ปนมาด้วย → parser ข้ามบรรทัดที่ไม่ขึ้นต้น "IMU,"
#
#  วิธีเปิดใช้:
#    1) pip install pyserial
#    2) ตั้งค่า IMU_ENABLE = True, IMU_PORT ให้ตรงกับ COM ของ Teensy
#    3) uncomment คลาส IMUReader + บล็อก start ใน main() + stop ใน _do_cleanup
#  ฝั่ง UI (script_index.js updateIMU + widget) พร้อมรับ {type:'imu',...} อยู่แล้ว
#
#  --- config (วางคู่กับ config อื่นด้านบนก็ได้) -------------------
#  IMU_ENABLE = False
#  IMU_PORT   = "COM6"      # ← แก้ให้ตรง COM ของ Teensy (ดูใน Device Manager)
#  IMU_BAUD   = 115200      # ตรงกับ Serial.begin(115200) ใน imu.ino
#  IMU_HZ     = 20          # อัตราส่งเข้า browser (throttle; .ino ส่ง ~20Hz อยู่แล้ว)
#
#  --- ตัวอ่าน ASCII line: "IMU,yaw,pitch,roll,sys,gyro,accel,mag" ---
#  import serial   # ← ต้อง pip install pyserial
#
#  class IMUReader:
#      """อ่าน BNO055 (ผ่าน Teensy/imu.ino) ทาง serial แล้ว push
#      {type:'imu', roll,pitch,yaw, cal} เข้า browser ผ่าน push_callback (= _push_ws)."""
#      def __init__(self, port, baud, push_callback, log_callback, send_hz=20):
#          self._port  = port
#          self._baud  = baud
#          self._push  = push_callback
#          self._log   = log_callback
#          self._min_dt = 1.0 / max(1, send_hz)
#          self._stop_evt = threading.Event()
#          self._thread   = threading.Thread(target=self._run, daemon=True)
#
#      def start(self):
#          self._thread.start()
#
#      def stop(self):
#          self._stop_evt.set()
#
#      def _run(self):
#          self._log(f"[IMU] opening {self._port} @ {self._baud}")
#          try:
#              ser = serial.Serial(self._port, self._baud, timeout=1)
#          except Exception as e:
#              self._log(f"[IMU] open failed: {e}")
#              return
#          try:
#              time.sleep(2.0)             # รอ Teensy reset หลังเปิด port
#              ser.reset_input_buffer()
#              ser.write(b"run\n")         # สั่งให้ออกจากโหมด cal → ส่ง IMU stream
#          except Exception:
#              pass
#          last_send = 0.0
#          while not self._stop_evt.is_set():
#              try:
#                  line = ser.readline().decode("utf-8", errors="ignore").strip()
#                  if not line.startswith("IMU,"):
#                      continue            # ข้าม INFO,* / ERROR,* / CAL,*
#                  parts = line.split(",")
#                  if len(parts) < 4:
#                      continue
#                  yaw   = float(parts[1])
#                  pitch = float(parts[2])
#                  roll  = float(parts[3])
#                  cal = {}
#                  if len(parts) >= 8:
#                      cal = {"sys": int(parts[4]), "gyro": int(parts[5]),
#                             "accel": int(parts[6]), "mag": int(parts[7])}
#                  now = time.time()
#                  if now - last_send >= self._min_dt:
#                      last_send = now
#                      self._push({"type":"imu","roll":roll,"pitch":pitch,"yaw":yaw,"cal":cal})
#              except Exception as e:
#                  self._log(f"[IMU] read error: {e}")
#                  self._stop_evt.wait(0.5)
#          try: ser.close()
#          except: pass


# ═══════════════════════════════════════════════════════════════
#  Robot Controller
# ═══════════════════════════════════════════════════════════════
class RobotController:
    def __init__(self):
        self._mu     = threading.Lock()
        self._log_mu = threading.Lock()
        self.running = True

        self.active_keys   = set()
        self.is_locked     = False
        self.linear_speed  = LINEAR_SPEED
        self.angular_speed = ANGULAR_SPEED
        self.base_rpm      = BASE_RPM

        self.angles      = list(SERVO_DEFAULTS)
        self.selected    = 0
        self.servo_step  = 1
        self.motor_state = "stop"
        self.serial_ok   = False
        self.log_lines   = []

        # ── Send Loop 50Hz ───────────────────────────────────
        self._current_key       = ''
        self._send_loop_running = True
        threading.Thread(target=self._send_loop, daemon=True).start()

        # ── Servo slew (ไล่องศาแบบค่อยเป็นค่อยไปขณะเปลี่ยนท่า) ──
        self._slew_thread = None
        self._slew_stop   = threading.Event()

    # ── Send Loop ────────────────────────────────────────────
    def _send_loop(self):
        """ส่ง move_start ทุก 20ms พร้อม lin/ang ล่าสุด"""
        _dbg_last = None
        while self._send_loop_running:
            key = self._current_key
            if key and not self.is_locked:
                _send_pi({
                    "type": "move_start",
                    "key":  key,
                    "lin":  self.linear_speed,
                    "ang":  self.angular_speed,
                })
                # DEBUG ชั่วคราว: log เฉพาะตอน key/lin/ang เปลี่ยน (ไม่สแปม 50Hz)
                _dbg = (key, round(self.linear_speed, 3), round(self.angular_speed, 3))
                if _dbg != _dbg_last:
                    self._log(f"[DBG] move_start → key:{key} lin:{self.linear_speed:.2f} ang:{self.angular_speed:.2f}")
                    _dbg_last = _dbg
            else:
                _dbg_last = None
            time.sleep(0.02)  # 50Hz

    # ── Logging ──────────────────────────────────────────────
    def _log(self, msg):
        ts   = time.strftime("%H:%M:%S")
        line = f"{ts}  {msg}"
        print(line)
        with self._log_mu:
            self.log_lines.append(line)
            if len(self.log_lines) > 50: self.log_lines.pop(0)
        _push_ws({"type":"log_event","line":line})
        _push_ws({"type":"state",**self._state_dict()})

    def _apply_from_pi(self, msg):
        t = msg.get("type","")
        if t == "arm_status":
            self.serial_ok = True
            self._log(f"Teensy: {msg.get('status','')}")
        elif t == "state":
            if "angles" in msg and not msg.get("from_serial", False):
                self.angles = msg["angles"]
            if "serial" in msg:
                self.serial_ok = msg["serial"]
            _push_ws({"type":"state",**self._state_dict()})
        elif t == "arm_log":
            text = msg.get("text","")
            if text:
                _push_ws({"type":"arm_log","data":text})  # ← แก้ bug indent
        elif t == "imu":
            # IMU จาก Pi (BWT901CL) → forward ตรงๆ ไป browser
            _push_ws(msg)

    # ── Servo ────────────────────────────────────────────────
    def _servo_cmd(self, idx, angle):
        self._slew_stop.set()   # สั่ง joint เดี่ยว → ยกเลิก ramp ท่าที่กำลังวิ่งอยู่
        safe = _sv_clamp(idx, angle)
        self.angles[idx] = safe
        # Gripper (idx 5) เป็น PWMServo ปรับองศาได้จริงบน Teensy (ช่วง 45–90°)
        # ส่ง servo_set ตรงๆ เหมือน joint อื่น เพื่อให้สั่งองศาได้อิสระ
        if idx == 5:
            self.motor_state = "reverse" if safe >= GRIP_MID else "grip"
        _send_pi({"type":"servo_set","index":idx,"angle":safe})

    # ── Servo ramp (ค่อยๆ ไล่องศา → target ให้ servo หมุนช้าลง + 3D/UI sync) ──
    def _ramp_to(self, target):
        """ไล่องศาจากท่าปัจจุบัน (self.angles) ไปยัง target ทีละขั้น.
        รันใน thread แยก ไม่ block ตัวจัดการคำสั่ง, และยกเลิก ramp เก่าทันที
        ถ้ามีท่าใหม่เข้ามาระหว่างทาง."""
        target = [_sv_clamp(i, a) for i, a in enumerate(target)]

        # ยกเลิก ramp เดิมที่ยังวิ่งอยู่
        self._slew_stop.set()
        if self._slew_thread and self._slew_thread.is_alive():
            self._slew_thread.join(timeout=0.5)

        if not SERVO_SLEW_ON:
            self.angles = list(target)
            for i, a in enumerate(target):
                _send_pi({"type": "servo_set", "index": i, "angle": a})
            _push_ws({"type": "state", **self._state_dict()})
            return

        stop  = threading.Event()
        self._slew_stop   = stop
        start = list(self.angles)

        def run():
            dt       = 1.0 / SERVO_SLEW_HZ
            max_step = max(0.5, SERVO_SLEW_DPS * dt)   # องศา/เฟรม
            cur  = [float(a) for a in start]
            last = [int(round(c)) for c in cur]
            while not stop.is_set():
                done = True
                for i in range(NUM_SERVOS):
                    d = target[i] - cur[i]
                    if abs(d) <= max_step:
                        cur[i] = float(target[i])
                    else:
                        cur[i] += max_step if d > 0 else -max_step
                        done = False
                ang = [int(round(c)) for c in cur]
                for i in range(NUM_SERVOS):
                    if ang[i] != last[i]:      # ส่งเฉพาะ servo ที่ค่าขยับ
                        _send_pi({"type": "servo_set", "index": i, "angle": ang[i]})
                        last[i] = ang[i]
                self.angles = ang
                _push_ws({"type": "state", **self._state_dict()})
                if done:
                    break
                time.sleep(dt)

        self._slew_thread = threading.Thread(target=run, daemon=True)
        self._slew_thread.start()

    # ── Posture ──────────────────────────────────────────────
    def _send_posture(self, name):
        if name == 'horizontal':
            _send_pi({"type":"arm_raw",
                      "cmd":f"GOTO {HOME_Y} {HOME_Z} {HOME_P}"})
            self._log("Posture → horizontal")
            return
        if name == 'home':
            # Python เป็นเจ้าของค่าองศา → ส่ง servo_set รายตัวให้ Teensy
            # (Teensy แค่รับค่ามาเขียนลง servo ไม่ต้องเก็บค่า home เอง)
            angles = POSTURE_ANGLES['home']
            self.motor_state = "stop"
            self._log(f"Posture → home: {angles}")
            self._ramp_to(angles)
            return
        # ── F7 QR SCAN — ส่ง GOTO ตรงๆ เหมือน F2 ──
        if name == 'custom_2':
            _send_pi({"type": "arm_raw", "cmd": "GOTO 35 18 2"})
            self._log("Posture → F7 QR SCAN (GOTO 35 18 2)")
            # ขยับ Flipper
            _send_pi({"type": "servo_set", "index": 6, "angle": 140})
            _send_pi({"type": "servo_set", "index": 7, "angle": 45})
            self.angles[6] = 140
            self.angles[7] = 45
            _push_ws({"type": "state", **self._state_dict()})
            return

        # ── Custom postures F4-F9 (giraff, stair, custom_1-4) ──
        if name in CUSTOM_POSTURES or name.startswith("custom_"):
            angles = CUSTOM_POSTURES.get(name)
            if angles is None:
                self._log(f"Posture {name} not set — กำหนดใน Settings UI")
                return
            self.motor_state = "stop"
            self._log(f"Posture → {name}: {angles}")
            self._ramp_to(angles)
            return

        angles = POSTURE_ANGLES.get(name)
        if not angles:
            self._log(f"Unknown posture: {name}"); return
        self.motor_state = "stop"
        self._log(f"Posture → {name}")
        self._ramp_to(angles)

    # ── Wheel ────────────────────────────────────────────────
    def do_lock(self):
        self._current_key = ''
        self.is_locked = True
        _send_pi({"type":"lock"})
        _send_pi({"type":"move_stop"})
        self._log("BRAKE ENGAGED")

    def do_unlock(self):
        self.is_locked = False
        _send_pi({"type":"unlock"})
        self._log("BRAKE RELEASED")

    # ── State ────────────────────────────────────────────────
    def _state_dict(self):
        return {
            "angles":      self.angles,
            "selected":    self.selected,
            "motor_state": self.motor_state,
            "serial":      self.serial_ok,
            "lr":          0,
            "rr":          0,
            "heading":     0.0,
            "battery":     0,
            "log":         self.log_lines[-8:],
            "lin_spd":     self.linear_speed,
            "ang_spd":     self.angular_speed,
            "locked":      self.is_locked,
        }

    def snapshot(self): return self._state_dict()

    # ── WebSocket handler ────────────────────────────────────
    def handle_ws(self, msg):
        t = msg.get("type","")

        # ── Wheel ─────────────────────────────────────────────
        if t == "move_start":
            k = msg.get("key","")
            if k in MOVE_KEYS:
                self.linear_speed  = float(msg.get("lin", self.linear_speed))
                self.angular_speed = float(msg.get("ang", self.angular_speed))
                if self.is_locked:
                    self.do_unlock()
                self.active_keys.add(k)
                self._current_key = k

        elif t == "move_stop":
            k = msg.get("key","")
            self.active_keys.discard(k)
            if not self.active_keys:
                self._current_key = ''
                _send_pi({"type":"move_stop"})
                _send_pi({"type":"lock"})
                self.is_locked = True
                self._log("Auto-brake")
            else:
                self._current_key = next(iter(self.active_keys))

        elif t == "lock":
            self.active_keys.clear()
            self.do_lock()

        elif t == "unlock":
            self.do_unlock()

        elif t == "set_speed":
            self.linear_speed  = float(msg.get("lin", self.linear_speed))
            self.angular_speed = float(msg.get("ang", self.angular_speed))
            self.base_rpm      = int(msg.get("rpm", self.base_rpm))
            _send_pi({
                "type": "set_speed",
                "lin":  self.linear_speed,
                "ang":  self.angular_speed,
            })
            self._log(f"Speed → lin:{self.linear_speed:.2f} ang:{self.angular_speed:.2f}")

        # ── Servo ─────────────────────────────────────────────
        elif t == "servo_set":
            self._servo_cmd(int(msg.get("index",0)),
                            float(msg.get("angle",90)))

        elif t == "servo_set_multi":
            for idx, angle in msg.get("joints",{}).items():
                self._servo_cmd(int(idx), float(angle))

        elif t == "select_servo":
            self.selected = int(msg.get("index",0))
            _push_ws({"type":"state",**self._state_dict()})

        elif t == "set_step":
            self.servo_step = int(msg.get("step",1))

        # ── Gripper (PWMServo ปรับองศาได้ — สั่ง servo_set ตรงๆ, คุม Gripper#1+#2 พร้อมกัน) ──
        elif t == "motor_grip":
            self._servo_cmd(5, GRIP_CLOSE)
            self._servo_cmd(6, GRIP2_CLOSE)
            self._log(f"Gripper → CLOSE ({self.angles[5]}° / {self.angles[6]}°)")

        elif t == "motor_reverse":
            self._servo_cmd(5, GRIP_OPEN)
            self._servo_cmd(6, GRIP2_OPEN)
            self._log(f"Gripper → OPEN ({self.angles[5]}° / {self.angles[6]}°)")

        elif t == "motor_stop":
            self._servo_cmd(5, self.angles[5])
            self._servo_cmd(6, self.angles[6])
            self.motor_state = "stop"
            self._log(f"Gripper → HOLD ({self.angles[5]}° / {self.angles[6]}°)")

        # ── Posture ───────────────────────────────────────────
        elif t == "posture":
            self._send_posture(msg.get("name",""))

        # ── Save custom posture angles from Browser Settings ──
        elif t == "save_custom_posture":
            name   = msg.get("name","")
            angles = msg.get("angles", [])
            if name in CUSTOM_POSTURES and len(angles) == NUM_SERVOS:
                CUSTOM_POSTURES[name] = [float(a) for a in angles]
                self._log(f"Custom posture saved: {name} = {angles}")
                _push_ws({"type":"custom_posture_ack","name":name,"ok":True})
            else:
                self._log(f"save_custom_posture: invalid data name={name} len={len(angles)}")

        # ── arm_raw ───────────────────────────────────────────
        elif t == "arm_raw":
            _send_pi(msg)
            self._log(f"arm_raw: {msg.get('cmd','')}")

        elif t == "leds":
            _send_pi(msg)

        elif t == "laser":
            val = bool(msg.get("value", False))
            _send_pi({"type": "laser", "value": val})
            self._log(f"Laser → {'ON' if val else 'OFF'}")

        elif t == "snapshot":
            self._log("Snapshot requested")

        elif t == "qr_result":
            # Browser ส่ง QR result มาให้ log
            codes = msg.get("codes", [])
            if codes:
                self._log(f"[QR] Browser result: {' | '.join(codes)}")
    
        elif t == 'motor':
            # {"type":"motor","id":1,"action":"setvel","value":50}
            _send_pi(msg)
            self._log(f"Motor M{msg.get('id')} → {msg.get('action')} {msg.get('value','')}")

        elif t == 'motor_sync':
            # {"type":"motor_sync","action":"syncvel","v1":50,"v2":-50}
            _send_pi(msg)
            self._log(f"MotorSync → {msg.get('action')}")

        elif t == 'motor_all':
            # {"type":"motor_all","action":"stop"}
            _send_pi(msg)
            self._log(f"MotorAll → {msg.get('action')}")

        elif t == 'motor_feedback':
            # feedback จาก Teensy ผ่าน Pi → push ไป browser
            _push_ws(msg)

        elif t == 'motor_ack':
            # ack จาก Teensy → push ไป browser
            _push_ws(msg)

    def cleanup(self):
        self.running = False
        self._send_loop_running = False
        self._current_key = ''
        _send_pi({"type":"move_stop"})
        _send_pi({"type":"lock"})
        time.sleep(0.1)          # ให้ move_stop/lock ออกจาก buffer ก่อนตัด
        _close_pi_sock(hard=True)  # ส่ง RST → Pi ปล่อย connection เก่าทันที


# ═══════════════════════════════════════════════════════════════
#  WebSocket Server
# ═══════════════════════════════════════════════════════════════
def _push_ws(payload):
    global _ws_loop, _ws_queue
    if _ws_loop and _ws_loop.is_running() and _ws_queue is not None:
        try:
            asyncio.run_coroutine_threadsafe(_ws_queue.put(payload), _ws_loop)
        except Exception as e:
            print(f"[WS PUSH ERROR] {e}")

def _push_log(msg):
    if _ws_ctrl:
        _ws_ctrl._log(msg)
    else:
        print(msg)

async def _send_one(ws, data):
    await asyncio.wait_for(ws.send(data), timeout=WS_SEND_TIMEOUT)


async def _broadcast(payload):
    global _ws_clients
    if not _ws_clients: return
    data = json.dumps(payload)
    clients = list(_ws_clients)
    tasks = [asyncio.create_task(_send_one(ws, data)) for ws in clients]
    results = await asyncio.gather(*tasks, return_exceptions=True)
    dead = set()
    for ws, result in zip(clients, results):
        if isinstance(result, Exception): dead.add(ws)
    if dead: _ws_clients -= dead


async def _ws_handler(websocket, path=None):
    global _ws_clients
    _ws_clients.add(websocket)
    if _ws_ctrl:
        await websocket.send(json.dumps({"type":"state",**_ws_ctrl.snapshot()}))
    try:
        async for raw in websocket:
            if raw == "__ping__":
                await websocket.send("__pong__"); continue
            try:    msg = json.loads(raw)
            except: continue
            if _ws_ctrl: _ws_ctrl.handle_ws(msg)
    except websockets.exceptions.ConnectionClosedOK: pass
    except Exception as e: print(f"[ws] error: {e}")
    finally: _ws_clients.discard(websocket)


async def _ws_event_loop():
    global _ws_queue
    while True:
        payload = await _ws_queue.get()
        try:    await _broadcast(payload)
        except Exception as e: print(f"[WS EVENT ERROR] {e}")


async def _ws_state_loop():
    while True:
        await asyncio.sleep(1.0 / STATE_PUSH_HZ)
        if _ws_ctrl and _ws_clients:
            try:    await _broadcast({"type":"state",**_ws_ctrl.snapshot()})
            except Exception as e: print(f"[WS STATE ERROR] {e}")


async def _ws_serve():
    global _ws_queue
    _ws_queue = asyncio.Queue(maxsize=EVENT_QUEUE_MAX)
    async with websockets.serve(
        _ws_handler, "0.0.0.0", WS_PORT,
        max_queue=64, ping_interval=10,
        ping_timeout=10, close_timeout=1,
    ):
        await asyncio.gather(_ws_event_loop(), _ws_state_loop())


# ═══════════════════════════════════════════════════════════════
#  HTTP Server
# ═══════════════════════════════════════════════════════════════
def _http_server(base):
    class CORSHandler(SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=base, **kwargs)

        def end_headers(self):
            self.send_header("Access-Control-Allow-Origin",   "*")
            self.send_header("Access-Control-Allow-Methods",  "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers",  "Content-Type")
            self.send_header("X-Frame-Options",               "ALLOWALL")
            self.send_header("Content-Security-Policy",       "frame-ancestors *")
            super().end_headers()

        def do_OPTIONS(self):
            self.send_response(200)
            self.end_headers()

        def log_message(self, *a):
            pass

    # ThreadingHTTPServer: เสิร์ฟหลาย request พร้อมกัน เพื่อให้หน้า control
    # ขึ้นทันทีโดยไม่ต้องรอ iframe 3D ดูด STL ก้อนใหญ่ (~71MB) ให้เสร็จก่อน
    ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), CORSHandler).serve_forever()

# ═══════════════════════════════════════════════════════════════
#  Main
# ═══════════════════════════════════════════════════════════════
def main():
    global _ws_ctrl, _ws_loop, _qr_worker

    if sys.platform == "win32":
        try: sys.stdout.reconfigure(encoding="utf-8")
        except Exception: pass
        try: sys.stderr.reconfigure(encoding="utf-8")
        except Exception: pass
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())

    if getattr(sys, "frozen", False):
        base = sys._MEIPASS
    else:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        base = os.path.dirname(script_dir)

    print(f"[HTTP] Serving from: {base}")
    threading.Thread(target=_http_server, args=(base,), daemon=True).start()

    ctrl = RobotController()
    _ws_ctrl = ctrl

    threading.Thread(target=_pi_connect_thread, args=(ctrl,), daemon=True).start()
    threading.Thread(target=_pi_recv_thread,    args=(ctrl,), daemon=True).start()

    if QR_ENABLE:
        _qr_worker = QRWorker(
            rtsp_url=RTSP_URL,
            push_callback=_push_ws,
            log_callback=_push_log,
            jpeg_quality=QR_JPEG_QUALITY,
            detect_width=QR_DETECT_WIDTH,
            send_fps=QR_SEND_FPS,
        )
        _qr_worker.start()

    # ── IMU Reader (BWT901CL) — เตรียมไว้ ยังไม่เปิด ──────────────────────
    # uncomment เมื่อจะอ่าน IMU จาก COM port ตรงที่เครื่องนี้ (ดูคลาส IMUReader
    # ที่คอมเมนต์ไว้ด้านบน). ต้อง: global _imu_reader + ตั้ง IMU_* config ก่อน
    # if IMU_ENABLE:
    #     global _imu_reader
    #     _imu_reader = IMUReader(
    #         port=IMU_PORT, baud=IMU_BAUD,
    #         push_callback=_push_ws, log_callback=_push_log, send_hz=IMU_HZ,
    #     )
    #     _imu_reader.start()

    def _open():
        time.sleep(1.2)
        webbrowser.open(f"http://localhost:{HTTP_PORT}/ui/index.html")
    threading.Thread(target=_open, daemon=True).start()

    print(f"\n{'='*64}")
    print("  RescueBot Windows Bridge + QR")
    print(f"  WS   → ws://localhost:{WS_PORT}")
    print(f"  HTTP → http://localhost:{HTTP_PORT}/ui/index.html")
    print(f"  Pi   → {PI_IP}:{PI_TCP_PORT} (Control Center)")
    print(f"  RTSP → {RTSP_URL}")
    print(f"  Flask QR → {FLASK_BASE}")
    print(f"{'='*64}\n")

    # ── Cleanup ที่กันเรียกซ้ำ — ทำงานจริงใน main thread เท่านั้น ──
    # สำคัญ: signal handler ห้ามทำงานหนัก (sleep / ปิด socket) เพราะจะค้าง
    # หน้าที่ handler มีแค่ "สั่ง event loop หยุด" เบาที่สุด แล้วปล่อยให้
    # finally ทำ cleanup จริงหลัง loop จบ
    _cleaned = threading.Event()
    def _do_cleanup():
        if _cleaned.is_set():
            return
        _cleaned.set()
        if _qr_worker:
            _qr_worker.stop()
        # if _imu_reader:        # ← uncomment คู่กับ IMUReader
        #     _imu_reader.stop()
        ctrl.cleanup()

    def _on_signal(*_a):
        # เบาที่สุด: แค่สั่งให้ event loop หยุด → run_until_complete คืนค่า
        # → ไปเข้า finally → _do_cleanup() ใน main thread
        try:
            _ws_loop.call_soon_threadsafe(_ws_loop.stop)
        except Exception:
            pass

    atexit.register(_do_cleanup)

    _ws_loop = asyncio.new_event_loop()
    asyncio.set_event_loop(_ws_loop)

    try:
        signal.signal(signal.SIGINT,  _on_signal)
        signal.signal(signal.SIGTERM, _on_signal)
    except (ValueError, OSError):
        pass

    # ใช้ run_forever + task แทน run_until_complete เพื่อให้ loop.stop()
    # จาก signal handler หยุดได้สะอาด (run_until_complete จะ raise
    # RuntimeError ถ้าถูก stop กลางคัน)
    _serve_task = _ws_loop.create_task(_ws_serve())

    def _serve_done(_t):
        # ถ้า _ws_serve จบ (มัก = error ตอน bind port ชน) → หยุด loop
        # เพื่อไม่ให้ run_forever ค้างวนเปล่าๆ โดยไม่มี WS server
        try:
            _ws_loop.stop()
        except Exception:
            pass
    _serve_task.add_done_callback(_serve_done)

    try:
        _ws_loop.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        _do_cleanup()

    # surface error จาก _ws_serve (เช่น port 8765/8766 ถูกใช้อยู่)
    if _serve_task.done() and not _serve_task.cancelled():
        exc = _serve_task.exception()
        if exc is not None:
            print(f"\n[FATAL] WS/HTTP server error: {exc}")
            print("        → มักเกิดจาก port ถูกใช้อยู่ (rescue.py ค้างอีกตัว)")

    print("\nStopped.")


if __name__ == "__main__":
    main()
