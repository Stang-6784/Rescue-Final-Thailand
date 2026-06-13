#!/usr/bin/env python3
"""
pi_imu_bwt901.py  –  ฝั่ง Raspberry Pi (ป้อนข้อมูล IMU เข้า bridge :9000)
==========================================================================
อ่าน IMU WitMotion BWT901CL ผ่าน serial (โปรโตคอล 0x55) แล้ว parse เป็น
roll/pitch/yaw + accel/gyro/temp ส่งออกผ่าน callback `on_imu(dict)`

dict ที่ส่งออกมีรูปแบบตรงกับที่ rescue.py / control.html รออยู่:
    {"type":"imu","roll":..,"pitch":..,"yaw":..,
     "ax":..,"ay":..,"az":..,"wx":..,"wy":..,"wz":..,"temp":..}

──────────────────────────────────────────────────────────────────────────
วิธีต่อเข้า bridge เดิมของ Pi (Pi เป็น TCP server บน :9000)
──────────────────────────────────────────────────────────────────────────
ใน bridge ของ Pi ที่มีฟังก์ชัน broadcast ไปยัง client ทุกตัวอยู่แล้ว
(สมมติชื่อ broadcast_to_clients(obj: dict)) ให้ wire แบบนี้:

    from pi_imu_bwt901 import BWT901Reader

    imu = BWT901Reader(port="/dev/ttyUSB0", baud=115200, rate_hz=20)
    imu.on_imu = broadcast_to_clients   # ส่งตรงเข้า client ทุกตัว
    imu.start()

rescue.py ฝั่ง Windows จะ forward {"type":"imu",...} นี้เข้า WebSocket
ให้ widget มุมขวาบนของ control.html เอง (มี handler รออยู่แล้ว)

ต้องลง pyserial บน Pi:  pip install pyserial
"""

import time
import threading
import logging

try:
    import serial   # pyserial
except ImportError:
    serial = None

log = logging.getLogger("imu_bwt901")
log.setLevel(logging.INFO)
if not log.handlers:
    h = logging.StreamHandler()
    h.setFormatter(logging.Formatter("[%(asctime)s][%(levelname)s] %(message)s", "%H:%M:%S"))
    log.addHandler(h)


def _int16le(b, off):
    """little-endian signed 16-bit จาก buffer"""
    v = (b[off + 1] << 8) | b[off]
    return v - 65536 if v > 32767 else v


class BWT901Reader:
    """
    อ่าน BWT901CL ผ่าน serial เป็น background thread (daemon)
    parse แพ็กเก็ต 0x55 ขนาด 11 ไบต์ (0x51 accel / 0x52 gyro / 0x53 angle)
    ตรวจ checksum แล้วเรียก on_imu(dict) ตาม rate_hz ที่กำหนด
    """

    def __init__(self, port="/dev/ttyUSB0", baud=115200,
                 rate_hz=20, auto_reconnect=True):
        self.port           = port
        self.baud           = baud
        self.rate_hz        = max(1, rate_hz)
        self.auto_reconnect = auto_reconnect

        # callbacks
        self.on_imu         = None   # fn(dict)  ← wire เข้า broadcast ของ bridge
        self.on_connect     = None   # fn()
        self.on_disconnect  = None   # fn()

        # latest values
        self.roll = self.pitch = self.yaw = 0.0
        self.ax = self.ay = self.az = 0.0
        self.wx = self.wy = self.wz = 0.0
        self.temp = 0.0

        self._running   = False
        self._ser       = None
        self._buf       = bytearray()
        self._last_emit = 0.0
        self._lock      = threading.Lock()

    # ── public API ────────────────────────────────────────────────
    def start(self):
        """เริ่ม background thread – เรียกครั้งเดียว"""
        if serial is None:
            log.error("pyserial ไม่ได้ติดตั้ง — pip install pyserial")
            return
        self._running = True
        t = threading.Thread(target=self._loop, daemon=True, name="BWT901Reader")
        t.start()

    def stop(self):
        self._running = False
        try:
            if self._ser:
                self._ser.close()
        except Exception:
            pass

    def snapshot(self) -> dict:
        """ค่าล่าสุดในรูปแบบ message พร้อมส่ง"""
        with self._lock:
            return {
                "type": "imu",
                "roll": round(self.roll, 2),
                "pitch": round(self.pitch, 2),
                "yaw": round(self.yaw, 2),
                "ax": round(self.ax, 3), "ay": round(self.ay, 3), "az": round(self.az, 3),
                "wx": round(self.wx, 2), "wy": round(self.wy, 2), "wz": round(self.wz, 2),
                "temp": round(self.temp, 1),
            }

    # ── internal ──────────────────────────────────────────────────
    def _open(self):
        while self._running:
            try:
                self._ser = serial.Serial(self.port, self.baud, timeout=0.1)
                self._buf = bytearray()
                log.info(f"IMU connected {self.port} @ {self.baud}")
                if self.on_connect:
                    try: self.on_connect()
                    except Exception: pass
                return True
            except Exception as e:
                log.warning(f"เปิด serial ไม่ได้ ({e}) — ลองใหม่ใน 3 วิ")
                time.sleep(3)
        return False

    def _loop(self):
        while self._running:
            if not self._open():
                return
            try:
                while self._running:
                    chunk = self._ser.read(64)
                    if chunk:
                        self._buf.extend(chunk)
                        self._scan()
            except Exception as e:
                log.warning(f"IMU serial error: {e}")
                if self.on_disconnect:
                    try: self.on_disconnect()
                    except Exception: pass
                try: self._ser.close()
                except Exception: pass
                if not self.auto_reconnect:
                    break
                time.sleep(2)

    def _scan(self):
        """หา + parse แพ็กเก็ต 0x55 ที่ครบ 11 ไบต์ใน buffer"""
        buf = self._buf
        while len(buf) >= 11:
            # หา header 0x55 ตามด้วย type ที่รู้จัก
            idx = -1
            for i in range(len(buf) - 1):
                if buf[i] == 0x55 and buf[i + 1] in (0x51, 0x52, 0x53):
                    idx = i
                    break
            if idx == -1:
                # ไม่เจอ header — เก็บไบต์ท้ายไว้กันตัดกลางแพ็กเก็ต
                del buf[:-1]
                return
            if idx > 0:
                del buf[:idx]
            if len(buf) < 11:
                return

            pkt = bytes(buf[:11])
            del buf[:11]

            # checksum = ผลรวม 10 ไบต์แรก & 0xFF
            if (sum(pkt[:10]) & 0xFF) != pkt[10]:
                continue
            self._parse(pkt)

    def _parse(self, p):
        t = p[1]
        with self._lock:
            if t == 0x51:      # acceleration (g) + temp
                self.ax = _int16le(p, 2) / 32768.0 * 16.0
                self.ay = _int16le(p, 4) / 32768.0 * 16.0
                self.az = _int16le(p, 6) / 32768.0 * 16.0
                self.temp = _int16le(p, 8) / 100.0
            elif t == 0x52:    # angular velocity (°/s)
                self.wx = _int16le(p, 2) / 32768.0 * 2000.0
                self.wy = _int16le(p, 4) / 32768.0 * 2000.0
                self.wz = _int16le(p, 6) / 32768.0 * 2000.0
            elif t == 0x53:    # angle (°)
                self.roll  = _int16le(p, 2) / 32768.0 * 180.0
                self.pitch = _int16le(p, 4) / 32768.0 * 180.0
                self.yaw   = _int16le(p, 6) / 32768.0 * 180.0

        # ส่งออกตาม rate (อิงแพ็กเก็ต angle เป็นจังหวะหลัก)
        if t == 0x53:
            now = time.time()
            if now - self._last_emit >= 1.0 / self.rate_hz:
                self._last_emit = now
                if self.on_imu:
                    try: self.on_imu(self.snapshot())
                    except Exception as e:
                        log.warning(f"on_imu callback error: {e}")


# ── โหมดทดสอบ standalone — แค่ print ค่าออกมา ──────────────────────
if __name__ == "__main__":
    import sys
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    imu = BWT901Reader(port=port, baud=baud, rate_hz=10)
    imu.on_imu = lambda d: print(
        f"R:{d['roll']:+7.2f}  P:{d['pitch']:+7.2f}  Y:{d['yaw']:+7.2f}  "
        f"T:{d['temp']:.1f}°C"
    )
    imu.start()
    print(f"อ่าน BWT901CL จาก {port} @ {baud} — Ctrl+C เพื่อหยุด")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        imu.stop()
        print("\nหยุดแล้ว")
