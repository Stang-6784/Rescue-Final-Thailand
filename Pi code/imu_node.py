#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
============================================================================
 imu_node.py  —  /teensy/feedback (BNO055 ASCII)  ⇄  sensor_msgs/Imu
============================================================================
 รันบน Pi/Ubuntu เป็น ROS2 node. หน้าที่:
   - subscribe /teensy/feedback (std_msgs/String) ที่ control_motor_servo.py
     (teensy_serial node) republish ทุกบรรทัดจาก serial Teensy มาให้
   - กรองเฉพาะบรรทัดที่ขึ้นต้น "IMU," แล้ว parse:
        IMU,yaw,pitch,roll,sys,gyro,accel,mag,gx,gy,gz,ax,ay,az
          [1..3]  yaw,pitch,roll   = Euler (deg)
          [4..7]  sys,gyro,accel,mag = calibration status 0..3
          [8..10] gx,gy,gz         = angular velocity (rad/s)
          [11..13] ax,ay,az        = linear acceleration (m/s^2, รวม gravity)
   - publish sensor_msgs/Imu ออก /imu/data ให้ Cartographer / lidar fusion ใช้

 *ทำไม subscribe /teensy/feedback แทนที่จะเปิด serial เอง?*
   เพราะ teensy_serial node ถือพอร์ต /dev/teensy อยู่แล้ว (เปิดซ้ำไม่ได้)
   และมัน republish ทุกบรรทัดออก /teensy/feedback อยู่แล้ว → เกาะ topic นั้นพอ

 *Cartographer ใช้ field ไหน?*
   ใช้ angular_velocity (gyro) + linear_acceleration (accel รวม gravity)
   ส่วน orientation มี BNO055 fuse ให้ (ใส่ไว้ด้วย เผื่อ node อื่นอยากใช้)

 params:
   frame_id   (string) : TF frame ของ IMU (default 'imu_link')
   imu_topic  (string) : ชื่อ topic ที่ publish (default '/imu/data')                           
============================================================================
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Imu

DEG2RAD = math.pi / 180.0


def euler_deg_to_quaternion(yaw_deg, pitch_deg, roll_deg):
    """Euler (deg, ลำดับ ZYX: yaw→pitch→roll) → quaternion (x,y,z,w).
    ตรงกับที่ .ino แมป yaw=euler.x, pitch=euler.y, roll=euler.z ของ BNO055."""
    y = yaw_deg   * DEG2RAD
    p = pitch_deg * DEG2RAD
    r = roll_deg  * DEG2RAD
    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class ImuNode(Node):
    def __init__(self):
        super().__init__("imu_node")

        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("imu_topic", "/imu/data")
        # topic ที่ tcp_bridge subscribe แล้ว forward JSON ขึ้น Windows (rescue.py)
        self.declare_parameter("win_topic", "/imu/win")
        self.frame_id  = self.get_parameter("frame_id").value
        imu_topic      = self.get_parameter("imu_topic").value
        win_topic      = self.get_parameter("win_topic").value

        # sensor data: BEST_EFFORT เหมาะกับสตรีมความถี่สูง (ตรงกับที่ Cartographer คาดหวัง)
        qos = QoSProfile(
            depth=50,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.pub_imu = self.create_publisher(Imu, imu_topic, qos)

        # ── ส่งค่า IMC แบบ JSON ขึ้น Windows ผ่าน tcp_bridge ──
        # ใช้ RELIABLE (depth เล็ก) เพราะ tcp_bridge แค่ relay ต่อ ไม่ต้องการสตรีมถี่
        self.pub_win = self.create_publisher(String, win_topic, 10)
        # ส่งขึ้น Windows ที่ ~10Hz พอสำหรับโชว์มุม (ลดโหลด TCP ไม่สแปมทุกบรรทัด)
        self.declare_parameter("win_rate_hz", 10.0)
        win_rate = float(self.get_parameter("win_rate_hz").value)
        self._win_period_ns = int(1e9 / win_rate) if win_rate > 0 else 0
        self._last_win_ns = 0

        self.create_subscription(String, "/teensy/feedback", self._on_feedback, 50)

        self._warned_short = False
        self._count = 0
        self.get_logger().info(
            f"imu_node: /teensy/feedback (IMU,...) -> {imu_topic} "
            f"(frame_id={self.frame_id})")

    def _on_feedback(self, msg: String):
        line = msg.data.strip()
        if not line.startswith("IMU,"):
            return

        parts = line.split(",")
        # ต้องครบ 14 ฟิลด์ (เฟิร์มแวร์ใหม่). ของเก่า 8 ฟิลด์ -> ไม่มี gyro/accel จริง, ข้าม
        if len(parts) < 14:
            if not self._warned_short:
                self._warned_short = True
                self.get_logger().warn(
                    f"IMU line มี {len(parts)} ฟิลด์ (<14): เฟิร์มแวร์ Teensy ยังไม่ส่ง "
                    f"gyro/accel จริง — แฟลช teensy_control_motor.ino เวอร์ชันใหม่ก่อน")
            return

        try:
            yaw   = float(parts[1])
            pitch = float(parts[2])
            roll  = float(parts[3])
            cal_sys, cal_gyro, cal_accel, cal_mag = (
                int(parts[4]), int(parts[5]), int(parts[6]), int(parts[7]))
            gx, gy, gz = float(parts[8]),  float(parts[9]),  float(parts[10])
            ax, ay, az = float(parts[11]), float(parts[12]), float(parts[13])
        except ValueError:
            return  # บรรทัดเพี้ยน -> ข้าม

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = self.frame_id

        qx, qy, qz, qw = euler_deg_to_quaternion(yaw, pitch, roll)
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw

        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz

        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az

        # covariance (diagonal). ค่าประมาณสำหรับ BNO055 — ปรับจูนได้ตามงาน.
        # element [0] != -1 = "มีข้อมูล" (ถ้าใส่ -1 consumer จะถือว่าไม่มี orientation)
        imu.orientation_covariance         = [0.0025, 0.0, 0.0,
                                              0.0, 0.0025, 0.0,
                                              0.0, 0.0, 0.0025]
        imu.angular_velocity_covariance    = [0.0004, 0.0, 0.0,
                                              0.0, 0.0004, 0.0,
                                              0.0, 0.0, 0.0004]
        imu.linear_acceleration_covariance = [0.04, 0.0, 0.0,
                                              0.0, 0.04, 0.0,
                                              0.0, 0.0, 0.04]

        self.pub_imu.publish(imu)

        # ── ส่ง JSON ขึ้น Windows (throttle ตาม win_rate_hz) ──
        now_ns = self.get_clock().now().nanoseconds
        if self._win_period_ns == 0 or (now_ns - self._last_win_ns) >= self._win_period_ns:
            self._last_win_ns = now_ns
            wmsg = String()
            wmsg.data = json.dumps({
                "type": "imu",
                "yaw": round(yaw, 2), "pitch": round(pitch, 2), "roll": round(roll, 2),
                "gyro": [round(gx, 4), round(gy, 4), round(gz, 4)],
                "accel": [round(ax, 3), round(ay, 3), round(az, 3)],
                "calib": {"sys": cal_sys, "gyro": cal_gyro,
                          "accel": cal_accel, "mag": cal_mag},
            })
            self.pub_win.publish(wmsg)

        self._count += 1
        if self._count % 100 == 1:   # log เบาๆ ทุก ~5 วิ (ที่ 20Hz)
            self.get_logger().info(
                f"imu #{self._count}: yaw={yaw:.1f} pitch={pitch:.1f} roll={roll:.1f} "
                f"gz={gz:.3f} az={az:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
