/*
 * ============================================================================
 *  Teensy 4.0 Firmware - MOTOR + FLIPPER + IMU + THERMAL
 * ============================================================================
 *  ตัดมาจาก firmware ตัวเต็ม เหลือเฉพาะส่วน "คุมมอเตอร์" + flipper servo
 *  รวม IMU (BNO055) และ Thermal IR (MLX90614) เข้าด้วยกัน
 *  เอาออก: gripper/arm servo (ยังไม่มี handler)
 *  มี handler: LED (pin 12) + LASER (pin 11) digital on/off ผ่าน JSON
 *  เก็บไว้: CAN MIT-mode drive ของ 2x CubeMars AK45-10, โปรโตคอล serial,
 *           telemetry motor_feedback, IMU, thermal, และ ping/pong
 *
 *  >>> โปรโตคอล serial เหมือนเดิมทุกอย่าง + เพิ่ม thermal broadcast <<<
 *  ดังนั้น ROS2 bridge (control_motor_servo.py) ใช้ต่อได้เลย ไม่ต้องแก้
 *  - /motor/command ส่ง JSON มา -> มอเตอร์ทำงาน
 *  - /arm/command ส่งอะไรมาที่ไม่ใช่ JSON -> ถูกข้ามเงียบ (ไม่มี handler แล้ว)
 *
 *  Hardware:
 *    - Teensy 4.0, Arduino framework
 *    - 2x CubeMars AK45-10 KV75 (left/right) บน CAN bus
 *      - CAN1 = pins 22 (TX) / 23 (RX), transceiver SN65HVD230
 *      - Bus speed: 1,000,000 bps (1 Mbit/s)
 *      - Motor CAN IDs: LEFT = 1, RIGHT = 2
 *      - Control mode: MIT mode, ใช้สำหรับ VELOCITY control
 *    - BNO055 IMU (I2C: SDA=18, SCL=19, addr 0x29, Wire)
 *    - MLX90614 Thermal IR (I2C: SDA=17, SCL=16, addr 0x5A, Wire1 — บัสแยกจาก BNO055)
 *    - QMC5883L Magnetometer/Compass (GY-271 รุ่นใหม่, I2C: SDA=17, SCL=16, addr 0x0D, Wire1)
 *
 *  Communication: USB Serial @ 115200 baud, line-based, ปิดท้ายด้วย '\n'
 *    คำสั่งที่รองรับ:
 *      {"type":"drive","linear":<-100..100>,"angular":<-100..100>}
 *      {"type":"motor","id":<1|2>,"action":"setvel","value":<-100..100>}
 *      {"type":"motor_sync","action":"syncvel","v1":<-100..100>,"v2":<-100..100>}
 *      {"type":"motor_all","action":"stop"}
 *      {"type":"ping"}                          -> ตอบ {"type":"pong"}
 *      {"type":"led","state":<0|1>}             -> LED HIGH/LOW, ตอบ {"type":"led_ack",...}
 *      {"type":"laser","state":<0|1>}           -> LASER HIGH/LOW, ตอบ {"type":"laser_ack",...}
 *    Feedback ที่ส่งกลับ (ฝั่ง Pi/Windows อ่านได้เหมือน IMU):
 *      {"type":"motor_feedback","motors":[velL,velR], ...}
 *      {"type":"thermal","ambient":<°C>,"object":<°C>}   <-- ใหม่ (5 Hz)
 *      {"type":"mag","x":<raw>,"y":<raw>,"z":<raw>,"heading":<deg>,"dir":"<N..NW>"}  <-- ใหม่ (5 Hz)
 *      IMU,yaw,pitch,roll,sys,gyro,accel,mag,gx,gy,gz,ax,ay,az   (20 Hz, CSV เดิม)
 *      LOG: ...   (ข้อความ text ทั่วไป)
 *
 *  !!! ก่อนใช้จริง ตรวจสอบ MIT-mode limits กับ datasheet CubeMars AK45-10 !!!
 *
 *  Library dependencies:
 *    - FlexCAN_T4       (tonton81/FlexCAN_T4)
 *    - ArduinoJson      (bblanchon/ArduinoJson, v6.x)
 *    - Adafruit_BNO055  (+ Adafruit_Sensor)
 *    - Adafruit_MLX90614
 * ============================================================================
 */

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <ArduinoJson.h>
#include <PWMServo.h>   // flipper servos (bundled with Teensyduino)
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>   // IMU BNO055 (I2C: SDA=18, SCL=19)
#include <Adafruit_MLX90614.h> // Thermal IR sensor (I2C, address 0x5A, share bus กับ BNO055)

// ============================================================================
// FLIPPER SERVOS (front + rear only — arm servos NOT attached in this build)
// ============================================================================ 

// ===== SERVOS 9 ตัว: [0]J1 [1]J2 [2]J3 [3]J4 [4]J5 [5]Gripper [6]Gripper2 [7]Flip-F [8]Flip-R =====
const uint8_t NUM_SERVOS = 9;

//                                       J1    J2  J3   J4   J5   Grip Grip2  Flip-F Flip-R
const uint8_t SERVO_PINS[NUM_SERVOS] = {  8,   9,  10,   5,   4,   3,   2,     6,     7  };
const int SERVO_MIN[NUM_SERVOS]  =     { 50,  10,   0,   0,   0,  40,   0,     0,     0 };
const int SERVO_MAX[NUM_SERVOS]  =     {150, 150, 180, 125, 180, 180, 180,   180,   180 };

// ค่า home ตรงกับฝั่ง Python (rescue.py: SERVO_DEFAULTS / POSTURE_ANGLES["home"])
const int SERVO_HOME[NUM_SERVOS] = { 98, 150, 150,  80,  100, 100,  0,   85,   90 };
PWMServo servos[NUM_SERVOS];
int servoDeg[NUM_SERVOS];   // ค่าองศาที่ "เขียนลง servo จริง" ล่าสุด

// ── Servo slew (จำกัดความเร็วการหมุน servo ขณะเปลี่ยนท่า) ───────────────────
// แทนที่จะกระชากไปค่าปลายทางทันที (เต็มสปีด servo) เราเก็บ "เป้าหมาย" ไว้แล้ว
// ค่อยๆ ขยับ servoCurrent เข้าหาเป้าหมายทีละนิดในทุก loop → servo หมุนช้า/นุ่มขึ้น.
//   SERVO_SLEW_DPS = องศาต่อวินาที สูงสุด (ลด = ช้าลง, เพิ่ม = เร็วขึ้น)
//                    *** ควรตั้งให้ตรงกับ SERVO_SLEW_DPS ใน rescue.py เพื่อให้
//                        โมเดล 3D / UI ขยับตรงกับ servo จริง ***
//   ตั้งค่าสูงมาก (เช่น 100000) = ปิด easing กลับไปวิ่งเต็มสปีดแบบเดิม
const float    SERVO_SLEW_DPS       = 90.0f;  // ~2 วินาที ต่อการกวาด 180°
const uint32_t SERVO_SLEW_PERIOD_MS = 15;     // อัปเดต ~66 Hz (นุ่มพอ ไม่กิน CPU)
float    servoCurrent[NUM_SERVOS];            // องศาปัจจุบัน (ทศนิยม) ระหว่างไล่
int      servoTarget[NUM_SERVOS];             // องศาเป้าหมายที่สั่งล่าสุด
uint32_t lastServoSlewMs = 0;


// ============================================================================
// CAN BUS SETUP
// ============================================================================
// Teensy 4.0 CAN1 = pins 22 (TX) / 23 (RX), wired to SN65HVD230 transceiver.
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can0;

const uint32_t CAN_BITRATE = 1000000; // 1 Mbit/s, per AK45-10 MIT mode default

// Motor CAN IDs (left / right)
const uint8_t MOTOR_ID_LEFT  = 1;
const uint8_t MOTOR_ID_RIGHT = 2;

// ============================================================================
// MIT MODE PARAMETER LIMITS  (verify against CubeMars AK45-10 datasheet!)
// ============================================================================
const float P_MIN  = -12.5f;   // rad
const float P_MAX  =  12.5f;   // rad
const float V_MIN  = -20.0f;   // rad/s  <-- VERIFY
const float V_MAX  =  20.0f;   // rad/s
const float KP_MIN = 0.0f;
const float KP_MAX = 500.0f;
const float KD_MIN = 0.0f;
const float KD_MAX = 5.0f;
const float T_MIN  = -20.0f;   // N*m  <-- VERIFY
const float T_MAX  =  20.0f;   // N*m

// Velocity-mode tuning: Kp = 0 สำหรับ velocity control ล้วน, Kd = damping
const float DRIVE_KP = 0.0f;
const float DRIVE_KD = 2.0f;   // <-- tune ตามโหลด/มอเตอร์
// Kd ตอน "เบรก" (ล้อถูกสั่ง vel=0): สูงกว่า DRIVE_KD → ต้านความเร็วแรงขึ้น
// หุ่นหยุดไวขึ้นตอนปล่อยปุ่ม โดยไม่ต้อง exit motor mode (พร้อมออกตัวต่อทันที)
// tune ได้ในช่วง (DRIVE_KD, KD_MAX=5.0]; แรงไปล้ออาจกระตุก
const float BRAKE_KD = 4.0f;
const float DRIVE_TORQUE_FF = 0.0f;
const float DRIVE_POSITION_DUMMY = 0.0f; // ไม่ใช้ใน velocity mode, ส่ง 0

// ความเร็วสูงสุด (rad/s) ที่ตรงกับ drive command = +/-100
// ===========================================================================
//  >>> ค่านี้ตั้งไว้ "ต่ำ" สำหรับเทสครั้งแรกโดยตั้งใจ <<<
//  ของเดิมคือ 20.0f — เมื่อมั่นใจว่าทิศ/พฤติกรรมถูกต้องแล้ว ค่อยขยับขึ้น
// ===========================================================================
const float MAX_VELOCITY_RADPS = 18.0f; // AK45-10 @24V: rated ~15.7, no-load ~18.8 rad/s
                                        // ใช้ 15.0 ถ้าต้องการแรงบิดเต็มสำหรับปีน/ดัน

// Per-motor spin direction (มอเตอร์ซ้าย/ขวาประกบแบบ mirror → ตัวขวาต้องกลับเครื่องหมาย)
// ถ้าตัวไหนยังหมุนผิดทาง ให้สลับ +1.0f <-> -1.0f ของตัวนั้น
const float MOTOR_DIR_LEFT  = +1.0f;
const float MOTOR_DIR_RIGHT = -1.0f;   // <-- กลับทิศ ID 2 เพื่อให้เดินหน้าไปทางเดียวกับ ID 1

// MIT mode special CAN commands (8-byte frames)
const uint8_t CAN_CMD_ENTER_MOTOR_MODE[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
const uint8_t CAN_CMD_EXIT_MOTOR_MODE[8]  = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
const uint8_t CAN_CMD_ZERO_POSITION[8]    = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE};

bool motorModeActive = false;

// ============================================================================
// IMU (BNO055) — I2C: SDA=18, SCL=19 บน Teensy 4.0
// ============================================================================
// คงฟอร์แมตเดิมของ imu.ino ไว้ทุกอย่าง:
//   - คำสั่ง "cal" -> โหมดแสดงค่า calibration (พิมพ์ "CAL,...")
//   - คำสั่ง "run" -> โหมดส่งค่าให้ Python/ROS2
//
// รูปแบบบรรทัด "run" (ต่อท้ายของเดิม เพื่อ backward-compatible กับ test.py / rescue.py
// ที่อ่านแค่ฟิลด์ 1..7):
//   IMU,yaw,pitch,roll,sys,gyro,accel,mag,gx,gy,gz,ax,ay,az
//     [1..3]  yaw,pitch,roll        = Euler (deg)  ← ของเดิม
//     [4..7]  sys,gyro,accel,mag    = calibration status 0..3  ← ของเดิม (ไม่ใช่ค่าเซนเซอร์)
//     [8..10] gx,gy,gz              = angular velocity (rad/s)  ← ใหม่ สำหรับ sensor_msgs/Imu
//     [11..13] ax,ay,az             = linear acceleration (m/s^2, รวม gravity)  ← ใหม่
//   ฝั่ง ROS2 (ผ่าน TCP bridge :9000 บน Pi) เอา gx..gz + ax..az ไปสร้าง sensor_msgs/Imu
//   ให้ Cartographer/lidar fusion ใช้ได้ตรงๆ
// IMU เป็น optional: ถ้า boot แล้วไม่เจอ จะ "ข้าม" ไม่ block เพื่อให้มอเตอร์ยังทำงานได้
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);
bool imuReady = false;
bool imuCalMode = false;
uint32_t lastImuMs = 0;
const uint32_t IMU_PERIOD_MS = 50;   // 20 Hz, แทน delay(50) เดิม

// ============================================================================
// THERMAL SENSOR (MLX90614) — I2C บัสที่ 2: Wire1 (SCL=16, SDA=17)
// ============================================================================
// ต่อคนละบัสกับ BNO055 (Wire: SDA=18/SCL=19) ตามที่ต่อจริง (pin 16,17)
// ส่งค่าออกเป็น JSON broadcast (คล้าย motor_feedback) เพื่อให้ฝั่ง Pi/Windows
// parse ได้ตรงๆ โดยไม่ต้องเดา field แบบ CSV เหมือน IMU:
//   {"type":"thermal","ambient":<°C>,"object":<°C>}
// Thermal เป็น optional เหมือน IMU: ถ้า boot แล้วไม่เจอ จะ "ข้าม" ไม่ block ระบบอื่น
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
bool thermalReady = false;
uint32_t lastThermalMs = 0;
const uint32_t THERMAL_PERIOD_MS = 200;   // 5 Hz, ตรงกับที่ระบุไว้ในสเปก

// ============================================================================
// MAGNETOMETER / COMPASS (QMC5883L) — I2C บัสที่ 2: Wire1 (SCL=16, SDA=17)
// ============================================================================
// GY-271 รุ่นใหม่ = ชิป QMC5883L (addr 0x0D) ไม่ใช่ HMC5883L (0x1E) → register
// map คนละแบบ จึงเขียน raw-register driver ฝังในไฟล์ ไม่พึ่ง Adafruit HMC lib
// อยู่บัสเดียวกับ MLX90614 (0x5A) — address ไม่ชนกัน ใช้ร่วม Wire1 ได้เลย
//
// ส่งค่าออกเป็น JSON broadcast (แบบเดียวกับ thermal):
//   {"type":"mag","x":<raw>,"y":<raw>,"z":<raw>,"heading":<deg 0..360>}
//   - x,y,z = ค่า raw 16-bit signed (LSB) จากเซนเซอร์
//   - heading = มุมเข็มทิศคำนวณจาก atan2(y,x) หน่วยองศา (ยังไม่ชด declination)
// Magnetometer เป็น optional เหมือน IMU/thermal: ไม่เจอ → ข้าม ไม่ block ระบบอื่น
const uint8_t QMC5883L_ADDR   = 0x0D;
// QMC5883L registers
const uint8_t QMC5883L_REG_X_LSB   = 0x00;  // data เริ่มที่นี่ (X_LSB..Z_MSB = 6 ไบต์)
const uint8_t QMC5883L_REG_STATUS  = 0x06;  // bit0 = DRDY
const uint8_t QMC5883L_REG_CONFIG1 = 0x09;  // OSR/RNG/ODR/MODE
const uint8_t QMC5883L_REG_CONFIG2 = 0x0A;  // soft reset ฯลฯ
const uint8_t QMC5883L_REG_SETRESET= 0x0B;  // ต้องเขียน 0x01 ตาม datasheet
// CONFIG1 = OSR=512(0b00<<6) | RNG=8G(0b01<<4) | ODR=200Hz(0b11<<2) | MODE=Continuous(0b01)
//         = 0x00 | 0x10 | 0x0C | 0x01 = 0x1D
const uint8_t QMC5883L_CONFIG1_VAL  = 0x1D;
bool magReady = false;
uint32_t lastMagMs = 0;
const uint32_t MAG_PERIOD_MS = 200;   // 5 Hz broadcast

// ============================================================================
// LED (digital on/off) — สั่ง HIGH/LOW ตรงๆ ผ่าน JSON จาก rescue.py
// ============================================================================
//   {"type":"led","state":1}  -> เปิด (HIGH)
//   {"type":"led","state":0}  -> ปิด (LOW)
// *** แก้ LED_PIN ให้ตรงกับ pin จริงที่ต่อ (ค่า default = 14) ***
const uint8_t LED_PIN = 12;
bool ledState = false;

// ============================================================================
// LASER (digital on/off) — สั่ง HIGH/LOW ตรงๆ ผ่าน JSON จาก rescue.py
// ============================================================================
//   {"type":"laser","state":1}  -> เปิด (HIGH)
//   {"type":"laser","state":0}  -> ปิด (LOW)
const uint8_t LASER_PIN = 11;
bool laserState = false;

// ============================================================================
// SERIAL COMMAND BUFFER
// ============================================================================
String serialLineBuffer = "";

// ============================================================================
// FAILSAFE / TIMING
// ============================================================================
const uint32_t DRIVE_TIMEOUT_MS   = 500;   // หยุดมอเตอร์ถ้าไม่มีคำสั่ง drive ในเวลานี้
const uint32_t CAN_SEND_PERIOD_MS = 10;    // ส่ง MIT frame ทุก 10 ms

uint32_t lastDriveCommandMs = 0;
uint32_t lastCanSendMs = 0;
bool driveActive = false; // true เมื่อได้รับคำสั่ง drive อย่างน้อยหนึ่งครั้ง

// ความเร็วที่สั่งอยู่ (rad/s) ของ left/right
float currentVelLeft = 0.0f;
float currentVelRight = 0.0f;

// feedback ล่าสุดจากมอเตอร์ (decode จาก CAN reply) สำหรับ telemetry
float fbPosLeft = 0, fbVelLeft = 0, fbTorqueLeft = 0; int fbTempLeft = 0;
float fbPosRight = 0, fbVelRight = 0, fbTorqueRight = 0; int fbTempRight = 0;
bool haveFbLeft = false, haveFbRight = false;

// ============================================================================
// MIT MODE PACKING HELPERS
// ============================================================================

uint32_t floatToUint(float x, float x_min, float x_max, uint8_t bits) {
  if (x < x_min) x = x_min;
  if (x > x_max) x = x_max;
  float span = x_max - x_min;
  float scale = (float)((1UL << bits) - 1);
  return (uint32_t)((x - x_min) * (scale / span));
}

float uintToFloat(uint32_t x_int, float x_min, float x_max, uint8_t bits) {
  float span = x_max - x_min;
  float scale = (float)((1UL << bits) - 1);
  return ((float)x_int) * (span / scale) + x_min;
}

// Byte layout (per CubeMars/MIT-mode spec):
//   [0] pos[15:8] [1] pos[7:0] [2] vel[11:4]
//   [3] vel[3:0]|kp[11:8] [4] kp[7:0] [5] kd[11:4]
//   [6] kd[3:0]|t[11:8] [7] t[7:0]
void packMitFrame(uint8_t *buf, float position, float velocity,
                  float kp, float kd, float torque) {
  uint32_t p_int  = floatToUint(position, P_MIN, P_MAX, 16);
  uint32_t v_int  = floatToUint(velocity, V_MIN, V_MAX, 12);
  uint32_t kp_int = floatToUint(kp, KP_MIN, KP_MAX, 12);
  uint32_t kd_int = floatToUint(kd, KD_MIN, KD_MAX, 12);
  uint32_t t_int  = floatToUint(torque, T_MIN, T_MAX, 12);

  buf[0] = (uint8_t)(p_int >> 8);
  buf[1] = (uint8_t)(p_int & 0xFF);
  buf[2] = (uint8_t)(v_int >> 4);
  buf[3] = (uint8_t)(((v_int & 0x0F) << 4) | (kp_int >> 8));
  buf[4] = (uint8_t)(kp_int & 0xFF);
  buf[5] = (uint8_t)(kd_int >> 4);
  buf[6] = (uint8_t)(((kd_int & 0x0F) << 4) | (t_int >> 8));
  buf[7] = (uint8_t)(t_int & 0xFF);
}

// Reply layout: [0]=id [1..2]=pos [3..4]=vel [5..6]=torque [7? temp]
void decodeMitReply(const uint8_t *buf, uint8_t &id, float &position,
                    float &velocity, float &torque, int &temperatureC) {
  id = buf[0];
  uint32_t p_int = ((uint32_t)buf[1] << 8) | buf[2];
  uint32_t v_int = ((uint32_t)buf[3] << 4) | (buf[4] >> 4);
  uint32_t t_int = (((uint32_t)buf[4] & 0x0F) << 8) | buf[5];
  position = uintToFloat(p_int, P_MIN, P_MAX, 16);
  velocity = uintToFloat(v_int, V_MIN, V_MAX, 12);
  torque   = uintToFloat(t_int, T_MIN, T_MAX, 12);
  temperatureC = (int8_t)buf[6]; // best-effort
}

void sendRawMotorCmd(uint8_t motorId, const uint8_t *data8) {
  CAN_message_t msg;
  msg.id = motorId;
  msg.len = 8;
  memcpy(msg.buf, data8, 8);
  can0.write(msg);
}

void sendMitCommand(uint8_t motorId, float position, float velocity,
                    float kp, float kd, float torque) {
  CAN_message_t msg;
  msg.id = motorId;
  msg.len = 8;
  packMitFrame(msg.buf, position, velocity, kp, kd, torque);
  can0.write(msg);
}

void enterMotorModeBoth() {
  sendRawMotorCmd(MOTOR_ID_LEFT, CAN_CMD_ENTER_MOTOR_MODE);
  delay(2);
  sendRawMotorCmd(MOTOR_ID_RIGHT, CAN_CMD_ENTER_MOTOR_MODE);
  motorModeActive = true;
  Serial.println("LOG: motor mode ENTER");
}

void exitMotorModeBoth() {
  sendRawMotorCmd(MOTOR_ID_LEFT, CAN_CMD_EXIT_MOTOR_MODE);
  delay(2);
  sendRawMotorCmd(MOTOR_ID_RIGHT, CAN_CMD_EXIT_MOTOR_MODE);
  motorModeActive = false;
  Serial.println("LOG: motor mode EXIT");
}

// ============================================================================
// DRIVE COMMAND -> MOTOR VELOCITY CONVERSION
// ============================================================================

float mapCommandToVelocity(float cmd) {
  if (cmd > 100) cmd = 100;
  if (cmd < -100) cmd = -100;
  return (cmd / 100.0f) * MAX_VELOCITY_RADPS;
}

// differential drive: left = linear+angular, right = linear-angular
void applyDriveCommand(float linear, float angular) {
  float leftCmd  = linear + angular;
  float rightCmd = linear - angular;
  if (leftCmd > 100) leftCmd = 100;
  if (leftCmd < -100) leftCmd = -100;
  if (rightCmd > 100) rightCmd = 100;
  if (rightCmd < -100) rightCmd = -100;

  currentVelLeft  = mapCommandToVelocity(leftCmd);
  currentVelRight = mapCommandToVelocity(rightCmd);

  lastDriveCommandMs = millis();
  driveActive = true;

  if (!motorModeActive) {
    enterMotorModeBoth();
  }
}

void stopMotorsVelocity() {
  currentVelLeft = 0.0f;
  currentVelRight = 0.0f;
}

// {"type":"motor","id":...,"action":"setvel","value":...}
void setSingleMotorVelocity(uint8_t motorId, float cmdValue) {
  float vel = mapCommandToVelocity(cmdValue);
  if (motorId == MOTOR_ID_LEFT) {
    currentVelLeft = vel;
  } else if (motorId == MOTOR_ID_RIGHT) {
    currentVelRight = vel;
  } else {
    return; // unknown id -> ignore
  }
  lastDriveCommandMs = millis();
  driveActive = true;
  if (!motorModeActive) {
    enterMotorModeBoth();
  }
}

// {"type":"motor_sync","action":"syncvel","v1":...,"v2":...}
void setSyncMotorVelocity(float v1, float v2) {
  currentVelLeft  = mapCommandToVelocity(v1);
  currentVelRight = mapCommandToVelocity(v2);
  lastDriveCommandMs = millis();
  driveActive = true;
  if (!motorModeActive) {
    enterMotorModeBoth();
  }
}

// ============================================================================
// JSON COMMAND HANDLER (motor only)
// ============================================================================

void handleJsonCommand(const String &line) {
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, line);
  if (err) {
    // JSON พัง -> ข้ามเงียบ
    return;
  }

  const char *type = doc["type"];
  if (type == nullptr) return;

  if (strcmp(type, "drive") == 0) {
    if (!doc.containsKey("linear") || !doc.containsKey("angular")) return;
    float linear = doc["linear"];
    float angular = doc["angular"];
    applyDriveCommand(linear, angular);

  } else if (strcmp(type, "motor_all") == 0) {
    const char *action = doc["action"];
    if (action != nullptr && strcmp(action, "stop") == 0) {
      stopMotorsVelocity();
      driveActive = false;
      lastDriveCommandMs = millis();
      // ไม่ exit motor mode: คง frame ต่อเนื่องเพื่อเบรกล้อให้นิ่ง (vel=0 + BRAKE_KD)
      // → หยุดไวกว่าเดิม (เดิม exit = ตัดไฟ แล้วล้อไหลตามแรงเฉื่อย)
      //   และพร้อมออกตัวทันทีโดยไม่ต้อง re-enter motor mode
    }

  } else if (strcmp(type, "motor") == 0) {
    const char *action = doc["action"];
    if (action != nullptr && strcmp(action, "setvel") == 0) {
      if (!doc.containsKey("id") || !doc.containsKey("value")) return;
      uint8_t id = doc["id"];
      float value = doc["value"];
      setSingleMotorVelocity(id, value);
    }

  } else if (strcmp(type, "motor_sync") == 0) {
    const char *action = doc["action"];
    if (action != nullptr && strcmp(action, "syncvel") == 0) {
      if (!doc.containsKey("v1") || !doc.containsKey("v2")) return;
      float v1 = doc["v1"];
      float v2 = doc["v2"];
      setSyncMotorVelocity(v1, v2);
    }

  } else if (strcmp(type, "ping") == 0) {
    Serial.println("{\"type\":\"pong\"}");

  } else if (strcmp(type, "led") == 0) {
    if (!doc.containsKey("state")) return;
    int state = doc["state"];
    ledState = (state != 0);
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
    // echo กลับให้ rescue.py ยืนยันสถานะ (optional แต่ช่วย debug)
    Serial.print("{\"type\":\"led_ack\",\"state\":");
    Serial.print(ledState ? 1 : 0);
    Serial.println("}");

  } else if (strcmp(type, "laser") == 0) {
    if (!doc.containsKey("state")) return;
    int state = doc["state"];
    laserState = (state != 0);
    digitalWrite(LASER_PIN, laserState ? HIGH : LOW);
    // echo กลับให้ rescue.py ยืนยันสถานะ
    Serial.print("{\"type\":\"laser_ack\",\"state\":");
    Serial.print(laserState ? 1 : 0);
    Serial.println("}");
  }
  // type อื่น (เช่น leds) -> ข้ามเงียบ
}

// ============================================================================
// FLIPPER COMMAND HANDLER
// ============================================================================
// รองรับ 2 รูปแบบ:
//   "SERVO 6 <deg>" / "SERVO 7 <deg>"   ← ตรงกับที่ rescue.py ส่ง (6=หน้า 7=หลัง)
//   "FLIPPER F <deg>" / "FLIPPER R <deg>"  ← พิมพ์ง่ายตอนเทสใน Serial Monitor
// องศาถูก clamp อยู่ใน [45,160] เสมอ


static int clampServo(int idx, int deg){
  if (deg < SERVO_MIN[idx]) deg = SERVO_MIN[idx];
  if (deg > SERVO_MAX[idx]) deg = SERVO_MAX[idx];
  return deg;
}
// ตั้ง "เป้าหมาย" ของ servo — การหมุนจริงเกิดทีละนิดใน updateServoEasing()
void setServo(int idx, int deg){
  if (idx < 0 || idx >= NUM_SERVOS) return;
  servoTarget[idx] = clampServo(idx, deg);
  Serial.print("LOG: servo "); Serial.print(idx);
  Serial.print(" -> "); Serial.println(servoTarget[idx]);
}

// ค่อยๆ ขยับ servoCurrent เข้าหา servoTarget (non-blocking, เรียกทุก loop)
void updateServoEasing(){
  uint32_t now = millis();
  uint32_t dt  = now - lastServoSlewMs;
  if (dt < SERVO_SLEW_PERIOD_MS) return;
  lastServoSlewMs = now;

  float maxStep = SERVO_SLEW_DPS * (dt / 1000.0f);   // องศาที่ขยับได้ในรอบนี้
  if (maxStep <= 0.0f) maxStep = 1000.0f;            // กัน 0 → เท่ากับปิด easing
  for (int i = 0; i < NUM_SERVOS; i++){
    float diff = (float)servoTarget[i] - servoCurrent[i];
    if (diff >  maxStep) diff =  maxStep;
    if (diff < -maxStep) diff = -maxStep;
    servoCurrent[i] += diff;
    int w = (int)lroundf(servoCurrent[i]);
    if (w != servoDeg[i]) {        // เขียนเฉพาะตอนค่าเปลี่ยน (ลดภาระ)
      servoDeg[i] = w;
      servos[i].write(w);
    }
  }
}
// "SERVO <idx> <deg>"  ใช้ได้ครบทั้ง 8 ตัว
void handleServoCommand(const String &line){
  int s1 = line.indexOf(' ');           if (s1 < 0) return;
  int s2 = line.indexOf(' ', s1 + 1);   if (s2 < 0) return;
  int idx = line.substring(s1 + 1, s2).toInt();
  int deg = line.substring(s2 + 1).toInt();
  setServo(idx, deg);
}

void applyPosture(const int *arr){
  for (int i = 0; i < NUM_SERVOS; i++) setServo(i, arr[i]);
}
// "POSTURE <name>"
// firmware เก็บไว้แค่ท่า "home" เท่านั้น (= SERVO_HOME)
// ท่าอื่น (guard/giraff/stair/custom) ฝั่ง Python สั่งเองผ่าน servo_set/SERVO <idx> <deg>
void handlePostureCommand(const String &line){
  int sp = line.indexOf(' ');  if (sp < 0) return;
  String name = line.substring(sp + 1); name.trim();
  if (name.equalsIgnoreCase("home")) applyPosture(SERVO_HOME);
  else Serial.println("LOG: unknown posture");
}
// ============================================================================
// IMU (BNO055) READ + OUTPUT  (คงฟอร์แมตเดิมของ imu.ino)
// ============================================================================

void readAndSendImu() {
  if (!imuReady) return;

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> acc   = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);  // m/s^2 (รวม gravity)
  imu::Vector<3> gyr   = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);      // deg/s (Adafruit default)

  float yaw_raw   = euler.x();
  float pitch_raw = euler.y();
  float roll_raw  = euler.z();

  // BNO055 (Adafruit lib) คืน gyro เป็น deg/s → แปลงเป็น rad/s ให้ตรงกับ sensor_msgs/Imu
  const float DEG2RAD = 0.01745329252f;
  float gx = gyr.x() * DEG2RAD;
  float gy = gyr.y() * DEG2RAD;
  float gz = gyr.z() * DEG2RAD;

  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);

  if (imuCalMode) {
    Serial.print("CAL,SYS=");
    Serial.print(sys);
    Serial.print(",G=");
    Serial.print(gyro);
    Serial.print(",A=");
    Serial.print(accel);
    Serial.print(",M=");
    Serial.print(mag);
    Serial.print(",AX=");
    Serial.print(acc.x(), 2);
    Serial.print(",AY=");
    Serial.print(acc.y(), 2);
    Serial.print(",AZ=");
    Serial.println(acc.z(), 2);
  } else {
    Serial.print("IMU,");
    Serial.print(yaw_raw, 2);
    Serial.print(",");
    Serial.print(pitch_raw, 2);
    Serial.print(",");
    Serial.print(roll_raw, 2);
    Serial.print(",");
    Serial.print(sys);
    Serial.print(",");
    Serial.print(gyro);
    Serial.print(",");
    Serial.print(accel);
    Serial.print(",");
    Serial.print(mag);
    // ── ค่าเซนเซอร์จริงสำหรับ ROS2 sensor_msgs/Imu ──
    Serial.print(",");
    Serial.print(gx, 4);   // angular velocity x (rad/s)
    Serial.print(",");
    Serial.print(gy, 4);   // angular velocity y (rad/s)
    Serial.print(",");
    Serial.print(gz, 4);   // angular velocity z (rad/s)
    Serial.print(",");
    Serial.print(acc.x(), 3);  // linear acceleration x (m/s^2)
    Serial.print(",");
    Serial.print(acc.y(), 3);  // linear acceleration y (m/s^2)
    Serial.print(",");
    Serial.println(acc.z(), 3); // linear acceleration z (m/s^2)
  }
}

// ============================================================================
// THERMAL (MLX90614) READ + OUTPUT
// ============================================================================
// รูปแบบ JSON เพื่อให้ parse ง่ายฝั่ง Pi/Windows (ต่างจาก IMU ที่เป็น CSV เดิม):
//   {"type":"thermal","ambient":<°C 2 ตำแหน่งทศนิยม>,"object":<°C 2 ตำแหน่งทศนิยม>}
void readAndSendThermal() {
  if (!thermalReady) return;

  float ambientC = mlx.readAmbientTempC();
  float objectC  = mlx.readObjectTempC();

  Serial.print("{\"type\":\"thermal\",\"ambient\":");
  Serial.print(ambientC, 2);
  Serial.print(",\"object\":");
  Serial.print(objectC, 2);
  Serial.println("}");
}

// ============================================================================
// MAGNETOMETER (QMC5883L on Wire1) — raw-register driver + JSON OUTPUT
// ============================================================================

// helper: เขียน 1 register บน Wire1
static bool qmcWriteReg(uint8_t reg, uint8_t val) {
  Wire1.beginTransmission(QMC5883L_ADDR);
  Wire1.write(reg);
  Wire1.write(val);
  return (Wire1.endTransmission() == 0);
}

// เริ่มต้น QMC5883L: soft reset → set/reset period → continuous mode
// คืน true ถ้าคุยกับ chip สำเร็จ (ACK ครบทุกขั้น)
bool initMagnetometer() {
  Wire1.beginTransmission(QMC5883L_ADDR);
  if (Wire1.endTransmission() != 0) return false;   // ไม่มี ACK ที่ 0x0D → ไม่เจอ chip

  bool ok = true;
  ok &= qmcWriteReg(QMC5883L_REG_CONFIG2,  0x80);   // soft reset
  delay(10);
  ok &= qmcWriteReg(QMC5883L_REG_SETRESET, 0x01);   // set/reset period (ตาม datasheet)
  ok &= qmcWriteReg(QMC5883L_REG_CONFIG1,  QMC5883L_CONFIG1_VAL); // OSR512/8G/200Hz/continuous
  return ok;
}

// อ่าน X/Y/Z (raw 16-bit signed) — คืน false ถ้าอ่านไม่ครบ 6 ไบต์
static bool qmcReadRaw(int16_t &mx, int16_t &my, int16_t &mz) {
  Wire1.beginTransmission(QMC5883L_ADDR);
  Wire1.write(QMC5883L_REG_X_LSB);
  if (Wire1.endTransmission(false) != 0) return false;   // repeated start

  uint8_t n = Wire1.requestFrom((int)QMC5883L_ADDR, 6);
  if (n < 6) return false;
  uint8_t b[6];
  for (int i = 0; i < 6; i++) b[i] = Wire1.read();

  mx = (int16_t)(b[0] | (b[1] << 8));   // LSB ก่อน (QMC little-endian)
  my = (int16_t)(b[2] | (b[3] << 8));
  mz = (int16_t)(b[4] | (b[5] << 8));
  return true;
}

// heading (deg 0..360) → ชื่อทิศ 8 ทิศ (N=เหนือ ... NW=ตะวันตกเฉียงเหนือ)
// แบ่งเป็นช่วงละ 45° โดยให้ N อยู่กึ่งกลางช่วง (337.5..360 และ 0..22.5)
static const char* headingToCardinal(float heading) {
  static const char* dirs[8] = {"N","NE","E","SE","S","SW","W","NW"};
  int idx = (int)((heading + 22.5f) / 45.0f) & 7;   // & 7 = mod 8 กันช่วง wrap 360→0
  return dirs[idx];
}

// รูปแบบ JSON (แบบเดียวกับ thermal):
//   {"type":"mag","x":<raw>,"y":<raw>,"z":<raw>,"heading":<deg 0..360>,"dir":"<N..NW>"}
void readAndSendMag() {
  if (!magReady) return;

  int16_t mx, my, mz;
  if (!qmcReadRaw(mx, my, mz)) return;   // อ่านพลาดรอบนี้ → ข้ามเงียบ

  // heading จากระนาบ X-Y (สมมติเซนเซอร์วางระนาบ) ยังไม่ชด magnetic declination
  float heading = atan2f((float)my, (float)mx) * 57.29577951f; // rad → deg
  if (heading < 0) heading += 360.0f;

  Serial.print("{\"type\":\"mag\",\"x\":");
  Serial.print(mx);
  Serial.print(",\"y\":");
  Serial.print(my);
  Serial.print(",\"z\":");
  Serial.print(mz);
  Serial.print(",\"heading\":");
  Serial.print(heading, 2);
  Serial.print(",\"dir\":\"");
  Serial.print(headingToCardinal(heading));
  Serial.println("\"}");
}

// ============================================================================
// TOP-LEVEL LINE DISPATCH
// ============================================================================

void processLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line.charAt(0) == '{') {
    handleJsonCommand(line);
    return;
  }

  if (line.startsWith("SERVO")) {   handleServoCommand(line);   return; }
  if (line.startsWith("POSTURE")) { handlePostureCommand(line); return; }

  // คำสั่ง IMU (เหมือน imu.ino เดิม)
  if (line == "cal") { imuCalMode = true;  Serial.println("INFO,MODE_CAL"); return; }
  if (line == "run") { imuCalMode = false; Serial.println("INFO,MODE_RUN"); return; }
  // คำสั่งอื่น (GRIP/LASER/LEDS) ยังไม่มีในบิลด์นี้ -> ข้ามเงียบ
}

// อ่าน serial แบบ non-blocking, สะสมจนเจอ '\n' แล้วค่อย dispatch
void pollSerialInput() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      processLine(serialLineBuffer);
      serialLineBuffer = "";
    } else if (c != '\r') {
      serialLineBuffer += c;
      if (serialLineBuffer.length() > 512) {
        serialLineBuffer = "";
      }
    }
  }
}

// ============================================================================
// CAN RX HANDLING (motor feedback)
// ============================================================================

void pollCanFeedback() {
  CAN_message_t msg;
  while (can0.read(msg)) {
    if (msg.len < 7) continue;

    uint8_t id;
    float pos, vel, torque;
    int temp;
    decodeMitReply(msg.buf, id, pos, vel, torque, temp);

    if (id == MOTOR_ID_LEFT) {
      fbPosLeft = pos; fbVelLeft = vel; fbTorqueLeft = torque; fbTempLeft = temp;
      haveFbLeft = true;
    } else if (id == MOTOR_ID_RIGHT) {
      fbPosRight = pos; fbVelRight = vel; fbTorqueRight = torque; fbTempRight = temp;
      haveFbRight = true;
    }
  }
}

// ============================================================================
// PERIODIC MOTOR CAN TRANSMIT
// ============================================================================

void sendMotorFrames() {
  if (!motorModeActive) return;
  // ล้อที่ถูกสั่ง vel=0 → ใช้ BRAKE_KD (เบรกแรงขึ้น ให้หยุดไว) ; ล้อที่ยังวิ่ง → DRIVE_KD
  float kdLeft  = (currentVelLeft  == 0.0f) ? BRAKE_KD : DRIVE_KD;
  float kdRight = (currentVelRight == 0.0f) ? BRAKE_KD : DRIVE_KD;
  // คูณ direction ตอนส่งออก CAN เท่านั้น → currentVel*/telemetry ยังเก็บค่าตรรกะปกติ
  sendMitCommand(MOTOR_ID_LEFT, DRIVE_POSITION_DUMMY, currentVelLeft  * MOTOR_DIR_LEFT,
                 DRIVE_KP, kdLeft, DRIVE_TORQUE_FF);
  sendMitCommand(MOTOR_ID_RIGHT, DRIVE_POSITION_DUMMY, currentVelRight * MOTOR_DIR_RIGHT,
                 DRIVE_KP, kdRight, DRIVE_TORQUE_FF);
}

// ============================================================================
// TELEMETRY (feedback to Pi over Serial)
// ============================================================================

void sendMotorFeedback() {
  Serial.print("{\"type\":\"motor_feedback\",\"motors\":[");
  Serial.print(currentVelLeft, 3);
  Serial.print(",");
  Serial.print(currentVelRight, 3);
  Serial.print("]");

  if (haveFbLeft) {
    Serial.print(",\"left\":{\"pos\":");
    Serial.print(fbPosLeft, 4);
    Serial.print(",\"vel\":");
    Serial.print(fbVelLeft, 4);
    Serial.print(",\"torque\":");
    Serial.print(fbTorqueLeft, 4);
    Serial.print(",\"temp\":");
    Serial.print(fbTempLeft);
    Serial.print("}");
  }
  if (haveFbRight) {
    Serial.print(",\"right\":{\"pos\":");
    Serial.print(fbPosRight, 4);
    Serial.print(",\"vel\":");
    Serial.print(fbVelRight, 4);
    Serial.print(",\"torque\":");
    Serial.print(fbTorqueRight, 4);
    Serial.print(",\"temp\":");
    Serial.print(fbTempRight);
    Serial.print("}");
  }
  Serial.println("}");
}

// ============================================================================
// SETUP / LOOP
// ============================================================================

uint32_t lastFeedbackMs = 0;
const uint32_t FEEDBACK_PERIOD_MS = 200; // telemetry rate

void setup() {
  Serial.begin(115200);

  can0.begin();
  can0.setBaudRate(CAN_BITRATE);

  lastDriveCommandMs = millis();
  lastCanSendMs = millis();
  lastFeedbackMs = millis();
  lastImuMs = millis();

  // IMU BNO055 (I2C SDA=18 / SCL=19) — optional, ไม่ block boot ถ้าไม่เจอ
  Wire.begin();
  Wire.setClock(100000);
  Serial.println("INFO,START_BNO055");
  if (bno.begin()) {
    delay(1000);
    bno.setExtCrystalUse(true);
    delay(100);
    imuReady = true;
    Serial.println("INFO,BNO055_READY");
  } else {
    imuReady = false;
    Serial.println("ERROR,BNO055_NOT_FOUND");  // มอเตอร์ยังทำงานต่อได้
  }

  // Thermal MLX90614 (I2C บัสที่ 2: Wire1, SCL=16/SDA=17) — optional, ไม่ block boot ถ้าไม่เจอ
  Wire1.begin();
  Wire1.setClock(100000);
  Serial.println("INFO,START_MLX90614");
  if (mlx.begin(MLX90614_I2CADDR, &Wire1)) {
    thermalReady = true;
    Serial.println("INFO,MLX90614_READY");
  } else {
    thermalReady = false;
    Serial.println("ERROR,MLX90614_NOT_FOUND");  // มอเตอร์/IMU ยังทำงานต่อได้
  }

  // Magnetometer QMC5883L (Wire1 บัสเดียวกับ MLX90614) — optional, ไม่ block boot
  Serial.println("INFO,START_QMC5883L");
  if (initMagnetometer()) {
    magReady = true;
    Serial.println("INFO,QMC5883L_READY");
  } else {
    magReady = false;
    Serial.println("ERROR,QMC5883L_NOT_FOUND");  // ระบบอื่นยังทำงานต่อได้
  }

  // LED (digital output) — เริ่มที่ปิด (LOW) ตอน boot
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  ledState = false;

  // LASER (digital output) — เริ่มที่ปิด (LOW) ตอน boot
  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);
  laserState = false;

  // Flipper servos: attach + ไปท่า home (กลางๆ) ทันทีตอน boot
  for (int i = 0; i < NUM_SERVOS; i++){
    servos[i].attach(SERVO_PINS[i]);
    servoDeg[i]     = SERVO_HOME[i];
    servoCurrent[i] = (float)SERVO_HOME[i];   // เริ่มที่ home (ไม่มีการไล่ตอน boot)
    servoTarget[i]  = SERVO_HOME[i];
    servos[i].write(SERVO_HOME[i]);
  }
  lastServoSlewMs = millis();

  // หมายเหตุ: ยังไม่ enter motor mode ตอน boot โดยตั้งใจ
  // มอเตอร์จะ "ปลุก" ก็ต่อเมื่อได้รับคำสั่ง drive/motor/motor_sync ครั้งแรก (ปลอดภัยกว่า)
  lastThermalMs = millis();
  lastMagMs = millis();
  Serial.println("LOG: Teensy MOTOR + FLIPPER + IMU + THERMAL + MAG firmware started");
}

void loop() {
  // 1) รับคำสั่งจาก Pi (non-blocking)
  pollSerialInput();

  // 2) ดึง CAN feedback ที่ค้างอยู่
  pollCanFeedback();

  // 3) Failsafe: ถ้าไม่มีคำสั่ง drive ในเวลาที่กำหนด -> บังคับความเร็ว 0
  if (driveActive && (millis() - lastDriveCommandMs > DRIVE_TIMEOUT_MS)) {
    stopMotorsVelocity();
    driveActive = false;
  }

  // 4) ส่ง MIT frame เป็นรอบๆ (มอเตอร์ต้องการ frame ต่อเนื่อง ไม่ใช่ครั้งเดียว)
  if (millis() - lastCanSendMs >= CAN_SEND_PERIOD_MS) {
    lastCanSendMs = millis();
    sendMotorFrames();
  }

  // 5) ส่ง telemetry กลับ Pi เป็นรอบๆ
  if (millis() - lastFeedbackMs >= FEEDBACK_PERIOD_MS) {
    lastFeedbackMs = millis();
    sendMotorFeedback();
  }

  // 6) อ่าน + ส่งค่า IMU เป็นรอบๆ (non-blocking แทน delay(50) เดิม)
  if (millis() - lastImuMs >= IMU_PERIOD_MS) {
    lastImuMs = millis();
    readAndSendImu();
  }

  // 7) อ่าน + ส่งค่า Thermal (MLX90614) เป็นรอบๆ (JSON broadcast, 5 Hz)
  if (millis() - lastThermalMs >= THERMAL_PERIOD_MS) {
    lastThermalMs = millis();
    readAndSendThermal();
  }

  // 8) อ่าน + ส่งค่า Magnetometer (QMC5883L) เป็นรอบๆ (JSON broadcast, 5 Hz)
  if (millis() - lastMagMs >= MAG_PERIOD_MS) {
    lastMagMs = millis();
    readAndSendMag();
  }

  // 9) ไล่องศา servo เข้าหาเป้าหมายทีละนิด (จำกัดความเร็วการหมุน, non-blocking)
  updateServoEasing();
}
