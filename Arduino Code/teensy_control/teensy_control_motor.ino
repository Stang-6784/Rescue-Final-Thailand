/*
 * ============================================================================
 *  Teensy 4.0 Firmware - MOTOR ONLY (test build)
 * ============================================================================
 *  ตัดมาจาก firmware ตัวเต็ม เหลือเฉพาะส่วน "คุมมอเตอร์" สำหรับเทสมอเตอร์ก่อน
 *  เอาออก: servo, gripper, laser, LED ทั้งหมด (รวม PWMServo)
 *  เก็บไว้: CAN MIT-mode drive ของ 2x CubeMars AK45-10, โปรโตคอล serial,
 *           telemetry motor_feedback, และ ping/pong
 *
 *  >>> โปรโตคอล serial เหมือนเดิมทุกอย่าง <<<
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
 *
 *  Communication: USB Serial @ 115200 baud, line-based, ปิดท้ายด้วย '\n'
 *    คำสั่งที่รองรับ:
 *      {"type":"drive","linear":<-100..100>,"angular":<-100..100>}
 *      {"type":"motor","id":<1|2>,"action":"setvel","value":<-100..100>}
 *      {"type":"motor_sync","action":"syncvel","v1":<-100..100>,"v2":<-100..100>}
 *      {"type":"motor_all","action":"stop"}
 *      {"type":"ping"}                          -> ตอบ {"type":"pong"}
 *    Feedback ที่ส่งกลับ:
 *      {"type":"motor_feedback","motors":[velL,velR], ...}
 *      LOG: ...   (ข้อความ text ทั่วไป)
 *
 *  !!! ก่อนใช้จริง ตรวจสอบ MIT-mode limits กับ datasheet CubeMars AK45-10 !!!
 *
 *  Library dependencies:
 *    - FlexCAN_T4   (tonton81/FlexCAN_T4)
 *    - ArduinoJson  (bblanchon/ArduinoJson, v6.x)
 * ============================================================================
 */

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <ArduinoJson.h>
#include <PWMServo.h>   // flipper servos (bundled with Teensyduino)

// ============================================================================
// FLIPPER SERVOS (front + rear only — arm servos NOT attached in this build)
// ============================================================================

// ===== SERVOS 8 ตัว: [0]J1 [1]J2 [2]J3 [3]J4 [4]J5 [5]Gripper [6]Flip-F [7]Flip-R =====
const uint8_t NUM_SERVOS = 8;

//                                      J1  J2  J3  J4  J5  Grip Flip-F Flip-R
const uint8_t SERVO_PINS[NUM_SERVOS] = { 8,  9,  10,  5,  4,  3,    6,    7 };
const int SERVO_MIN[NUM_SERVOS]  = { 50, 10,  0,  0,  0, 45, 45, 45 };
const int SERVO_MAX[NUM_SERVOS]  = {150, 150, 180, 125, 180, 90, 160, 160 };
const int SERVO_HOME[NUM_SERVOS] = { 98, 90, 157, 90, 90, 70, 90, 90 };  // = POSTURE home
PWMServo servos[NUM_SERVOS];
int servoDeg[NUM_SERVOS];



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
const float DRIVE_KD = 1.0f;   // <-- tune ตามโหลด/มอเตอร์
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
      if (motorModeActive) {
        exitMotorModeBoth();
      }
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
void setServo(int idx, int deg){
  if (idx < 0 || idx >= NUM_SERVOS) return;
  servoDeg[idx] = clampServo(idx, deg);
  servos[idx].write(servoDeg[idx]);
  Serial.print("LOG: servo "); Serial.print(idx);
  Serial.print(" -> "); Serial.println(servoDeg[idx]);
}
// "SERVO <idx> <deg>"  ใช้ได้ครบทั้ง 8 ตัว
void handleServoCommand(const String &line){
  int s1 = line.indexOf(' ');           if (s1 < 0) return;
  int s2 = line.indexOf(' ', s1 + 1);   if (s2 < 0) return;
  int idx = line.substring(s1 + 1, s2).toInt();
  int deg = line.substring(s2 + 1).toInt();
  setServo(idx, deg);
}


const int POSTURE_HOME[NUM_SERVOS]   = { 98,170,157, 90, 90, 70, 90, 90 };
const int POSTURE_GUARD[NUM_SERVOS]  = { 50,130,  0, 90, 90, 70, 45,150 };
const int POSTURE_GIRAFF[NUM_SERVOS] = { 50,130,  0, 90, 90, 70, 57, 80 };
const int POSTURE_STAIR[NUM_SERVOS]  = { 50,130, 90,110, 90, 70,160, 45 };

void applyPosture(const int *arr){
  for (int i = 0; i < NUM_SERVOS; i++) setServo(i, arr[i]);
}
// "POSTURE <name>"
void handlePostureCommand(const String &line){
  int sp = line.indexOf(' ');  if (sp < 0) return;
  String name = line.substring(sp + 1); name.trim();
  if      (name.equalsIgnoreCase("home"))   applyPosture(POSTURE_HOME);
  else if (name.equalsIgnoreCase("guard"))  applyPosture(POSTURE_GUARD);
  else if (name.equalsIgnoreCase("giraff")) applyPosture(POSTURE_GIRAFF);
  else if (name.equalsIgnoreCase("stair"))  applyPosture(POSTURE_STAIR);
  else Serial.println("LOG: unknown posture");
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
  // คำสั่งอื่น (POSTURE/GRIP/LASER/LEDS) ยังไม่มีในบิลด์นี้ -> ข้ามเงียบ
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
  // คูณ direction ตอนส่งออก CAN เท่านั้น → currentVel*/telemetry ยังเก็บค่าตรรกะปกติ
  sendMitCommand(MOTOR_ID_LEFT, DRIVE_POSITION_DUMMY, currentVelLeft  * MOTOR_DIR_LEFT,
                 DRIVE_KP, DRIVE_KD, DRIVE_TORQUE_FF);
  sendMitCommand(MOTOR_ID_RIGHT, DRIVE_POSITION_DUMMY, currentVelRight * MOTOR_DIR_RIGHT,
                 DRIVE_KP, DRIVE_KD, DRIVE_TORQUE_FF);
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

  // Flipper servos: attach + ไปท่า home (กลางๆ) ทันทีตอน boot
  for (int i = 0; i < NUM_SERVOS; i++){
    servos[i].attach(SERVO_PINS[i]);
    servoDeg[i] = SERVO_HOME[i];
    servos[i].write(SERVO_HOME[i]);
  }

  // หมายเหตุ: ยังไม่ enter motor mode ตอน boot โดยตั้งใจ
  // มอเตอร์จะ "ปลุก" ก็ต่อเมื่อได้รับคำสั่ง drive/motor/motor_sync ครั้งแรก (ปลอดภัยกว่า)
  Serial.println("LOG: Teensy MOTOR + FLIPPER firmware started");
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
}
