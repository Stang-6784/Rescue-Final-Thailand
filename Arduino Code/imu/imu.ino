#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// Teensy 4.0 I2C
// SDA = pin 18
// SCL = pin 19

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

bool calMode = false;

void setup()
{
  Serial.begin(115200);
  delay(1500);

  Wire.begin();
  Wire.setClock(100000);

  Serial.println("INFO,START_BNO055");

  if (!bno.begin())
  {
    Serial.println("ERROR,BNO055_NOT_FOUND");
    while (1)
    {
      delay(100);
    }
  }

  delay(1000);

  bno.setExtCrystalUse(true);
  delay(100);

  Serial.println("INFO,BNO055_READY");
  Serial.println("INFO,COMMANDS:");
  Serial.println("INFO,cal = show calibration");
  Serial.println("INFO,run = send IMU to Python");
  Serial.println("INFO,FORMAT:");
  Serial.println("INFO,IMU,yaw_raw,pitch_raw,roll_raw,sys,gyro,accel,mag");
}

void loop()
{
  if (Serial.available())
  {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "cal")
    {
      calMode = true;
      Serial.println("INFO,MODE_CAL");
    }
    else if (cmd == "run")
    {
      calMode = false;
      Serial.println("INFO,MODE_RUN");
    }
  }

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  float yaw_raw = euler.x();
  float pitch_raw = euler.y();
  float roll_raw = euler.z();

  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);

  if (calMode)
  {
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
  }
  else
  {
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
    Serial.println(mag);
  }

  delay(50);
}