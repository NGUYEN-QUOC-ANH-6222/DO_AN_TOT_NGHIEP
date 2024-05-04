#include <Arduino.h>
#include <Wire.h>

#include "EEPROM.h"
#include "ServoTimer2.h"

#define PWM_1 9
#define PWM_2 10
#define DIR_1 7
#define DIR_2 5
#define ENC_1 3
#define ENC_2 2
#define BRAKE 8

#define STEERING_MAX 350
#define SPEED_MAX 80
#define STEERING_CENTER 1500
#define ST_LIMIT 5
#define SPEED_LIMIT 4

// define for MPU
#define MPU6050 0x68       // Device address
#define ACCEL_CONFIG 0x1C  // Accelerometer configuration address
#define GYRO_CONFIG 0x1B   // Gyro configuration address

// Registers: Accelerometer, Temp, Gyroscope
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C

#define accSens 0   // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define gyroSens 1  // 0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s

// Define for remote
#define STX 0x02
#define ETX 0x03
byte cmd[8] = {0, 0, 0, 0, 0, 0, 0, 0};
byte buttonStatus = 0;
//

float Gyro_amount = 0.996;  // 0.996

bool vertical = false;
bool calibrating = false;
bool calibrated = false;

float K1 = 690;          // 115 //-4047//5
float K2 = 300;         // 15 //-150
float K3 = 0.5;      // 8 //-2
float K4 = 0.1;        // 0.6
float loop_time = 10;  // 10

struct OffsetsObj {
  int ID;
  int16_t AcY;
  int16_t AcZ;
};
OffsetsObj offsets;

float alpha = 0.4;  // 0.4

int16_t AcY, AcZ, GyX, gyroX, gyroXfilt;

int16_t AcYc, AcZc;
int16_t GyX_offset = 0;  // GyX's offset value
int32_t GyX_offset_sum = 0;

float robot_angle;
float Acc_angle;

volatile byte pos;
volatile int motor_counter = 0, enc_count = 0;
int16_t motor_speed;
int32_t motor_pos;
int steering_remote = 0, speed_remote = 0, speed_value = 0, steering_value = STEERING_CENTER;

long currentT, previousT_1, previousT_2 = 0;

ServoTimer2 steering_servo;

// Line 104 - 180 from remote
void getButtonState(int bStatus) {
  switch (bStatus) {
      // -----------------  BUTTON #1  -----------------------
    case 'A':
      buttonStatus |= B000001;  // ON
      break;
    case 'B':
      buttonStatus &= B111110;  // OFF
      break;
      // -----------------  BUTTON #2  -----------------------
    case 'C':
      buttonStatus |= B000010;  // ON
      break;
    case 'D':
      buttonStatus &= B111101;  // OFF
      break;
  }
}

// Chuyển đổi data ASCII sang số nguyên
void getJoystickState(byte data[8]) {
  int joyX = (data[1] - 48) * 100 + (data[2] - 48) * 10 + (data[3] - 48);  // obtain the Int from the ASCII representation
  int joyY = (data[4] - 48) * 100 + (data[5] - 48) * 10 + (data[6] - 48);
  joyX = joyX - 200;  // Offset to avoid
  joyY = joyY - 200;  // transmitting negative numbers

  if (joyX < -100 || joyX > 100 || joyY < -100 || joyY > 100)
    return;                       // commmunication error
  if (joyX < -10 || joyX > 10) {  // dead zone
    if (joyX > 0)                 // exponential
      steering_remote = (-joyX * joyX + 0.1 * joyX) / 100.0;
    else
      steering_remote = (joyX * joyX + 0.1 * joyX) / 100.0;
  } else
    steering_remote = 0;
  steering_remote = -map(steering_remote, -100, 100, -STEERING_MAX, STEERING_MAX);
  if (joyY < -10 || joyY > 10)  // dead zone
    speed_remote = map(joyY, 100, -100, SPEED_MAX, -SPEED_MAX);
  else
    speed_remote = 0;
}

void readControlParameters() {
  if (Serial.available()) {  // data received from smartphone
    // delay(1);
    cmd[0] = Serial.read();
    if (cmd[0] == STX) {
      int i = 1;
      while (Serial.available()) {
        // delay(1);
        cmd[i] = Serial.read();
        if (cmd[i] > 127 || i > 7) break;  // Communication error
        if ((cmd[i] == ETX) && (i == 2 || i == 7))
          break;  // Button or Joystick data
        i++;
      }
      if (i == 2)
        getButtonState(cmd[1]);  // 3 Bytes  ex: < STX "C" ETX >
      else if (i == 7)
        getJoystickState(cmd);  // 6 Bytes  ex: < STX "200" "180" ETX >
    }
  }
}

String getButtonStatusString() {
  String bStatus = "";
  for (int i = 0; i < 6; i++) {
    if (buttonStatus & (B100000 >> i))
      bStatus += "1";
    else
      bStatus += "0";
  }
  return bStatus;
}

void sendControlParameters() {
  Serial.print((char)STX);  // Start of Transmission
  Serial.print(getButtonStatusString());
  Serial.print((char)0x1);  // buttons status feedback
  Serial.print(0);
  Serial.print((char)0x4);  // datafield #1
  Serial.print(0);
  Serial.print((char)0x5);  // datafield #2
  Serial.print(0);          // datafield #3
  Serial.print((char)ETX);  // End of Transmission
}

void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

void printValues() {  // Done
  Serial.print("K1: ");
  Serial.print(K1);
  Serial.print(" K2: ");
  Serial.print(K2);
  Serial.print(" K3: ");
  Serial.print(K3, 4);
  Serial.print(" K4: ");
  Serial.println(K4, 4);
}

void save() {  // save value calid into EPPROM
  EEPROM.put(0, offsets);
  delay(100);
  EEPROM.get(0, offsets);
  if (offsets.ID == 35) calibrated = true;
  calibrating = false;
  Serial.println("calibrating off");
}

void angle_calc() {  // Done
  Wire.beginTransmission(MPU6050);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true);
  GyX = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(MPU6050);
  Wire.write(ACCEL_YOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true);
  AcY = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(MPU6050);
  Wire.write(ACCEL_ZOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true);
  AcZ = Wire.read() << 8 | Wire.read();

  AcYc = AcY - offsets.AcY;
  AcZc = AcZ - offsets.AcZ;
  GyX -= GyX_offset;

  robot_angle += GyX * loop_time / 1000 / 65.536;  // calculate angle and convert to deg
  Acc_angle = -atan2(AcYc, -AcZc)*57.2958;  // calculate angle and convert from rad to deg (180/pi)
  robot_angle = robot_angle * Gyro_amount + Acc_angle * (1.0 - Gyro_amount);  // pitch = 0.96*(angle +GyroXangle/dt) + 0.4*(accelXangle)

  if (abs(robot_angle) > 10) vertical = false;  // 10
  if (abs(robot_angle) < 0.5) vertical = true;  // 0.4
}

void angle_setup() {  // Angle calibration
  Wire.begin();
  delay(100);
  writeTo(MPU6050, PWR_MGMT_1, 0);
  writeTo(MPU6050, ACCEL_CONFIG,
          accSens << 3);  // Specifying output scaling of accelerometer
  delay(100);

  for (int i = 0; i < 1024; i++) {
    angle_calc();
    GyX_offset_sum += GyX;
    delay(3);
  }
  GyX_offset = GyX_offset_sum >> 10;
  Serial.print("GyX offset: ");
  Serial.println(GyX_offset);
}

void Motor1_control(int sp) {
  if (sp > 0)
    digitalWrite(DIR_1, LOW);
  else
    digitalWrite(DIR_1, HIGH);
  analogWrite(PWM_1, 255 - abs(sp));  // 255
}

void Motor2_control(int sp) {
  if (sp > 0)
    digitalWrite(DIR_2, LOW);
  else
    digitalWrite(DIR_2, HIGH);
  analogWrite(PWM_2, 255 - abs(sp));
}

int Tuning() {
  if (!Serial.available()) return 0;
  delay(2);
  char param = Serial.read();  // get parameter byte
  if (!Serial.available()) return 0;
  char cmd = Serial.read();  // get command byte
  Serial.flush();
  switch (param) {
    case 'p':
      if (cmd == '+') K1 += 1;
      if (cmd == '-') K1 -= 1;
      printValues();
      break;
    case 'i':
      if (cmd == '+') K2 += 0.5;
      if (cmd == '-') K2 -= 0.5;
      printValues();
      break;
    case 's':
      if (cmd == '+') K3 += 0.2;
      if (cmd == '-') K3 -= 0.2;
      printValues();
      break;
    case 'a':
      if (cmd == '+') K4 += 0.05;
      if (cmd == '-') K4 -= 0.05;
      printValues();
      break;
    case 'c':
      if (cmd == '+' && !calibrating) {
        calibrating = true;
        Serial.println("calibrating on");
      }
      if (cmd == '-' && calibrating) {
        offsets.ID = 35;
        offsets.AcZ = AcZ + 16384;
        offsets.AcY = AcY;
        Serial.print("AcY: ");
        Serial.print(offsets.AcY);
        Serial.print(" AcZ: ");
        Serial.println(offsets.AcZ);
        save();
      }
      break;
  }
  return 1;
}

void ENC_READ() {
  byte cur = (!digitalRead(ENC_1) << 1) + !digitalRead(ENC_2);
  byte old = pos & B00000011;
  byte dir = (pos & B00110000) >> 4;

  if (cur == 3)
    cur = 2;
  else if (cur == 2)
    cur = 3;

  if (cur != old) {
    if (dir == 0) {
      if (cur == 1 || cur == 3) dir = cur;
    } else {
      if (cur == 0) {
        if (dir == 1 && old == 3)
          enc_count--;
        else if (dir == 3 && old == 1)
          enc_count++;
        //  Serial.print("chieu:" );Serial.println(dir);
        dir = 0;
      }
    }
    pos = (dir << 4) + (old << 2) + cur;
  }
}
//

void setup() {
  Serial.begin(9600);

  // Pins D9 and D10 - 7.8 kHz
  TCCR1A = 0b00000001;
  TCCR1B = 0b00001010;

  steering_servo.attach(A3);
  steering_servo.write(STEERING_CENTER);

  pinMode(DIR_1, OUTPUT);
  pinMode(DIR_2, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  pinMode(ENC_1, INPUT);
  pinMode(ENC_2, INPUT);

  Motor1_control(0);
  Motor2_control(0);

  attachInterrupt(0, ENC_READ, CHANGE);
  attachInterrupt(1, ENC_READ, CHANGE);

  EEPROM.get(0, offsets);
  if (offsets.ID == 35)
    calibrated = true;
  else
    calibrated = false;
  delay(3000);
  angle_setup();
}

void loop() {
  currentT = millis();

  if (currentT - previousT_1 >= loop_time) {
    //Tuning();
    readControlParameters();
    angle_calc();
    Serial.println(robot_angle);
    motor_speed = -enc_count;
    enc_count = 0;

    if (vertical && calibrated && !calibrating) {
      digitalWrite(BRAKE, HIGH);
      gyroX = GyX / 131.0;  // Convert to deg/s

      gyroXfilt = alpha * gyroX + (1 - alpha) * gyroXfilt;  // filter

      motor_pos += motor_speed;
      motor_pos = constrain(motor_pos, -110, 110);  // constrain(motor_pos, -110, 110)

      int pwm = constrain(K1 * robot_angle + K2 * gyroXfilt + K3 * motor_speed + K4 * motor_pos, -255, 255);
      Motor1_control(-pwm);
      // Serial.print("vong: "); Serial.println(enc_count);
      // Serial.print("chieu:" );Serial.println(pos);
      if ((speed_value - speed_remote) > SPEED_LIMIT)
        speed_value -= SPEED_LIMIT;
      else if ((speed_value - speed_remote) < -SPEED_LIMIT)
        speed_value += SPEED_LIMIT;
      else
        speed_value = speed_remote;
      if ((steering_value - STEERING_CENTER - steering_remote) > ST_LIMIT)
        steering_value -= ST_LIMIT;
      else if ((steering_value - STEERING_CENTER - steering_remote) < -ST_LIMIT)
        steering_value += ST_LIMIT;
      else
        steering_value = STEERING_CENTER + steering_remote;

      steering_servo.write(steering_value);
      Motor2_control(speed_value);
    } else {
      digitalWrite(BRAKE, LOW);
      steering_value = STEERING_CENTER;
      steering_servo.write(STEERING_CENTER);
      speed_value = 0;
      Motor1_control(0);
      Motor2_control(0);
      motor_pos = 0;
    }
    previousT_1 = currentT;
  }

  if (currentT - previousT_2 >= 2000) {
    if (!calibrated && !calibrating) {
      Serial.println("first you need to calibrate the balancing point...");
    }
    previousT_2 = currentT;
  }
}
