#include <Arduino.h>
#include <Wire.h>
#include "EEPROM.h"
#include "ServoTimer2.h"
#include <SoftwareSerial.h>

#define PWM_1 9
#define PWM_2 10
#define DIR_1 7
#define DIR_2 5
#define ENC_1 3
#define ENC_2 2
#define BRAKE 8

#define SERVO_CENTER 1450 //-50
int SPEED_MOTOR_2 = 10;
int SPEED_MAX_F = 50;
int SPEED_MAX_B = -50;
int angle_L = 90;
ServoTimer2 my_servo;

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


float Gyro_amount = 0.996;  // 0.996

bool vertical = false;
bool calibrating = false;
bool calibrated = false;

float K1 = 400;          // 400 //-4047//5
float K2 = 8;         // 8.0 //-150
float K3 = 50;      // 50.1 //-2
float K4 = 0.0006;        // 0.0006
float alpha = 0.4;  // 0.4   `

struct OffsetsObj {
  int ID;
  int16_t AcY;
  int16_t AcZ;
};
OffsetsObj offsets;


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
int speed_remote = 0, speed_value = 0;
float loop_time = 10;  
long currentT, previousT_1, previousT_2 = 0;

unsigned long previousMillis = 0 ;
SoftwareSerial bluetooth(0, 1); // Khai báo chân kết nối với module Bluetooth

char receivedChar = ' ';

void Read_HC_05(){
  if (bluetooth.available()) { // Kiểm tra nếu có dữ liệu nhận được từ Bluetooth
    receivedChar = bluetooth.read(); // Đọc dữ liệu nhận được từ Bluetooth
    Serial.println(receivedChar); // In dữ liệu nhận được ra Serial Monitor
  }
}

int cal_servo(int convert) {
 return map(convert, 0, 180, 750, 2250) -50;
}

void Homing_Sevor() {
    my_servo.write(SERVO_CENTER);
}

void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
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

  // AcYc = AcY - 308; 
  // AcZc = AcZ - 31340; 
  // GyX -= -665; //GyX_offset
  AcYc = AcY - offsets.AcY;
  AcZc = AcZ - offsets.AcZ;
  GyX -= GyX_offset;

  robot_angle += GyX * loop_time / 1000 / 65.536;  // calculate angle and convert to deg
  Acc_angle = -atan2(AcYc, -AcZc)*57.2958;  // calculate angle and convert from rad to deg (180/pi)
  robot_angle = robot_angle * Gyro_amount + Acc_angle * (1.0 - Gyro_amount);  // pitch = 0.96*(angle +GyroXangle/dt) + 0.4*(accelXangle)

  if (abs(robot_angle) > 10) vertical = false;  // 10
  if (abs(robot_angle) < 0.4) vertical = true;  // 0.4
}

void angle_setup() {  // Angle calibration
  Wire.begin();
  delay(100);
  writeTo(MPU6050, PWR_MGMT_1, 0);
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3);  // Specifying output scaling of accelerometer
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

int Tuning() {
  if (!Serial.available()) return 0;
  delay(2);
  char param = Serial.read();  // get parameter byte
  if (!Serial.available()) return 0;
  char cmd = Serial.read();  // get command byte
  Serial.flush();
  switch (param) {
    case 'p':
      if (cmd == '+') K1 += 5;
      if (cmd == '-') K1 -= 5;
      printValues();
      break;
    case 'i':
      if (cmd == '+') K2 += 0.5;
      if (cmd == '-') K2 -= 0.5;
      printValues();
      break;
    case 's':
      if (cmd == '+') K3 += 1.0;
      if (cmd == '-') K3 -= 1.0;
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
        dir = 0;
      }
    }
    pos = (dir << 4) + (old << 2) + cur;
  }
}

void setup() {
  Serial.begin(115200);
  bluetooth.begin(115200);

  // Pins D9 and D10 - 7.8 kHz
  TCCR1A = 0b00000001;
  TCCR1B = 0b00001010;
  my_servo.attach(A3);

  pinMode(DIR_1, OUTPUT);
  pinMode(DIR_2, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  pinMode(ENC_1, INPUT);
  pinMode(ENC_2, INPUT);

  Motor1_control(0);
  Motor2_control(0);

  attachInterrupt(0, ENC_READ, CHANGE);
  attachInterrupt(1, ENC_READ, CHANGE);
  Homing_Sevor();
  
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
    // Tuning();
    Read_HC_05();
    angle_calc();
    motor_speed = -enc_count;
    enc_count = 0;
    if (vertical && calibrated && !calibrating) {
      digitalWrite(BRAKE, HIGH);

      gyroX = GyX / 131.0;  // Convert to deg/s
      gyroXfilt = alpha * gyroX + (1 - alpha) * gyroXfilt;  // low pass fillter
      motor_pos += motor_speed;
      motor_pos = constrain(motor_pos, -110, 110);  // constrain(motor_pos, -110, 110)

      int pwm = constrain(K1 * robot_angle + K2 * gyroXfilt + K3 * motor_speed + K4 * motor_pos, -255, 255);
      Motor1_control(-pwm);
      
      switch (receivedChar){
      case 'H':
        Homing_Sevor();
        angle_L = 90;
        receivedChar = ' ';
        break;
      case 'S':
        digitalWrite(BRAKE, LOW);
        Motor2_control(0);
        receivedChar = ' ';
        SPEED_MOTOR_2 = 0;
        break; 
      case 'F':
        if (SPEED_MOTOR_2 <= SPEED_MAX_F && millis() - previousMillis >= loop_time)
        {
        digitalWrite(BRAKE, HIGH);
        Motor2_control(SPEED_MOTOR_2++);
        previousMillis = millis();
        }
        break; 
      case 'B':
        if (SPEED_MOTOR_2 >= SPEED_MAX_B && millis() - previousMillis >= loop_time)
        {
        digitalWrite(BRAKE, HIGH);
        Motor2_control(SPEED_MOTOR_2--);
        previousMillis = millis();
        }
        break; 
      case 'R':
        if (angle_L <= 135 && millis() - previousMillis >= loop_time)
        {
        my_servo.write(cal_servo(angle_L++));
        previousMillis = millis();
        }
        break; 
      case 'L':
        if (angle_L >= 45 && millis() - previousMillis >= loop_time)
        {
        my_servo.write(cal_servo(angle_L--));
        previousMillis = millis();
        }
        break; 
      }
    } else {
      digitalWrite(BRAKE, LOW);
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
