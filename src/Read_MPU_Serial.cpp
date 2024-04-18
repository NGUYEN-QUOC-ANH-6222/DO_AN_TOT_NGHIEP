#include  <Arduino.h>
#include <Wire.h>
#include "SimpleKalmanFilter.h"
SimpleKalmanFilter filterX(2, 2, 0.1),filterY(2, 2, 0.1),filterZ(2, 2, 0.1);
 
float u0 = 100.0; // giá trị thực (không đổi)
float x, y, z; // nhiễu
float x_kalman, y_kalman, z_kalman; // giá được lọc nhiễu

float gyroX, gyroY, gyroZ;
unsigned long prevTime =0;
unsigned long waitTime = 10;

void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission(); 
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8); 
  Wire.endTransmission(); 
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyX=Wire.read()<<8 | Wire.read();
  int16_t GyY=Wire.read()<<8 | Wire.read();
  int16_t GyZ=Wire.read()<<8 | Wire.read();
  gyroX=(float)GyX/65.5;
  gyroY=(float)GyY/65.5;
  gyroZ=(float)GyZ/65.5;
  y_kalman = filterY.updateEstimate(gyroY);
}
void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
}
void loop() {
    while(millis() - prevTime < waitTime){
  }
  prevTime = millis();
  gyro_signals();

  //Serial.println(accX + "," + accY + "," + accZ);/
  Serial.print(y_kalman);
  Serial.print(",");
  Serial.print(gyroY);
  Serial.print(",");
  Serial.println(gyroZ);
  
}
