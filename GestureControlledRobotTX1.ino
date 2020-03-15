#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include<Wire.h>
RF24 radio(7, 8); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};
boolean buttonState = 0;
const int MPU6050_addr=0x68;
int16_t AccX,AccY,AccZ,Temp,GyroX,GyroY,GyroZ,x,y,z;
void setup() {
  
  radio.begin();
  radio.openWritingPipe(addresses[1]); // 00002
  radio.openReadingPipe(1, addresses[0]); // 00001
  radio.setPALevel(RF24_PA_MIN);
  Wire.begin();
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}
void loop() {
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_addr,14,true);
  AccX=Wire.read()<<8|Wire.read();
  x=(AccX/500)+50;String x1=String(x);
  AccY=Wire.read()<<8|Wire.read();
  y=(AccY/500)+50;String y1=String(y);
  AccZ=Wire.read()<<8|Wire.read();
  z=(AccZ/1000)+50;String z1=String(z);
  Temp=Wire.read()<<8|Wire.read();
  GyroX=Wire.read()<<8|Wire.read();
  GyroY=Wire.read()<<8|Wire.read();
  GyroZ=Wire.read()<<8|Wire.read();
  String a = x1+y1+z1;
  long int b = a.toInt();
  delay(5);
  radio.stopListening();
  radio.write(&b, sizeof(b));
  delay(5);
  radio.startListening();
  while (!radio.available());
}
