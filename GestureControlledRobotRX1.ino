#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#define button 3
const int a=4,b=5,c=6,d=7;\
int x,y,z;
RF24 radio(9, 10); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};
boolean buttonState = 0;

void setup() {
  pinMode(button, INPUT);
  radio.begin();
  radio.openWritingPipe(addresses[0]); // 00001
  radio.openReadingPipe(1, addresses[1]); // 00002
  radio.setPALevel(RF24_PA_MIN);
}

void loop() {
  delay(5);
  lcd.clear();
  radio.startListening();
  if ( radio.available()) {
    while (radio.available()) {
      long int angleV = 0;
      radio.read(&angleV, sizeof(angleV));
      x=angleV/10000;
      y=(angleV/100)%100;
      z=angleV%100;
      Serial.println(x+" "+y+""+z);
    }
    delay(5);
    radio.stopListening();
    buttonState = digitalRead(button);
    radio.write(&buttonState, sizeof(buttonState));
  }
}
