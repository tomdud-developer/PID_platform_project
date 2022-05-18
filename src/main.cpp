#include <Arduino.h>
#include <ESP32Servo.h>
#include <string.h>
#include <iostream>

#define MAX_BUFF_LEN 255

Servo servoX1;
Servo servoX2;
Servo servoY1;
Servo servoY2;

 
float posX = 0; 
float posY = 0; 
float setpointX = 0.5;
float setpointY = 0.5;
float UX = 0;
float UY = 0;
float angleX = 90;
float angleY = 90;
float sumX = 0;
float sumY = 0;
float prevX = 0;
float prevY = 0;

float kp = 40;
float ki = 50;
float kd = -25;

unsigned long loopTime = millis();
const int FREQUENCY = 33.333;
const unsigned long TIME = 1000 / FREQUENCY;
float dt = 1.0 / FREQUENCY;

int minAngle = 60;
int maxAngle = 180 - minAngle ;

int servoX1Pin = 13;
int servoX2Pin = 12;
int servoY1Pin = 14;
int servoY2Pin = 27;
int ii = 0;
int minUs = 1000;
int maxUs = 2000;
int cosVal = 0;
int sinVal = 0;
int X = 0;
int Y = 0;
float i = 0.0;

char c;
char str[MAX_BUFF_LEN];
uint8_t idx = 0;

void pid();

void setup() {
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
  servoX1.setPeriodHertz(50);
  servoX2.setPeriodHertz(50);
  servoY1.setPeriodHertz(50);
  servoY2.setPeriodHertz(50);
 
	servoX1.attach(servoX1Pin, minUs, maxUs);
  servoX2.attach(servoX2Pin, minUs, maxUs);
  servoY1.attach(servoY1Pin, minUs, maxUs);
  servoY2.attach(servoY2Pin, minUs, maxUs);
  Serial.begin(500000);
}

 
void loop() {

 while(Serial.available()>0){
   c = Serial.read();
   if(c != '\n'){
     str[idx++] = c;
   }
   else{
      str[idx] = '\0';
      idx = 0;
      String s = str;
      int pos = 7;
      String subX = s.substring(0 , pos);
      posX = subX.toFloat();
      String subY = s.substring(pos + 1 , s.length());
      posY = subY.toFloat();
      Serial.printf("ESP: X:%f, Y:%f, sumX:%f\n", UX, UY, sumX);


      cosVal = cos(i * 0.01745322535604579726333426417202);
      sinVal = sin(i * 0.01745322535604579726333426417202);
      i += dt / 80;
      if(i >= 360.0) i = 0.0;

      setpointX = 0.5 + cosVal / 2;
      setpointY = 0.5 + sinVal / 2;

      pid();

      servoX2.write(90 - angleX);
      servoY2.write(90 - angleY);

      
   }
  }
  
  
  
  /*
  servoX2.write(90);
  servoY2.write(90);
  */

}

void pid(){
  //Uchyb
  float eX = setpointX - posX;
  float eY = setpointY - posY;
  
  //Całka
  sumX += eX * ki * dt;
  sumY += eY * ki * dt;

  if(sumX > 10){
    sumX = 10;
  }
  else if( sumX < -10){
    sumX = -10;
  }
  if(sumY > 10){
    sumY = 10;
  }
  else if( sumY < -10){
    sumY = -10;
  }

  //Sterowanie
  UX = kp * eX + sumX + kd * (posX - prevX) / dt;
  UY = kp * eY + sumY + kd * (posY - prevY) / dt;

  angleX = UX;
  angleY = UY;


  //Ograniczenie sterowania
  if(angleX > maxAngle - minAngle){
    angleX = maxAngle - minAngle;
  }
  else if(angleX < -1*(maxAngle - minAngle)){
    angleX = -1*(maxAngle - minAngle);
  }
  if(angleY > maxAngle - minAngle){
    angleY = maxAngle - minAngle;
  }
  else if(angleY < -1*(maxAngle - minAngle)){
    angleY = -1*(maxAngle - minAngle);
  }


  prevX = posX;
  prevY = posY;
}
