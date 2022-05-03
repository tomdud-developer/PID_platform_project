#include <Arduino.h>

#include <ESP32Servo.h>
 
#define MAX_BUFF_LEN 255

Servo servoX1;
Servo servoX2;
Servo servoY1;
Servo servoY2;

 
int posX = 90; 
int posY = 90; 

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

char c;
char str[MAX_BUFF_LEN];
uint8_t idx = 0;


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
  Serial.begin(115200);
}

 
void loop() {

  //Serial.write(Serial.read());
 // 
 while(Serial.available()>0){
   c = Serial.read();
   if(c != '\n'){
     str[idx++] = c;
   }
   else{
     str[idx] = '\0';
     idx = 0;
     X = (str[0] - '0') * 100 + (str[1] - '0') + (str[2] - '0');
     Y = (str[3] - '0') * 100 + (str[4] - '0') + (str[5] - '0');
     Serial.write(str);
   }
 }
  servoY1.write(90);
    servoY2.write(90);

	/*	servoX1.write(90);
    servoX2.write(90);
    servoY1.write(90);
    servoY2.write(90);*/
 /*
	for (posX = minAngle; posX <= maxAngle; posX += 1) {
		servoX1.write(posX);
    servoX2.write(maxAngle-posX);
    //servoY1.write(posX);
   // servoY2.write(maxAngle-posX);
		delay(10);       
    printf("%d", posX);
     Serial.println( posX);
	}
	for (posX = maxAngle; posX >= minAngle; posX -= 1) { 
		servoX1.write(posX);
    servoX2.write(maxAngle-posX);
   // servoY1.write(posX);
   // servoY2.write(maxAngle-posX);
   Serial.println( posX);
     printf("%d", posX);
		delay(10);            
	}*/

/*
  for (int i = 0; i <= 360; i += 1) {
    cosVal = cos(i * 0.01745322535604579726333426417202) * (double)(maxAngle - minAngle) + (double)minAngle;
    sinVal = sin(i * 0.01745322535604579726333426417202) * (double)(maxAngle - minAngle) + (double)minAngle;
		servoX1.write(cosVal);
    servoX2.write(maxAngle-cosVal);
    servoY1.write(sinVal);
    servoY2.write(maxAngle-sinVal);
		delay(2);       
     //Serial.printf( " cos(%d) = %lf", i, cos(i * 1000.0 / 57296.0));
  }*/
  if(X>322){
    servoX1.write(90-30);
    servoX2.write(90+30);
  }
  else{
    servoX2.write(90-30);
    servoX1.write(90+30);
  }
}

/*
void rotateX(int angle){
  posX
  myservo.write(pos);
}*/