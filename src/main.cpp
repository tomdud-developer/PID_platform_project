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
const int num = 7;
float* values = new float[num];

float filtered_e_X = 0;
float filtered_e_Y = 0;
const int deriv_filter_N = 3;
float e_X_total = 0;
float e_Y_total = 0;
float e_X_buf[deriv_filter_N];
float e_Y_buf[deriv_filter_N];
uint8_t e_X_i = 0;
uint8_t e_Y_i = 0;

float angleX = 90;
float angleY = 90;
float sumX = 0;
float sumY = 0;
float prevX = 0;
float prevY = 0;

//***************************     PID parameters regulation   *****************************
float kp = 40; //50
float ki = 0; // 50
float kd = -25;

float circural_move_multiplier = -1;
float eight_move_multiplier = -1;
unsigned long loopTime = millis();
const int FREQUENCY = 33.333;
const unsigned long TIME = 1000 / FREQUENCY;
float dt = 1.0 / FREQUENCY;

int minAngle = 70;
int maxAngle = 180 - minAngle ;

int servoX1Pin = 13;
int servoX2Pin = 12;
int servoY1Pin = 14;
int servoY2Pin = 27;
int ii = 0;
int minUs = 1000;
int maxUs = 2000;
float cosVal = 0;
float sinVal = 0;
float cosVal2 = 0;
int X = 0;
int Y = 0;
float i = 0.0;
bool cycle = false;

char c;
char str[MAX_BUFF_LEN];
uint8_t idx = 0;

void pid();
float moving_average_X(float newValue);
float moving_average_Y(float newValue);
void circural_move(float multiplier);
void figure8_move(float multiplier);
void get_values(String str);

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

  for(int i = 0; i < deriv_filter_N; i++){
    e_X_buf[i] = 0;
    e_Y_buf[i] = 0;
  }
  
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
      get_values(s);
      posX = values[0];
      posY = values[1];
      kp = values[2];
      ki = values[3];
      kd = values[4];
      circural_move_multiplier = values[5];
      eight_move_multiplier = values[6];
      //Serial.printf("ESP: X:%f, Y:%f, sumX:%f, sin:%f, posX:%f, posY:%f, kp:%f, ki:%f, kd:%f, cm:%f\n", UX, UY, sumX, sinVal, posX, posY, kp, ki, kd, circural_move_multiplier);
      Serial.printf("ESP: X:%.2f, Y:%.2f\n", UX, UY);

      pid();
      setpointX = 0.5;
      setpointY = 0.5;
      circural_move(circural_move_multiplier);
      figure8_move(eight_move_multiplier);
      
      servoX1.write(90 + angleX);
      servoX2.write(90 - angleX);
      servoY2.write(90 - angleY);
   }
  }
  
  
  /*
  servoX1.write(90);
  servoX2.write(90);
  servoY2.write(90);
  */

}

void get_values(String str){
  int q = 0;
  for(int i = 0; i < num; i++){
    int p = 0;
    String sub = "00000000";
    while(q < str.length() && str[q] != ','){
      sub[p] = str[q];
      q++;
      p++;
    }
    q++;
    values[i] = sub.toFloat();
  }
}


void pid(){
  //Uchyb
  float eX = setpointX - posX;
  float eY = setpointY - posY;
  
  //Całka
  sumX += eX * ki * dt;
  sumY += eY * ki * dt;

  int i_limit = 2;
  if(sumX > i_limit){
    sumX = i_limit;
  }
  else if( sumX < -1 * i_limit){
    sumX = -1 * i_limit;
  }
  if(sumY > i_limit){
    sumY = i_limit;
  }
  else if( sumY < -1 * i_limit){
    sumY = -1 * i_limit;
  }

  //Sterowanie
  filtered_e_X = moving_average_X(posX - prevX);
  filtered_e_Y = moving_average_Y(posY - prevY);

  UX = kp * eX + sumX + kd * filtered_e_X / dt;
  UY = kp * eY + sumY + kd * filtered_e_Y / dt;

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

float moving_average_X(float newValue){
  e_X_total -= e_X_buf[e_X_i];
  e_X_buf[e_X_i] = newValue;
  e_X_total += e_X_buf[e_X_i];
  e_X_i++;
  if(e_X_i >= deriv_filter_N){
    e_X_i = 0;
  }
  return (float)(e_X_total / deriv_filter_N);
}

float moving_average_Y(float newValue){
  e_Y_total -= e_Y_buf[e_Y_i];
  e_Y_buf[e_Y_i] = newValue;
  e_Y_total += e_Y_buf[e_Y_i];
  e_Y_i++;
  if(e_Y_i >= deriv_filter_N){
    e_Y_i = 0;
  }
  return (float)(e_Y_total / deriv_filter_N);
}

void circural_move(float multiplier){
  if(multiplier > 0.0){
    cosVal = cos(i * 0.01745322535604579726333426417202);
    sinVal = sin(i * 0.01745322535604579726333426417202);
    i += dt * multiplier;
    if(i >= 360.0) 
      i = 0.0;
    setpointX = 0.5 + cosVal / 4;
    setpointY = 0.5 + sinVal / 4;
  }
}

void figure8_move(float multiplier){
  if(multiplier > 0.0){
    cosVal = cos(i * 0.01745322535604579726333426417202);
    sinVal = sin(i * 0.01745322535604579726333426417202);

    cosVal2 = cos(i * 0.01745322535604579726333426417202 + 3.14);
    i += dt * multiplier;
    if(i >= 360.0){ 
        i = 0.0;
        cycle = !cycle;
    }

    if(cycle == 0){
      setpointX = 0.7 + cosVal2 / 6;
      setpointY = 0.5 + sinVal / 6;
    }
    else{
      setpointX = 0.3 + cosVal / 6;
      setpointY = 0.5 + sinVal / 6;
    }

  }
}