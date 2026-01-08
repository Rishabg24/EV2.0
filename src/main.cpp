#include <Arduino.h>
#include "Motor.h"
#include "PID.h"
#include "Drive.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;
Encoder Rencoder(3 /*Encoder Pin A*/, 5 /*Encoder Pin B*/);
Motor Rmotor(8 /*IN1*/, 9 /*IN2*/, 11 /*ENA*/);
Encoder Lencoder(2 /*Encoder Pin A*/, 4 /*Encoder Pin B*/);
Motor Lmotor(6 /*IN3*/ , 7 /*IN4*/ ,10 /*ENB*/);
PID right(0.1f/*Kp*/, 0.999f/*Ki*/, 0.f/*Kd*/, 40.f/*Diameter of the wheels*/, 75.81f /*Gear Ratio of Motor*/, 12 /*CPR*/);
PID left(0.11f/*Kp*/, 1.03f/*Ki*/, 0.f/*Kd*/, 40.f/*Diameter of the wheels*/, 75.81f /*Gear Ratio of Motor*/, 12 /*CPR*/);
uint8_t buttonState;

Drive drive(left, right, Lmotor, Rmotor, Lencoder, Rencoder, mpu, 106 /* The distance between the wheels*/, 40 /* The diameter of the wheels*/);


void setup() {
  pinMode(12, INPUT);
  drive.begin();
}

void loop() {
  sensors_event_t a , g, temp;
  mpu.getEvent(&a, &g, &temp);
  buttonState = digitalRead(12);
  bool runState = false;
  if(buttonState == HIGH){
    runState = true;
    Serial.println("button pressed");
  }else{
    runState = false;
  }

  if(runState){
    drive.accel(300,50);
    drive.driveDistance(1000,300);
    delay(5000);
    drive.stop();
    runState = false;
  }
}

