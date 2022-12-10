#include "Arduino.h"
#include "Robot.h"

/******************************************************************************************************
  I2C communication with IMU (MPU6050)
  Register addresses for the IMU are specified below (refer to Adafruit_MPU6050.h or MPU6050 datasheet)
******************************************************************************************************/

// If no potentiometer is attached, use default speed
#ifndef potPin
#define SPEED 3
#endif

#define SWITCH 2

enum LR {
  L = 0,
  R = 1,
};


/************************************************************
---------------------Constructor-----------------------------
************************************************************/
Robot::Robot(int P1, int P2, int P3, int P4, int P5, int P6, int P7, int P8) {
  pos[0] = 0;
  pos[1] = 0;


  int tmp1[4] = { P1, P4, P2, P3 };
  memcpy(&pins[L][0], &tmp1, sizeof(tmp1));
  int tmp2[4] = { P1, P3, P2, P4 };
  memcpy(&pins[L][1], &tmp2, sizeof(tmp2));
  int tmp3[4] = { P5, P8, P6, P7 };
  memcpy(&pins[R][0], &tmp3, sizeof(tmp3));
  int tmp4[4] = { P5, P7, P6, P8 };
  memcpy(&pins[R][1], &tmp4, sizeof(tmp4));

  for (int i = 0; i < 10; i++) {
    dist[i] = 0;
  }
}


/*********************************************************************
---------------------Robot Initialization-----------------------------
*********************************************************************/
void Robot::begin() {

  //Prompt user to place robot at origin
  Serial.println("---Place robot at origin---");
  delay(2000);

  //Turn on motor switch (PMOS at pin 2)
  digitalWrite(SWITCH, HIGH);
  
  //configurate stepper pins
  for (int i = 0; i < 4; i++) {
    pinMode(pins[L][0][i], OUTPUT);
    pinMode(pins[R][0][i], OUTPUT);
  }

  Serial.println("---Robot starting---");
}


/******************************************************
---------------------Motor-----------------------------
******************************************************/
void Robot::stepper(int dir, int side, float speed) {
  int i, j;
  int tmp = speed * 1000;
  for (i = 0; i < 4; i++) {
    digitalWrite(this->pins[side][dir][i], HIGH);
    for (j = 0; j < 4; j++)
      if (i != j)
        digitalWrite(this->pins[side][dir][j], LOW);
    delayMicroseconds(tmp);
  }
}


/***************************************************************
---------------------Robot Movement-----------------------------
***************************************************************/
void Robot::Translate(int dir, int speed) {
  this->stepper(dir, 0, speed);
  this->stepper(!dir, 1, speed);
}

void Robot::Translate(int dir) {
  this->stepper(dir, 0, SPEED);
  this->stepper(!dir, 1, SPEED);
}

void Robot::Rotate(int dir, int speed) {
  this->stepper(dir, 0, speed);
  this->stepper(dir, 1, speed);
}

void Robot::Rotate(int dir) {
  this->stepper(dir, 0, SPEED);
  this->stepper(dir, 1, SPEED);
}


/*******************************************************************************
---------------------Register Surrounding Obstacles-----------------------------
*******************************************************************************/
int Robot::getDist(int pin) {
  int k[12]
  int i, tmp(0);

  // Take the moving average of sensor data to reduce noise
  for (i = 0; i < 11; i++) {
    tmp += k[i];
  }

  k[i] = analogRead(pin);
  return (tmp + k[i]) / 12;
}
