#include "Arduino.h"
#include "MPU.h"
#include <Wire.h>
#include <math.h>

/******************************************************************************************************
  I2C communication with IMU (MPU6050)
  Register addresses for the IMU are specified below (refer to Adafruit_MPU6050.h or MPU6050 datasheet)
******************************************************************************************************/

#define ID 0x68           //MPU6050 register, returned from device ID register 0x75
#define ACEL 0x3B         //accelerometer data status register
#define GYRO 0x43         //gyroscope data status register
#define PWR 0x6B          //power/sleep control register
#define ACEL_CONFIG 0x1C  //accelerometer configuration register
#define GYRO_CONFIG 0x1B  //gyroscope configuration register
#define DLPF_CONFIG 0x1A  //digital low pass filter configuration register
#define INT_CONFIG 0x37   //interrupt configuration register
#define INT_ENABLE 0x38   //interrupt enable configuration register
#define INT_STATUS 0x3A   //interrupt status register
#define MOT_THR 0x1F      //motion detection threshold configuration register
#define MOT_DUR 0x20      //duration counter threshold for motion interrupt
#define MOT_CONFIG 0x69   //I2C Master control register

enum xyz {
  X = 0,
  Y = 1,
  Z = 2,
};

enum range {          // 4 settings encoded in 2 bits (bit 3 and 4 from data sheet)
  G2 = 0b01 << 3,     // +/- 2g 0x01 for enable 5Hz high pass filter
  G4 = 0b01 << 3,     // +/- 4g
  G8 = 0b10 << 3,     // +/- 8g
  G16 = 0b11 << 3,    // +/- 16g
  D250 = 0b00 << 3,   // +/- 250 deg/s
  D500 = 0b01 << 3,   // +/- 500 deg/s
  D1000 = 0b10 << 3,  // +/- 1000 deg/s
  D2000 = 0b11 << 3,  // +/- 2000 deg/s
};


/******************************************************************************************************************
---------------------Constructor based on input sensitivities or copies another object-----------------------------
******************************************************************************************************************/
MPU::MPU(int acel, int gyro) {

  switch (acel) {
    case (2):
      coef[0] = 16384.0;
      range[0] = G2;
      break;
    case (4):
      coef[0] = 8192.0;
      range[0] = G4;
      break;
    case (8):
      coef[0] = 4096.0;
      range[0] = G8;
      break;
    case (16):
      coef[0] = 2048.0;
      range[0] = G16;
      break;
  }

  switch (gyro) {
    case (250):
      coef[1] = 131;
      range[0] = D250;
      break;
    case (500):
      coef[1] = 65.5;
      range[0] = D500;
      break;
    case (1000):
      coef[1] = 32.8;
      range[0] = D1000;
      break;
    case (2000):
      coef[1] = 16.4;
      range[0] = D2000;
      break;
  }

  for (int i = X; i <= Z; i++) {
    Error[0][i] = 0;
    Error[1][i] = 0;
    Angle[0][i] = 0;
    Angle[1][i] = 0;
  }
}

MPU::MPU(MPU& obj) {
  for (int i = 0; i < 2; i++) {
    range[i] = obj.range[i];
    coef[i] = obj.coef[i];
  }

  for (int i = X; i <= Z; i++) {
    Error[0][i] = obj.Error[0][i];
    Error[1][i] = obj.Error[1][i];
    Angle[0][i] = 0;
    Angle[1][i] = 0;
  }
}


/*********************************************************************
---------------------Data I/O through I2C-----------------------------
*********************************************************************/
void write(uint8_t device, uint8_t reg, uint8_t data) {  // Device address, register address, data
  Wire.beginTransmission(device);                        // Start communication with IMU / device
  Wire.write(reg);                                       // Talk to the target register
  Wire.write(data);                                      // Send data through SDA
  Wire.endTransmission();
}

void read(uint8_t device, uint8_t reg, int num) {  // Device address, register address, num of bytes requesting
  Wire.beginTransmission(device);                  // Start communication with IMU / device
  Wire.write(reg);                                 // Talk to the target register
  Wire.endTransmission(false);                     // Reset the SDA line
  Wire.requestFrom(ID, (uint8_t)num, true);        // Request n bytes after which reading is available and transmission ends
}


/***********************************************************************************
---------------------MPU initialization and calibration-----------------------------
***********************************************************************************/
void MPU::begin() {
  //Power up device and reset
  write(ID, PWR, 0x00);
  write(ID, ID, 0x07);

  //Sensor config based on input sensitivities
  write(ID, ACEL_CONFIG, this->range[0]);
  write(ID, GYRO_CONFIG, this->range[1]);

  //Configurate the DLPF bandwidth to 5kHz to minimize noise in exchange of sampling rate
  //Sampling rate of 1kHz is used instead of the default 8kHz
  write(ID, DLPF_CONFIG, 0x06);

  //Interrupts
  //Configurate ISR to active low, push-pull, latched, and clears interrupt flag only by reading INT_STATUS
  write(ID, INT_CONFIG, 0xA0);
  //Enable motion detection interrupt from slave to master (bit 6)
  write(ID, INT_ENABLE, 0x40);

  //Motion detection
  //Set motion threshold
  write(ID, MOT_THR, 25);
  //Set motion detect duration
  write(ID, MOT_DUR, 20);
  //Configurate motion detect
  write(ID, MOT_CONFIG, 0x15);

  delay(20);

  Serial.println("---MPU6050 Setup Complete---");
}

void MPU::calibrate() {  //set device to home position and obtain steady state error at that config
  float* tmp;
  for (int i = 0; i < 100; i++) {
    tmp = this->getAcc();
    this->Error[0][X] += (atan(tmp[Y] / sqrt(pow(tmp[X], 2) + pow(tmp[Z], 2))) * 180.0 / PI);
    this->Error[0][Y] += (atan(-1 * tmp[X] / sqrt(pow(tmp[Y], 2) + pow(tmp[Z], 2))) * 180.0 / PI);
  }

  for (int i = 0; i < 200; i++) {
    tmp = this->getGyro();
    for (int j = X; j <= Z; j++) {

      this->Error[1][j] += tmp[j];
    }
  }
  for (int i = X; i <= Z; i++) {
    this->Error[0][i] /= 100.0;
    this->Error[1][i] /= 200.0;
  }
}


/**********************************************************************
---------------------Device sleep and wake-----------------------------
**********************************************************************/
void MPU::sleep() {
  // Set accelerometer HPF to enable interrupt
  write(ID, ACEL_CONFIG, 0x01);
}

void MPU::wake() {
  // Reset accelerometer based on input sensitivity
  write(ID, ACEL_CONFIG, this->range[0]);

  // Clear INT_STATUS buffer in IMU
  read(0x68, 0x3A, 1);
  Wire.read();
  read(0x68, 0x37, 1);
  Wire.read();
}


/***********************************************************************************************************
---------------------Obtain and decode gyroscope, accelerometer, and angle data-----------------------------
***********************************************************************************************************/
float* MPU::getGyro() {
  static float gyy[3];  //static array to reserve data at memory location outside of function
  read(ID, GYRO, 6);
  // Read 6 registers, each axis value is stored in 2 registers
  // Divide input data by a constant based on the specified range / sensitivity, according to datasheet
  // Two's complements are taken
  for (int i = X; i <= Z; i++) {
    gyy[i] = ~((Wire.read() << 8 | Wire.read()) & ~(uint8_t)1) / this->coef[1];
  }

  return gyy;
}

float* MPU::getAcc() {
  static float acc[3];
  read(ID, ACEL, 6);
  // Read 6 registers, each axis value is stored in 2 registers
  // Divide input data by a constant based on the specified range / sensitivity, according to datasheet
  // Two's complements are taken
  for (int i = X; i <= Z; i++) {
    acc[i] = ~((Wire.read() << 8 | Wire.read()) & ~(uint8_t)1) / this->coef[0];
  }

  return acc;
}

void MPU::getAngle(int time) {
  float* tmp;
  tmp = this->getAcc();
  // Calculate orientation based on accel data
  this->Angle[0][X] = atan(tmp[Y] / sqrt(pow(tmp[X], 2) + pow(tmp[Z], 2))) * 180.0 / PI - this->Error[0][X];
  this->Angle[0][Y] = atan(-1 * tmp[X] / sqrt(pow(tmp[Y], 2) + pow(tmp[Z], 2))) * 180.0 / PI - this->Error[0][Y];

  tmp = this->getGyro();
  // Calculate orientation by intergrating gyro data
  this->Angle[1][X] += (tmp[X] - this->Error[1][X]) * time / 1000.0;  // deg/s * s = deg
  this->Angle[1][Y] += (tmp[Y] - this->Error[1][Y]) * time / 1000.0;
  this->Angle[1][Z] += (tmp[Z] - this->Error[1][Z]) * time / 1000.0;

  // Mitigate drifting of the gyroscope
  this->Angle[1][X] = this->roll();
  this->Angle[1][Y] = this->pitch();
  this->Angle[1][Z] = this->yaw();
}


/********************************************************************
---------------------Output Euler Angles-----------------------------
********************************************************************/
float MPU::roll() {
  return 0.85 * this->Angle[0][X] + 0.15 * this->Angle[0][X];
  // The weight of gyro and accel data are emperically adjusted to optimize this filter
  // -90 90 -> -95 95
}

float MPU::pitch() {
  return 0.85 * this->Angle[0][Y] + 0.15 * this->Angle[0][Y];
  // -90 90 -> -95 95
}

float MPU::yaw() {
  return this->Angle[1][Z];
}