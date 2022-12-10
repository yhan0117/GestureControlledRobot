#ifndef MPU_h
#define MPU_h
#include "Arduino.h"

class MPU {
public:

  // Constructors
  MPU(int, int);
  MPU(MPU&);

  // Functions called in main sketch
  void begin();
  void calibrate();
  float pitch();
  float roll();
  float yaw();
  void getAngle(int);
  void sleep();
  void wake();

private:

  // Functions / variables used within calss
  float Angle[2][3];
  float Error[2][3];

  uint8_t range[2];
  float coef[2];

  float* getGyro();
  float* getAcc();
};
#endif