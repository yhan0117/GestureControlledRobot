#ifndef Robot_h
#define Robot_h
#include "Arduino.h"

class Robot {
public:

  // Constructors
  Robot(int, int, int, int, int, int, int, int);

  // Functions called in main sketch
  void begin();
  void Translate(int, int);
  void Translate(int);
  void Rotate(int, int);
  void Rotate(int);
  int getDist(int);

private:

  // Functions / variables used within calss
  float pos[2];
  int pins[2][2][4];  //side, dir, pins
  int dist[10];
  void stepper(int, int, float);
};
#endif