#include "Robot.h"
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 myRadio(10, 4);  // CE, CSN
const byte address[][6] = { "00001", "00002" };
const byte sensorPin = 0;
Robot myRobot(24, 25, 51, 50, 22, 23, 53, 52);

struct data {
  float speed;
  char dir;
};

struct data Data;

void setup() {
  Serial.begin(9600);
  myRadio.begin();
  myRadio.openWritingPipe(address[1]);   // 00002
  myRadio.openReadingPipe(0, address[0]);  // 00001
  myRadio.setPALevel(RF24_PA_MAX);

  myRobot.begin();
  myRadio.startListening();
}

void loop() {

  if (myRadio.available()) {
    while (myRadio.available()) {
      myRadio.read(&Data, sizeof(data));
      Serial.print(String(Data.dir) + "\t");
      Serial.print(String(Data.speed) + "\t");
      Serial.println(myRobot.getDist(sensorPin));
      if (Data.dir == 'l')
        myRobot.Rotate(1, Data.speed);
      else if (Data.dir == 'r')
        myRobot.Rotate(0, Data.speed);
      else if (Data.dir == 'f')
        myRobot.Translate(1, Data.speed);
      else if (Data.dir == 'b')
        myRobot.Translate(0, Data.speed);
      else if (Data.dir == 'z')
        ;
    }
  }
}