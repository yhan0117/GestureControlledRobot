#include <Wire.h>
#include "MPU.h"
#include <avr/sleep.h>
//#include "ArduinoLowPower.h"
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// MPU(accelerometer range [g], gyroscope range [degree/s])
MPU myMPU(8, 1000);
//RF24(chip enable, chip select not)
RF24 myRadio(7, 8);

unsigned long timePrev;
unsigned long timeOut;
unsigned long timeSample;
unsigned long timeLoop;


const byte intPin = 2;
const byte LED = 3;
const byte address[6] = "00001";

static float prevAngles[3];
static float curAngles[3];

struct data {
  float speed;
  char dir;
};

struct data Data;

void setup() {
  // Initialize communicaton
  Serial.begin(19200);
  while (!Serial)
    ;

  Wire.begin();  // Initialize MCU I2C

  myMPU.begin();  // Initialize IMU I2C

  myRadio.begin();  // Initialize nRF SPI

  // Pin setup
  pinMode(intPin, INPUT_PULLUP);
  pinMode(LED, OUTPUT);

  // Calibrate to normalized data based on error at home orientation
  myMPU.calibrate();

  // nRF config and setup
  myRadio.openWritingPipe(address[0]);     // 00001
  //myRadio.openReadingPipe(1, addresses[1]);  // 00002
  myRadio.setPALevel(RF24_PA_MAX);
  myRadio.stopListening();

  // Variable initialization
  timePrev = millis();
  timeOut = millis();
  timeSample = millis();
  timeLoop = millis();
  Data.dir = ' ';
  Data.speed = 0;

  // Blinks the built in L LED to indicate the end of initialization
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED, 0);
    delay(100);
    digitalWrite(LED, 1);
    delay(100);
  }
}

void loop() {
  int elapsedTime = millis() - timePrev;
  timePrev = millis();
  myMPU.getAngle(elapsedTime);

  curAngles[0] = myMPU.roll();
  curAngles[1] = myMPU.pitch();
  curAngles[2] = myMPU.yaw();

  // If changes in position is significant in the past 50ms
  // Then reset timeout
  for (int i = 0; i < 2; i++) {
    if (prevAngles[i] - curAngles[i] > 10 || prevAngles[i] - curAngles[i] < -10) {
      timeOut = millis();
    }
  }

  // Sample the angles every 50ms, based on movements taking 0.5s on average
  // If sampling f is to high, then change in angle would be minimal such that noise dominates
  // If sampling f is to low, then user might have returned to previous position, thus causing no change to be registered
  if (millis() - timeSample > 50) {
    for (int i = 0; i < 3; i++) {
      prevAngles[i] = curAngles[i];
    }
    timeSample = millis();
  }

  // If device has been idle, put to sleep
  if (millis() - timeOut > 30000)
    Sleep();

  //delay for data to be recieved
  while (millis() - timeLoop < 10)
    ;
  // Read the user hand position
  position();

  // Read speed
  int potVal = analogRead(0);
  Data.speed = potVal * 5.0 / 712.0 + 3.1;
  Serial.print(Data.speed);
  Serial.print("\t");
}

void Boot() {
  // Avoid interruption of normal loop
  detachInterrupt(0);
}

void Sleep() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // sleep mode is set here
  sleep_enable();                       // enables the sleep bit in the mcucr register
  delay(200);
  Serial.println("---System timeout---");
  Serial.println("---Device Sleeping---");
  delay(200);
  // Dim LED for indication
  for (int i = 200; i > 0; i--) {
    float DC = i / 200.0;
    digitalWrite(LED, 1);
    delayMicroseconds(floor(2000 * DC));
    digitalWrite(LED, 0);
    delayMicroseconds(floor(2000 * (1 - DC)));
  }
  myMPU.sleep();
  attachInterrupt(0, Boot, LOW);
  // LowPower.attachInterruptWakeup(0, Boot, FALLING);  // Set interrupt 0 to boot device
  // LowPower.idle();
  // Once interrupt is closed
  delay(500);
  sleep_mode();  // here the device is actually put to sleep...!!

  Serial.println("---Device Starting---");
  // Brighten LED for indication
  for (int i = 0; i < 1000; i++) {
    float DC = i / 1000.0;
    digitalWrite(LED, 0);
    delayMicroseconds(floor(2000 * (1 - DC)));
    digitalWrite(LED, 1);
    delayMicroseconds(floor(2000 * DC));
  }
  sleep_disable();  // Disables the sleep bit in the MCU Control register
  myMPU.wake();

  // Reset timeout
  timeOut = millis();
}

void position() {

  // Categorize into vertical and horizontal gestures
  // Horizontal controls rotation of ground robot
  // Vertical controls translation of ground robot
  // Exact values are taken emperically

  char pos = 'a';
  if (myMPU.roll() < 20) {
    pos = horizontal();
  }
  if (20 < myMPU.roll()) {
    pos = vertical();
  }

  Serial.println(pos);
  Data.dir = pos;
  myRadio.write(&Data, sizeof(Data));
}

char horizontal() {
  if (myMPU.pitch() > 20) {
    return 'b';  //forward
  } else if (myMPU.pitch() > -20) {
    return 'z';  //zero position
  } else {
    return 'f';  //backward
  }
}

char vertical() {
  if (myMPU.pitch() > 20) {
    return 'l';  //left
  } else if (myMPU.pitch() > -45) {
    return 'z';  //zero position
  } else {
    return 'r';  //right
  }
}