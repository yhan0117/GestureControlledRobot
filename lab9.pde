//import processing.serial.*; // add the serial library
//Serial myPort; // define a serial port object to monitor


float w = 0;
long time;
float dir = 0;
int x = 0, a = 1;

void setup() {
  size(1500, 900); // set the window size
  background(125); // 125 gray level

  //myPort = new Serial(this, Serial.list()[1], 9600); // define input port
  //myPort.clear(); // clear the port of any initial junk
  time = millis();
}

void draw() {

  //while (myPort.available () > 0) { // make sure port is open
  //String inString = myPort.readStringUntil('\n'); // read input string


  //if (inString != null) { // ignore null strings
  //inString = trim(inString); // trim off any whitespace
  //String[] data = splitTokens(inString, "\t"); // extract x & y into an array

  //if (data.length == 3) {

  //char direction = char(data[0]);
  //float speed = float(data[1]);
  //int dist = int(data[2]);


  if (millis() - time > 10) {
    w += PI/60;//speed;
    x+=a;
    if (x > 40 || x < 0)
      a = -a;
    time = millis();
  }

  /*if (direction == 'f') {
   dir = 1;
   } else if (direction == 'z') {
   dir = 0;
   } else if (direction == 'b') {
   dir = -1;
   }*/
  background(125); // 125 gray level

  drawRobot(0);//dir/speed*10.0);
  drawObstacle(2*x);

  //}
  //}
}
void drawRobot(float dir) {

  fill(255, 255, 255);
  noStroke();
  ellipse(600, 650+dir, 105, 200);
  ellipse(900, 650+dir, 105, 200);

  //draw wheel frame
  fill(100, 100, 100);
  arc(900, 650+dir, 60, 145, -PI/3 + w, w);
  arc(900, 650+dir, 60, 145, -PI + w, -2*PI/3 + w);
  arc(900, 650+dir, 60, 145, PI/3 + w, 2*PI/3 + w);

  arc(600, 650+dir, 60, 145, -PI/3 - w, -w);
  arc(600, 650+dir, 60, 145, -PI - w, -2*PI/3 - w);
  arc(600, 650+dir, 60, 145, PI/3 - w, 2*PI/3 - w);

  fill(255, 255, 255);
  ellipse(900, 650+dir, 45, 113);
  ellipse(600, 650+dir, 45, 113);


  fill(50, 205, 135);
  quad(620, 650+dir, 870, 650+dir, 860, 475+dir, 630, 475+dir);
  fill(30, 165, 75);

  quad(860, 475+dir, 630, 475+dir, 650, 430+dir, 840, 430+dir);

  fill(255, 255, 255);
  rectMode(CENTER);
  rect(687, 420, 120, 100);
}

void drawObstacle(int dist) {
  rectMode(CENTER);
  fill(120, 150, 255, 180-dist*2);
  quad(600, 200 - dist, 900, 200 - dist, 880, 150 - dist, 620, 150 - dist);
  fill(80, 80, 215, 180-dist*2);

  rect(750, 300 - dist, 300, 200);
}
