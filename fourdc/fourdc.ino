#include "Wire.h"
#include "SparkFunISL29125.h"
#include <stdint.h>

#define TCAADDR 0x70

bool seesRed;
int P; // CALIBRATE THIS
int speed;

int values[5];

SFE_ISL29125 RGB_sensors[8];

const int front = 4;
const int leftMid = 2;
const int rightMid = 6;
const int leftBack = 5;
const int rightBack = 7;
const int leftPID = 1;
const int rightPID = 0;

unsigned int rgb[3][8];

// right controller

const int enAR = 46;
const int enBR = 44;
const int in1R = 45;
const int in2R = 47;
const int in3R = 49;
const int in4R = 51;

// left controller

const int enAL = 2;
const int enBL = 3;
const int in1L = 35;
const int in2L = 37;
const int in3L = 39;
const int in4L = 41;

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void checkMulti() {
  for (uint8_t t=0; t<8; t++) {
    tcaselect(t);
    Serial.print("TCA Port #"); Serial.println(t);

    for (uint8_t addr = 0; addr<=127; addr++) {
      if (addr == TCAADDR) continue;
        Wire.beginTransmission(addr);
        if (!Wire.endTransmission()) {
          Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
        }
      }
    }
  Serial.println("\ndone");
}

void getRGBs() {
  for(int i = 0; i < 8; i++) { // epic array :D
    tcaselect(i);
    //Serial.print("Sensor "); Serial.print(i); Serial.println(":");
    rgb[0][i] = RGB_sensors[i].readRed();
    rgb[1][i] = RGB_sensors[i].readGreen();
    rgb[2][i] = RGB_sensors[i].readBlue();
  }
}

void leftDrive(int vel) {
  analogWrite(enAL, vel);
  analogWrite(enBL, vel);
  digitalWrite(in1L, LOW);
  digitalWrite(in2L, HIGH);
  digitalWrite(in3L, LOW);
  digitalWrite(in4L, HIGH);
}

void rightDrive(int vel) {
  analogWrite(enAR, vel);
  analogWrite(enBR, vel);
  digitalWrite(in1R, LOW);
  digitalWrite(in2R, HIGH);
  digitalWrite(in3R, LOW);
  digitalWrite(in4R, HIGH);
}

void stop() {
  digitalWrite(in1R, LOW);
  digitalWrite(in2R, LOW);
  digitalWrite(in3R, LOW);
  digitalWrite(in4R, LOW);
  digitalWrite(in1L, LOW);
  digitalWrite(in2L, LOW);
  digitalWrite(in3L, LOW);
  digitalWrite(in4L, LOW);
}

int checkColor(int sensor) {
  if((rgb[0][sensor] > 40000) && (rgb[1][sensor] > 40000) && (rgb[2][sensor] > 30000)) {
    return 6; // white
  }
  else if((rgb[0][sensor] < 1000) && (rgb[1][sensor] < 1000) && (rgb[2][sensor] < 500)) {
    return 1; // black
  }
  else if((rgb[0][sensor] < 15000) && (rgb[1][sensor] > 20000) && (rgb[2][sensor] < 15000)) {
    return 3; // green
  }
  else if((rgb[0][sensor] > 15000) && (rgb[1][sensor] < 15000) && (rgb[2][sensor] < 7000)) {
    return 5; // red
    seesRed = true;
  }
  else return 0;
}

int takeAve(int sensor1, int sensor2) {
  int r = rgb[0][sensor1] - rgb[0][sensor2];
  int g = rgb[1][sensor1] - rgb[1][sensor2];
  int b = rgb[2][sensor1] - rgb[2][sensor2];
  return (r + b + g) / 3;
}

void linefollowing() {
  if(checkColor(front) == 1) {
    int aveDif = takeAve(leftPID, rightPID);
    leftDrive(speed + (aveDif * P));
    rightDrive(speed - (aveDif * P));
  }
  else if(checkColor(front) == 6) {
    if((checkColor(leftPID) == 1) && (checkColor(rightPID) == 6)) {
      leftDrive(speed);
      rightDrive(speed / 2);
    }
    else if((checkColor(leftPID) == 6) && (checkColor(rightPID) == 1)) {
      leftDrive(speed / 2);
      rightDrive(speed);
    }
    else if((checkColor(leftPID) == 1) && (checkColor(rightPID) == 1)) {
      leftDrive(speed);
      rightDrive(speed);
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  seesRed = false;
  Wire.begin();

  pinMode(enAR, OUTPUT);
	pinMode(enBR, OUTPUT);
	pinMode(in1R, OUTPUT);
	pinMode(in2R, OUTPUT);
	pinMode(in3R, OUTPUT);
	pinMode(in4R, OUTPUT);

  pinMode(enAL, OUTPUT);
	pinMode(enBL, OUTPUT);
	pinMode(in1L, OUTPUT);
	pinMode(in2L, OUTPUT);
	pinMode(in3L, OUTPUT);
	pinMode(in4L, OUTPUT);

  stop();

  for(int i = 0; i < 8; i++) {
    tcaselect(i);
    if (RGB_sensors[i].init()) {
      Serial.print("Sensor "); Serial.print(i); Serial.print(" Initialization Successful\n\r");
      Serial.println();
    }
  }
  Serial.println("\nTCAScanner ready!");


  Serial.begin(9600);
  Serial.println("Yippee!! Let's go!!");
}

void loop() {
  // put your main code here, to run repeatedly:
  getRGBs();

  leftDrive(75);
  rightDrive(75);
  /*  
  for(int i = 25; i < 256; i++) {
    leftDrive(i);
    rightDrive(i);
    delay(100);
  }
  delay(2000);

  for(int i = 25; i < 256; i++) {
    leftDrive(-i);
    rightDrive(-i);
    delay(100);
  }
  delay(2000);
  */
}
