#include "Wire.h"
#include "SparkFunISL29125.h"
#include <stdint.h>

#define TCAADDR 0x70

SFE_ISL29125 RGB_sensors[8];
unsigned int mins[3][8];
unsigned int maxes[3][8];

int counter;

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

void leftDrive(int vel) {
  if (vel < 0) {
    analogWrite(enAL, abs(vel));
    analogWrite(enBL, abs(vel));
    digitalWrite(in1L, HIGH);
    digitalWrite(in2L, LOW);
    digitalWrite(in3L, HIGH);
    digitalWrite(in4L, LOW);
  } else {
    analogWrite(enAL, vel);
    analogWrite(enBL, vel);
    digitalWrite(in1L, LOW);
    digitalWrite(in2L, HIGH);
    digitalWrite(in3L, LOW);
    digitalWrite(in4L, HIGH);
  }
}

void rightDrive(int vel) {
  if (vel < 0) {
    analogWrite(enAR, abs(vel));
    analogWrite(enBR, abs(vel));
    digitalWrite(in1R, HIGH);
    digitalWrite(in2R, LOW);
    digitalWrite(in3R, HIGH);
    digitalWrite(in4R, LOW);
  } else {
    analogWrite(enAR, vel);
    analogWrite(enBR, vel);
    digitalWrite(in1R, LOW);
    digitalWrite(in2R, HIGH);
    digitalWrite(in3R, LOW);
    digitalWrite(in4R, HIGH);
  }
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

void circle() {
  for (int i = 0; i < 8; i++) {
    unsigned int red = RGB_sensors[i].readRed();
    if (red > maxes[0][i]) {
      maxes[0][i] = red;
    }
    if (red < mins[0][i]) {
      mins[0][i] = red;
    }
    unsigned int green = RGB_sensors[i].readGreen();
    if (green > maxes[1][i]) {
      maxes[1][i] = green;
    }
    if (green < mins[1][i]) {
      mins[1][i] = green;
    }

    unsigned int blue = RGB_sensors[i].readBlue();
    if (blue > maxes[2][i]) {
      maxes[2][i] = blue;
    }
    if (blue < mins[2][i]) {
      mins[2][i] = blue;
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600);

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

  for (int i = 0; i < 8; i++) {
    tcaselect(i);
    if (RGB_sensors[i].init()) {
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.print(" Initialization Successful\n\r");
      Serial.println();
    }

    mins[0][i] = 65535;
    maxes[0][i] = -1;

    mins[1][i] = 65535;
    maxes[1][i] = -1;

    mins[2][i] = 65535;
    maxes[2][i] = -1;
  }

  counter = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  counter++;
  if (counter < 50) {
    leftDrive(75);
    rightDrive(-75);
    delay(100);
    stop();
    circle();
    delay(100);
  }
  else if(counter == 50) {
    for(int i = 0; i < 8; i++) {
      Serial.print("Sensor "); Serial.println(i);
      for(int j = 0; i < 3; j++) {
        Serial.print(mins[j][i]); Serial.print(" ");
      }
      Serial.println();
      for(int j = 0; i < 3; j++) {
        Serial.print(maxes[j][i]); Serial.print(" ");
      }
      Serial.println();
    }
  }
  else {
    stop();
  }
}
