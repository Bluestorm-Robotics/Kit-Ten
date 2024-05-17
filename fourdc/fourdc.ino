#include "Wire.h"
#include "SparkFunISL29125.h"
#include <stdint.h>

#define TCAADDR 0x70

#define trigPin 13
#define echoPin 12

bool seesRed;
bool seesSilver;
int P = 0.5;  // CALIBRATE THIS
int speed = 75;

int counter;

int values[5];

float distance;

SFE_ISL29125 RGB_sensors[8];

/*
  PCB Wiring Colors
    White - 5V
    Purple - SDA
    Black - SCL
    Green - Ground
    Blue - 3.3V
*/

const int front = 4;
const int leftMid = 2;
const int rightMid = 6;
const int leftBack = 5;
const int rightBack = 7;
const int leftPID = 1;
const int rightPID = 0;
const int back = 3;

unsigned int rgb[3][8];

const int encoder = 4;
const int encoder2 = 5;

// right controller

const int enAR = 46;
const int enBR = 44;
const int in1R = 45;
const int in2R = 47;
const int in3R = 49;
const int in4R = 51;

// left controller

const int enAL = 4;
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
  for (uint8_t t = 0; t < 8; t++) {
    tcaselect(t);
    Serial.print("TCA Port #");
    Serial.println(t);

    for (uint8_t addr = 0; addr <= 127; addr++) {
      if (addr == TCAADDR) continue;
      Wire.beginTransmission(addr);
      if (!Wire.endTransmission()) {
        Serial.print("Found I2C 0x");
        Serial.println(addr, HEX);
      }
    }
  }
  Serial.println("\ndone");
}

void getRGBs() {
  for (int i = 0; i < 8; i++) {  // epic array :D
    tcaselect(i);
    //Serial.print("Sensor "); Serial.print(i); Serial.println(":");
    rgb[0][i] = RGB_sensors[i].readRed();
    rgb[1][i] = RGB_sensors[i].readGreen();
    rgb[2][i] = RGB_sensors[i].readBlue();
  }
}

void printNums() {
  Serial.print("Front: ");Serial.print(checkColor(front));
  Serial.print(" LeftPID: ");Serial.print(checkColor(leftPID)); 
  Serial.print(" RightPID: ");Serial.print(checkColor(rightPID)); 
  Serial.print(" LeftMid: ");Serial.print(checkColor(leftMid)); 
  Serial.print(" RightMid: ");Serial.print(checkColor(rightMid)); 
  Serial.print(" LeftBack: ");Serial.print(checkColor(leftBack)); 
  Serial.print(" RightBack: ");Serial.print(checkColor(rightBack)); 
  Serial.print(" Back: ");Serial.print(checkColor(back)); 
  Serial.println();
}

void printVals() {
  Serial.print("Left: ");
  Serial.print(rgb[0][leftPID]);
  Serial.print(" ");
  Serial.print(rgb[1][leftPID]);
  Serial.print(" ");
  Serial.println(rgb[3][leftPID]);
  Serial.print("Right: ");
  Serial.print(rgb[0][rightPID]);
  Serial.print(" ");
  Serial.print(rgb[1][rightPID]);
  Serial.print(" ");
  Serial.println(rgb[3][rightPID]);
}

void leftFwd(int vel) {
  if (vel > 0) {
    leftBkd(abs(vel));
  } else {
    analogWrite(enAL, vel);
    analogWrite(enBL, vel);
    digitalWrite(in1L, LOW);
    digitalWrite(in2L, HIGH);
    digitalWrite(in3L, LOW);
    digitalWrite(in4L, HIGH);
  }
}

void rightFwd(int vel) {
  if (vel < 0) {
    rightBkd(abs(vel));
  } else {
    analogWrite(enAR, vel);
    analogWrite(enBR, vel);
    digitalWrite(in1R, LOW);
    digitalWrite(in2R, HIGH);
    digitalWrite(in3R, LOW);
    digitalWrite(in4R, HIGH);
  }
}

void leftBkd(int vel) {
  if (vel < 0) {
    leftFwd(abs(vel));
  } else {
    analogWrite(enAL, vel);
    analogWrite(enBL, vel);
    digitalWrite(in1L, HIGH);
    digitalWrite(in2L, LOW);
    digitalWrite(in3L, HIGH);
    digitalWrite(in4L, LOW);
  }
}

void rightBkd(int vel) {
  if (vel > 0) {
    rightFwd(abs(vel));
  } else {
    analogWrite(enAR, vel);
    analogWrite(enBR, vel);
    digitalWrite(in1R, HIGH);
    digitalWrite(in2R, LOW);
    digitalWrite(in3R, HIGH);
    digitalWrite(in4R, LOW);
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

/*
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
  else if((rgb[0][sensor] > 15000) && (rgb[1][sensor] < 15000) && (rgb[2][sensor] < 7000)) { //RECODE TO SILVER
    return 7; //silver
    seesSilver = !seesSilver;
  }
  else return 0;
}*/
int checkColor(int sensor) {
  if ((rgb[0][sensor] > 1400) && (rgb[1][sensor] > 1500)) {
    return 6;  // white
  } else if ((rgb[0][sensor] < 200) && (rgb[1][sensor] < 200)) {
    return 1;  // black
  } else if ((rgb[0][sensor] < 1500) && (rgb[1][sensor] > 1500)) {
    return 3;  // green
  } else if ((rgb[0][sensor] > 1500) && (rgb[1][sensor] < 1500)) {
    return 5;  // red
    seesRed = true;
  }
  /*
  else if((rgb[0][sensor] > 15000) && (rgb[1][sensor] < 15000) && (rgb[2][sensor] < 7000)) { //RECODE TO SILVER
    return 7; //silver
    seesSilver = !seesSilver;
  }
  */
  else
    return 0;
}


int takeAve(int sensor1, int sensor2) {
  int r = rgb[0][sensor1] - rgb[0][sensor2];
  int g = rgb[1][sensor1] - rgb[1][sensor2];
  int b = rgb[2][sensor1] - rgb[2][sensor2];
  return (r + b + g) / 3;
}

//RETURNS THE DISTANCE MEASURED BY THE HC-SR04 DISTANCE SENSOR
float getDistance() {
  float echoTime;            //variable to store the time it takes for a ping to bounce off an object
  float calculatedDistance;  //variable to store the distance calculated from the echo time

  //send out an ultrasonic pulse that's 10ms long
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  echoTime = pulseIn(echoPin, HIGH);  //use the pulsein command to see how long it takes for the pulse to bounce back to the sensor

  calculatedDistance = echoTime / 148.0;  //calculate the distance of the object that reflected the pulse (half the bounce time multiplied by the speed of sound)

  return calculatedDistance;  //send back the distance that was calculated
}

void leftTurn() {
  rightFwd(speed);
  leftBkd(speed);
  delay((1000 / 75) * speed);
  stop();
}

void rightTurn() {
  leftFwd(speed);
  rightBkd(speed);
  delay((1000 / 75) * speed);
  stop();
}

void convertToInt() {
  values[0] = checkColor(front);
  values[1] = checkColor(leftMid);
  values[2] = checkColor(rightMid);
  values[3] = checkColor(leftBack);
  values[4] = checkColor(rightBack);
}

bool truth() {  // 1984
  //Serial.println("truth");
  if (values[0] == 1) {
    //front sensor reads black
    if (values[1] == 1) {
      //left mid reads black
      if (values[2] == 1) {
        //right mid reads black
        if ((values[3] == 6) && (values[4] == 3)) {
          //left rear reads white
          //right rear reads green
          // BBBWG
          rightTurn();
          return true;
        } else if (values[3] == 3) {
          //left rear reads green
          if (values[4] == 6) {
            //right rear reads white
            // BBBGW
            leftTurn();
            return true;
          } else if (values[4] == 3) {
            //right rear reads green
            // BBBGG
            rightTurn();
            rightTurn();
            return true;
          }
        }
      } else if ((values[2] == 6) && (values[3] == 3) && (values[4] == 6)) {
        //right mid reads white
        //left rear reads green
        //right rear reads white
        // BBWGW
        leftTurn();
        return true;
      }
    } else if ((values[1] == 6) && (values[2] == 1) && (values[3] == 6) && (values[4] == 3)) {
      //left mid reads white
      //right mid reads black
      //left rear reads white
      //right rear reads green
      // BWBWG
      rightTurn();
      return true;
    }
  } else if (values[0] == 6) {
    //front sensor reads white
    if (values[1] == 1) {
      //left mid reads black
      if (values[2] == 1) {
        //right mid reads black
        if (values[3] == 6) {
          //left rear reads white
          if (values[4] == 6) {
            //right rear reads white
            // WBBWW
            // Initialization
            return true;
          } else if (values[4] == 3) {
            //right rear reads green
            // WBBWG
            rightTurn();
            return true;
          }
        } else if (values[3] == 3) {
          //left rear reads green
          if (values[4] == 6) {
            //right rear reads white
            // WBBGW
            leftTurn();
            return true;
          } else if (values[4] == 3) {
            //right rear reads green
            // WBBGG
            rightTurn();
            rightTurn();
            return true;
          }
        }
      } else if (values[2] == 6) {
        //right mid reads white
        if ((values[3] == 1) && (values[4] == 6)) {
          //left rear reads black
          //right rear reads white
          // WBWBW
          // Shift left
          return true;
        } else if (values[3] == 6) {
          //left rear reads white
          if (values[4] == 1) {
            //right rear reads black
            // WBWWB
            // Turn left (~60 deg)
            return true;
          } else if (values[4] == 6) {
            //right rear reads white
            // WBWWW
            // Check rear sensor
            return true;
          }
        } else if ((values[3] == 3) && (values[4] == 6)) {
          //left rear reads green
          //right rear reads white
          // WBWGW
          leftTurn();
          return true;
        }
      }
    } else if (values[1] == 6) {
      //left mid reads white
      if (values[2] == 1) {
        //right mid reads black
        if ((values[3] == 1) && (values[4] == 6)) {
          //left rear reads black
          //right rear reads white
          // WWBBW
          // Turn right (60 deg)
          return true;
        } else if (values[3] == 6) {
          //left rear reads white
          if (values[4] == 1) {
            //right rear reads black
            // WWBWB
            // Shift right
            return true;
          } else if (values[4] == 6) {
            //right rear reads white
            // WWBWW
            // Check rear sensor
            return true;
          } else if (values[4] == 3) {
            //right rear reads green
            // WWBWG
            rightTurn();
            return true;
          }
        }
      } else if ((values[2] == 6) && (values[3] == 6) && (values[4] == 6)) {  //hope didnt break anything here, swaped %% with &&. compiler error :(
        //right mid reads white
        //left rear reads white
        //right rear reads white
        // WWWWW
        // Straight with a timer
        return true;
      }
    }
  }
  return false;
}

void simple() {
  if((checkColor(leftPID) == 6) && (checkColor(rightPID) == 1)) {
    leftFwd(speed);
    rightBkd(speed);
    delay(50);
    stop();
    delay(50);
  }
  else if((checkColor(leftPID) == 1) && (checkColor(rightPID) == 6)) {
    leftBkd(speed);
    rightFwd(speed);
    delay(50);
    stop();
    delay(50);
  }
  else if((checkColor(leftPID) == 6) && (checkColor(rightPID) == 6)) {
    leftFwd(speed);
    rightFwd(speed);
    delay(50);
    stop();
    delay(50);
  }
  else if((checkColor(leftPID) == 1) && (checkColor(rightPID) == 1)) {
    leftFwd(speed);
    rightFwd(speed);
    delay(50);
    stop();
    delay(50);
  }
}

void linefollowing() {
  //Serial.println("line");
  int aveDif = takeAve(leftPID, rightPID);
  //Serial.println(aveDif);
  leftFwd(speed + (aveDif * P));
  rightFwd(speed - (aveDif * P));
  delay(50);
  stop();
  delay(10);
  /*
  if(checkColor(front) == 1) {
    int aveDif = takeAve(leftPID, rightPID);
    leftFwd(speed + (aveDif * P));
    rightFwd(speed - (aveDif * P));
  }
  else if(checkColor(front) == 6) {
    if((checkColor(leftPID) == 1) && (checkColor(rightPID) == 6)) {
      leftFwd(speed);
      rightFwd(speed / 2);
    }
    else if((checkColor(leftPID) == 6) && (checkColor(rightPID) == 1)) {
      leftFwd(speed / 2);
      rightFwd(speed);
    }
    else if((checkColor(leftPID) == 1) && (checkColor(rightPID) == 1)) {
      leftFwd(speed);
      rightFwd(speed);
    }
  }
  */
}

void setup() {
  // put your setup code here, to run once:
  seesRed = false;
  distance = 0;
  counter = 0;
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

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  stop();

  for (int i = 0; i < 8; i++) {
    tcaselect(i);
    if (RGB_sensors[i].init()) {
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.print(" Initialization Successful\n\r");
      Serial.println();
    }
  }
  Serial.println("\nTCAScanner ready!");


  Serial.begin(9600);
  Serial.println("Yippee!! Let's go!!");
}

void loop() {
  // put your main code here, to run repeatedly:
  distance = getDistance();
  //Serial.println(distance);
  /*
  Serial.print("Left: ");
  Serial.print(rgb[0][leftPID]);
  Serial.print(" ");
  Serial.print(rgb[1][leftPID]);
  Serial.print(" ");
  Serial.println(rgb[3][leftPID]);
  Serial.print("Right: ");
  Serial.print(rgb[0][rightPID]);
  Serial.print(" ");
  Serial.print(rgb[1][rightPID]);
  Serial.print(" ");
  Serial.println(rgb[3][rightPID]);
  */
  getRGBs();
  checkColor(front);
  //Serial.print(rgb[0][front]); Serial.print(" "); Serial.print(rgb[1][front]); Serial.print(" "); Serial.println(rgb[3][front]);
  //Serial.println(checkColor(front));
  //Serial.print(checkColor(leftPID));Serial.print(" "); Serial.println(checkColor(rightPID));
  
  

  if (seesRed) {
    stop();
  } else if (distance < 10) {
    //go around
    stop();
  } else {
    // testing color detection
    /*
    if(checkColor(leftPID) == 6) {
      leftFwd(speed);
      rightFwd(speed);
      delay(100);
      stop();
      delay(10);
    }
    else if(checkColor(leftPID) == 1) {
      stop();
    }
    */
    printVals();
    if (!truth()) {
      //linefollowing();
      simple();
    }
  }

  //delay(100);

  //leftBkd(speed);
  //rightBkd(speed);

  //counter++;
  //delay(10);

  /*  
  for(int i = 25; i < 256; i++) {
    leftFwd(i);
    rightFwd(i);
    delay(100);
  }
  delay(2000);

  for(int i = 25; i < 256; i++) {
    leftBkd(i);
    rightBkd(i);
    delay(100);
  }
  delay(2000);
  */
}
