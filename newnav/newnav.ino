/*
team Neo Eclipse

written by
bellatheITgirl :D

*/
#include "Wire.h"
#include "SparkFunISL29125.h"

#include <stdint.h>


int combination;
int values[5];
bool seesRed;

#define TCAADDR 0x70

SFE_ISL29125 RGB_sensors[8];

const int front = 4;
const int leftMid = 2;
const int rightMid = 6;
const int leftBack = 5;
const int rightBack = 7;
const int leftPID = 1;
const int rightPID = 0;

unsigned int rgb[3][8];
const int enPin=8;
// front left
const int stepXPin = 2; //X.STEP
const int dirXPin = 5; // X.DIR
// front right
const int stepYPin = 3; //Y.STEP
const int dirYPin = 6; // Y.DIR
// back left
const int stepZPin = 4; //Z.STEP
const int dirZPin = 7; // Z.DIR
// back right
const int stepAPin = 12; // A.STEP
const int dirAPin = 13; //A.DIR

const int stepsPerRev=200;
int pulseWidthMicros = 20; 	// microseconds
int millisBtwnSteps = 2500; // ~30 rpm

int P; // CALIBRATE THIS

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}


// standard Arduino setup()
void setup()
{
  seesRed = false;
    while (!Serial);
    delay(1000);

    Wire.begin();
    
    Serial.begin(115200);

    for(int i = 0; i < 8; i++) {
      tcaselect(i);
      if (RGB_sensors[i].init()) {
        Serial.print("Sensor "); Serial.print(i); Serial.print(" Initialization Successful\n\r");
        Serial.println();
      }
    }
    
    Serial.println("\nTCAScanner ready!");
    
    
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

  pinMode(enPin, OUTPUT);
 	digitalWrite(enPin, LOW);
 	pinMode(stepXPin, OUTPUT);
 	pinMode(dirXPin, OUTPUT);
  pinMode(stepYPin, OUTPUT);
 	pinMode(dirYPin, OUTPUT);
  pinMode(stepZPin, OUTPUT);
 	pinMode(dirZPin, OUTPUT);
  pinMode(stepAPin, OUTPUT);
 	pinMode(dirAPin, OUTPUT);
 	Serial.println(F("CNC Shield Initialized"));
  
  delay(2000);
}

void leftStep(int num) {
  for(int i = 0; i < num; i++) {
    digitalWrite(stepXPin, HIGH);
    delayMicroseconds(pulseWidthMicros);
    digitalWrite(stepXPin, LOW);
    delayMicroseconds(millisBtwnSteps);
          
    digitalWrite(stepZPin, HIGH);
    delayMicroseconds(pulseWidthMicros);
    digitalWrite(stepZPin, LOW);
    delayMicroseconds(millisBtwnSteps);
  }
}

void rightStep(int num) {
  for(int i = 0; i < num; i++) {
    digitalWrite(stepYPin, HIGH);
    delayMicroseconds(pulseWidthMicros);
    digitalWrite(stepYPin, LOW);
    delayMicroseconds(millisBtwnSteps);
  
    digitalWrite(stepAPin, HIGH);
    delayMicroseconds(pulseWidthMicros);
    digitalWrite(stepAPin, LOW);
    delayMicroseconds(millisBtwnSteps);
  }
}

void moveForward(int steps) {
  digitalWrite(dirXPin, HIGH);
  digitalWrite(dirYPin, LOW);
 	digitalWrite(dirZPin, HIGH);
  digitalWrite(dirAPin, LOW); // Enables the motor to move in a particular direction
 	// Makes 200 pulses for making one full cycle rotation
 	for (int i = 0; i < steps; i++) {
 			leftStep(1);
      rightStep(1);
 	}
}

void moveBackward(int steps) {
  digitalWrite(dirXPin, LOW);
  digitalWrite(dirYPin, HIGH);
 	digitalWrite(dirZPin, LOW);
  digitalWrite(dirAPin, HIGH); // Enables the motor to move in a particular direction
 	// Makes 200 pulses for making one full cycle rotation
 	for (int i = 0; i < steps; i++) {
 			leftStep(1);
      rightStep(1);
 	}
}

void scoop() {
  /*
  gone,
  reduced to atoms
  */
  
}

bool isGreen(int sensor) {
  if((rgb[0][sensor] < 15000) && (rgb[1][sensor] > 20000) && (rgb[2][sensor] < 15000)) {
    return true;
  }
  else return false;
}

bool isRed(int sensor) {
  if((rgb[0][sensor] > 15000) && (rgb[1][sensor] < 15000) && (rgb[2][sensor] < 7000)) {
    return true;
    seesRed = true;
  }
  else return false;
}

bool isWhite(int sensor) {
  if((rgb[0][sensor] > 40000) && (rgb[1][sensor] > 40000) && (rgb[2][sensor] > 30000)) {
    return true;
  }
  else return false;
}

bool isBlack(int sensor) {
  if((rgb[0][sensor] < 1000) && (rgb[1][sensor] < 1000) && (rgb[2][sensor] < 500)) {
    return true;
  }
  else return false;
}

int checkValue(int sensor) {
  if(isBlack(sensor)) {
    return 0;
  }
  else if(isWhite(sensor)) {
    return 1;
  }
  else if(isGreen(sensor)) {
    return 2;
  }
  else if(isRed(sensor)) {
    return 3;
  }
}

void convertToInt() {
  values[0] = checkValue(rightBack);
  values[1] = checkValue(leftBack);
  values[2] = checkValue(rightMid);
  values[3] = checkValue(leftMid);
  values[4] = checkValue(front);
}

int takeAve(int sensor1, int sensor2) {
  int r = rgb[0][sensor1] - rgb[0][sensor2];
  int g = rgb[1][sensor1] - rgb[1][sensor2];
  int b = rgb[2][sensor1] - rgb[2][sensor2];
  return (r + b + g) / 3;
}

void truth() { // 1984
  if(values[0] == 0) {
    //front sensor reads black
    if(values[1] == 0) {
      //left mid reads black
      if(values[2] == 0) {
        //right mid reads black
        if((values[3] == 1) && (values[4] == 2)) {
          //left rear reads white
          //right rear reads green
          // BBBWG
          // Right turn
        }
        else if(values[3] == 2) {
          //left rear reads green
          if(values[4] == 1) {
            //right rear reads white
            // BBBGW
            // Left turn
          }
          else if(values[4] == 2) {
            //right rear reads green
            // BBBGG
            // U turn
          }
        }
      }
      else if((values[2] == 1) && (values[3] == 2) && (values[4] == 1)) {
        //right mid reads white
        //left rear reads green
        //right rear reads white
        // BBWGW
        // Left turn
      }
    }
    else if((values[1] == 1) && (values[2] == 0) && (values[3] == 1) && (values[4] == 2)) {
      //left mid reads white
      //right mid reads black
      //left rear reads white
      //right rear reads green
      // BWBWG
      // Right turn
    }
  }
  else if(values[0] == 1) {
    //front sensor reads white
    if(values[1] == 0) {
      //left mid reads black
      if(values[2] == 0) {
        //right mid reads black
        if(values[3] == 1) {
          //left rear reads white
          if(values[4] == 1) {
            //right rear reads white
            // WBBWW
            // Initialization
          }
          else if(values[4] == 2) {
            //right rear reads green
            // WBBWGlinefollowing
            // Right turn
          }
        }
        else if(values[3] == 2) {
          //left rear reads green
          if(values[4] == 1) {
            //right rear reads white
            // WBBGW
            // Left turn
          }
          else if(values[4] == 2) {
            //right rear reads green
            // WBBGG
            // U turn
          }
        }
      }
      else if(values[2] == 1) {
        //right mid reads white
        if((values[3] == 0) && (values[4] == 1)) {
          //left rear reads black
          //right rear reads white
          // WBWBW
          // Shift left
        }
        else if(values[3] == 1) {
          //left rear reads white
          if(values[4] == 0) {
            //right rear reads black
            // WBWWB
            // Turn left (~60 deg)
          }
          else if(values[4] == 1) {
            //right rear reads white
            // WBWWW
            // Check rear sensor
          }
        }
        else if((values[3] == 2) && (values[4] == 1)) {
          //left rear reads green
          //right rear reads white
          // WBWGW
          // Left turn
        }
      }
    }
    else if(values[1] == 1) {
      //left mid reads white
      if(values[2] == 0) {
        //right mid reads black
        if((values[3] == 0) && (values[4] == 1)) {
          //left rear reads black
          //right rear reads white
          // WWBBW
          // Turn right (60 deg)

        }
        else if(values[3] == 1) {
          //left rear reads white
          if(values[4] == 0) {
            //right rear reads black
            // WWBWB
            // Shift right
          }
          else if(values[4] == 1) {
            //right rear reads white
            // WWBWW
            // Check rear sensor
          }
          else if(values[4] == 2) {
            //right rear reads green
            // WWBWG
            // Right turn
          }
        }
      }
      else if((values[2] == 1) && (values[3] == 1) && (values[4] == 1)) { //hope didnt break anything here, swaped %% with &&. compiler error :(
        //right mid reads white
        //left rear reads white
        //right rear reads white
        // WWWWW
        // Straight with a timer

      }
    }
  }
}

void linefollowing() {
  digitalWrite(dirXPin, HIGH);
  digitalWrite(dirYPin, LOW);
  digitalWrite(dirZPin, HIGH);
  digitalWrite(dirAPin, LOW);
  if(isBlack(front)) {
    int aveDif = takeAve(leftPID, rightPID);
    leftStep(5 + (aveDif * P));
    rightStep(5 - (aveDif * P));
  }
  else if(isWhite(front)) {
    if(isBlack(leftPID) && isWhite(rightPID)) {
      leftStep(1);
    }
    else if(isWhite(leftPID) && isBlack(rightPID)) {
      rightStep(1);
    }
    else if(isBlack(leftPID) && isBlack(rightPID)) {
      moveForward(1);
    }
  }
  
  /*
  if(isWhite(leftPID) && isWhite(rightPID)) {
    moveForward(1);
  }
  else if(isWhite(leftPID) && isBlack(rightPID)) {
    digitalWrite(dirXPin, HIGH);
    digitalWrite(dirYPin, LOW);
    digitalWrite(dirZPin, HIGH);
    digitalWrite(dirAPin, LOW); // Enables the motor to move in a particular direction

    digitalWrite(stepYPin, HIGH);
 		delayMicroseconds(pulseWidthMicros);
		digitalWrite(stepYPin, LOW);
		delayMicroseconds(millisBtwnSteps);

    digitalWrite(stepAPin, HIGH);
 		delayMicroseconds(pulseWidthMicros);
		digitalWrite(stepAPin, LOW);
 		delayMicroseconds(millisBtwnSteps);
  }
  else if(isBlack(leftPID) && isWhite(rightPID)) {
    digitalWrite(dirXPin, HIGH);
    digitalWrite(dirYPin, LOW);
    digitalWrite(dirZPin, HIGH);
    digitalWrite(dirAPin, LOW); // Enables the motor to move in a particular direction
      
    digitalWrite(stepXPin, HIGH);
 		delayMicroseconds(pulseWidthMicros);
 		digitalWrite(stepXPin, LOW);
		delayMicroseconds(millisBtwnSteps);
      
    digitalWrite(stepZPin, HIGH);
 		delayMicroseconds(pulseWidthMicros);
		digitalWrite(stepZPin, LOW);
    delayMicroseconds(millisBtwnSteps);
  }
  else if(isBlack(leftPID) && isBlack(rightPID)) {
    moveForward(1);
  }
  */
}

void loop() {

  for(int i = 0; i < 8; i++) { // epic array :D
    tcaselect(i);
    //Serial.print("Sensor "); Serial.print(i); Serial.println(":");
    rgb[0][i] = RGB_sensors[i].readRed();
    rgb[1][i] = RGB_sensors[i].readGreen();
    rgb[2][i] = RGB_sensors[i].readBlue();
    /*
    Serial.print("Red: "); Serial.println(rgb[0][i],DEC);
    Serial.print("Green: "); Serial.println(rgb[1][i],DEC);
    Serial.print("Blue: "); Serial.println(rgb[2][i],DEC);
    */
    //Serial.println();
  }

  Serial.print("Red: "); Serial.print(rgb[0][leftPID],DEC);
  Serial.print(" Green: "); Serial.print(rgb[1][leftPID],DEC);
  Serial.print(" Blue: "); Serial.println(rgb[2][leftPID],DEC);

  Serial.print("Red: "); Serial.print(rgb[0][rightPID],DEC);
  Serial.print(" Green: "); Serial.print(rgb[1][rightPID],DEC);
  Serial.print(" Blue: "); Serial.println(rgb[2][rightPID],DEC);

  Serial.println("---");
  /*

  if(!seesRed) {
    convertToInt();


    delay(2000);
  }
  else {
    digitalWrite(stepXPin, LOW);
    digitalWrite(stepYPin, LOW);
    digitalWrite(stepZPin, LOW);
    digitalWrite(stepAPin, LOW);
    Serial.println("End.");
  }
  */

  linefollowing();
}
