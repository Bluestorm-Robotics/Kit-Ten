#include "Wire.h"
#include "SparkFunISL29125.h"
#include "TimerOne.h"
#include <stdint.h>
unsigned int counterLeft = 0;
unsigned int counterRight = 0;

#define TCAADDR 0x70

#define trigPin 13
#define echoPin 12

#define BLACK 1
#define WHITE 6

bool seesRed;
bool seesSilver;
int P = 0.5;  // CALIBRATE THIS
int Kp_speedInv =20;   // Proportional feedback gain for speed sensor feedback (Inversed to stay as an integer, the SMALLER the value, the HIGHER the gain)
int Ki_speedInv = 20;  // Integral feedback gain for speed sensor feedback (Inversed to stay as an integer, the SMALLER the value, the HIGHER the gain)
int Kd_speedInv = 10;   // Derivative feedback gain for speed sensor feedback (Inversed to stay as an integer, the SMALLER the value, the HIGHER the gain)

int errLeft = 0;   // error from the left wheels
int errRight = 0;  // error from the right wheels

int errPreviousLeft = 0;   // previous error from the left wheels
int errPreviousRight = 0;  // previous error from the right wheels

int err_I_Left = 0;   // error for integral feed1back control for the left wheels
int err_I_Right = 0;  // error for integral feedback control for the right wheels

int err_D_Left = 0;   // error for Derivative feedback control for the left wheels
int err_D_Right = 0;  // error for Derivative feedback control for the right wheels

int P_ValueLeft = 0;   // P control left wheels
int P_ValueRight = 0;  // P control right wheels
int I_ValueLeft = 0;   // I control left wheels
int I_ValueRight = 0;  // I control right wheels
int D_ValueLeft = 0;   // D control left wheels
int D_ValueRight = 0;  // D control right wheels

int speedMax = 150;  // maximum allowed wheel speed command
int speedMin = 50;  // minimum allowed wheel speed command

int speedTargetNominal = 75;  // rpm, target speed before adjustment by RGB sensor for line tracking

int speedTargetLeft = 65;   // rpm, target speed for left wheels, adjusted by RGB sensor for line tracking
int speedTargetRight = 65;  // rpm, target speed for right wheels adjusted by RGB sensor for line tracking

int speedFeedbackLeft = 0; // rpm, feedback value for left wheels
int speedFeedbackRight = 0; // rpm, feedback value for right wheels

int speedCommandLeft = 50;   // rpm, command speed for left wheels, adjusted speed sensor feedback
int speedCommandRight = 50;  // rpm, command speed for right wheels, adjusted speed sensor feedback

//int speed; //depricated
int delayMs = 50;


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
const int leftMid = 6;
const int rightMid = 2;
const int leftBack = 7;
const int rightBack = 5;
const int leftPID = 0;
const int rightPID = 1;
const int back = 3;

unsigned int rgb[3][8];

/*const int encoder = 4;
const int encoder2 = 3;*/

// right controller

const int enAR = 46;
const int enBR = 44;
const int in1R = 45;
const int in2R = 47;
const int in3R = 49;
const int in4R = 51;

// left controller

const int enAL = 4;
const int enBL = 5;
const int in1L = 35;
const int in2L = 37;
const int in3L = 39;
const int in4L = 41;


void docount()  // counts from the speed sensor
{
  counterLeft++;  // increase +1 the counterLeft value
}

void docount2()  // counts from the speed sensor
{
  counterRight++;  // increase +1 the counterRight value
}
void timerIsr() {
  Timer1.detachInterrupt();  //stop the timer
  //Serial.print("Motor Speed: ");
  int rotationLeft = (counterLeft * 5 * 3) / 7.5;  // divide by number of holes in Disc (20), divide by period (0.2 sec), then multiply by 60 to get RPM
  // 7.5 is a fudge factor to account for the difference between demand and the actual measurement wiht the tt motor
  if (speedTargetLeft < 0) rotationLeft = -rotationLeft;
  int rotationRight = counterRight * 5 * 3 / 7.5;
  if (speedTargetRight < 0) rotationRight = -rotationRight;
  //Serial.print(rotationLeft, DEC);
  //Serial.println(" RPM and");
  //Serial.print(rotationRight, DEC);
  //Serial.println(" RPM");
  counterLeft = 0;
  counterRight = 0;                      //  reset counter to zero
  Timer1.attachInterrupt(timerIsr);  //enable the timer
  // below is the code for speed tracking inside timer ISP

  errPreviousLeft = errLeft;    // save the error value from the left wheel
  errPreviousRight = errRight;  // save the error value from the right wheel

  errLeft = speedTargetLeft - rotationLeft;     // update error for proportional feedback control for the left wheels
  errRight = speedTargetRight - rotationRight;  // update error for proportional feedback control for the right wheels

  if (speedCommandLeft < speedMax && speedCommandLeft > speedMin) { // integrator anti-windup
    err_I_Left = err_I_Left + errLeft;
    //Serial.print("err_I_Left: ");
    //Serial.print(err_I_Left, DEC);
    //Serial.println(" RPM");                              // error accumulation for integral feedback control for the left wheels
    //Serial.print("speedCommandLeft: ");
    //Serial.print(speedCommandLeft, DEC);
    //Serial.println(" RPM");    }
  }
  if (speedCommandRight < speedMax && speedCommandRight > speedMin)  // integrator anti-windup
    err_I_Right = err_I_Right + errRight;                             // error accumulation for integral feedback control for the right wheels

  err_D_Left = errLeft - errPreviousLeft;     // error calculation for derivation feedback control for the left wheels
  err_D_Right = errRight - errPreviousRight;  // error calculation for derivation feedback control for the right wheels

  P_ValueLeft = 0; //errLeft / Kp_speedInv;       // Proportional feedback value for speed sensor feedback (Inversed gain to stay as an integer)
  P_ValueRight = 0; //errRight / Kp_speedInv;     // Proportional feedback value for speed sensor feedback (Inversed gain to stay as an integer)
  I_ValueLeft = 0; //err_I_Left / Ki_speedInv;    // Integral feedback value for speed sensor feedback (Inversed gain to stay as an integer)
  I_ValueRight = 0; //err_I_Right / Ki_speedInv;  // Integral feedback value for speed sensor feedback (Inversed gain to stay as an integer)
  D_ValueLeft = 0; //err_D_Left / Kd_speedInv;    // Derivative feedback value for speed sensor feedback (Inversed gain to stay as an integer)
  D_ValueRight = 0; //err_D_Right / Kd_speedInv;  // Derivative feedback value for speed sensor feedback (Inversed gain to stay as an integer)

  speedFeedbackLeft = P_ValueLeft + I_ValueLeft + D_ValueLeft;      // feedback control for the left wheels
  speedFeedbackRight = P_ValueRight + I_ValueRight + D_ValueRight;  // feedback control for the right wheels

  /*Serial.print("P_ValueLeft: ");
  Serial.print(P_ValueLeft, DEC);
  Serial.println(" RPM");

  Serial.print("P_ValueRight: ");
  Serial.print(P_ValueRight, DEC);
  Serial.println(" RPM");

  Serial.print("I_ValueLeft: ");
  Serial.print(I_ValueLeft, DEC);
  Serial.println(" RPM");

  Serial.print("I_ValueRight: ");
  Serial.print(I_ValueRight, DEC);
  Serial.println(" RPM");

  Serial.print("D_ValueLeft: ");
  Serial.print(D_ValueLeft, DEC);
  Serial.println(" RPM");

  /*Serial.print("D_ValueRight: ");
  Serial.print(D_ValueRight, DEC);
  Serial.println(" RPM");

  Serial.print("speedFeedbackLeft: ");
  Serial.print(speedFeedbackLeft, DEC);
  Serial.println(" RPM");

  Serial.print("rotationLeft: ");
  Serial.print(rotationLeft, DEC);
  Serial.println(" RPM");*/

  /*Serial.print("speedFeedbackRight: ");
  Serial.print(speedFeedbackRight, DEC);
  Serial.println(" RPM");

  Serial.print("speedFeedbackRight: ");
  Serial.print(speedFeedbackRight, DEC);
  Serial.println(" RPM");*/

  // end of timer ISR
}

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
  Serial.print("Front: ");
  Serial.print(checkColor(front));
  Serial.print(" LeftPID: ");
  Serial.print(checkColor(leftPID));
  Serial.print(" RightPID: ");
  Serial.print(checkColor(rightPID));
  Serial.print(" LeftMid: ");
  Serial.print(checkColor(leftMid));
  Serial.print(" RightMid: ");
  Serial.print(checkColor(rightMid));
  Serial.print(" LeftBack: ");
  Serial.print(checkColor(leftBack));
  Serial.print(" RightBack: ");
  Serial.print(checkColor(rightBack));
  Serial.print(" Back: ");
  Serial.print(checkColor(back));
  Serial.println();
}

void printVals() {
  /*Serial.print("Left: ");
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
  Serial.println(rgb[3][rightPID]);/*
  Serial.print("front: ");
  Serial.print(rgb[0][front]);
  Serial.print(" ");
  Serial.print(rgb[1][front]);
  Serial.print(" ");
  Serial.println(rgb[3][front]);*/
  Serial.print("leftMid: ");
  Serial.print(rgb[0][leftMid]);
  Serial.print(" ");
  Serial.print(rgb[1][leftMid]);
  Serial.print(" ");
  Serial.println(rgb[3][leftMid]);
  /*Serial.println("Left: ");
  Serial.print(checkColor(leftPID));
  Serial.println("right: ");
  Serial.print(checkColor(rightPID));
  Serial.println("---------");*/
  delay(700);
}

void leftFwd(int vel) {
  if (vel < 0) {
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
  if (vel < 0) {
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
    return WHITE; // white
  }
  else if((rgb[0][sensor] < 1000) && (rgb[1][sensor] < 1000) && (rgb[2][sensor] < 500)) {
    return BLACK; // black
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
  if(sensor == leftPID){
    if ((rgb[0][leftPID] > 1300) && (rgb[1][leftPID] > 6000)) {
      return WHITE;  // white
    } else if ((rgb[0][leftPID] < 1300 ) && (rgb[1][leftPID] < 2500)) {
      return BLACK;  // black
    } else if ((rgb[0][leftPID] < 1500) && (rgb[1][leftPID] > 1500)) {
      return 3;  // green
    } else if ((rgb[0][leftPID] > 1500) && (rgb[1][leftPID] < 1500)) {
      return 5;  // red
      seesRed = true;
    }
  }
  else if(sensor == rightPID){
    if ((rgb[0][rightPID] > 2000) && (rgb[1][rightPID] > 4000)) {
      return WHITE;  // white
    } else if ((rgb[0][rightPID] < 2000 ) && (rgb[1][rightPID] < 4000)) {
      return BLACK;  // black
    } else if ((rgb[0][rightPID] < 1500) && (rgb[1][rightPID] > 1500)) {
      return 3;  // green
    } else if ((rgb[0][rightPID] > 1500) && (rgb[1][rightPID] < 1500)) {
      return 5;  // red
      seesRed = true;
    }
    else if(sensor == rightMid){
      if ((rgb[0][rightMid] > 2000) && (rgb[1][rightMid] > 4000)) {
        return WHITE;  // white
      } else if ((rgb[0][rightMid] < 2000 ) && (rgb[1][rightMid] < 4000)) {
        return BLACK;  // black
      } else if ((rgb[0][rightMid] < 1500) && (rgb[1][rightMid] > 1500)) {
        return 3;  // green
      } else if ((rgb[0][rightMid] > 1500) && (rgb[1][rightMid] < 1500)) {
        return 5;  // red
        seesRed = true;
      }
    }
    else if(sensor == leftMid){
      if ((rgb[0][leftMid] > 2000) && (rgb[1][leftMid] > 4000)) {
        return WHITE;  // white
      } else if ((rgb[0][leftMid] < 2000 ) && (rgb[1][leftMid] < 4000)) {
        return BLACK;  // black
      } else if ((rgb[0][leftMid] < 1500) && (rgb[1][leftMid] > 1500)) {
        return 3;  // green
      } else if ((rgb[0][leftMid] > 1500) && (rgb[1][leftMid] < 1500)) {
        return 5;  // red
        seesRed = true;
      }
    }
  }
    
  /*
  else if((rgb[0][sensor] > 15000) && (rgb[1][sensor] < 15000) && (rgb[2][sensor] < 7000)) { //RECODE TO SILVER
    return 7; //silver
    seesSilver = !seesSilver;
  }
  */
  else {
    return 0;
  }
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
  rightFwd(speedCommandRight);
  leftBkd(speedCommandLeft);
  delay((1000 / 75) * speedTargetLeft);
  stop();
}

void rightTurn() {
  leftFwd(speedCommandLeft);
  rightFwd(speedCommandRight);
  delay((1000 / 75) * speedTargetRight);
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
/*
  if ((checkColor(leftPID) == WHITE) && (checkColor(rightPID) == BLACK)) {
   speedTargetLeft = 0;   
   speedTargetRight = speedTargetNominal;  
   speedCommandLeft = speedTargetLeft + speedFeedbackLeft;     // PID control for the left wheels based on speed sensor feedback
   speedCommandRight = speedTargetRight + speedFeedbackRight;  // PID control for the right wheels based on speed sensor feedback
    leftFwd(speedCommandLeft);
    rightFwd(speedCommandRight);
 //   delay(delayMs);
 //   stop();
 //   delay(delayMs);
    Serial.print("left ");
  } else 
*/

if ((checkColor(leftPID) != BLACK) && (checkColor(rightPID) == BLACK)) {
   speedTargetLeft = 0;   
   speedTargetRight = speedTargetNominal;  
   speedCommandLeft = speedTargetLeft + speedFeedbackLeft;     // PID control for the left wheels based on speed sensor feedback
   speedCommandRight = speedTargetRight + speedFeedbackRight;  // PID control for the right wheels based on speed sensor feedback
  if (speedCommandLeft < 0) speedCommandLeft = 0;
  if (speedCommandRight < 0) speedCommandRight = 0;
  leftFwd(speedTargetLeft+50);
  rightBkd(speedTargetRight+50);
  delay(delayMs);
  stop();
  delay(delayMs);
  Serial.println("right"); 
} 
/*
else if ((checkColor(leftPID) != WHITE) && (checkColor(rightPID) == BLACK)) {
   speedTargetLeft = 0;   
   speedTargetRight = speedTargetNominal;  
   speedCommandLeft = speedTargetLeft + speedFeedbackLeft;     // PID control for the left wheels based on speed sensor feedback
   speedCommandRight = speedTargetRight + speedFeedbackRight;  // PID control for the right wheels based on speed sensor feedback
    leftFwd(speedCommandLeft);
    rightFwd(speedCommandRight);
 //   delay(delayMs);
 //   stop();
 //   delay(delayMs);
    Serial.print("left ");
  } 

else if ((checkColor(leftPID) == BLACK) && (checkColor(rightPID) == WHITE)) {
    leftFwd(speedCommandLeft);
    rightFwd(speedCommandRight);
    delay(delayMs);
    stop();
    delay(delayMs);
    Serial.print("right ");
  } 

*/

else if ((checkColor(leftPID) != WHITE) && (checkColor(rightPID) == WHITE)) {
   speedTargetLeft = speedTargetNominal;   
   speedTargetRight = 0;  
   speedCommandLeft = speedTargetLeft + speedFeedbackLeft;     // PID control for the left wheels based on speed sensor feedback
   speedCommandRight = speedTargetRight + speedFeedbackRight;  // PID control for the right wheels based on speed sensor feedback
  if (speedCommandLeft < 0) speedCommandLeft = 0;
  if (speedCommandRight < 0) speedCommandRight = 0;
  leftBkd(speedTargetLeft+50);
  rightFwd(speedTargetRight+50);
  delay(delayMs);
  stop();
  delay(delayMs);
  Serial.println("left");
 
}

/*
else if ((checkColor(leftPID) != BLACK) && (checkColor(rightPID) == BLACK)) {
    leftFwd(speedCommandLeft);
    rightFwd(speedCommandRight);
    delay(delayMs);
    stop();
    delay(delayMs);
    Serial.print("right ");
  } 
*/

else if ((checkColor(leftPID) == WHITE) && (checkColor(rightPID) == WHITE)) {
   speedTargetLeft = speedTargetNominal;   
   speedTargetRight = speedTargetNominal;  
   speedCommandLeft = speedTargetLeft + speedFeedbackLeft;     // PID control for the left wheels based on speed sensor feedback
   speedCommandRight = speedTargetRight + speedFeedbackRight;  // PID control for the right wheels based on speed sensor feedback
  if (speedCommandLeft < 0) speedCommandLeft = 0;
  if (speedCommandRight < 0) speedCommandRight = 0;
  if((checkColor(leftMid) == BLACK) && (checkColor(rightMid) != BLACK)) {
    leftBkd(speedTargetLeft+50);
    rightFwd(speedTargetRight+50);
    delay(delayMs);
    stop();
    delay(delayMs);
    Serial.println("left");
  }
  else if((checkColor(leftMid) != BLACK) && (checkColor(rightMid) == BLACK)) {
    leftFwd(speedTargetLeft+50);
    rightBkd(speedTargetRight+50);
    delay(delayMs);
    stop();
    delay(delayMs);
    Serial.println("right");
  }
  else {
    leftFwd(speedTargetLeft+50);
    rightFwd(speedTargetRight+50);
    delay(delayMs);
    stop();
    delay(delayMs);
    Serial.println("all white ");
  }
  } 

/*
else if ((checkColor(leftPID) == 1) && (checkColor(rightPID) == 1)) {
    leftFwd(speedCommandLeft);
    rightFwd(speedCommandRight);
    delay(delayMs);
    stop();
    delay(delayMs);
    Serial.print("straight ");
  }
*/
  else { // leftPID != WHITE && rightPID != WHITE
    speedTargetLeft = speedTargetNominal;   
    speedTargetRight = speedTargetNominal;  
    speedCommandLeft = speedTargetLeft + speedFeedbackLeft;     // PID control for the left wheels based on speed sensor feedback
    speedCommandRight = speedTargetRight + speedFeedbackRight;  // PID control for the right wheels based on speed sensor feedback
    if (speedCommandLeft < 0) speedCommandLeft = 0;
    if (speedCommandRight < 0) speedCommandRight = 0;
    leftFwd(speedTargetLeft);
    rightFwd(speedTargetRight);
    delay(delayMs);
    stop();
    delay(delayMs);
    Serial.println("both non-white ");
  }
}

/*void linefollowing() {
  //Serial.println("line");
  int aveDif = takeAve(leftPID, rightPID);
  //Serial.println(aveDif);
  leftFwd(speed + (aveDif * P));
  rightFwd(speed - (aveDif * P));
  delay(delayMs);
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

}*/

void setup() {
  // put your setup code here, to run once:
  Timer1.initialize(200000);           // set timer for 0.2sec
  attachInterrupt(0, docount, RISING);  // “0” – sensor connector to pin 2 on Arduino. increase counter when speed sensor pin goes High
  attachInterrupt(1, docount2, RISING);
  Timer1.attachInterrupt(timerIsr);  // enable the timer
  seesRed = false;
  distance = 0;
// counter = 0;
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
  


  if (speedCommandLeft > speedMax) speedCommandLeft = speedMax;      // stay within speed limit
  if (speedCommandLeft < -speedMax) speedCommandLeft = -speedMax;    // stay within speed limit
  if (speedCommandRight > speedMax) speedCommandRight = speedMax;    // stay within speed limit
  if (speedCommandRight < -speedMax) speedCommandRight = -speedMax;  // stay within speed limit
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
    else if(Color(leftPID) == 1) {
      stop();
    }
    */
    //simple();
    printVals();
    /*if (!truth()) {
      //linefollowing();
      //simple();
    }*/
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

