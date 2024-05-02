
#include "Wire.h"
#include "SparkFunISL29125.h"

#include <stdint.h>


#include <AccelStepper.h>
#include <Adafruit_MotorShield.h>

int combination;
int values[5];
bool seesRed;

Adafruit_MotorShield AFMSbot(0x60); // bottom
Adafruit_MotorShield AFMSmid(0x61); // middle
Adafruit_MotorShield AFMStop(0x62); // top

// Connect two steppers with 200 steps per revolution (1.8 degree)
// to the top shield
Adafruit_StepperMotor *top1 = AFMStop.getStepper(200, 1);
Adafruit_StepperMotor *top2 = AFMStop.getStepper(200, 2);

// Connect two steppers with 200 steps per revolution (1.8 degree)
// to the middle shield
Adafruit_StepperMotor *mid1 = AFMSmid.getStepper(200, 1);
Adafruit_StepperMotor *mid2 = AFMSmid.getStepper(200, 2);

// Connect two steppers with 200 steps per revolution (1.8 degree)
// to the bottom shield
Adafruit_StepperMotor *bot1 = AFMSbot.getStepper(200, 1);
Adafruit_StepperMotor *bot2 = AFMSbot.getStepper(200, 2);

// you can change these to DOUBLE or INTERLEAVE or MICROSTEP!
// wrappers for the first motor!
void forwardstep1() {
  top1->onestep(FORWARD, SINGLE);
}
void backwardstep1() {
  top1->onestep(BACKWARD, SINGLE);
}
// wrappers for the second motor!
void forwardstep2() {
  top2->onestep(FORWARD, DOUBLE);
}
void backwardstep2() {
  top2->onestep(BACKWARD, DOUBLE);
}
// wrappers for the third motor!
void forwardstep3() {
  mid1->onestep(FORWARD, INTERLEAVE);
}
void backwardstep3() {
  mid1->onestep(BACKWARD, INTERLEAVE);
}
// wrappers for the fourth motor!
void forwardstep4() {
  mid2->onestep(FORWARD, SINGLE);
}
void backwardstep4() {
  mid2->onestep(BACKWARD, SINGLE);
}
// wrappers for the fifth motor!
void forwardstep5() {
  bot1->onestep(FORWARD, DOUBLE);
}
void backwardstep5() {
  bot1->onestep(BACKWARD, DOUBLE);
}
// wrappers for the sixth motor!
void forwardstep6() {
  bot2->onestep(FORWARD, INTERLEAVE);
}
void backwardstep6() {
  bot2->onestep(BACKWARD, INTERLEAVE);
}

/*
  frontScoop
  backScoop
  leftFront
  rightFront
  leftBack
  rightBack
  */
AccelStepper frontScoop(forwardstep1, backwardstep1);
AccelStepper backScoop(forwardstep2, backwardstep2);
AccelStepper leftFront(forwardstep3, backwardstep3);
AccelStepper leftBack(forwardstep4, backwardstep4);
AccelStepper rightFront(forwardstep5, backwardstep5);
AccelStepper rightBack(forwardstep6, backwardstep6);

#define TCAADDR 0x70

SFE_ISL29125 RGB_sensors[8];

/*
  Front - 4
  Left Middle - 2
  Right Middle - 6
  Left Back - 5
  Right Back - 7
  PID Left - 1
  PID Right - 0
*/

unsigned int rgb[3][8];

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

  if(AFMSbot.begin()) {
    Serial.println("Bottom motor shield found!");
  }
  if(AFMSmid.begin()) {
    Serial.println("Middle motor shield found!");
  }
  if(AFMStop.begin()) {
    Serial.println("Top motor shield found!");
  }

  /*
  stepper1.setMaxSpeed(100.0);
  stepper1.setAcceleration(100.0);
  stepper1.moveTo(24);

  stepper2.setMaxSpeed(200.0);
  stepper2.setAcceleration(100.0);
  stepper2.moveTo(50000);

  stepper3.setMaxSpeed(300.0);
  stepper3.setAcceleration(100.0);
  stepper3.moveTo(1000000);

  stepper4.setMaxSpeed(300.0);
  stepper4.setAcceleration(100.0);
  stepper4.moveTo(1000000);

  stepper5.setMaxSpeed(300.0);
  stepper5.setAcceleration(100.0);
  stepper5.moveTo(1000000);

  stepper6.setMaxSpeed(300.0);
  stepper6.setAcceleration(100.0);
  stepper6.moveTo(1000000);
  */
    
  delay(2000);
}

// one step is 1.8 degrees with a 2 inch radius
// so each step is .06 inches aka 1.6mm * .07 = 1.1mm forward per step

void moveForward(int steps, int speed) {
  leftFront.setMaxSpeed(speed);
  leftFront.setAcceleration(100.0);
  leftFront.moveTo(-steps);
  rightFront.setMaxSpeed(speed);
  rightFront.setAcceleration(100.0);
  rightFront.moveTo(steps);
  leftBack.setMaxSpeed(speed);
  leftBack.setAcceleration(100.0);
  leftBack.moveTo(-steps);
  rightBack.setSpeedsetMaxSpeed(speed);
  rightBack.setAcceleration(100.0);
  rightBack.moveTo(steps);
}

void scoop() {
  frontScoop.setSpeed(100);
  frontScoop.moveTo(72);
  
}

bool isGreen(int sensor) {
  if((rgb[1][sensor] > 200) && (rgb[0][sensor] < 50) && (rgb[2][sensor] < 50)) {
    return true;
  }
  else return false;
}

bool isRed(int sensor) {
  if((rgb[0][sensor] > 200) && (rgb[1][sensor] < 50) && (rgb[2][sensor] < 50)) {
    return true;
    seesRed = true;
  }
  else return false;
}

bool isWhite(int sensor) {
  if((rgb[0][sensor] > 200) && (rgb[1][sensor] > 200) && (rgb[2][sensor] > 200)) {
    return true;
  }
  else return false;
}

bool isBlack(int sensor) {
  if((rgb[0][sensor] < 50) && (rgb[1][sensor] < 50) && (rgb[2][sensor] < 50)) {
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
  values[0] = checkValue(7);
  values[1] = checkValue(5);
  values[2] = checkValue(6);
  values[3] = checkValue(2);
  values[4] = checkValue(4);
}

void truth() {
  if(values[0] == 0) {
    //front sensor reads black
    if(values[1] == 0) {
      //left mid reads black
      if(values[2] == 0) {
        //right mid reads black
        if(values[3] == 1) {
          //left rear reads white
          if(values[4] == 2) {
            //right rear reads green
            // BBBWG
            // Right turn
          }
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
      else if(values[2] == 1) {
        //right mid reads white
        if(values[3] == 2) {
          //left rear reads green
          if(values[4] == 1) {
            //right rear reads white
            // BBWGW
            // Left turn
          }
        }
      }
    }
    else if(values[1] == 1) {
      //left mid reads white
      if(values[2] == 0) {
        //right mid reads black
        if(values[3] == 1) {
          //left rear reads white
          if(values[4] == 2) {
            //right rear reads green
            // BWBWG
            // Right turn
          }
        }
      }
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
            // WBBWG
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
        if(values[3] == 0) {
          //left rear reads black
          if(values[4] == 1) {
            //right rear reads white
            // WBWBW
            // Shift left
          }
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
        else if(values[3] == 2) {
          //left rear reads green
          if(values[4] == 1) {
            //right rear reads white
            // WBWGW
            // Left turn
          }
        }
      }
    }
    else if(values[1] == 1) {
      //left mid reads white
      if(values[2] == 0) {
        //right mid reads black
        if(values[3] == 0) {
          //left rear reads black
          if(values[4] == 1) {
            //right rear reads white
            // WWBBW
            // Turn right (60 deg)
          }
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
      else if(values[2] == 1) {
        //right mid reads white
        if(values[3] == 1) {
          //left rear reads white
          if(values[4] == 1) {
            //right rear reads white
            // WWWWW
            // Straight with a timer
          }
        }
      }
    }
  }
}

void loop() {

  /*
   if (stepper1.distanceToGo() == 0) {
      stepper1.moveTo(-stepper1.currentPosition());
    }

    if (stepper2.distanceToGo() == 0) {
      stepper2.moveTo(-stepper2.currentPosition());
    }
    if (stepper3.distanceToGo() == 0) {
      stepper3.moveTo(-stepper3.currentPosition());
    }

  if (stepper4.distanceToGo() == 0) {
    stepper4.moveTo(-stepper4.currentPosition());
  }

  if (stepper5.distanceToGo() == 0) {
    stepper5.moveTo(-stepper5.currentPosition());
  }

  if (stepper6.distanceToGo() == 0) {
    stepper6.moveTo(-stepper6.currentPosition());
  }

  stepper1.run();
  stepper2.run();
  stepper3.run();
  stepper4.run();
  stepper5.run();
  stepper6.run();
  */

  for(int i = 0; i < 8; i++) {
    tcaselect(i);
    Serial.print("Sensor "); Serial.print(i); Serial.println(":");
    rgb[0][i] = RGB_sensors[i].readRed();
    rgb[1][i] = RGB_sensors[i].readGreen();
    rgb[2][i] = RGB_sensors[i].readBlue();
    Serial.print("Red: "); Serial.println(rgb[0][i],HEX);
    Serial.print("Green: "); Serial.println(rgb[1][i],HEX);
    Serial.print("Blue: "); Serial.println(rgb[2][i],HEX);
    Serial.println();
  }
  /*

  if(!seesRed) {
    convertToInt();


    delay(2000);
  }
  else {
  }
  */

  delay(2000);
}
