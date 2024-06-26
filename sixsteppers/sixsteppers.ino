// Shows how to run three Steppers at once with varying speeds
//
// Requires the Adafruit_Motorshield v2 library
//   https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library
// And AccelStepper with AFMotor support
//   https://github.com/adafruit/AccelStepper

// This tutorial is for Adafruit Motorshield v2 only!
// Will not work with v1 shields

#include "Arduino_LED_Matrix.h"

#include <stdint.h>

ArduinoLEDMatrix matrix;


#include <AccelStepper.h>
#include <Adafruit_MotorShield.h>

int maxSpeed = 200;
int maxAcc = 50;

Adafruit_MotorShield AFMSbot(0x60); // bottom
Adafruit_MotorShield AFMSmid(0x61); // middle
Adafruit_MotorShield AFMStop(0x62); // top

// Connect two steppers with 200 steps per revolution (1.8 degree)
// to the top shield
Adafruit_StepperMotor *myStepper1 = AFMStop.getStepper(200, 1);
Adafruit_StepperMotor *myStepper2 = AFMStop.getStepper(200, 2);

// Connect two steppers with 200 steps per revolution (1.8 degree)
// to the middle shield
Adafruit_StepperMotor *myStepper3 = AFMSmid.getStepper(200, 1);
Adafruit_StepperMotor *myStepper4 = AFMSmid.getStepper(200, 2);

// Connect two steppers with 200 steps per revolution (1.8 degree)
// to the bottom shield
Adafruit_StepperMotor *myStepper5 = AFMSbot.getStepper(200, 1);
Adafruit_StepperMotor *myStepper6 = AFMSbot.getStepper(200, 2);

// you can change these to DOUBLE or SINGLE or MICROSTEP!
// wrappers for the first motor!
void forwardstep1() {
  myStepper1->onestep(FORWARD, DOUBLE);
}
void backwardstep1() {
  myStepper1->onestep(BACKWARD, DOUBLE);
}
// wrappers for the second motor!
void forwardstep2() {
  myStepper2->onestep(FORWARD, DOUBLE);
}
void backwardstep2() {
  myStepper2->onestep(BACKWARD, DOUBLE);
}
// wrappers for the third motor!
void forwardstep3() {
  myStepper3->onestep(FORWARD, DOUBLE);
}
void backwardstep3() {
  myStepper3->onestep(BACKWARD, DOUBLE);
}
// wrappers for the fourth motor!
void forwardstep4() {
  myStepper4->onestep(FORWARD, DOUBLE);
}
void backwardstep4() {
  myStepper4->onestep(BACKWARD, DOUBLE);
}
// wrappers for the fifth motor!
void forwardstep5() {
  myStepper5->onestep(FORWARD, DOUBLE);
}
void backwardstep5() {
  myStepper5->onestep(BACKWARD, DOUBLE);
}
// wrappers for the sixth motor!
void forwardstep6() {
  myStepper6->onestep(FORWARD, DOUBLE);
}
void backwardstep6() {
  myStepper6->onestep(BACKWARD, DOUBLE);
}

// Now we'll wrap the 6 steppers in an AccelStepper object
AccelStepper stepper1(forwardstep1, backwardstep1);
AccelStepper stepper2(forwardstep2, backwardstep2);
AccelStepper stepper3(forwardstep3, backwardstep3);
AccelStepper stepper4(forwardstep4, backwardstep4);
AccelStepper stepper5(forwardstep5, backwardstep5);
AccelStepper stepper6(forwardstep6, backwardstep6);

void setup()
{
  Serial.begin(9600);
  Serial.println("Setup begun!");

  if(AFMSbot.begin()) {
    Serial.println("Bottom motor found!");
  }
  if(AFMSmid.begin()) {
    Serial.println("Middle motor found!");
  }
  if(AFMStop.begin()) {
    Serial.println("Top motor found!");
  }

   // Start the bottom shield
   // Start the middle shield
   // Start the top shield

  stepper1.setMaxSpeed(maxSpeed);
  stepper1.setAcceleration(maxAcc);
  stepper1.moveTo(1000000);
/*
  stepper2.setMaxSpeed(maxSpeed);
  stepper2.setAcceleration(maxAcc);
  stepper2.moveTo(1000000);

*/
  stepper3.setMaxSpeed(maxSpeed);
  stepper3.setAcceleration(maxAcc);
  stepper3.moveTo(-1000000);

  stepper4.setMaxSpeed(maxSpeed);
  stepper4.setAcceleration(maxAcc);
  stepper4.moveTo(-1000000);

  stepper5.setMaxSpeed(maxSpeed);
  stepper5.setAcceleration(maxAcc);
  stepper5.moveTo(1000000);

  stepper6.setMaxSpeed(maxSpeed);
  stepper6.setAcceleration(maxAcc);
  stepper6.moveTo(1000000);

  matrix.begin();

}

void loop()
{
  /*
  matrix.setCursor(0,0);
  matrix.setTextColor(matrix.color565(0, 255, 0)); 
  matrix.print("Hello, world!");

  matrix.drawLine(0, 0, 5, 5, matrix.color565(255,0,0));
  matrix.drawLine(0, 5, 5, 0, matrix.color565(255,0,0));
  */

    // Change direction at the limits
  if (stepper1.distanceToGo() == 0) {
    stepper1.setCurrentPosition(0);
    stepper1.moveTo(1000000);
  }
  if (stepper2.distanceToGo() == 0) {
    stepper2.setCurrentPosition(0);
    stepper2.moveTo(1000000);
  }
  if (stepper3.distanceToGo() == 0) {
    stepper3.setCurrentPosition(0);
    stepper3.moveTo(1000000);
  }

  if (stepper4.distanceToGo() == 0) {
    stepper4.setCurrentPosition(0);
    stepper4.moveTo(1000000);
  }

  if (stepper5.distanceToGo() == 0) {
    stepper5.setCurrentPosition(0);
    stepper5.moveTo(-1000000);
  }

  if (stepper6.distanceToGo() == 0) {
    stepper6.setCurrentPosition(0);
    stepper6.moveTo(-1000000);
  }

  //stepper1.run();
  //stepper2.run();
  stepper3.run();
  stepper4.run();
  stepper5.run();
  stepper6.run();
}

