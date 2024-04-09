#include "vex.h"

using namespace vex;

//-----------
const int uTurnSpeed = 500;
const int pointTurnSpeed = 200;
const int defaultSpeed = 5;
const float turnSpeed = 10;
const float lineWidthCM = 4.0;
const float wheelDiameterCM = 7.4;
const float robotDiameterCM = 22.0;
const int wallDistanceCM = 8;
const float searchTime = 2;

//motors make robot go vrooommm 
motor motorB = motor(PORT1, ratio18_1, false);
motor motorC = motor(PORT2, ratio18_1, false);

void resetMotors(){
    motorB.resetRotation();
    motorC.resetRotation();
}

void moveForward(int speed = defaultSpeed){
    motorB.spin(forward, speed, velocityUnits::pct);
    motorC.spin(forward, speed, velocityUnits::pct);
}

void stop(){
    resetMotors(); 
    motorB.stop();
    motorC.stop(); 
}

float convertCMToDegrees(float distanceInCM){
    return distanceInCM / (PI * wheelDiameterCM) * 360.0;
}

void moveCM(float distanceInCM){ 
    float degrees = convertCMToDegrees(distanceInCM);
    resetMotors(); 
    motorB.startRotateFor(degrees, rotationUnits::deg, defaultSpeed, velocityUnits::pct);
    motorC.startRotateFor(degrees, rotationUnits::deg, defaultSpeed, velocityUnits::pct);
    waitUntilMotorStop(motorC);
}

void turnDegrees(float degrees){
    resetMotors(); 
    motorB.startRotateFor(degrees, rotationUnits::deg, turnSpeed, velocityUnits::pct);
    motorC.startRotateFor(-degrees, rotationUnits::deg, turnSpeed, velocityUnits::pct);
    waitUntilMotorStop(motorC);
}

void turnRight(){
    turnDegrees(90);
} 

void turnLeft(){
    turnDegrees(-90);
} 

void pointTurnRight(){
    resetMotors(); 
    motorB.spin(forward, pointTurnSpeed, velocityUnits::pct);
    motorC.spin(reverse, pointTurnSpeed, velocityUnits::pct);
}

void pointTurnLeft(){
    resetMotors(); 
    motorB.spin(reverse, pointTurnSpeed, velocityUnits::pct);
    motorC.spin(forward, pointTurnSpeed, velocityUnits::pct);
}

void uTurn(){
    resetMotors(); 
    motorB.spin(forward, uTurnSpeed, velocityUnits::pct);
    motorC.spin(reverse, uTurnSpeed, velocityUnits::pct);
}   

/* void leftNudge(){
    resetMotors(); 
    motorB.spin(reverse, defaultSpeed, velocityUnits::pct);
    motorC.spin(forward, defaultSpeed, velocityUnits::pct);
// }

// void rightNudge(){
    resetMotors(); 
    motorB.spin(forward, defaultSpeed, velocityUnits::pct);
    motorC.spin(reverse, defaultSpeed, velocityUnits::pct);
} */

//not sure if this is needed tbh

task main() {
    wait(1, timeUnits::sec);
    timer T1;
    T1.clear();
   
    while(T1.time(timeUnits::msec) < 10000){
        if(SensorValue[S1] == 1){
            moveCM(10);
        } else {
            turnRight();
        }
    }

    moveCM(10);
    
    return 0;
}

