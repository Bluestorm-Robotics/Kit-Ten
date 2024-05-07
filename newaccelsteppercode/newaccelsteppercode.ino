#include <AccelStepper.h>

// Voor de Arduino Uno + CNC shield V3
#define MOTOR_X_ENABLE_PIN 8
#define MOTOR_X_STEP_PIN 2
#define MOTOR_X_DIR_PIN 5


#define MOTOR_Y_ENABLE_PIN 8
#define MOTOR_Y_STEP_PIN 3
#define MOTOR_Y_DIR_PIN 6


#define MOTOR_Z_ENABLE_PIN 8
#define MOTOR_Z_STEP_PIN 4
#define MOTOR_Z_DIR_PIN 7

#define MOTOR_A_ENABLE_PIN 8
#define MOTOR_A_STEP_PIN 12
#define MOTOR_A_DIR_PIN 13

AccelStepper motorX(1, MOTOR_X_STEP_PIN, MOTOR_X_DIR_PIN);
AccelStepper motorY(1, MOTOR_Y_STEP_PIN, MOTOR_Y_DIR_PIN);
AccelStepper motorZ(1, MOTOR_Z_STEP_PIN, MOTOR_Z_DIR_PIN);
AccelStepper motorA(1, MOTOR_A_STEP_PIN, MOTOR_A_DIR_PIN);

void setup()
{
   pinMode(MOTOR_X_ENABLE_PIN, OUTPUT);
   pinMode(MOTOR_Y_ENABLE_PIN, OUTPUT);
   pinMode(MOTOR_Z_ENABLE_PIN, OUTPUT);
   pinMode(MOTOR_A_ENABLE_PIN, OUTPUT);

   motorX.setEnablePin(MOTOR_X_ENABLE_PIN);
   motorX.setPinsInverted(false, false, true);
   motorX.setAcceleration(100);
   //motorX.setMaxSpeed(100);
   //motorX.setSpeed(100);
   motorX.enableOutputs();

   motorY.setEnablePin(MOTOR_Y_ENABLE_PIN);
   motorY.setPinsInverted(false, false, true);
   motorY.setAcceleration(100);
   //motorY.setMaxSpeed(100);
   //motorY.setSpeed(100);
   motorY.enableOutputs();

   motorZ.setEnablePin(MOTOR_Z_ENABLE_PIN);
   motorZ.setPinsInverted(false, false, true);
   motorZ.setAcceleration(100);
   //motorZ.setMaxSpeed(100);
   //motorZ.setSpeed(100);
   motorZ.enableOutputs();

   motorA.setEnablePin(MOTOR_Z_ENABLE_PIN);
   motorA.setPinsInverted(false, false, true);
   motorA.setAcceleration(100);
   //motorA.setMaxSpeed(100);
   //motorA.setSpeed(100);
   motorA.enableOutputs();
}

void loop()
{
   motorX.move(3000);
   motorX.run();
   motorY.move(3000);
   motorY.run();
   motorZ.move(3000);
   motorZ.run();
   motorA.move(3000);
   motorA.run();
}