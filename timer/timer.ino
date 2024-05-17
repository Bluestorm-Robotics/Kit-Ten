#include "TimerOne.h"
unsigned int counter = 0;
unsigned int counter2 = 0;

// This is for PID control on wheel speeds (independent between left and right)
// The code assumes the left wheel speed measurement is stored in rotation
// The code assumes the right wheel speed measurement is stored in rotation2


// initialize variables
int Kp_speedInv = 2;   // Proportional feedback gain for speed sensor feedback (Inversed to stay as an integer, the SMALLER the value, the HIGHER the gain)
int Ki_speedInv = 10;  // Integral feedback gain for speed sensor feedback (Inversed to stay as an integer, the SMALLER the value, the HIGHER the gain)
int Kd_speedInv = 5;   // Derivative feedback gain for speed sensor feedback (Inversed to stay as an integer, the SMALLER the value, the HIGHER the gain)

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

int speedMax = 250;  // maximum allowed wheel speed command

int speedTargetNominal = 100;  // rpm, target speed before adjustment by RGB sensor for line tracking

int speedTargetLeft = 100;   // rpm, target speed for left wheels, adjusted by RGB sensor for line tracking
int speedTargetRight = 100;  // rpm, target speed for right wheels adjusted by RGB sensor for line tracking

int speedFeedbackLeft = 0;   // rpm, feedback control value for left wheels
int speedFeedbackRight = 0;  // rpm, feedback control value for right wheels

int speedCommandLeft = 100;   // rpm, command speed for left wheels, adjusted speed sensor feedback
int speedCommandRight = 100;  // rpm, command speed for right wheels, adjusted speed sensor feedback

// This is where to send speedCommandLeft to the left motor control, and to send speedCommandRight to the right motor control


void docount()  // counts from the speed sensor
{
  counter++;  // increase +1 the counter value
}

void docount2()  // counts from the speed sensor
{
  counter2++;  // increase +1 the counter value
}

void timerIsr() {
  Timer1.detachInterrupt();  //stop the timer
  Serial.print("Motor Speed: ");
  int rotation = (counter * 3);  // divide by number of holes in Disc (20), divide by period (1 sec), then multiply by 60 to get RPM
  int rotation2 = (counter2 * 3);
  Serial.print(rotation, DEC);
  Serial.println(" RPM and");
  Serial.print(rotation2, DEC);
  Serial.println(" RPM");
  counter = 0;
  counter2 = 0;                      //  reset counter to zero
  Timer1.attachInterrupt(timerIsr);  //enable the timer
  // below is the code for speed tracking inside timer ISP

  errPreviousLeft = errLeft;    // save the error value from the left wheel
  errPreviousRight = errRight;  // save the error value from the right wheel

  errLeft = speedTargetLeft * rotation;     // update error for proportional feedback control for the left wheels
  errRight = speedTargetRight * rotation2;  // update error for proportional feedback control for the right wheels

  if (speedCommandLeft < speedMax || speedCommandLeft > -speedMax)  // integrator anti-windup
    err_I_Left = err_I_Left + errLeft;                              // error accumulation for integral feedback control for the left wheels

  if (speedCommandRight < speedMax || speedCommandRight > -speedMax)  // integrator anti-windup
    err_I_Right = err_I_Right + errRight;                             // error accumulation for integral feedback control for the right wheels

  err_D_Left = errLeft - errPreviousLeft;     // error calculation for derivation feedback control for the left wheels
  err_D_Right = errRight - errPreviousRight;  // error calculation for derivation feedback control for the right wheels

  P_ValueLeft = errLeft / Kp_speedInv;       // Proportional feedback value for speed sensor feedback (Inversed gain to stay as an integer)
  P_ValueRight = errRight / Kp_speedInv;     // Proportional feedback value for speed sensor feedback (Inversed gain to stay as an integer)
  I_ValueLeft = err_I_Left / Ki_speedInv;    // Integral feedback value for speed sensor feedback (Inversed gain to stay as an integer)
  I_ValueRight = err_I_Right / Ki_speedInv;  // Integral feedback value for speed sensor feedback (Inversed gain to stay as an integer)
  D_ValueLeft = err_D_Left / Kd_speedInv;    // Derivative feedback value for speed sensor feedback (Inversed gain to stay as an integer)
  D_ValueRight = err_D_Right / Kd_speedInv;  // Derivative feedback value for speed sensor feedback (Inversed gain to stay as an integer)

  speedFeedbackLeft = P_ValueLeft + I_ValueLeft + D_ValueLeft;      // feedback control for the left wheels
  speedFeedbackRight = P_ValueRight + I_ValueRight + D_ValueRight;  // feedback control for the right wheels

  Serial.print("P_ValueLeft: ");
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

  Serial.print("D_ValueRight: ");
  Serial.print(D_ValueRight, DEC);
  Serial.println(" RPM");

  Serial.print("speedFeedbackLeft: ");
  Serial.print(speedFeedbackLeft, DEC);
  Serial.println(" RPM");

  Serial.print("speedFeedbackRight: ");
  Serial.print(speedFeedbackRight, DEC);
  Serial.println(" RPM");

  // end of timer ISR
}

void setup() {
  Serial.begin(9600);
  Timer1.initialize(1000000);           // set timer for 1sec
  attachInterrupt(0, docount, RISING);  // “0” – sensor connector to pin 2 on Arduino. increase counter when speed sensor pin goes High
  attachInterrupt(1, docount2, RISING);
  Timer1.attachInterrupt(timerIsr);  // enable the timer
}

void loop() {
  // below is code inside while loop for line tracking based on RGB sensors

  speedTargetLeft = speedTargetNominal + 0;   // This line needs to be modified to adjust left wheel target speed by RGB sensor readings for line tracking
  speedTargetRight = speedTargetNominal + 0;  // This line needs to be modified to adjust left wheel target speed by RGB sensor readings for line tracking

  speedCommandLeft = speedTargetLeft + speedFeedbackLeft;     // PID control for the left wheels based on speed sensor feedback
  speedCommandRight = speedTargetRight + speedFeedbackRight;  // PID control for the right wheels based on speed sensor feedback

  if (speedCommandLeft > speedMax) speedCommandLeft = speedMax;      // stay within speed limit
  if (speedCommandLeft < -speedMax) speedCommandLeft = -speedMax;    // stay within speed limit
  if (speedCommandRight > speedMax) speedCommandRight = speedMax;    // stay within speed limit
  if (speedCommandRight < -speedMax) speedCommandRight = -speedMax;  // stay within speed limit
}