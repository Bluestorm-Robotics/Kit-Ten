#include "TimerOne.h"
unsigned int counter = 0;
unsigned int counter2 = 0;

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
  counter2 = 0;                       //  reset counter to zero
  Timer1.attachInterrupt(timerIsr);  //enable the timer
}

void setup() {
  Serial.begin(9600);
  Timer1.initialize(1000000);           // set timer for 1sec
  attachInterrupt(0, docount, RISING);  // “0” – sensor connector to pin 2 on Arduino. increase counter when speed sensor pin goes High
  attachInterrupt(1, docount2, RISING);
  Timer1.attachInterrupt(timerIsr);     // enable the timer
}

void loop() {
  
}