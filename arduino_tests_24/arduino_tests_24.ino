/*
#define enA 9
#define in1 4
#define in2 5
#define enB 10
#define in3 6
#define in4 7

int motorSpeedA = 75;
int motorSpeedB = 75;

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, motorSpeedA); // Send PWM signal to motor A
  analogWrite(enB, motorSpeedB); // Send PWM signal to motor B
}
*/

/*
 # This Sample code is to test the Digital Servo Shield.
 # Editor : Leff
 # Date   : 2016-1-19
 # Ver    : 1.1
 # Product: Digital Servo Shield for Arduino

 # Hardwares:
 1. Arduino UNO
 2. Digital Servo Shield for Arduino
 3. Digital Servos( Compatible with AX-12,CDS55xx...etc)
 4. Power supply:6.5 - 12V

 # How to use:
 If you don't know your Servo ID number, please
 1. Open the serial monitor, and choose NewLine,115200
 2. Send command:'d',when it's finished, please close the monitor and re-open it
 3. Send the command according to the function //controlServo()//
*/

/*
#include <SPI.h>
#include <ServoCds55.h>
ServoCds55 myservo;

int servoNum = 1;
char inputCommand ;         // a string to hold incoming data
boolean inputComplete = false;

void setup () {
  Serial.begin (115200);
  myservo.begin ();
}

void loop () {
  for(int i = 0; i < 30; i++) {
    myservo.rotate(servoNum, 10 * i);
    delay(50);
  }
  for(int i = 30; i > 0; i--) {
    myservo.rotate(servoNum, 10 * i);
    delay(50);
  }
  myservo.rotate(servoNum, 0);
  /*
  serialEvent();
  if (inputComplete) {
    Serial.print("Your command is: "); Serial.println(inputCommand); Serial.println("");
    controlServo(inputCommand);
    // clear the command:
    inputCommand = 0;
    inputComplete = false;
  }
  */ /*
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      inputComplete = true;
      break;
    }
    inputCommand += inChar;
  }
}

void controlServo(char val) {
  switch (val) {
    case 'p':
      myservo.write(servoNum, 300); //ID:1  Pos:300  velocity:150
      delay(3000);
      myservo.write(servoNum, 0); //ID:1  Pos:0  velocity:150
      break;
    case 'v':
      myservo.setVelocity(200);// set velocity to 100(range:0-300) in Servo mode
      break;
    case 'm':
      myservo.rotate(servoNum, 150); //   Anti CW    ID:1  Velocity: 150_middle velocity  300_max
      delay(2000);
      myservo.rotate(servoNum, -150); //  CW     ID:1  Velocity: -150_middle velocity  -300_max
      delay(2000);
      myservo.rotate(servoNum, 0); //Stop
      myservo.Reset(servoNum);    //Only Dynamixel AX need this instruction while changing working mode
      //CDS55xx don't need this, it can switch freely between its working mode
      break;
    case 'r':
      myservo.Reset(servoNum);//Restore ID2 servo to factory Settings ( ID:1  Baud rate:1000000)
      break;
    //        case 'i':
    //        myservo.SetID(2,1);//ID:1   newID:2
    //        break;
    case 'd':  //Reset servo to ID>>servoNum. If you don't know your Servo ID, please send "d".
      Serial.print("Please wait..");
      for (int buf = 0; buf < 255; buf++) {
        myservo.SetID(buf, servoNum);
        if (buf % 50 == 0) Serial.print(".");
      }
      delay(2000);
      Serial.println("");   Serial.println("Please close the monitor and re-open it to play your servo! ");
      break;
    default:
      Serial.println("Please give me an available instruction:");
      Serial.println("  Servo mode: p_Set position; v_Set velocity.");
      Serial.println("  Motor mode: m_Rotate; v_Set velocity.");
      Serial.println("  Others: r_Reset servo to factory settings; i_Change servo ID."); Serial.println("");
  }
}
*/

#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <SPI.h>
#include <ServoCds55.h>
ServoCds55 myservo;
int servoNum = 1;

/* Example code for the Adafruit TCS34725 breakout library */

/* Connect SCL    to analog 5
   Connect SDA    to analog 4
   Connect VDD    to 3.3V DC
   Connect GROUND to common ground */

/* Initialise with default values (int time = 2.4ms, gain = 1x) */
// Adafruit_TCS34725 tcs = Adafruit_TCS34725();

/* Initialise with specific int time and gain values */

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

void setup(void) {
  Serial.begin(9600);
  myservo.begin();
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }

  // Now we're ready to get readings!
}

void loop(void) {
  /*
  myservo.rotate(1, 40);
  myservo.rotate(2, 80);
  myservo.rotate(3, 120);
  myservo.rotate(4, 160);
  myservo.rotate(5, 200);
  myservo.rotate(6, 240);
  myservo.rotate(7, 280);
  */
  
  for(int i = 0; i < 30; i++) {
    myservo.rotate(servoNum, 10 * i);
    delay(50);
  }
  for(int i = 30; i > 0; i--) {
    myservo.rotate(servoNum, 10 * i);
    delay(50);
  }
  
  
  uint16_t r, g, b, c, colorTemp, lux;
  tcs.getRawData(&r, &g, &b, &c);
  // colorTemp = tcs.calculateColorTemperature(r, g, b);
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  lux = tcs.calculateLux(r, g, b);

  Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
  Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
  Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
  Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
  Serial.println(" ");

    int vel = lux / 30;
  if(vel > 300) {
    vel = 300;
  }
  Serial.println(vel);
  myservo.rotate(servoNum, vel);
  delay(50);
  
}

/*
  Ultrasonic Sensor HC-SR04 and Arduino Tutorial

  by Dejan Nedelkovski,
  www.HowToMechatronics.com

*/
/*
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <SPI.h>
#include <ServoCds55.h>
ServoCds55 myservo;
int servoNum = 1;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

// defines pins numbers
const int trigPin = 9;
const int echoPin = 10;
// defines variables
long duration;
int distance;

void setup() {
  Serial.begin(9600); // Starts the serial communication
  myservo.begin();
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
}
void loop() {

  uint16_t r, g, b, c, colorTemp, lux;
  tcs.getRawData(&r, &g, &b, &c);
  // colorTemp = tcs.calculateColorTemperature(r, g, b);
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  lux = tcs.calculateLux(r, g, b);

  Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
  Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
  Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
  Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
  Serial.println(" ");

  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(distance);
  int vel = distance * 5;
  if(vel > 300) {
    vel = 300;
  }
  Serial.println(vel);
}
*/