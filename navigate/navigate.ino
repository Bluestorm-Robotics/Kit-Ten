
#include "Wire.h"
#include "SparkFunISL29125.h"

#include <stdint.h>


#include <AccelStepper.h>
#include <Adafruit_MotorShield.h>

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

// you can change these to DOUBLE or INTERLEAVE or MICROSTEP!
// wrappers for the first motor!
void forwardstep1() {
  myStepper1->onestep(FORWARD, SINGLE);
}
void backwardstep1() {
  myStepper1->onestep(BACKWARD, SINGLE);
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
  myStepper3->onestep(FORWARD, INTERLEAVE);
}
void backwardstep3() {
  myStepper3->onestep(BACKWARD, INTERLEAVE);
}
// wrappers for the fourth motor!
void forwardstep4() {
  myStepper4->onestep(FORWARD, SINGLE);
}
void backwardstep4() {
  myStepper4->onestep(BACKWARD, SINGLE);
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
  myStepper6->onestep(FORWARD, INTERLEAVE);
}
void backwardstep6() {
  myStepper6->onestep(BACKWARD, INTERLEAVE);
}

// Now we'll wrap the 6 steppers in an AccelStepper object
AccelStepper stepper1(forwardstep1, backwardstep1);
AccelStepper stepper2(forwardstep2, backwardstep2);
AccelStepper stepper3(forwardstep3, backwardstep3);
AccelStepper stepper4(forwardstep4, backwardstep4);
AccelStepper stepper5(forwardstep5, backwardstep5);
AccelStepper stepper6(forwardstep6, backwardstep6);

#define TCAADDR 0x70

SFE_ISL29125 RGB_sensor0;
SFE_ISL29125 RGB_sensor1;
SFE_ISL29125 RGB_sensor2;
SFE_ISL29125 RGB_sensor3;
SFE_ISL29125 RGB_sensor4;
SFE_ISL29125 RGB_sensor5;
SFE_ISL29125 RGB_sensor6;
SFE_ISL29125 RGB_sensor7;

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
    while (!Serial);
    delay(1000);

    Wire.begin();
    
    Serial.begin(115200);

    tcaselect(0);
    if (RGB_sensor0.init()) {
      Serial.println("Sensor 0 Initialization Successful\n\r");
    }
    tcaselect(1);
    if (RGB_sensor1.init()) {
      Serial.println("Sensor 1 Initialization Successful\n\r");
    }
    tcaselect(2);
    if(RGB_sensor2.init()) {
      Serial.println("Sensor 2 Initialization Successful\n\r");
    }
    tcaselect(3);
    if(RGB_sensor3.init()) {
      Serial.println("Sensor 3 Initialization Successful\n\r");
    }
    tcaselect(4);
    if(RGB_sensor4.init()) {
      Serial.println("Sensor 4 Initialization Successful\n\r");
    }
    tcaselect(5);
    if(RGB_sensor5.init()) {
      Serial.println("Sensor 5 Initialization Successful\n\r");
    }
    tcaselect(6);
    if(RGB_sensor6.init()) {
      Serial.println("Sensor 6 Initialization Successful\n\r");
    }
    tcaselect(7);
    if(RGB_sensor7.init()) {
      Serial.println("Sensor 7 Initialization Successful\n\r");
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
    Serial.println("Bottom motor found!");
  }
  if(AFMSmid.begin()) {
    Serial.println("Middle motor found!");
  }
  if(AFMStop.begin()) {
    Serial.println("Top motor found!");
  }
    
  delay(2000);
}

void loop() {
  tcaselect(0);
  Serial.println("Sensor 0:");
  // Read sensor values (16 bit integers)
  rgb[0][0] = RGB_sensor0.readRed();
  rgb[1][0] = RGB_sensor0.readGreen();
  rgb[2][0] = RGB_sensor0.readBlue();
    // Print out readings, change HEX to DEC if you prefer decimal output
  Serial.print("Red: "); Serial.println(rgb[0][0],HEX);
  Serial.print("Green: "); Serial.println(rgb[1][0],HEX);
  Serial.print("Blue: "); Serial.println(rgb[2][0],HEX);
  Serial.println();


  tcaselect(1);
  Serial.println("Sensor 1:");
  // Read sensor values (16 bit integers)
    rgb[0][1] = RGB_sensor1.readRed();
  rgb[1][1] = RGB_sensor1.readGreen();
  rgb[2][1] = RGB_sensor1.readBlue();
    // Print out readings, change HEX to DEC if you prefer decimal output
  Serial.print("Red: "); Serial.println(rgb[0][1],HEX);
  Serial.print("Green: "); Serial.println(rgb[1][1],HEX);
  Serial.print("Blue: "); Serial.println(rgb[2][1],HEX);
  Serial.println();


  tcaselect(2);
  Serial.println("Sensor 2:");
  // Read sensor values (16 bit integers)
  rgb[0][2] = RGB_sensor2.readRed();
  rgb[1][2] = RGB_sensor2.readGreen();
  rgb[2][2] = RGB_sensor2.readBlue();
    // Print out readings, change HEX to DEC if you prefer decimal output
  Serial.print("Red: "); Serial.println(rgb[0][2],HEX);
  Serial.print("Green: "); Serial.println(rgb[1][2],HEX);
  Serial.print("Blue: "); Serial.println(rgb[2][2],HEX);
  Serial.println();


  tcaselect(3);
  Serial.println("Sensor 3:");
  // Read sensor values (16 bit integers)
  rgb[0][3] = RGB_sensor3.readRed();
  rgb[1][3] = RGB_sensor3.readGreen();
  rgb[2][3] = RGB_sensor3.readBlue();
    // Print out readings, change HEX to DEC if you prefer decimal output
  Serial.print("Red: "); Serial.println(rgb[0][3],HEX);
  Serial.print("Green: "); Serial.println(rgb[1][3],HEX);
  Serial.print("Blue: "); Serial.println(rgb[2][3],HEX);
  Serial.println();


  tcaselect(4);
  Serial.println("Sensor 4:");
  // Read sensor values (16 bit integers)
  rgb[0][4] = RGB_sensor4.readRed();
  rgb[1][4] = RGB_sensor4.readGreen();
  rgb[2][4] = RGB_sensor4.readBlue();
    // Print out readings, change HEX to DEC if you prefer decimal output
  Serial.print("Red: "); Serial.println(rgb[0][4],HEX);
  Serial.print("Green: "); Serial.println(rgb[1][4],HEX);
  Serial.print("Blue: "); Serial.println(rgb[2][4],HEX);
  Serial.println();


  tcaselect(5);
  Serial.println("Sensor 5:");
  // Read sensor values (16 bit integers)
  rgb[0][5] = RGB_sensor5.readRed();
  rgb[1][5] = RGB_sensor5.readGreen();
  rgb[2][5] = RGB_sensor5.readBlue();
    // Print out readings, change HEX to DEC if you prefer decimal output
  Serial.print("Red: "); Serial.println(rgb[0][5],HEX);
  Serial.print("Green: "); Serial.println(rgb[1][5],HEX);
  Serial.print("Blue: "); Serial.println(rgb[2][5],HEX);
  Serial.println();


  tcaselect(6);
  Serial.println("Sensor 6:");
  // Read sensor values (16 bit integers)
  rgb[0][6] = RGB_sensor6.readRed();
  rgb[1][6] = RGB_sensor6.readGreen();
  rgb[2][6] = RGB_sensor6.readBlue();
    // Print out readings, change HEX to DEC if you prefer decimal output
  Serial.print("Red: "); Serial.println(rgb[0][6],HEX);
  Serial.print("Green: "); Serial.println(rgb[1][6],HEX);
  Serial.print("Blue: "); Serial.println(rgb[2][6],HEX);
  Serial.println();


  tcaselect(7);
  Serial.println("Sensor 7:");
  // Read sensor values (17 bit integers)
  rgb[0][7] = RGB_sensor7.readRed();
  rgb[1][7] = RGB_sensor7.readGreen();
  rgb[2][7] = RGB_sensor7.readBlue();
    // Print out readings, change HEX to DEC if you prefer decimal output
  Serial.print("Red: "); Serial.println(rgb[0][7],HEX);
  Serial.print("Green: "); Serial.println(rgb[1][7],HEX);
  Serial.print("Blue: "); Serial.println(rgb[2][7],HEX);
  Serial.println();

  delay(2000);
  
}
