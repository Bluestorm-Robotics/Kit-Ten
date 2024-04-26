
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

SFE_ISL29125 RGB_sensors[8];

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
    Serial.println("Bottom motor found!");
  }
  if(AFMSmid.begin()) {
    Serial.println("Middle motor found!");
  }
  if(AFMStop.begin()) {
    Serial.println("Top motor found!");
  }

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
    
  delay(2000);
}

void loop() {

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

  tcaselect(0);
  Serial.println("Sensor 0:");
  // Read sensor values (16 bit integers)
  rgb[0][0] = RGB_sensors[0].readRed();
  rgb[1][0] = RGB_sensors[0].readGreen();
  rgb[2][0] = RGB_sensors[0].readBlue();
    // Print out readings, change HEX to DEC if you prefer decimal output
  Serial.print("Red: "); Serial.println(rgb[0][0],HEX);
  Serial.print("Green: "); Serial.println(rgb[1][0],HEX);
  Serial.print("Blue: "); Serial.println(rgb[2][0],HEX);
  Serial.println();


  tcaselect(1);
  Serial.println("Sensor 1:");
  // Read sensor values (16 bit integers)
    rgb[0][1] = RGB_sensors[1].readRed();
  rgb[1][1] = RGB_sensors[1].readGreen();
  rgb[2][1] = RGB_sensors[1].readBlue();
    // Print out readings, change HEX to DEC if you prefer decimal output
  Serial.print("Red: "); Serial.println(rgb[0][1],HEX);
  Serial.print("Green: "); Serial.println(rgb[1][1],HEX);
  Serial.print("Blue: "); Serial.println(rgb[2][1],HEX);
  Serial.println();


  tcaselect(2);
  Serial.println("Sensor 2:");
  // Read sensor values (16 bit integers)
  rgb[0][2] = RGB_sensors[2].readRed();
  rgb[1][2] = RGB_sensors[2].readGreen();
  rgb[2][2] = RGB_sensors[2].readBlue();
    // Print out readings, change HEX to DEC if you prefer decimal output
  Serial.print("Red: "); Serial.println(rgb[0][2],HEX);
  Serial.print("Green: "); Serial.println(rgb[1][2],HEX);
  Serial.print("Blue: "); Serial.println(rgb[2][2],HEX);
  Serial.println();


  tcaselect(3);
  Serial.println("Sensor 3:");
  // Read sensor values (16 bit integers)
  rgb[0][3] = RGB_sensors[3].readRed();
  rgb[1][3] = RGB_sensors[3].readGreen();
  rgb[2][3] = RGB_sensors[3].readBlue();
    // Print out readings, change HEX to DEC if you prefer decimal output
  Serial.print("Red: "); Serial.println(rgb[0][3],HEX);
  Serial.print("Green: "); Serial.println(rgb[1][3],HEX);
  Serial.print("Blue: "); Serial.println(rgb[2][3],HEX);
  Serial.println();


  tcaselect(4);
  Serial.println("Sensor 4:");
  // Read sensor values (16 bit integers)
  rgb[0][4] = RGB_sensors[4].readRed();
  rgb[1][4] = RGB_sensors[4].readGreen();
  rgb[2][4] = RGB_sensors[4].readBlue();
    // Print out readings, change HEX to DEC if you prefer decimal output
  Serial.print("Red: "); Serial.println(rgb[0][4],HEX);
  Serial.print("Green: "); Serial.println(rgb[1][4],HEX);
  Serial.print("Blue: "); Serial.println(rgb[2][4],HEX);
  Serial.println();


  tcaselect(5);
  Serial.println("Sensor 5:");
  // Read sensor values (16 bit integers)
  rgb[0][5] = RGB_sensors[5].readRed();
  rgb[1][5] = RGB_sensors[5].readGreen();
  rgb[2][5] = RGB_sensors[5].readBlue();
    // Print out readings, change HEX to DEC if you prefer decimal output
  Serial.print("Red: "); Serial.println(rgb[0][5],HEX);
  Serial.print("Green: "); Serial.println(rgb[1][5],HEX);
  Serial.print("Blue: "); Serial.println(rgb[2][5],HEX);
  Serial.println();


  tcaselect(6);
  Serial.println("Sensor 6:");
  // Read sensor values (16 bit integers)
  rgb[0][6] = RGB_sensors[6].readRed();
  rgb[1][6] = RGB_sensors[6].readGreen();
  rgb[2][6] = RGB_sensors[6].readBlue();
    // Print out readings, change HEX to DEC if you prefer decimal output
  Serial.print("Red: "); Serial.println(rgb[0][6],HEX);
  Serial.print("Green: "); Serial.println(rgb[1][6],HEX);
  Serial.print("Blue: "); Serial.println(rgb[2][6],HEX);
  Serial.println();


  tcaselect(7);
  Serial.println("Sensor 7:");
  // Read sensor values (17 bit integers)
  rgb[0][7] = RGB_sensors[7].readRed();
  rgb[1][7] = RGB_sensors[7].readGreen();
  rgb[2][7] = RGB_sensors[7].readBlue();
    // Print out readings, change HEX to DEC if you prefer decimal output
  Serial.print("Red: "); Serial.println(rgb[0][7],HEX);
  Serial.print("Green: "); Serial.println(rgb[1][7],HEX);
  Serial.print("Blue: "); Serial.println(rgb[2][7],HEX);
  Serial.println();

  delay(2000);
  
}
