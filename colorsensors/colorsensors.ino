#include <ISL29125_SoftWire.h>

/******************************************************************************
ISL29125_basics.ino
Simple example for using the ISL29125 RGB sensor library.
Jordan McConnell @ SparkFun Electronics
11 Apr 2014
https://github.com/sparkfun/SparkFun_ISL29125_Breakout_Arduino_Library

This example declares an SFE_ISL29125 object called RGB_sensor. The 
object/sensor is initialized with a basic configuration so that it continuously
samples the light intensity of red, green and blue spectrums. These values are
read from the sensor every 2 seconds and printed to the Serial monitor.

Developed/Tested with:
Arduino Uno
Arduino IDE 1.0.5

Requires:
SparkFun_ISL29125_Arduino_Library

This code is beerware.
Distributed as-is; no warranty is given. 
******************************************************************************/

#include <Wire.h>
#include "SparkFunISL29125.h"

// SFE_ISL29125 RGB_sensor;
// SFE_ISL29125 RGB_sensor2;

// Declare sensor object
static int SDA1 = 1;
static int SCL1 = 2;
ISL29125_SOFT RGB_sensor;

static int SDA2 = 3;
static int SCL2 = 4;
ISL29125_SOFT RGB_sensor2;


void setup()
{
  // Initialize serial communication
  Serial.begin(115200);

  // Initialize the ISL29125 with simple configuration so it starts sampling
  if (RGB_sensor.init(SDA,SCL))
  {
    Serial.println("Sensor 1 Initialization Successful\n\r");
    if(RGB_sensor2.init(SDA2,SCL2)) {
      Serial.println("Sensor 2 Initialization Successful\n\r");
    }
  }
}

// Read sensor values for each color and print them to serial monitor
void loop()
{
  Serial.println("Sensor 1:");
  // Read sensor values (16 bit integers)
  unsigned int red = RGB_sensor.readRed();
  unsigned int green = RGB_sensor.readGreen();
  unsigned int blue = RGB_sensor.readBlue();
  
  // Print out readings, change HEX to DEC if you prefer decimal output
  Serial.print("Red: "); Serial.println(red,DEC);
  Serial.print("Green: "); Serial.println(green,DEC);
  Serial.print("Blue: "); Serial.println(blue,DEC);
  Serial.println();

  /*

  Serial.println("Sensor 2:");
  // Read sensor values (16 bit integers)
  unsigned int red2 = RGB_sensor2.readRed();
  unsigned int green2 = RGB_sensor2.readGreen();
  unsigned int blue2 = RGB_sensor2.readBlue();
  
  // Print out readings, change HEX to DEC if you prefer decimal output
  Serial.print("Red: "); Serial.println(red2,HEX);
  Serial.print("Green: "); Serial.println(green2,HEX);
  Serial.print("Blue: "); Serial.println(blue2,HEX);
  Serial.println();
  */

  delay(1000);
}
