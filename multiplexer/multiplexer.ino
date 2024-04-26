/**
 * TCA9548 I2CScanner.ino -- I2C bus scanner for Arduino
 *
 * Based on https://playground.arduino.cc/Main/I2cScanner/
 *
 */

#include "Wire.h"
#include "SparkFunISL29125.h"

#define TCAADDR 0x70

SFE_ISL29125 RGB_sensor0;
SFE_ISL29125 RGB_sensor1;
SFE_ISL29125 RGB_sensor2;
SFE_ISL29125 RGB_sensor3;
SFE_ISL29125 RGB_sensor4;
SFE_ISL29125 RGB_sensor5;
SFE_ISL29125 RGB_sensor6;
SFE_ISL29125 RGB_sensor7;

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
    
  delay(2000);
}

void loop() {
  tcaselect(0);
  Serial.println("Sensor 0:");
  // Read sensor values (16 bit integers)
  unsigned int red0 = RGB_sensor0.readRed();
  unsigned int green0 = RGB_sensor0.readGreen();
  unsigned int blue0 = RGB_sensor0.readBlue();
  
  // Print out readings, change HEX to DEC if you prefer decimal output
  Serial.print("Red: "); Serial.println(red0,HEX);
  Serial.print("Green: "); Serial.println(green0,HEX);
  Serial.print("Blue: "); Serial.println(blue0,HEX);
  Serial.println();

  tcaselect(1);
  Serial.println("Sensor 1:");
  // Read sensor values (16 bit integers)
  unsigned int red1 = RGB_sensor1.readRed();
  unsigned int green1 = RGB_sensor1.readGreen();
  unsigned int blue1 = RGB_sensor1.readBlue();
  
  // Print out readings, change HEX to DEC if you prefer decimal output
  Serial.print("Red: "); Serial.println(red1,HEX);
  Serial.print("Green: "); Serial.println(green1,HEX);
  Serial.print("Blue: "); Serial.println(blue1,HEX);
  Serial.println();

  tcaselect(2);
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

  tcaselect(3);
  Serial.println("Sensor 3:");
  // Read sensor values (16 bit integers)
  unsigned int red3 = RGB_sensor3.readRed();
  unsigned int green3 = RGB_sensor3.readGreen();
  unsigned int blue3 = RGB_sensor3.readBlue();
    // Print out readings, change HEX to DEC if you prefer decimal output
  Serial.print("Red: "); Serial.println(red3,HEX);
  Serial.print("Green: "); Serial.println(green3,HEX);
  Serial.print("Blue: "); Serial.println(blue3,HEX);
  Serial.println();

  tcaselect(4);
  Serial.println("Sensor 4:");
  // Read sensor values (16 bit integers)
  unsigned int red4 = RGB_sensor4.readRed();
  unsigned int green4 = RGB_sensor4.readGreen();
  unsigned int blue4 = RGB_sensor4.readBlue();
    // Print out readings, change HEX to DEC if you prefer decimal output
  Serial.print("Red: "); Serial.println(red4,HEX);
  Serial.print("Green: "); Serial.println(green4,HEX);
  Serial.print("Blue: "); Serial.println(blue4,HEX);
  Serial.println();

  tcaselect(5);
  Serial.println("Sensor 5:");
  // Read sensor values (16 bit integers)
  unsigned int red5 = RGB_sensor5.readRed();
  unsigned int green5 = RGB_sensor5.readGreen();
  unsigned int blue5 = RGB_sensor5.readBlue();
    // Print out readings, change HEX to DEC if you prefer decimal output
  Serial.print("Red: "); Serial.println(red5,HEX);
  Serial.print("Green: "); Serial.println(green5,HEX);
  Serial.print("Blue: "); Serial.println(blue5,HEX);
  Serial.println();

  tcaselect(6);
  Serial.println("Sensor 6:");
  // Read sensor values (16 bit integers)
  unsigned int red6 = RGB_sensor6.readRed();
  unsigned int green6 = RGB_sensor6.readGreen();
  unsigned int blue6 = RGB_sensor6.readBlue();
    // Print out readings, change HEX to DEC if you prefer decimal output
  Serial.print("Red: "); Serial.println(red6,HEX);
  Serial.print("Green: "); Serial.println(green6,HEX);
  Serial.print("Blue: "); Serial.println(blue6,HEX);
  Serial.println();

  tcaselect(7);
  Serial.println("Sensor 7:");
  // Read sensor values (17 bit integers)
  unsigned int red7 = RGB_sensor7.readRed();
  unsigned int green7 = RGB_sensor7.readGreen();
  unsigned int blue7 = RGB_sensor7.readBlue();
    // Print out readings, change HEX to DEC if you prefer decimal output
  Serial.print("Red: "); Serial.println(red7,HEX);
  Serial.print("Green: "); Serial.println(green7,HEX);
  Serial.print("Blue: "); Serial.println(blue7,HEX);
  Serial.println();

  delay(2000);
  
}
