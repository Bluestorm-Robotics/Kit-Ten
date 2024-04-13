/**
 * TCA9548 I2CScanner.ino -- I2C bus scanner for Arduino
 *
 * Based on https://playground.arduino.cc/Main/I2cScanner/
 *
 */

#include "Wire.h"
#include "SparkFunISL29125.h"

#define TCAADDR 0x70

SFE_ISL29125 RGB_sensor1;
SFE_ISL29125 RGB_sensor2;

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

    tcaselect(1);
    if (RGB_sensor1.init()) {
      Serial.println("Sensor 1 Initialization Successful\n\r");
    }
    tcaselect(2);
    if(RGB_sensor2.init()) {
      Serial.println("Sensor 2 Initialization Successful\n\r");
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

  delay(2000);
  
}
