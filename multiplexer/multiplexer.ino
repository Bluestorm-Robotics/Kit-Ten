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
          Serial.print("Found I2C 0x");  Serial.println(addr,DEC);
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
  rgb[0][0] = RGB_sensor0.readRed();
  rgb[1][0] = RGB_sensor0.readGreen();
  rgb[2][0] = RGB_sensor0.readBlue();
    // Print out readings, change HEX to DEC if you prefer decimal output
  Serial.print("Red: "); Serial.println(rgb[0][0],DEC);
  Serial.print("Green: "); Serial.println(rgb[1][0],DEC);
  Serial.print("Blue: "); Serial.println(rgb[2][0],DEC);
  Serial.println();

  
  tcaselect(1);
  Serial.println("Sensor 1:");
  // Read sensor values (16 bit integers)
    rgb[0][1] = RGB_sensor1.readRed();
  rgb[1][1] = RGB_sensor1.readGreen();
  rgb[2][1] = RGB_sensor1.readBlue();
    // Print out readings, change HEX to DEC if you prefer decimal output
  Serial.print("Red: "); Serial.println(rgb[0][1],DEC);
  Serial.print("Green: "); Serial.println(rgb[1][1],DEC);
  Serial.print("Blue: "); Serial.println(rgb[2][1],DEC);
  Serial.println();


  tcaselect(2);
  Serial.println("Sensor 2:");
  // Read sensor values (16 bit integers)
  rgb[0][2] = RGB_sensor2.readRed();
  rgb[1][2] = RGB_sensor2.readGreen();
  rgb[2][2] = RGB_sensor2.readBlue();
    // Print out readings, change HEX to DEC if you prefer decimal output
  Serial.print("Red: "); Serial.println(rgb[0][2],DEC);
  Serial.print("Green: "); Serial.println(rgb[1][2],DEC);
  Serial.print("Blue: "); Serial.println(rgb[2][2],DEC);
  Serial.println();


  tcaselect(3);
  Serial.println("Sensor 3:");
  // Read sensor values (16 bit integers)
  rgb[0][3] = RGB_sensor3.readRed();
  rgb[1][3] = RGB_sensor3.readGreen();
  rgb[2][3] = RGB_sensor3.readBlue();
    // Print out readings, change HEX to DEC if you prefer decimal output
  Serial.print("Red: "); Serial.println(rgb[0][3],DEC);
  Serial.print("Green: "); Serial.println(rgb[1][3],DEC);
  Serial.print("Blue: "); Serial.println(rgb[2][3],DEC);
  Serial.println();


  tcaselect(4);
  Serial.println("Sensor 4:");
  // Read sensor values (16 bit integers)
  rgb[0][4] = RGB_sensor4.readRed();
  rgb[1][4] = RGB_sensor4.readGreen();
  rgb[2][4] = RGB_sensor4.readBlue();
    // Print out readings, change HEX to DEC if you prefer decimal output
  Serial.print("Red: "); Serial.println(rgb[0][4],DEC);
  Serial.print("Green: "); Serial.println(rgb[1][4],DEC);
  Serial.print("Blue: "); Serial.println(rgb[2][4],DEC);
  Serial.println();


  tcaselect(5);
  Serial.println("Sensor 5:");
  // Read sensor values (16 bit integers)
  rgb[0][5] = RGB_sensor5.readRed();
  rgb[1][5] = RGB_sensor5.readGreen();
  rgb[2][5] = RGB_sensor5.readBlue();
    // Print out readings, change HEX to DEC if you prefer decimal output
  Serial.print("Red: "); Serial.println(rgb[0][5],DEC);
  Serial.print("Green: "); Serial.println(rgb[1][5],DEC);
  Serial.print("Blue: "); Serial.println(rgb[2][5],DEC);
  Serial.println();


  tcaselect(6);
  Serial.println("Sensor 6:");
  // Read sensor values (16 bit integers)
  rgb[0][6] = RGB_sensor6.readRed();
  rgb[1][6] = RGB_sensor6.readGreen();
  rgb[2][6] = RGB_sensor6.readBlue();
    // Print out readings, change HEX to DEC if you prefer decimal output
  Serial.print("Red: "); Serial.println(rgb[0][6],DEC);
  Serial.print("Green: "); Serial.println(rgb[1][6],DEC);
  Serial.print("Blue: "); Serial.println(rgb[2][6],DEC);
  Serial.println();


  tcaselect(7);
  Serial.println("Sensor 7:");
  // Read sensor values (17 bit integers)
  rgb[0][7] = RGB_sensor7.readRed();
  rgb[1][7] = RGB_sensor7.readGreen();
  rgb[2][7] = RGB_sensor7.readBlue();
    // Print out readings, change HEX to DEC if you prefer decimal output
  Serial.print("Red: "); Serial.println(rgb[0][7],DEC);
  Serial.print("Green: "); Serial.println(rgb[1][7],DEC);
  Serial.print("Blue: "); Serial.println(rgb[2][7],DEC);
  Serial.println();
  

  delay(2000);
  
}
