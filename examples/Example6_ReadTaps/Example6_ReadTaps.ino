/******************************************************************************
  Change scale of the MMA8452Q
  SFE_MMA8452Q Library Orientation Sketch
  Jim Lindblom @ SparkFun Electronics
  Original Creation Date: June 3, 2014
  https://github.com/sparkfun/MMA8452_Accelerometer

  This sketch uses the SparkFun_MMA8452Q library to initialize the
  accelerometer, and stream x, y, z acceleration values from it.

  Hardware hookup:
  Arduino --------------- MMA8452Q Breakout
    3.3V  ---------------     3.3V
    GND   ---------------     GND
  SDA (A4) --\/330 Ohm\/--    SDA
  SCL (A5) --\/330 Ohm\/--    SCL

  The MMA8452Q is a 3.3V max sensor, so you'll need to do some
  level-shifting between the Arduino and the breakout. Series
  resistors on the SDA and SCL lines should do the trick.

  Development environment specifics:
  IDE: Arduino 1.0.5
  Hardware Platform: Arduino Uno

  **Updated for Arduino 1.6.4 5/2015**

  This code is beerware; if you see me (or any other SparkFun employee) at the
  local, and you've found our code helpful, please buy us a round!

  Distributed as-is; no warranty is given.
******************************************************************************/
#include <Wire.h>                 // Must include Wire library for I2C
#include "SparkFun_MMA8452Q.h"    // Click here to get the library: http://librarymanager/All#SparkFun_MMA8452Q

MMA8452Q accel;                   // create instance of the MMA8452 class

void setup() {
  Serial.begin(9600);
  Serial.println("MMA8452Q Read Tap Code!");
  Wire.begin();

  if (accel.begin() == false) {
    Serial.println("Not Connected. Please check connections and read the hookup guide.");
    while (1);
  }
}

void loop() {
  if (accel.available()) {      // Wait for new data from accelerometer
    // Prints "Tap" each time accelerometer detects a tap
    if (accel.readTap() > 0) {
      Serial.println("Tap");
      delay(10);
    }
  }
}
