/*
  Library for the MMA8452Q
  By: Jim Lindblom and Andrea DeVore
  SparkFun Electronics

  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/14587

  This sketch demonstrates how to use the SparkFun_MMA8452Q
  library with multiple sensors.

  Hardware hookup:
  Arduino --------------- MMA8452Q Breakout
    3.3V  ---------------     3.3V
    GND   ---------------     GND
  SDA (A4) --\/330 Ohm\/--    SDA
  SCL (A5) --\/330 Ohm\/--    SCL

  The MMA8452Q is a 3.3V max sensor, so you'll need to do some
  level-shifting between the Arduino and the breakout. Series
  resistors on the SDA and SCL lines should do the trick.

  When you have two sensors, you need to solder together the
  jumper on the back of one of the sensors. By soldering
  together the jumper, the board's I2C address changes.
  I2C Address:
  Jumper Open (default) = 0x1D
  Jumper Closed = 0x1C

  License: This code is public domain, but if you see me
  (or any other SparkFun employee) at the local, and you've
  found our code helpful, please buy us a round (Beerware
  license).

  Distributed as is; no warrenty given.
*/

#include <Wire.h>                 // Must include Wire library for I2C
#include "SparkFun_MMA8452Q.h"    // Click here to get the library: http://librarymanager/All#SparkFun_MMA8452Q

// Create two instances of the MMA8452Q class
MMA8452Q accel1;
MMA8452Q accel2;

void setup() {
  Serial.begin(9600);
  Serial.println("MMA8452Q Basic Reading Code!");
  Wire.begin();

  /* accel.begin() takes in two arguments, the wire port and
     the device address in the form:
     accel.begin([deviceAddress], [wirePort])
     Default device address is 0x1D
     Default wire port is Wire

     The first instance of the MMA8452Q class uses the default
     settings, so nothing is passed into accel1.begin().
  */

  if (accel1.begin() == false) {
    Serial.println("First accelerometer not detected. Please check connections and read the hookup guide.");
    while (1);
  }
  /* The second instance of the MMA8452Q class uses the default
     wire port and the unique I2C address of the board with the
     soldered jumper. Since we need to change the second argument,
     we pass both the default wire port and the unique I2C address
     into accel2.begin(). (Note, the harware hookup above explains
     how to change the I2C address of the second MMA8452Q board.)
  */
  if (accel2.begin(Wire, 0x1c) == false) {
    Serial.println("Second accelerometer not detected. Please check connections and read the hookup guide.");
    while (1);
  }
}

void loop() {
  // Wait for new data from from both accelerometers
  if (accel1.available() && accel2.available()) {
    /* Acceleration of x, y, and z directions in g units
       for each of the sensors
    */
    float x1 = accel1.getCalculatedX();
    float x2 = accel2.getCalculatedX();

    float y1 = accel1.getCalculatedY();
    float y2 = accel2.getCalculatedY();

    float z1 = accel1.getCalculatedZ();
    float z2 = accel2.getCalculatedZ();

    Serial.print("x: ");
    Serial.print(x1, 3);
    Serial.print(" ");
    Serial.print(x2, 3);

    Serial.print("\t");
    Serial.print("\t");

    Serial.print("y: ");
    Serial.print(y1, 3);
    Serial.print(" ");
    Serial.print(y2, 3);

    Serial.print("\t");
    Serial.print("\t");
    
    Serial.print("z: ");
    Serial.print(z1, 3);
    Serial.print(" ");
    Serial.print(z2, 3);

    Serial.println();
  }
}
