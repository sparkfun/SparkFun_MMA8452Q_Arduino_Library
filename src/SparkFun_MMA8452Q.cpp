/******************************************************************************
SparkFun_MMA8452Q.cpp
SparkFun_MMA8452Q Library Source File
Jim Lindblom and Andrea DeVore @ SparkFun Electronics
Original Creation Date: June 3, 2014
https://github.com/sparkfun/MMA8452_Accelerometer

This file implements all functions of the MMA8452Q class. Functions here range
from higher level stuff, like reading/writing MMA8452Q registers to low-level,
hardware I2C reads and writes.

Development environment specifics:
	IDE: Arduino 1.0.5
	Hardware Platform: Arduino Uno

	**Updated for Arduino 1.8.5 2/2019**

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#include "SparkFun_MMA8452Q.h"
#include <Arduino.h>
#include <Wire.h>

//#define DEBUG

// CONSTRUCTUR
//   This function, called when you initialize the class will simply write the
//   supplied address into a private variable for future use.
//   The variable addr should be either 0x1C or 0x1D, depending on which voltage
//   the SA0 pin is tied to (GND or 3.3V respectively).
MMA8452Q::MMA8452Q(byte addr)
{
	_deviceAddress = addr; // Store address into private variable
}

// BEGIN INITIALIZATION (New Implementation of Init)
// 	This will be used instead of init in future sketches
// 	to match Arudino guidelines. We will maintain init
// 	for backwards compatability purposes.
bool MMA8452Q::begin(TwoWire &wirePort, uint8_t deviceAddress)
{
	_deviceAddress = deviceAddress;
	_i2cPort = &wirePort;

	byte c = readRegister(WHO_AM_I); // Read WHO_AM_I register

	if (c != 0x2A) // WHO_AM_I should always be 0x2A
	{
		return false;
	}

	scale = SCALE_2G;
	odr = ODR_800;

	setScale(scale);  	  // Set up accelerometer scale
	setDataRate(odr); 	  // Set up output data rate
	setupPL();		  // Set up portrait/landscape detection

	// Multiply parameter by 0.0625g to calculate threshold.
	setupTap(0x80, 0x80, 0x08); // Disable x, y, set z to 0.5g

	return true;
}

// INITIALIZATION
//	This function initializes the MMA8452Q. It sets up the scale (either 2, 4,
//	or 8g), output data rate, portrait/landscape detection and tap detection.
//	It also checks the WHO_AM_I register to make sure we can communicate with
//	the sensor. Returns a 0 if communication failed, 1 if successful.
byte MMA8452Q::init(MMA8452Q_Scale fsr, MMA8452Q_ODR odr)
{
	scale = fsr; // Haul fsr into our class variable, scale

	if (_i2cPort == NULL)
	{
		_i2cPort = &Wire;
	}

	_i2cPort->begin(); // Initialize I2C

	byte c = readRegister(WHO_AM_I); // Read WHO_AM_I register

	if (c != 0x2A) // WHO_AM_I should always be 0x2A
	{
		return 0;
	}

	standby(); // Must be in standby to change registers

	setScale(scale);  // Set up accelerometer scale
	setDataRate(odr); // Set up output data rate
	setupPL();	  // Set up portrait/landscape detection
	// Multiply parameter by 0.0625g to calculate threshold.
	setupTap(0x80, 0x80, 0x08); // Disable x, y, set z to 0.5g

	active(); // Set to active to start reading

	return 1;
}

byte MMA8452Q::readID()
{
	return readRegister(WHO_AM_I);
}

// GET FUNCTIONS FOR RAW ACCELERATION DATA
// Returns raw X acceleration data
short MMA8452Q::getX()
{
	byte rawData[2];
	readRegisters(OUT_X_MSB, rawData, 2); // Read the X data into a data array
	return ((short)(rawData[0] << 8 | rawData[1])) >> 4;
}

// Returns raw Y acceleration data
short MMA8452Q::getY()
{
	byte rawData[2];
	readRegisters(OUT_Y_MSB, rawData, 2); // Read the Y data into a data array
	return ((short)(rawData[0] << 8 | rawData[1])) >> 4;
}

// Returns raw Z acceleration data
short MMA8452Q::getZ()
{
	byte rawData[2];
	readRegisters(OUT_Z_MSB, rawData, 2); // Read the Z data into a data array
	return ((short)(rawData[0] << 8 | rawData[1])) >> 4;
}

// GET FUNCTIONS FOR CALCULATED ACCELERATION DATA
// Returns calculated X acceleration data
float MMA8452Q::getCalculatedX()
{
	x = getX();
	return (float)x / (float)(1 << 11) * (float)(scale);
}

// Returns calculated Y acceleration data
float MMA8452Q::getCalculatedY()
{
	y = getY();
	return (float)y / (float)(1 << 11) * (float)(scale);
}

// Returns calculated Z acceleration data
float MMA8452Q::getCalculatedZ()
{
	z = getZ();
	return (float)z / (float)(1 << 11) * (float)(scale);
}

// READ ACCELERATION DATA
//  This function will read the acceleration values from the MMA8452Q. After
//	reading, it will update two triplets of variables:
//		* int's x, y, and z will store the signed 12-bit values read out
//		  of the acceleromter.
//		* floats cx, cy, and cz will store the calculated acceleration from
//		  those 12-bit values. These variables are in units of g's.
void MMA8452Q::read()
{
	byte rawData[6]; // x/y/z accel register data stored here

	readRegisters(OUT_X_MSB, rawData, 6); // Read the six raw data registers into data array

	x = ((short)(rawData[0] << 8 | rawData[1])) >> 4;
	y = ((short)(rawData[2] << 8 | rawData[3])) >> 4;
	z = ((short)(rawData[4] << 8 | rawData[5])) >> 4;
	cx = (float)x / (float)(1 << 11) * (float)(scale);
	cy = (float)y / (float)(1 << 11) * (float)(scale);
	cz = (float)z / (float)(1 << 11) * (float)(scale);
}

// CHECK IF NEW DATA IS AVAILABLE
//	This function checks the status of the MMA8452Q to see if new data is availble.
//	returns 0 if no new data is present, or a 1 if new data is available.
byte MMA8452Q::available()
{
	return (readRegister(STATUS_MMA8452Q) & 0x08) >> 3;
}

// SET FULL-SCALE RANGE
//	This function sets the full-scale range of the x, y, and z axis accelerometers.
//	Possible values for the fsr variable are SCALE_2G, SCALE_4G, or SCALE_8G.
void MMA8452Q::setScale(MMA8452Q_Scale fsr)
{
	// Must be in standby mode to make changes!!!
	// Change to standby if currently in active state
	if (isActive() == true)
		standby();

	byte cfg = readRegister(XYZ_DATA_CFG);
	cfg &= 0xFC;	   // Mask out scale bits
	cfg |= (fsr >> 2); // Neat trick, see page 22. 00 = 2G, 01 = 4A, 10 = 8G
	writeRegister(XYZ_DATA_CFG, cfg);

	// Return to active state when done
	// Must be in active state to read data
	active();
}

// SET THE OUTPUT DATA RATE
//	This function sets the output data rate of the MMA8452Q.
//	Possible values for the odr parameter are: ODR_800, ODR_400, ODR_200,
//	ODR_100, ODR_50, ODR_12, ODR_6, or ODR_1
void MMA8452Q::setDataRate(MMA8452Q_ODR odr)
{
	// Must be in standby mode to make changes!!!
	// Change to standby if currently in active state
	if (isActive() == true)
		standby();

	byte ctrl = readRegister(CTRL_REG1);
	ctrl &= 0xC7; // Mask out data rate bits
	ctrl |= (odr << 3);
	writeRegister(CTRL_REG1, ctrl);

	// Return to active state when done
	// Must be in active state to read data
	active();
}

// SET UP TAP DETECTION
//	This function can set up tap detection on the x, y, and/or z axes.
//	The xThs, yThs, and zThs parameters serve two functions:
//		1. Enable tap detection on an axis. If the 7th bit is SET (0x80)
//			tap detection on that axis will be DISABLED.
//		2. Set tap g's threshold. The lower 7 bits will set the tap threshold
//			on that axis.
void MMA8452Q::setupTap(byte xThs, byte yThs, byte zThs)
{
	// Must be in standby mode to make changes!!!
	// Change to standby if currently in active state
	if (isActive() == true)
		standby();

	// Set up single and double tap - 5 steps:
	// for more info check out this app note:
	// http://cache.freescale.com/files/sensors/doc/app_note/AN4072.pdf
	// Set the threshold - minimum required acceleration to cause a tap.
	byte temp = 0;
	if (!(xThs & 0x80)) // If top bit ISN'T set
	{
		temp |= 0x3;					 // Enable taps on x
		writeRegister(PULSE_THSX, xThs); // x thresh
	}
	if (!(yThs & 0x80))
	{
		temp |= 0xC;					 // Enable taps on y
		writeRegister(PULSE_THSY, yThs); // y thresh
	}
	if (!(zThs & 0x80))
	{
		temp |= 0x30;					 // Enable taps on z
		writeRegister(PULSE_THSZ, zThs); // z thresh
	}
	// Set up single and/or double tap detection on each axis individually.
	writeRegister(PULSE_CFG, temp | 0x40);
	// Set the time limit - the maximum time that a tap can be above the thresh
	writeRegister(PULSE_TMLT, 0x30); // 30ms time limit at 800Hz odr
	// Set the pulse latency - the minimum required time between pulses
	writeRegister(PULSE_LTCY, 0xA0); // 200ms (at 800Hz odr) between taps min
	// Set the second pulse window - maximum allowed time between end of
	//	latency and start of second pulse
	writeRegister(PULSE_WIND, 0xFF); // 5. 318ms (max value) between taps max

	// Return to active state when done
	// Must be in active state to read data
	active();
}

// This function will read the motion detection source register and
// print motion direction
// Return 0 for no motion
// 0x02 X motion, 0x08 Y motion, 0x20 Z motion
uint8_t MMA8452Q::readMotionType()
{
    byte source = readRegister(FF_MT_SRC);
    uint8_t flags;
  if((source >> 7) == 1) {  // If Event Active flag set in the FF_MT_SRC register

  if ((source & 0x02)==0x02)  // If XHE bit is set, x-motion detected
	  flags |= 0x02;
  if ((source & 0x08)==0x08)  // If YHE bit is set, y-motion detected
	  flags |= 0x08;
  if ((source & 0x20)==0x20)  // If ZHE bit is set, z-motion detected
	  flags |= 0x20;
  if ((source & 0x10)==0x10)  // If ZHP is set
	  flags |= 0x10;
  return flags;
  }
  return 0;
}

// Set up sensor software reset
void MMA8452Q::reset() 
{
  writeRegister(CTRL_REG2, 0x40); // set reset bit to 1 to assert software reset to zero at end of boot process
  delay(2);			  // here states ~1ms https://community.nxp.com/docs/DOC-332453
}

// Allow user compensation of acceleration errors

// READ TAP STATUS
//	This function returns any taps read by the MMA8452Q. If the function
//	returns no new taps were detected. Otherwise the function will return the
//	lower 7 bits of the PULSE_SRC register.
byte MMA8452Q::readTap()
{
	byte tapStat = readRegister(PULSE_SRC);
	if (tapStat & 0x80) // Read EA bit to check if a interrupt was generated
	{
		return tapStat & 0x7F;
	}
	else
		return 0;
}

// SET UP PORTRAIT/LANDSCAPE DETECTION
//	This function sets up portrait and landscape detection.
void MMA8452Q::setupPL(uint8_t debounce)
{
	// Must be in standby mode to make changes!!!
	// Change to standby if currently in active state
	if (isActive() == true)
		standby();

	// For more info check out this app note:
	//	http://cache.freescale.com/files/sensors/doc/app_note/AN4068.pdf
	// 1. Enable P/L
	writeRegister(PL_CFG, readRegister(PL_CFG) | 0x40); // Set PL_EN (enable)
	// 2. Set the debounce rate
	writeRegister(PL_COUNT, 0x50); // Debounce counter at 100ms (at 800 hz)

	// Return to active state when done
	// Must be in active state to read data
	active();
}

// READ PORTRAIT/LANDSCAPE STATUS
//	This function reads the portrait/landscape status register of the MMA8452Q.
//	It will return either PORTRAIT_U, PORTRAIT_D, LANDSCAPE_R, LANDSCAPE_L,
//	or LOCKOUT. LOCKOUT indicates that the sensor is in neither p or ls.
byte MMA8452Q::readPL()
{
	byte plStat = readRegister(PL_STATUS);

	if (plStat & 0x40) // Z-tilt lockout
		return LOCKOUT;
	else // Otherwise return LAPO status
		return (plStat & 0x6) >> 1;
}

// CHECK FOR ORIENTATION
bool MMA8452Q::isRight()
{
	if (readPL() == LANDSCAPE_R)
		return true;
	return false;
}

bool MMA8452Q::isLeft()
{
	if (readPL() == LANDSCAPE_L)
		return true;
	return false;
}

bool MMA8452Q::isUp()
{
	if (readPL() == PORTRAIT_U)
		return true;
	return false;
}

bool MMA8452Q::isDown()
{
	if (readPL() == PORTRAIT_D)
		return true;
	return false;
}

bool MMA8452Q::isFlat()
{
	if (readPL() == LOCKOUT)
		return true;
	return false;
}

// Enable / Disable auto sleep mode. check AN4074
// Returns false if something went bad.
//
// ovr (Oversampling mode, or Power mode) 0 to 4. I think 1 is a sane value for low power devices (2ua difference with 0) but limits to 4g
// ovr is on bits [3-4] for Active Sleep and [0-1] for Active Wake.
// For simplification we use the same value
// TODO / EVAL: Bad combination is ovrW = 1 and ovrS = 4!
//
// ssr Sleep Sample Rate (0 to 3) 0 = 1.56Hz, 6.25Hz, 12.5Hz, 50Hz
// wsr Wake  Sample Rate (0 to 7) 0 = 1.56Hz, 6.25Hz, 12.5Hz, 50Hz, 100Hz, 200Hz, 400Hz, 800Hz
//
// timeout to sleep [seconds] (0 to 81). if ODR 1.56 (0-162) (not supported)
// firstPin. If true, route to INT1 (default is Pin2)
bool MMA8452Q::setupSleep(bool enable, bool firstPin, uint8_t ovrS, uint8_t ovrW, uint8_t ssr, uint8_t wsr, uint8_t timeout)
{
	if (ovrS > 4 || ovrW > 4 || timeout > 81 || ssr > 3 || wsr > 7)  // wrong values. Bigger values messes registers
		return false;

	// Must be in standby mode to make changes!!!
	// Change to standby if currently in active state
	if (isActive() == true)
		standby();

	uint8_t temp;

	if (enable == true) {
		// set Sleep Sample Rate and Wake Sample Rate
		temp = readRegister(CTRL_REG1) & 0b0111;                        // get CTRL_REG1 for ACTIVE[0], F_READ[1], LNOISE[2] without ssr[7-6] and swr[5-3]
		writeRegister(CTRL_REG1, temp | ssr << 6 | wsr << 2 | 0b0100);  // apply ssr and wsr with lnoise (this limits to 4G)

		// set Sleep bit together with ovr's
		writeRegister(CTRL_REG2, ovrW | ovrS << 3 | 0b0100);   // apply ovr for sleep and wake. Enable bit2 for sleep mode
		// wake up causes 
		temp = readRegister(CTRL_REG3) &  0b10000111;         // store setting without FIFO_GATE [7], IPOL[1], PP_OD[0] (pushpull/open drain) [2 NO VALUE]
		writeRegister(CTRL_REG3, temp | ~(0b10000111));	      // wake with Transient[6], Orientation[5], Tap[4], Motion/Freefall[3]

		// wake up causes 
		temp = readRegister(CTRL_REG3) &  0b10000111;         // store setting without FIFO_GATE [7], IPOL[1], PP_OD[0] (pushpull/open drain) [2 NO VALUE]
		writeRegister(CTRL_REG3, temp | ~(0b10000111));	      // wake with Transient[6], Orientation[5], Tap[4], Motion/Freefall[3]

		// set interrupt. Default mapping is to int2
		temp = readRegister(CTRL_REG4) | 0b10000000;         // read settings and apply wake interrupt
		// wake up causes 
		temp = readRegister(CTRL_REG3) &  0b10000111;         // store setting without FIFO_GATE [7], IPOL[1], PP_OD[0] (pushpull/open drain) [2 NO VALUE]
		writeRegister(CTRL_REG3, temp | ~(0b10000111));	      // wake with Transient[6], Orientation[5], Tap[4], Motion/Freefall[3]
		writeRegister(CTRL_REG4, temp);                      // apply wake interrupt
		if (firstPin == true)
		   temp = readRegister(CTRL_REG5) & 0b01111111;      // read settings and apply wake interrupt
		   writeRegister(CTRL_REG5, temp | 0b10000000);      // route to firstPin

	} else {
		temp = readRegister(CTRL_REG2) & 0b11011011;           // store setting of FIFO_GATE [7], IPOL[1], PP_OD[0] (pushpull/open drain) [2 NO VALUE]
		writeRegister(CTRL_REG2, (temp | ovrW) & 0b10111011);   // apply ovr and disable SLPE (SLEEP) [2], omit RESET bit [6]
	}

		// set timeout
		writeRegister(ASLP_COUNT, timeout);                  // apply timeout

	// Return to active state when done
	// Must be in active state to read data
	active();

	return true; // all OK
	
}

// check if we are sleeping or not
// copy / paste from p. 9 AN4074
// return 2 for sleep, 1 for wake 0 if neither.
uint8_t MMA8452Q::wakeOrSleep(){
	uint8_t temp = readRegister(INT_SOURCE); // determine the source of interrupt
	if ( ( temp &= 0x80) == 0x80){           // set up case statement. We have Auto-sleep flag
		temp = readRegister(SYSMOD);     // Read system mode to clear the interrupt
		if ( temp == 0x02) {             // sleep mode
			return temp;
		} else if ( temp == 0x01 ) {     // Wake mode
			return temp;
		}
	}
	return temp;
}

// Setup Motion event
// return 0 for error
// latch
// axes 1 = X, 3 = X + Y, 7 = X+Y+Z. 2 = only Y, 4 = only Z
void MMA8452Q::setupMotion(bool latch, bool freefall, uint8_t axes, uint8_t threshold, uint8_t debounce)
{
	if ( axes > 7 )			// take care of wrong input
		axes = 7;
	if ( threshold > 127 )		// take care of wrong input. Max is 8G even with full scale of 2G
		threshold = 127;

	// Must be in standby mode to make changes!!!
	// Change to standby if currently in active state
	if (isActive() == true)
		standby();

	/*writeRegister(FF_MT_CFG, latch << 7 | freefall << 6 | axes << 3);
	writeRegister(FF_MT_THS, threshold);
	writeRegister(FF_MT_COUNT, debounce);
	*/

	// EVAL
	writeRegister(FF_MT_CFG, 0x58); // Set motion flag on x and y axes
	writeRegister(FF_MT_THS, 0x84); // Clear debounce counter when condition no longer obtains, set threshold to 0.25 g
	writeRegister(FF_MT_COUNT, 0x8); // Set debounce to 0.08 s at 100 Hz

	// Return to active state when done
	// Must be in active state to read data
	active();
}

// Enable / disable events for interrupts. Last two bits configure polarity and push-pull / OpenDrain
void MMA8452Q::enableEvents(bool sleep, bool transient, bool orientation, bool tap, bool motion)
{
	if (isActive() == true)
		standby();

	// apply interrupts to INT2 or INT1
	uint8_t temp = readRegister(CTRL_REG4) & 0b01000011;         // store settings without our bits (7, 5, 4, 3, 2)
	writeRegister(CTRL_REG4, temp | sleep << 7 | transient << 5 | orientation << 4 | tap << 3 | motion << 2); // apply setting and our values
	writeRegister(CTRL_REG4, readRegister(CTRL_REG4) | 0b01); 			// EVAL: DRDY on INT1
	writeRegister(CTRL_REG5, readRegister(CTRL_REG5) | 0b01); 			// EVAL: DRDY on INT1

	// Return to active state when done
	// Must be in active state to read data
	active();
}

// Select interrupt
// 0 for INT2 (factory default) 1 for INT1
void MMA8452Q::interruptPin(bool sleep, bool transient, bool orientation, bool tap, bool motion, bool polarity, bool openDrain )
{
	if (isActive() == true)
		standby();

	// aplly interrupts to INT2 or INT1
	uint8_t temp = readRegister(CTRL_REG5); 		        // store settings without our bits (7, 5, 4, 3, 2)
	writeRegister(CTRL_REG5, temp | sleep << 7 | transient << 5 | orientation << 4 | tap << 3 | motion << 2);

	// apply pin polarity and openDrain
	temp = readRegister(CTRL_REG3) & 0b00;				    // store settings without IPOL [1] and PP_OD [0]
	writeRegister(CTRL_REG3, temp | polarity << 1 | openDrain); 	    // apply our bits

	// Return to active state when done
	// Must be in active state to read data
	active();
}

// Read the cause of INT event
// 0x08=Tap, 0x10=orientation, 0x04=Motion, 0x20=Transient
uint8_t MMA8452Q::readIRQEvent(){

//	wakeOrSleep();					// check mode and clear the sleep / normal register

	uint8_t temp = readRegister(INT_SOURCE);	// examine the cause of interrupt
	uint8_t reason = 0;

	if (( temp & 0x10)==0x10)			// Orientation is set. Read it.
		reason |= 0x10;
	 if ( (temp & 0x08) == 0x08 ) 			// Tap is set. Read it.
		reason |= 0x08;
	 if ( (temp & 0x20) == 0x20 ) 			// Transient is set. Read it.
		reason |= 0x20;
		temp = readRegister(TRANSIENT_SRC);     // TODO: read data
	 if ( (temp & 0x04) == 0x04 )            	// Motion
		reason |= 0x04;
	 return reason;
}

// SET STANDBY MODE
//	Sets the MMA8452 to standby mode. It must be in standby to change most register settings
void MMA8452Q::standby()
{
	byte c = readRegister(CTRL_REG1);
	writeRegister(CTRL_REG1, c & ~(0x01)); //Clear the active bit to go into standby
}

// SET ACTIVE MODE
//	Sets the MMA8452 to active mode. Needs to be in this mode to output data
void MMA8452Q::active()
{
	byte c = readRegister(CTRL_REG1);
	writeRegister(CTRL_REG1, c | 0x01); //Set the active bit to begin detection
}

// CHECK STATE (ACTIVE or STANDBY)
//	Returns true if in Active State, otherwise return false
bool MMA8452Q::isActive()
{
	byte currentState = readRegister(SYSMOD);
	currentState &= 0b00000011;

	// Wake and Sleep are both active SYSMOD states (pg. 10 datasheet)
	if (currentState == SYSMOD_STANDBY)
		return false;
	return true;
}

// WRITE A SINGLE REGISTER
// 	Write a single byte of data to a register in the MMA8452Q.
void MMA8452Q::writeRegister(MMA8452Q_Register reg, byte data)
{
	writeRegisters(reg, &data, 1);
}

// WRITE MULTIPLE REGISTERS
//	Write an array of "len" bytes ("buffer"), starting at register "reg", and
//	auto-incrmenting to the next.
void MMA8452Q::writeRegisters(MMA8452Q_Register reg, byte *buffer, byte len)
{
	_i2cPort->beginTransmission(_deviceAddress);
	_i2cPort->write(reg);
#ifdef DEBUG // EVAL
		Serial.print(_deviceAddress, HEX);Serial.print(":");Serial.println(buffer[0], HEX);
#endif
	for (int x = 0; x < len; x++)
		_i2cPort->write(buffer[x]);
	_i2cPort->endTransmission(); //Stop transmitting
}

// READ A SINGLE REGISTER
//	Read a byte from the MMA8452Q register "reg".
byte MMA8452Q::readRegister(MMA8452Q_Register reg)
{
#ifdef _VARIANT_ARDUINO_DUE_X_
	_i2cPort->requestFrom((uint8_t)_deviceAddress, (uint8_t)1, (uint32_t)reg, (uint8_t)1, true);
#else
	_i2cPort->beginTransmission(_deviceAddress);
	_i2cPort->write(reg);
	_i2cPort->endTransmission(false); //endTransmission but keep the connection active

	_i2cPort->requestFrom(_deviceAddress, (byte)1); //Ask for 1 byte, once done, bus is released by default
#endif
	if (_i2cPort->available())
	{							 //Wait for the data to come back
#ifdef DEBUG
		uint8_t b = _i2cPort->read();
		Serial.print(_deviceAddress, HEX);Serial.print(":");Serial.println(b, HEX);
		return b;
#endif
		return _i2cPort->read(); //Return this one byte
	}
	else
	{
		return 0;
	}
}

// READ MULTIPLE REGISTERS
//	Read "len" bytes from the MMA8452Q, starting at register "reg". Bytes are stored
//	in "buffer" on exit.
void MMA8452Q::readRegisters(MMA8452Q_Register reg, byte *buffer, byte len)
{
#ifdef _VARIANT_ARDUINO_DUE_X_
	_i2cPort->requestFrom((uint8_t)_deviceAddress, (uint8_t)len, (uint32_t)reg, (uint8_t)1, true);
#else
	_i2cPort->beginTransmission(_deviceAddress);
	_i2cPort->write(reg);
	_i2cPort->endTransmission(false);			//endTransmission but keep the connection active
	_i2cPort->requestFrom(_deviceAddress, len); //Ask for bytes, once done, bus is released by default
#endif
	if (_i2cPort->available() == len)
	{
		for (int x = 0; x < len; x++)
			buffer[x] = _i2cPort->read();
	}
}

// ERASE
// Feel free to modify any values, these are settings that work well for me.
void MMA8452Q::initOld(byte fsr, byte dataRate)
{
  standby();  // Must be in standby to change registers

  // Set up the full scale range to 2, 4, or 8g.
  if ((fsr==2)||(fsr==4)||(fsr==8))
    writeRegister(XYZ_DATA_CFG, fsr >> 2);  
  else
    writeRegister(XYZ_DATA_CFG, 0);

  // Setup the 3 data rate bits, from 0 to 7
  writeRegister(CTRL_REG1, readRegister(CTRL_REG1) & ~(0x38));
  if (dataRate <= 7)
    writeRegister(CTRL_REG1, readRegister(CTRL_REG1) | (dataRate << 3));  
    
// These settings have to do with setting up the sleep mode and should probably be broken up into a separate function
// set Auto-WAKE sample frequency when the device is in sleep mode

     writeRegister(ASLP_COUNT, 0x40 ); // sleep after ~36 seconds of inactivity at 6.25 Hz ODR

     writeRegister(CTRL_REG1, readRegister(CTRL_REG1) & ~(0xC0)); // clear bits 7 and 8
     writeRegister(CTRL_REG1, readRegister(CTRL_REG1) |  (0xC0)); // select 1.56 Hz sleep mode sample frequency for low power

  // set sleep power mode scheme
     writeRegister(CTRL_REG2, readRegister(CTRL_REG2) & ~(0x18)); // clear bits 3 and 4
     writeRegister(CTRL_REG2, readRegister(CTRL_REG2) |  (0x18)); // select low power mode
     
  // Enable auto SLEEP
     writeRegister(CTRL_REG2, readRegister(CTRL_REG2) & ~(0x04)); // clear bit 2
     writeRegister(CTRL_REG2, readRegister(CTRL_REG2) |  (0x04)); // enable auto sleep mode

  // set sleep mode interrupt scheme
     writeRegister(CTRL_REG3, readRegister(CTRL_REG3) & ~(0x3C)); // clear bits 3, 4, 5, and 6
     writeRegister(CTRL_REG3, readRegister(CTRL_REG3) |  (0x3C)); // select wake on transient, orientation change, pulse, or freefall/motion detect
     
   // Enable Auto-SLEEP/WAKE interrupt
     writeRegister(CTRL_REG4, readRegister(CTRL_REG4) & ~(0x80)); // clear bit 7
     writeRegister(CTRL_REG4, readRegister(CTRL_REG4) |  (0x80)); // select  Auto-SLEEP/WAKE interrupt enable
   
  // Set up portrait/landscape registers - 4 steps:
  // 1. Enable P/L
  // 2. Set the back/front angle trigger points (z-lock)
  // 3. Set the threshold/hysteresis angle
  // 4. Set the debouce rate
  // For more info check out this app note: http://cache.freescale.com/files/sensors/doc/app_note/AN4068.pdf
  writeRegister(PL_CFG, 0x40);        // 1. Enable P/L
 // writeRegister(PL_BF_ZCOMP, 0x44); // 2. 29deg z-lock (don't think this register is actually writable)
 // writeRegister(P_L_THS_REG, 0x84); // 3. 45deg thresh, 14deg hyst (don't think this register is writable either)
  writeRegister(PL_COUNT, 0x50);      // 4. debounce counter at 100ms (at 800 hz)

  /* Set up single and double tap - 5 steps:
   1. Set up single and/or double tap detection on each axis individually.
   2. Set the threshold - minimum required acceleration to cause a tap.
   3. Set the time limit - the maximum time that a tap can be above the threshold
   4. Set the pulse latency - the minimum required time between one pulse and the next
   5. Set the second pulse window - maximum allowed time between end of latency and start of second pulse
   for more info check out this app note: http://cache.freescale.com/files/sensors/doc/app_note/AN4072.pdf */
  writeRegister(PULSE_CFG, 0x7F);  // 1. enable single/double taps on all axes
  // writeRegister(PULSE_CFS, 0x55);  // 1. single taps only on all axes
  // writeRegister(PULSE_CFS, 0x6A);  // 1. double taps only on all axes
  writeRegister(PULSE_THSX, 0x04);  // 2. x thresh at 0.25g, multiply the value by 0.0625g/LSB to get the threshold
  writeRegister(PULSE_THSY, 0x04);  // 2. y thresh at 0.25g, multiply the value by 0.0625g/LSB to get the threshold
  writeRegister(PULSE_THSZ, 0x04);  // 2. z thresh at 0.25g, multiply the value by 0.0625g/LSB to get the threshold
  writeRegister(PULSE_TMLT, 0x30);  // 3. 2.55s time limit at 100Hz odr, this is very dependent on data rate, see the app note
  writeRegister(PULSE_LTCY, 0xA0);  // 4. 5.1s 100Hz odr between taps min, this also depends on the data rate
  writeRegister(PULSE_WIND, 0xFF);  // 5. 10.2s (max value)  at 100 Hz between taps max

  // Set up motion detection
  writeRegister(FF_MT_CFG, 0x58); // Set motion flag on x and y axes
  writeRegister(FF_MT_THS, 0x84); // Clear debounce counter when condition no longer obtains, set threshold to 0.25 g
  writeRegister(FF_MT_COUNT, 0x8); // Set debounce to 0.08 s at 100 Hz

  // Set up interrupt 1 and 2
  writeRegister(CTRL_REG3, readRegister(CTRL_REG3) & ~(0x02)); // clear bits 0, 1 
  writeRegister(CTRL_REG3, readRegister(CTRL_REG3) |  (0x02)); // select ACTIVE HIGH, push-pull interrupts
     
 // writeRegister(0x2C, 0x02);  // Active high, push-pull interrupts

  writeRegister(CTRL_REG4, readRegister(CTRL_REG4) & ~(0x1D)); // clear bits 0, 3, and 4
  writeRegister(CTRL_REG4, readRegister(CTRL_REG4) |  (0x1D)); // DRDY, Freefall/Motion, P/L and tap ints enabled
   
  writeRegister(CTRL_REG5, 0x01);  // DRDY on INT1, P/L and taps on INT2

  active();  // Set to active to start reading
}

void MMA8452Q::lowPowerEVAL()
{
  standby();  // Must be in standby to change registers

  writeRegister(CTRL_REG2, 0x1b | 0x04);
  writeRegister(ASLP_COUNT, 0x10);                           //1 count = 320ms
  writeRegister(CTRL_REG4, readRegister(CTRL_REG4) | 0x80);
  writeRegister(CTRL_REG1, 0xC1);

  active();
}

void MMA8452Q::lowPowerAN()
{
  standby();  // Must be in standby to change registers

  uint8_t t = readRegister(CTRL_REG2);
  writeRegister(CTRL_REG2, t | 0x04);
  t = readRegister(CTRL_REG1) & 0xE4;
  //writeRegister(CTRL_REG1, t | 0x1A);				// Wake Hi-Res, Sleep low-power: OK!
  //writeRegister(CTRL_REG1, t | 0x19);				// Wake Low Noise and low-power, Sleep low-power: FAILED
  writeRegister(CTRL_REG1, t | 0x18);				// Wake Normal, Sleep low-power: OK!
  writeRegister(CTRL_REG4, 0x9C);				// Wake events
  writeRegister(CTRL_REG5, 0x80);				// Route Sleep to INT1 others to INT2
  writeRegister(CTRL_REG3, 0x18);				// Wake events

  t = readRegister(XYZ_DATA_CFG) & 0xFC;
  writeRegister(XYZ_DATA_CFG, t);				// 2G

  writeRegister(ASLP_COUNT, 0x10);                           //1 count = 320ms MAX is 0x51 (81)

  active();
}
