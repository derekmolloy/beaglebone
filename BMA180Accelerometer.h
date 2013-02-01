/*
 * BMAAccelerator.h
 * Definition of a class to interface with the Bosch BMA180 3 Axis Accelerometer
 * over the I2C bus
 *
 * Copyright Derek Molloy, School of Electronic Engineering, Dublin City University
 * www.eeng.dcu.ie/~molloyd/
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL I
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef BMA180ACCELEROMETER_H_
#define BMA180ACCELEROMETER_H_
#define BMA180_I2C_BUFFER 0x80

enum BMA180_RANGE {
	PLUSMINUS_1_G 		= 0,
	PLUSMINUS_1POINT5_G = 1,
	PLUSMINUS_2G 		= 2,
	PLUSMINUS_3G 		= 3,
	PLUSMINUS_4G 		= 4,
	PLUSMINUS_8G 		= 5,
	PLUSMINUS_16G 		= 6
};

enum BMA180_BANDWIDTH {
	BW_10HZ 	= 0,
	BW_20HZ 	= 1,
	BW_40HZ 	= 2,
	BW_75HZ 	= 3,
	BW_150HZ 	= 4,
	BW_300HZ 	= 5,
	BW_600HZ 	= 6,
	BW_12OOHZ 	= 7,
	BW_HIGHPASS = 8,
	BW_BANDPASS = 9
};

enum BMA180_MODECONFIG {
	MODE_LOW_NOISE = 0,
	MODE_LOW_POWER = 3
};

class BMA180Accelerometer {

private:
	int I2CBus, I2CAddress;
	char dataBuffer[BMA180_I2C_BUFFER];

	int accelerationX;
	int accelerationY;
	int accelerationZ;

	double pitch;  //in degrees
	double roll;   //in degrees

	float temperature; //accurate to 0.5C
	BMA180_RANGE range;
	BMA180_BANDWIDTH bandwidth;
	BMA180_MODECONFIG modeConfig;

	int  convertAcceleration(int msb_addr, int lsb_addr);
	int  writeI2CDeviceByte(char address, char value);
	//char readI2CDeviceByte(char address);
	void calculatePitchAndRoll();

public:
	BMA180Accelerometer(int bus, int address);
	void displayMode(int iterations);

	int  readFullSensorState();
	// The following do physical reads and writes of the sensors
	int setRange(BMA180_RANGE range);
	BMA180_RANGE getRange();
	int setBandwidth(BMA180_BANDWIDTH bandwidth);
	BMA180_BANDWIDTH getBandwidth();
	int setModeConfig(BMA180_MODECONFIG mode);
	BMA180_MODECONFIG getModeConfig();
	float getTemperature();

	int getAccelerationX() { return accelerationX; }
	int getAccelerationY() { return accelerationY; }
	int getAccelerationZ() { return accelerationZ; }

	float getPitch() { return pitch; }  // in degrees
	float getRoll() { return roll; }  // in degrees

	virtual ~BMA180Accelerometer();
};

#endif /* BMA180ACCELEROMETER_H_ */
