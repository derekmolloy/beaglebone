/*
 * BMAAccelerator.cpp
 * Implementation of a class to interface with the Bosch BMA180 3 Axis Accelerometer
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

#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <stropts.h>
#include <stdio.h>
#include "BMA180Accelerometer.h"
#include <iostream>
#include <math.h>
using namespace std;
#define MAX_BUS 64


#define ACC_X_LSB 	0x02
#define ACC_X_MSB 	0x03
#define ACC_Y_LSB 	0x04
#define ACC_Y_MSB 	0x05
#define ACC_Z_LSB 	0x06
#define ACC_Z_MSB 	0x07
#define TEMP	  	0x08  //Temperature
#define RANGE	  	0x35  //bits 3,2,1
#define BANDWIDTH 	0x20  //bits 7,6,5,4
#define MODE_CONFIG 0x30  //bits 1,0


BMA180Accelerometer::BMA180Accelerometer(int bus, int address) {
	I2CBus = bus;
	I2CAddress = address;
	readFullSensorState();
}

void BMA180Accelerometer::calculatePitchAndRoll() {

	double accelerationXSquared = this->accelerationX * this->accelerationX;
	double accelerationYSquared = this->accelerationY * this->accelerationY;
	double accelerationZSquared = this->accelerationZ * this->accelerationZ;
	this->pitch = 180 * atan(accelerationX/sqrt(accelerationYSquared + accelerationZSquared))/M_PI;
	this->roll = 180 * atan(accelerationY/sqrt(accelerationXSquared + accelerationZSquared))/M_PI;
}

int BMA180Accelerometer::readFullSensorState(){

   //cout << "Starting BMA180 I2C sensor state read" << endl;
    char namebuf[MAX_BUS];
   	snprintf(namebuf, sizeof(namebuf), "/dev/i2c-%d", I2CBus);
    int file;
    if ((file = open(namebuf, O_RDWR)) < 0){
            cout << "Failed to open BMA180 Sensor on " << namebuf << " I2C Bus" << endl;
            return(1);
    }
    if (ioctl(file, I2C_SLAVE, I2CAddress) < 0){
            cout << "I2C_SLAVE address " << I2CAddress << " failed..." << endl;
            return(2);
    }

    // According to the BMA180 datasheet on page 59, you need to send the first address
    // in write mode and then a stop/start condition is issued. Data bytes are
    // transferred with automatic address increment.
    char buf[1] = { 0x00 };
    if(write(file, buf, 1) !=1){
    	cout << "Failed to Reset Address in readFullSensorState() " << endl;
    }

    int numberBytes = BMA180_I2C_BUFFER;
    int bytesRead = read(file, this->dataBuffer, numberBytes);
    if (bytesRead == -1){
    	cout << "Failure to read Byte Stream in readFullSensorState()" << endl;
    }
    close(file);

    if (this->dataBuffer[0]!=0x03){
    	cout << "MAJOR FAILURE: DATA WITH BMA180 HAS LOST SYNC!" << endl;
    }

   // cout << "Number of bytes read was " << bytesRead << endl;
   /// for (int i=0; i<8; i++){
    //       printf("Byte %02d is 0x%02x\n", i, dataBuffer[i]);
    //}
    //cout << "Closing BMA180 I2C sensor state read" << endl;

    this->accelerationX = convertAcceleration(ACC_X_MSB, ACC_X_LSB);
    this->accelerationY = convertAcceleration(ACC_Y_MSB, ACC_Y_LSB);
    this->accelerationZ = convertAcceleration(ACC_Z_MSB, ACC_Z_LSB);
    this->calculatePitchAndRoll();
    //cout << "Pitch:" << this->getPitch() << "   Roll:" << this->getRoll() <<  endl;
    return 0;
}

int BMA180Accelerometer::convertAcceleration(int msb_reg_addr, int lsb_reg_addr){
//	cout << "Converting " << (int) dataBuffer[msb_reg_addr] << " and " << (int) dataBuffer[lsb_reg_addr] << endl;;
	short temp = dataBuffer[msb_reg_addr];
	temp = (temp<<8) | dataBuffer[lsb_reg_addr];
	temp = temp>>2;
	temp = ~temp + 1;
//	cout << "The X acceleration is " << temp << endl;
	return temp;
}

void BMA180Accelerometer::displayMode(int iterations){

	for(int i=0; i<iterations; i++){
		this->readFullSensorState();
		printf("Rotation (%d, %d, %d)", accelerationX, accelerationY, accelerationZ);
	}
}

//  Temperature in 2's complement has a resolution of 0.5K/LSB
//  80h is lowest temp - approx -40C and 00000010 is 25C in 2's complement
//  this value is offset at room temperature - 25C and accurate to 0.5K

float BMA180Accelerometer::getTemperature(){

	int offset = -40;  // -40 degrees C
	this->readFullSensorState();
	char temp = dataBuffer[TEMP]; // = -80C 0b10000000  0b00000010; = +25C
	//char temp = this->readI2CDeviceByte(TEMP);
	//this->readFullSensorState();
    //char temp = dataBuffer[TEMP];
	int temperature;
	if(temp&0x80)	{
		temp = ~temp + 0b00000001;
		temperature = 128 - temp;
	}
	else {
		temperature = 128 + temp;
	}
	this->temperature = offset + ((float)temperature*0.5f);
	//cout << "The temperature is " << this->temperature << endl;
	//int temp_off = dataBuffer[0x37]>>1;
	//cout << "Temperature offset raw value is: " << temp_off << endl;
	return this->temperature;
}

BMA180_RANGE BMA180Accelerometer::getRange(){
	this->readFullSensorState();
	char temp = dataBuffer[RANGE];
	//char temp = this->readI2CDeviceByte(RANGE);  //bits 3,2,1
	temp = temp & 0b00001110;
	temp = temp>>1;
	//cout << "The current range is: " << (int)temp << endl;
	this->range = (BMA180_RANGE) temp;
	return this->range;
}

int BMA180Accelerometer::setRange(BMA180_RANGE range){
	//char current = this->readI2CDeviceByte(RANGE);  //bits 3,2,1
	this->readFullSensorState();
	char current = dataBuffer[RANGE];
	char temp = range << 1; //move value into bits 3,2,1
	current = current & 0b11110001; //clear the current bits 3,2,1
	temp = current | temp;
	if(this->writeI2CDeviceByte(RANGE, temp)!=0){
		cout << "Failure to update RANGE value" << endl;
		return 1;
	}
	return 0;
}

BMA180_BANDWIDTH BMA180Accelerometer::getBandwidth(){
	this->readFullSensorState();
	char temp = dataBuffer[BANDWIDTH];   //bits 7->4
	//char temp = this->readI2CDeviceByte(BANDWIDTH);  //bits 7,6,5,4
//	cout << "The value of bandwidth returned is: " << (int)temp << endl;
	temp = temp & 0b11110000;
	temp = temp>>4;
//	cout << "The current bandwidth is: " << (int)temp << endl;
	this->bandwidth = (BMA180_BANDWIDTH) temp;
	return this->bandwidth;
}

int BMA180Accelerometer::setBandwidth(BMA180_BANDWIDTH bandwidth){
	//char current = this->readI2CDeviceByte(BANDWIDTH);  //bits 7,6,5,4
	this->readFullSensorState();
    char current = dataBuffer[BANDWIDTH];   //bits 7->4
	char temp = bandwidth << 4; //move value into bits 7,6,5,4
	current = current & 0b00001111; //clear the current bits 7,6,5,4
	temp = current | temp;
	if(this->writeI2CDeviceByte(BANDWIDTH, temp)!=0){
		cout << "Failure to update BANDWIDTH value" << endl;
		return 1;
	}
	return 0;
}

BMA180_MODECONFIG BMA180Accelerometer::getModeConfig(){
	//char temp = dataBuffer[MODE_CONFIG];   //bits 1,0
	//char temp = this->readI2CDeviceByte(MODE_CONFIG);  //bits 1,0
	this->readFullSensorState();
    char temp = dataBuffer[MODE_CONFIG];
	temp = temp & 0b00000011;
	//cout << "The current mode config is: " << (int)temp << endl;
	this->modeConfig = (BMA180_MODECONFIG) temp;
	return this->modeConfig;
}

int BMA180Accelerometer::writeI2CDeviceByte(char address, char value){

    cout << "Starting BMA180 I2C sensor state write" << endl;
    char namebuf[MAX_BUS];
   	snprintf(namebuf, sizeof(namebuf), "/dev/i2c-%d", I2CBus);
    int file;
    if ((file = open(namebuf, O_RDWR)) < 0){
            cout << "Failed to open BMA180 Sensor on " << namebuf << " I2C Bus" << endl;
            return(1);
    }
    if (ioctl(file, I2C_SLAVE, I2CAddress) < 0){
            cout << "I2C_SLAVE address " << I2CAddress << " failed..." << endl;
            return(2);
    }

    // need to set the ctrl_reg0 ee_w bit. With that set the image registers change properly.
    // need to do this or can't write to 20H ... 3Bh
    // Very Important... wrote a 0x10 to 0x0D and it worked!!!
    //   char buf[2];
    //     buf[0] = BANDWIDTH;
    //     buf[1] = 0x28;
    //     buf[2] = 0x65;
    //  if ( write(file,buf,2) != 2) {
    //	  cout << "Failure to write values to I2C Device " << endl;
    //  }

    char buffer[2];
    	buffer[0] = address;
    	buffer[1] = value;
    if ( write(file, buffer, 2) != 2) {
        cout << "Failure to write values to I2C Device address." << endl;
        return(3);
    }
    close(file);
    cout << "Finished BMA180 I2C sensor state write" << endl;
    return 0;
}

/*
char BMA180Accelerometer::readI2CDeviceByte(char address){

  //  cout << "Starting BMA180 I2C sensor state byte read" << endl;
    char namebuf[MAX_BUS];
   	snprintf(namebuf, sizeof(namebuf), "/dev/i2c-%d", I2CBus);
    int file;
    if ((file = open(namebuf, O_RDWR)) < 0){
            cout << "Failed to open BMA180 Sensor on " << namebuf << " I2C Bus" << endl;
            return(1);
    }
    if (ioctl(file, I2C_SLAVE, I2CAddress) < 0){
            cout << "I2C_SLAVE address " << I2CAddress << " failed..." << endl;
            return(2);
    }

    // According to the BMA180 datasheet on page 59, you need to send the first address
    // in write mode and then a stop/start condition is issued. Data bytes are
    // transferred with automatic address increment.
    char buf[1] = { 0x00 };
    if(write(file, buf, 1) !=1){
    	cout << "Failed to Reset Address in readFullSensorState() " << endl;
    }

    char buffer[1];
    	buffer[0] = address;
    if ( read(file, buffer, 2) != 2) {
        cout << "Failure to read value from I2C Device address." << endl;
    }
    close(file);
   // cout << "Finished BMA180 I2C sensor state read" << endl;
    return buffer[0];
}*/


BMA180Accelerometer::~BMA180Accelerometer() {
	// TODO Auto-generated destructor stub
}

