/*
 * EasyDriver.cpp
 *
 * Copyright Derek Molloy, School of Electronic Engineering, Dublin City University
 * www.eeng.dcu.ie/~molloyd/
 *
 * YouTube Channel: http://www.youtube.com/derekmolloydcu/
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

#include "EasyDriver.h"
#include <iostream>
#include <unistd.h>
using namespace std;

EasyDriver::EasyDriver(int gpio_MS1, int gpio_MS2, int gpio_STEP, int gpio_SLP,
					   int gpio_DIR, int speedRPM, int stepsPerRevolution){

	this->gpio_MS1  = gpio_MS1;
	this->gpio_MS2  = gpio_MS2;
	this->gpio_STEP = gpio_STEP;
	this->gpio_SLP  = gpio_SLP;
	this->gpio_DIR  = gpio_DIR;

	gpio_export(this->gpio_MS1);
	gpio_set_dir(this->gpio_MS1, OUTPUT_PIN);
	gpio_export(this->gpio_MS2);
	gpio_set_dir(this->gpio_MS2, OUTPUT_PIN);
	gpio_export(this->gpio_STEP);
	gpio_set_dir(this->gpio_STEP, OUTPUT_PIN);
	gpio_export(this->gpio_SLP);
	gpio_set_dir(this->gpio_SLP, OUTPUT_PIN);
	gpio_export(this->gpio_DIR);
	gpio_set_dir(this->gpio_DIR, OUTPUT_PIN);

	// default to clockwise direction
	clockwise = true;
	// default to full stepping
	setStepMode(STEP_FULL);
	// the default number of steps per revolution
	setStepsPerRevolution(stepsPerRevolution);
	// the default speed in rpm
	setSpeed(speedRPM);
	//wake up the controller - holding torque..
	wake();
}

void EasyDriver::setStepMode(STEP_MODE mode) {
	this->stepMode = mode;
	switch(stepMode){
	case STEP_FULL:
		gpio_set_value(this->gpio_MS1, LOW);
		gpio_set_value(this->gpio_MS2, LOW);
		this->delayFactor = 1;
		break;
	case STEP_HALF:
		gpio_set_value(this->gpio_MS1, HIGH);
		gpio_set_value(this->gpio_MS2, LOW);
		this->delayFactor = 2;
		break;
	case STEP_QUARTER:
		gpio_set_value(this->gpio_MS1, LOW);
		gpio_set_value(this->gpio_MS2, HIGH);
		this->delayFactor = 4;
		break;
	case STEP_EIGHT:
		gpio_set_value(this->gpio_MS1, HIGH);
		gpio_set_value(this->gpio_MS2, HIGH);
		this->delayFactor = 8;
		break;
	}
}

void EasyDriver::setSpeed(float rpm) {
	this->speed = rpm;
	float delayPerSec = (60/rpm)/stepsPerRevolution;    // delay per step in seconds
	this->uSecDelay = (int)(delayPerSec * 1000 * 1000); // in microseconds
}

void EasyDriver::step(int numberOfSteps){
	cout << "Doing "<< numberOfSteps << " steps and going to sleep for " << uSecDelay/delayFactor << "uS\n";
	int sleepDelay = uSecDelay/delayFactor;
	if(numberOfSteps>=0) {
		if(clockwise) gpio_set_value(this->gpio_DIR, LOW);
		else gpio_set_value(this->gpio_DIR, HIGH);
		for(int i=0; i<numberOfSteps; i++){
			gpio_set_value(this->gpio_STEP, LOW);
			gpio_set_value(this->gpio_STEP, HIGH);
			usleep(sleepDelay);
		}
	}
	else { // going in reverse (numberOfSteps is negative)
		if(clockwise) gpio_set_value(this->gpio_DIR, HIGH);
		else gpio_set_value(this->gpio_DIR, LOW);
		for(int i=numberOfSteps; i<=0; i++){
			gpio_set_value(this->gpio_STEP, LOW);
			gpio_set_value(this->gpio_STEP, HIGH);
			usleep(sleepDelay);
		}
	}
}

void EasyDriver::rotate(int degrees){
	float degreesPerStep = 360.0f/getStepsPerRevolution();
	int numberOfSteps = degrees/degreesPerStep;
	cout << "The number of steps is " << numberOfSteps << endl;
	cout << "The delay factor is " << delayFactor << endl;
	step(numberOfSteps*delayFactor);
}

EasyDriver::~EasyDriver() {
	gpio_unexport(this->gpio_MS1);
	gpio_unexport(this->gpio_MS2);
	gpio_unexport(this->gpio_STEP);
	gpio_unexport(this->gpio_SLP);
	gpio_unexport(this->gpio_DIR);
}

