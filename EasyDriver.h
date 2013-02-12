/*
 * EasyDriver.h
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

#ifndef EASYDRIVER_H_
#define EASYDRIVER_H_

#include "SimpleGPIO.h"

enum STEP_MODE {
	STEP_FULL,
	STEP_HALF,
	STEP_QUARTER,
	STEP_EIGHT
};

class EasyDriver {

	private:
		// The GPIO pins MS1, MS2 (Microstepping options), STEP (The low->high step)
		// SLP (Sleep - active low) and DIR (Direction)
		int 			gpio_MS1, gpio_MS2, gpio_STEP, gpio_SLP, gpio_DIR;
		unsigned int 	uSecDelay;
		bool 			clockwise;
		int 			delayFactor;  // keep constant rpm even with microstepping

	protected:
		STEP_MODE 	stepMode;
		float 		speed;
		int   		stepsPerRevolution;
		bool  		asleep;

	public:
		EasyDriver(int gpio_MS1, int gpio_MS2, int gpio_STEP, int gpio_SLP, int gpio_DIR,
				   int speedRPM = 60, int stepsPerRevolution = 200);

		void step(int numberOfSteps = 1); // defaults to 1 if no value is passed
		void rotate(int degrees);
		void reverseDirection() { clockwise = !clockwise; }
		void setStepMode(STEP_MODE mode);
		STEP_MODE getStepMode() { return stepMode; }

		void  setSpeed(float rpm);
		float getSpeed() { return speed; }

		void setStepsPerRevolution(int steps) { stepsPerRevolution = steps; }
		int  getStepsPerRevolution() { return stepsPerRevolution; }

		void sleep()    { asleep = true;  gpio_set_value(this->gpio_SLP, LOW);  }
		void wake()     { asleep = false; gpio_set_value(this->gpio_SLP, HIGH); }
		bool isAsleep() { return asleep; }

		virtual ~EasyDriver();  // unexport the gpios
};

#endif /* EASYDRIVER_H_ */
