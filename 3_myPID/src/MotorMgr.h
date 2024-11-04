/*
 * MotorMgr.h
 *
 *  Created on: 28 May 2023
 *      Author: jondurrant
 */

#ifndef ENCODER_SRC_MOTORMGR_H_
#define ENCODER_SRC_MOTORMGR_H_

#include <pico/stdlib.h>
#include "stdio.h"
#include "hardware/pwm.h"
#include "GPIOInputMgr.h"
#include "GPIOObserver.h"



class MotorMgr : public GPIOObserver {
public:
    /***
	 * Constructor
	 * @param gpCW - GP Pad for PWM for Clockwise
	 * @param gpCCW - GP Pad for PWM for Clockwise
	 * @param gpA - GP Pad for ROTENC A
	 * @param gpB - GP Pad for ROTENC AB
	 */
    MotorMgr(uint8_t gpCW, uint8_t gpCCW, uint8_t gpA, uint8_t gpB);
    ~MotorMgr();

    /***
	 * Set throttle as a percentage
	 * @param percent: 0.0 < p <= 1.0
	 * @param cw - true of clockwise
	 */
	void setThrottle(float percent, bool cw);

    void updateVelocities(float time); 
	void updateVelocity_method2(bool cw); 


	float getRPS1();
	float getRPS2();


	int16_t getPos();

	bool getDirection();  // Returns true for CW, false for CCW


	/***
	 * Get the throttle speed
	 * @return 0.0 < o <= 1.0
	 */
	float getThrottle();


protected:
    /***
	 * Handle GPIO event on teh ROTENC
	 * @param gpio
	 * @param events
	 */
	virtual void handleGPIO(uint gpio, uint32_t events);


	bool xCW = true;
    

private:

	uint8_t xGPCW=0;
	uint8_t xGPCCW=0;
	uint8_t xGPA;
	uint8_t xGPB;

	float xThrottle = 0.0;	
    
	float xFiltRPS1 = 0.0;
    float xFiltRPS2 = 0.0;
	float xFiltRPS1_Prev = 0.0;
    float xFiltRPS2_Prev = 0.0;

	// variables for method 1 
	// Use the "volatile" directive for variables when the variables are
	// used in ISR and main
// used in an interrupt
	volatile int32_t xPos = 0;
    int32_t posPrev = 0;
    uint32_t xLastTime1 = 0;
	volatile float xActRPS1 = 0.0;  //rev per second

    //variables for method 2
	//bool xPosReset = false;
    uint32_t xLastTime2 = 0;
	volatile float xActRPS2 = 0.0;
            

	// Number of ticks to track in total rotation
	float xNumTicks=211.8;

	// Last position of switches
	uint8_t xLast=0;

	// Count way through the signal
	int8_t xCount=0;

	bool xDirection;  // true for CW, false for CCW


	//These are lookups for the sequence to see in CW and CCW motion
	uint8_t xRotEncCW[4] ={2,0,3,1};
	uint8_t xRotEncCCW[4]={1,3,0,2};
	
};

#endif /* ENCODER_SRC_MOTORMGR_H_ */