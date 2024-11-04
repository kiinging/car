/*
 * MotorMgr.cpp
 *
 *  Created on: 28 May 2023
 *      Author: jondurrant
 */
// Note: i have tried using single rising edge of a single encoder (encoder A) 
// to trigger the calculation. It failed. Not sure why. (21 Oct 2024)
//
// Now, i go back to the original one: using both rising edge and falling edge
// to trigger the calculation. It works.

#include "MotorMgr.h"
#include "pico/sync.h"

#include <math.h>

MotorMgr::MotorMgr(uint8_t gpCW, uint8_t gpCCW, uint8_t gpA, uint8_t gpB)
    : xGPCW(gpCW), xGPCCW(gpCCW), xGPA(gpA), xGPB(gpB) {
    
    gpio_init(xGPCW);
	gpio_set_function(xGPCW, GPIO_FUNC_PWM);
	pwm_set_gpio_level(xGPCW, 0);
	uint slice_num = pwm_gpio_to_slice_num(xGPCW);
	pwm_set_enabled(slice_num, true);

	gpio_init(xGPCCW);
	gpio_set_function(xGPCCW, GPIO_FUNC_PWM);
	pwm_set_gpio_level(xGPCCW, 0);
	slice_num = pwm_gpio_to_slice_num(xGPCCW);
	pwm_set_enabled(slice_num, true);

    GPIOInputMgr::getMgr()->addObserver(xGPA, this);
    GPIOInputMgr::getMgr()->addObserver(xGPB, this);


}

MotorMgr::~MotorMgr() {
	// TODO Auto-generated destructor stub
}


void MotorMgr::setThrottle(float percent, bool cw) {
    xThrottle = percent;
    xCW = cw;

    if (xThrottle < 0) { xThrottle = 0.0;}

    if (xThrottle == 0.0) {
        xActRPS2 = 0.0;  //reset when motor stops.
        xLastTime2 = 0;
        pwm_set_gpio_level(xGPCW, 0);
        pwm_set_gpio_level(xGPCCW, 0);
        return;
    }

    if (xThrottle > 1.0) { xThrottle = 1.0; }

    int pwm = (int)((float)(0xffff) * xThrottle);
    if (cw) {
        pwm_set_gpio_level(xGPCCW, 0);
        pwm_set_gpio_level(xGPCW, pwm);
    } else {
        pwm_set_gpio_level(xGPCW, 0);
        pwm_set_gpio_level(xGPCCW, pwm);
    }
}

void MotorMgr::handleGPIO(uint gpio, uint32_t events) {
    uint8_t c;  // Current state of encoder inputs
    c = gpio_get(xGPA);  // Read encoder A
    c = c << 1;
    c = (gpio_get(xGPB)) | c;  // Combine with encoder B to form a 2-bit state

     // Handle clockwise (CW) rotation
    if (xRotEncCW[xLast] == c) {
        xCount++;  // Increment the count for CW movement

        // Update position if xCount crosses the threshold
        if (xCount > 3) {
            xPos++;  // Increment position for CW rotation

            // // Check for overflow and reset xPos if necessary
            // if (xPos == INT32_MAX) {
            //     xPos = 0;  // Reset position to 0
            // } 
            updateVelocity_method2(true);  // Trigger method 2
            xDirection = 1; // indicate CW
            xCount = 0;  // Reset count after handling rotation
        }

        xLast = c;  // Store current state as the last state
    }

    // Handle counterclockwise (CCW) rotation
    if (xRotEncCCW[xLast] == c) {
        xCount--;  // Decrement the count for CCW movement

        // Update position if xCount crosses the threshold
        if (xCount < -3) {
            xPos--;  // Decrement position for CCW rotation

            // Check for underflow and reset xPos if necessary
            // if (xPos == INT32_MIN) {
            //     xPos = 0;  // Reset position to 0
            // } 
            updateVelocity_method2(false);  // Trigger method 2
            xDirection = 0; // indicate CCW
            xCount = 0;  // Reset count after handling rotation
        }

        xLast = c;  // Store current state as the last state
    }
}

void MotorMgr::updateVelocity_method2(bool cw){
    uint32_t now = to_us_since_boot(get_absolute_time());
    if (xLastTime2 != 0) {
        uint32_t us = now - xLastTime2;
        float rps = 1000000.0 / (float)us;
        rps = rps / xNumTicks;  // Convert to Rev per second

        // Apply direction: positive for clockwise, negative for counterclockwise
        xActRPS2 = (cw) ? rps : -rps;
    }
    xLastTime2 = now;
}



//Use Method 1 to calculate velocity
void MotorMgr::updateVelocity_method1(float ms) {  
    int32_t xPos_1;  
    float rps_2;
    // Disable interrupts before entering critical section  
    uint32_t save = save_and_disable_interrupts();  // ATOMIC BLOCK START
    xPos_1 = xPos;
    rps_2 = xActRPS2;
    // Enable interrupts again after exiting the critical section
    restore_interrupts(save);  // ATOMIC BLOCK END


    // Calculate Revolutions Per Second (RPS)
    float rps = 1000.0 / ms; // Time-based RPS calculation
    rps = rps * ((float)(xPos_1-posPrev)) / xNumTicks;  // ticks * (RPS per tick)
    xActRPS1 = rps;                 // Store the actual RPS
    
    posPrev = xPos_1;
      // Low-pass filter (25 Hz cutoff)
    xFiltRPS1 = 0.854*xFiltRPS1 + 0.0728*xActRPS1 + 0.0728*xFiltRPS1_Prev;
    xFiltRPS1_Prev = xFiltRPS1;
    xFiltRPS2 = 0.854*xFiltRPS2 + 0.0728*rps_2 + 0.0728*xFiltRPS2_Prev;
    xFiltRPS2_Prev = xFiltRPS2;

    // Send the filtered values over serial
    printf("%.2f,%.2f\n", xFiltRPS1, xFiltRPS2);
 
    
    // xLastTime1 = now;
    
}

float MotorMgr::getRPS1(){
    return xActRPS1;
}

float MotorMgr::getRPS2(){
    return xActRPS2;
}

int16_t MotorMgr::getPos(){
    return xPos;
}

// Method to get the direction (CW or CCW)
bool MotorMgr::getDirection() {
    return xDirection;  // true if CW, false if CCW
}