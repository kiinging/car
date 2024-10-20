/*
 * MotorMgr.cpp
 *
 *  Created on: 28 May 2023
 *      Author: jondurrant
 */

#include "MotorMgr.h"
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

}

MotorMgr::~MotorMgr() {
	// TODO Auto-generated destructor stub
}


void MotorMgr::setThrottle(float percent, bool cw) {
    xThrottle = percent;
    xCW = cw;

    if (xThrottle < 0) { xThrottle = 0.0;}

    if (xThrottle == 0.0) {
        // xActRPM1 = 0.0;
        // xActRPM2 = 0.0;
        // xLastTime1 = 0;
        // xLastTime2 = 0;
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
    // Your existing encoder logic
    int b = gpio_get(xGPB);
    int increment = (b > 0) ? 1 : -1;
    xPos += increment;

    // Use Method 2 for velocity calculation if selected
    // rev per second is used (bcoz of easy to debug the motor speed)
    uint32_t now = to_ms_since_boot(get_absolute_time());
    uint32_t ms = now - xLastTime2; //ms per i tick
    
    //assume motor has stopped.
    if (ms > 250){
        xActRPS2 = 0.0;
        xLastTime2 = 0;
    }

    if (xLastTime2 != 0) {
        float rps = 1000.0 / (float)ms;
        rps = rps / xNumTicks;  // Convert to Rev per second
        xActRPS2 = rps * increment;     // Apply direction to rev per second
    //    xMvAvgRPS2 = (rps * 1.0 + xMvAvgRPS2 * 3.0) / 4.0;  // Moving average of RPM
    }
    xLastTime2 = now;
    
}


//Use Method 1 to calculate velocity
void MotorMgr::updateVelocity_method1() {       
    uint32_t now  = to_ms_since_boot(get_absolute_time());
    uint32_t ms = (now - xLastTime1) ; //ms per one slot
    int16_t ticks;
    
     // Handle overflow when xPos wraps from INT16_MAX (32767) to INT16_MIN (-32768)
    if (xPos < 0 && posPrev > 0 && (posPrev - xPos) > 32000) {
        // Overflow occurred, adjust the tick difference
        ticks = (INT16_MAX - posPrev) + (xPos - INT16_MIN) + 1;
    } 
    // Handle underflow when xPos wraps from INT16_MIN (-32768) to INT16_MAX (32767)
    else if (xPos > 0 && posPrev < 0 && (xPos - posPrev) > 32000) {
        // Underflow occurred, adjust the tick difference
        ticks = (INT16_MIN - posPrev) + (xPos - INT16_MAX) - 1;
    } 
    else {
        // Normal case, no overflow or underflow
        ticks = xPos - posPrev;
    }
    
    float rps = 1000.0 / (float)ms;
    rps = ticks * rps / xNumTicks; // rev per second
    xActRPS1 = rps;

      // Low-pass filter (25 Hz cutoff)
    xFiltRPS1 = 0.854*xFiltRPS1 + 0.0728*xActRPS1 + 0.0728*xFiltRPS1_Prev;
    xFiltRPS1_Prev = xFiltRPS1;
    xFiltRPS2 = 0.854*xFiltRPS2 + 0.0728*xActRPS2 + 0.0728*xFiltRPS2_Prev;
    xFiltRPS2_Prev = xFiltRPS2;

    // Send the filtered values over serial
    printf("%.2f,%.2f\n", xFiltRPS1, xFiltRPS2);
 
    posPrev = xPos;
    xLastTime1 = now;
    
}

