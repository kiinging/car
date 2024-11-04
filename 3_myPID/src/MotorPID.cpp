#include "MotorPID.h"
#include "pico/stdlib.h"

MotorPID::MotorPID(uint8_t gpCW, uint8_t gpCCW, uint8_t gpA, uint8_t gpB, int method)
    : MotorMgr(gpCW, gpCCW, gpA, gpB), method_(method), xKP(0), xKI(0), xKD(0), xCumErr(0), xLastErr(0), xTargetRPS(0) {}



void MotorPID::setSpeedRPS(float rps, bool cw) {
    xTargetRPS = rps;  // Set the target speed
    xCW = cw;
    xCumErr = 0.0;  // Reset integral term
    xLastErr = 0.0;  // Reset previous error
}



void MotorPID::configPID(float kp, float ki, float kd) {
    xKP = kp;
    xKI = ki;
    xKD = kd;
}


void MotorPID::doPID() {
    float pv;  // Process variable (current RPS)
    
    
    // Select between RPS calculation methods
    if (method_ == 1) {
        pv = getRPS1();  // Use Method 1 for RPS calculation
    } else {
        pv = getRPS2();  // Use Method 2 for RPS calculation
    }

    float error = xTargetRPS - pv;  // Calculate the error
    xCumErr += error;  // Accumulate the integral term
    float derivative = error - xLastErr;  // Calculate the derivative term

    // PID formula: output = Kp * error + Ki * integral + Kd * derivative
    float output = xKP * error + xKI * xCumErr + xKD * derivative;
    output = output / 15; //15 rps is the maximum speed.

    float t = getThrottle() + output;

    // Set the motor throttle based on the PID output
    if (t > 1.0) {
        t = 1.0;  // Limit to max throttle
    } else if (t < 0) {
        t = 0;  // No reverse throttle here, set to 0
    }

    setThrottle(t, xCW);  // Set the motor throttle
    xLastErr = error;  // Store the error for the next cycle
 

 //   printf("%.2f\n",pv);
}
