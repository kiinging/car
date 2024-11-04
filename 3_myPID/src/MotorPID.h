#ifndef MOTORPID_H_
#define MOTORPID_H_

#include "MotorMgr.h"

class MotorPID : public MotorMgr {
public:
    /***
     * Constructor
     * @param gpCW - GPIO Pin for Clockwise control
     * @param gpCCW - GPIO Pin for Counterclockwise control
     * @param gpA - GPIO Pin for Encoder A
     * @param gpB - GPIO Pin for Encoder B
     * @param method - 1 for Method 1, 2 for Method 2
     */
    MotorPID(uint8_t gpCW, uint8_t gpCCW, uint8_t gpA, uint8_t gpB, int method = 1);

    /***
     * Configure the PID constants
     * @param kp - Proportional gain
     * @param ki - Integral gain
     * @param kd - Derivative gain
     */
    void configPID(float kp, float ki, float kd);

    /***
     * Set the target speed in RPS and the direction
     * @param rps - Setpoint for speed (revolutions per second)
     * @param cw - Direction (true for clockwise, false for counterclockwise)
     */
    void setSpeedRPS(float rps, bool cw);

    void doPID(); // Internal PID calculation method

protected:
    float xTargetRPS;   // Desired speed (RPS)
private:
    float xPrevError;
    float xKP, xKI, xKD; // PID constants
    float xCumErr;     // Accumulated integral value
    float xLastErr;    // Previous error value for derivative calculation
  
    int method_;         // Method for RPS calculation (1 or 2)

    
};

#endif /* MOTORPID_H_ */
