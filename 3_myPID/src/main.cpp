///home/pizza/ros2_projects/jondurrant/DDD-Exp/3_myPID/build/src/DDD.elf
// https://www.youtube.com/watch?v=HRaZLCBFVDE
//https://github.com/curiores/ArduinoTutorials/blob/main/SpeedControl/SpeedControl/SpeedControl.ino
#include <cstdio> // Add this include
#include "pico/stdlib.h"
#include "MotorMgr.h"
#include "MotorPID.h"

// NumTick = 211.8 


// Left Motor connects to TB6612 A inputs
#define LEFT_PWR_CW     12
#define LEFT_PWR_CCW    13
#define LEFT_ROTENC_A   6  
#define LEFT_ROTENC_B   7  

// Right Motor connects to TB6612 B inputs
#define RIGHT_PWR_CW    19
#define RIGHT_PWR_CCW   18
#define RIGHT_ROTENC_A  8 
#define RIGHT_ROTENC_B  9 

// PID constants
#define KP  0.021 // 0.02
#define KI  0.000005
#define KD  0.0000009


int main( void ){
    stdio_init_all();
    sleep_ms(5000);
    printf("GO\n");

    MotorPID leftMotor(LEFT_PWR_CW,   LEFT_PWR_CCW,  LEFT_ROTENC_A,  LEFT_ROTENC_B );
    MotorPID rightMotor(RIGHT_PWR_CW, RIGHT_PWR_CCW, RIGHT_ROTENC_A, RIGHT_ROTENC_B);
     
    bool cw = true;    
    uint32_t count = 0;  // Counter for timing rps adjustment
    float rps = 1.0;  // Initial throttle value

    leftMotor.setSpeedRPS(rps, cw);
    rightMotor.setSpeedRPS(rps, cw);

    leftMotor.configPID(  KP, KI, KD);
    rightMotor.configPID( KP, KI, KD);

    uint32_t lastUpdate = to_us_since_boot(get_absolute_time());  // Store last update time
    uint32_t lastThrottleChange = lastUpdate;  // For RPS adjustments
    const uint32_t pidInterval = 1000;  // 1 ms in microseconds
    const uint32_t throttleChangeInterval = 10000000;  // 10 seconds in microseconds
    
    while (true) {
        uint32_t currentTime = to_us_since_boot(get_absolute_time());

        // Check if 1 ms (1000 µs) has passed for PID and velocity updates
        if (currentTime - lastUpdate >= pidInterval) {
            float deltaT = (currentTime - lastUpdate) / 1e3;  // Convert to milliseconds


            // Update velocities and call PID for both motors
            leftMotor.updateVelocities(deltaT); 
            rightMotor.updateVelocities(deltaT);  

            leftMotor.doPID();
            rightMotor.doPID();

            printf("Left(SP): %.2f, Left(PV): %.2f, throttle: %.3f, cw: %d\n",rps, leftMotor.getRPS1(), leftMotor.getThrottle(), cw);   

            lastUpdate = currentTime;  // Reset last update time
        }
        // Check if 10 seconds have passed to adjust the throttle
        if (currentTime - lastThrottleChange >= throttleChangeInterval) {
            rps += 1.0;
            if (rps >= 5.0) {
                rps = 1.0; // Reset throttle back to 1.0 rps after reaching 3.0 
                cw = !cw;  // Change motor direction
                
                // Stop the motors before reversing
                leftMotor.setSpeedRPS(0.0,  cw);
                rightMotor.setSpeedRPS(0.0, cw);
                sleep_ms(1000);
            }
            leftMotor.setSpeedRPS(rps, cw);
            rightMotor.setSpeedRPS(rps, cw);

            // Reset the last throttle change time
            lastThrottleChange = currentTime;
        }    	
    }
}
