///home/pizza/ros2_projects/jondurrant/DDD-Exp/3_myPID/build/src/DDD.elf
#include <cstdio> // Add this include
#include "pico/stdlib.h"
#include "MotorMgr.h"

// NumTick = 211.8 


// Left Motor
#define LEFT_PWR_CW     12
#define LEFT_PWR_CCW    13
#define LEFT_ROTENC_A   6  
#define LEFT_ROTENC_B   7  

// Right Motor
#define RIGHT_PWR_CW    19
#define RIGHT_PWR_CCW   18
#define RIGHT_ROTENC_A  8 
#define RIGHT_ROTENC_B  9 

// PID constants
// #define KP  0.55
// #define KI  0.019
// #define KD  0.24

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    printf("GO\n");

    // Create motor managers using Method 1 or Method 2
    MotorMgr leftMotor (LEFT_PWR_CW, LEFT_PWR_CCW, LEFT_ROTENC_A, LEFT_ROTENC_B);  
    MotorMgr rightMotor(RIGHT_PWR_CW, RIGHT_PWR_CCW, RIGHT_ROTENC_A, RIGHT_ROTENC_B); 
   
    leftMotor.setThrottle(0, 1);
    

    bool cw = true;
    uint32_t lastUpdate = to_ms_since_boot(get_absolute_time());  // Store last update time
    uint32_t count = 0;  // Counter for timing throttle adjustment
    uint32_t count2 = 0;
    float throttle = 0.25;  // Initial throttle value

    for (;;) {
        // Get the current time in milliseconds
        uint32_t currentTime = to_ms_since_boot(get_absolute_time());
        
        // Call updateVelocity_method1() every 1 ms
        if (currentTime - lastUpdate >= 1) {  // 1 ms has passed
            rightMotor.updateVelocity_method1();  // Update left motor velocity
        //    rightMotor.updateVelocity_method1();  // Update right motor velocity
            lastUpdate = currentTime;  // Reset last update time
            count++;  // Increment count after every 1 ms
            count2++;

            if (count2 >= 500) {
             //   printf("%.2f\n", leftMotor.getRPM());
                printf("%d,%d,%.2f,%.2f\n",rightMotor.getPos(), rightMotor.getDirection(), rightMotor.getRPS1(), rightMotor.getRPS2() );

                count2 = 0;
            
            }
            
            // After 10,000 iterations (approx. 10 seconds), adjust throttle
            if (count >= 5000) {               
                throttle += 0.05;  // Increase throttle by 0.1
                if (throttle > 0.6) {
                    throttle = 0.25;  // Reset throttle back to 0.3 after reaching 0.6
                    cw = !cw;  // Change motor direction
                }
                rightMotor.setThrottle(throttle, cw);  // Set new throttle for left motor
         //       rightMotor.setThrottle(throttle, cw);  // Set new throttle for right motor
                count = 0;  // Reset counter after throttle adjustment
            }
        }
   }
}
