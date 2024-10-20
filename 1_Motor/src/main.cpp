/***
 * Control DDD for a Motor Throttle test
 *
 * Jon Durrant
 * 15-Aug-2022
 */
#include <stdio.h>

#include "pico/stdlib.h"
#include "MotorMgr.h"

//Left Motor
#define LEFT_PWR_CW		12
#define LEFT_PWR_CCW	13
#define LEFT_ROTENC_A 	6  
#define LEFT_ROTENC_B	7  

//Right Motor
#define RIGHT_PWR_CW	19
#define RIGHT_PWR_CCW	18
#define RIGHT_ROTENC_A 	8 
#define RIGHT_ROTENC_B	9 

char ROBOT_NAME[]="ddd";




/***
 * Main
 * @return
 */
int main( void ){
    stdio_init_all();
    sleep_ms(2000);
    printf("GO\n");

    MotorMgr left(LEFT_PWR_CW,   LEFT_PWR_CCW,  LEFT_ROTENC_A,  LEFT_ROTENC_B);
    MotorMgr right(RIGHT_PWR_CW, RIGHT_PWR_CCW, RIGHT_ROTENC_A, RIGHT_ROTENC_B);

    bool cw = true;
    for (;;) {
        for (float throttle = 0.3; throttle < 0.8; throttle += 0.2) {
            // Single line to print throttle and direction
            printf("Throttle set to %.2f and running %s\n", throttle, cw ? "Clockwise (CW)" : "Counterclockwise (CCW)");

            left.setThrottle(throttle, cw);  //cw goes backward, ccw go forward
            right.setThrottle(throttle, cw); 
            sleep_ms(10000);
        }

        // Toggle the direction
        cw = !cw;
    }
}
