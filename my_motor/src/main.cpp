/***
 * Control DDD for a Motor Speed Test
 *
 * Jon Durrant
 * 15-Aug-2022
 */


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

// NumTick =211


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
    for (;;){
    	for (float throttle = 0.25; throttle <= 0.4; throttle += 0.1){
    		printf("Throttle %.2f\n", throttle);
    		left.setThrottle(throttle, cw);
    		right.setThrottle(throttle, cw);

    		for (int i=0; i < 10; i++){
    			printf("Left %0.3f Right %0.3f Rads Per Second\n",
    					left.getMovingAvgRPM(), //getAvgRadPerSec(),
						right.getMovingAvgRPM()); //getAvgRadPerSec());
    			sleep_ms(2000);
    		}

    	}
		printf("Reversing direction\n");
    	cw = !cw;
		sleep_ms(2000);

    }
}