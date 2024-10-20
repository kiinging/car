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

    MotorMgr left(LEFT_PWR_CW,   LEFT_PWR_CCW,  LEFT_ROTENC_A,  LEFT_ROTENC_B, 211.8);
    MotorMgr right(RIGHT_PWR_CW, RIGHT_PWR_CCW, RIGHT_ROTENC_A, RIGHT_ROTENC_B, 211.8);

	bool cw = true;
    left.setThrottle(0.3, cw);
    right.setThrottle(0.3, cw);

	while (1) {	
        printf("Left and right Motor Encoder Position: %d and %d \n", left.EncoderPosition(), right.EncoderPosition()); 
        printf("Left %0.3f Right %0.3f Rads Per Second\n", left.getAvgRadPerSec(), right.getAvgRadPerSec());
       
		
		sleep_ms(1000);  // Adjust the delay as needed
	}

   
}


 // bool cw = true;
    // for (;;){
    // 	for (float throttle = 0.3; throttle <= 1.0; throttle += 0.1){
    // 		printf("Throttle set to %.2f\n", throttle);
    // 		left.setThrottle(throttle, cw);
    // 		right.setThrottle(throttle, cw);

    // 		for (int i=0; i < 20; i++){
    // 			printf("Left %0.3f Right %0.3f Rads Per Second\n",
    // 					left.getAvgRadPerSec(),
	// 					right.getAvgRadPerSec());
    // 			sleep_ms(500);
    // 		}

    // 	}
    // 	cw = !cw;
    // }