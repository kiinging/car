/***
 * Control DDD for a Motor Speed Test
 *
 * Jon Durrant
 * 15-Aug-2022
 */


#include "pico/stdlib.h"
#include "MotorMgr.h"
#include "MotorPID.h"

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

//PID
#define KP	0.55
#define KI	0.019
#define KD	0.24




/***
 * Main
 * @return
 */
int main( void ){
    stdio_init_all();
    sleep_ms(2000);
    printf("GO\n");

    MotorPID left(LEFT_PWR_CW,   LEFT_PWR_CCW,  LEFT_ROTENC_A,  LEFT_ROTENC_B);
    MotorPID right(RIGHT_PWR_CW, RIGHT_PWR_CCW, RIGHT_ROTENC_A, RIGHT_ROTENC_B);

    left.configPID(    KP, KI, KD);
    right.configPID( KP, KI, KD);


    bool cw = true;
    float sp, pv, err, p, i, d;
    for (;;){
    	// for (float radps = 1.5; radps <= 6.3; radps += 0.78){
		for (float radps = 6.3; radps <= 25.2; radps += 6.3){
    		printf("Test Speed %.2f\n", radps);

    		left.setSpeedRadPS(  radps,   cw);
    		right.setSpeedRadPS(radps, cw);

    		printf("Time,  SP,  PV,  ERR,  P,  I,  D\n");

    		for (int t=0; t < 50; t++){
    			printf("%u, ", t);

    			left.pid(sp, pv, err, p, i, d);
    			printf("%f, %f, %f, %f, %f, %f",
    					sp, pv, err, p, i, d);

				left.doPID();
				right.doPID();

				printf("\n");
    			sleep_ms(200);
    		}

    		//STOP
    		left.setThrottle(0.0, true);
    		right.setThrottle(0.0, true);
    		sleep_ms(1000);

    		cw = !cw;

    	}

    }
}
