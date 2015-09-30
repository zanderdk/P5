#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_base.h"
#include "ecrobot_private.h"
#include "ecrobot_interface.h"
#include "dist_nx.h"
#include "PID.h"


#define SAMPLESIZE 3
#define WALL 100
#define TARGET_NO 0
#define TARGET_LEFT 1
#define TARGET_RIGHT 2
#define TARGET_CENTER 3

    /* nxtOSEK hook to be invoked from an ISR in category 2 */
    void user_1ms_isr_type2(void){ /* do nothing */ }

    void ecrobot_device_initialize(void)
    {
        ecrobot_init_sonar_sensor(NXT_PORT_S1);
        ecrobot_init_sonar_sensor(NXT_PORT_S2);
        ecrobot_set_motor_mode_speed(NXT_PORT_A,0,0);
    }
    
    void ecrobot_device_terminate(void)
    {
    	ecrobot_term_sonar_sensor(NXT_PORT_S1);
    	ecrobot_term_sonar_sensor(NXT_PORT_S2);
    }

void sweep(){

	static U8 t = 0;

	if(t == 0)
		MotorPID(100,NXT_PORT_A);
	else if(t == 1)
		MotorPID(-10,NXT_PORT_A);

	if(nxt_motor_get_count(NXT_PORT_A) > 80)
		t = 1;
	else if(nxt_motor_get_count(NXT_PORT_A) < 0)
		t = 0;
}


U8 directionCheck(S32 sensorLeft, S32 sensorRight){

//	static S32 lcbuf[SAMPLESIZE] = {{0}};
//	static S32 rcbuf[SAMPLESIZE] = {{0}};
//	static S32 i = 0;
//
//	lcbuf[i] = sensor1;
//	rcbuf[i] = sensor2;
//
//	i = (i == SAMPLESIZE-1) ? 0 : i+1;
//
//	S32 sensorLeft = 0;
//	S32 sensorRight = 0;
//
//	U8 j;
//	for(j = 0; j < SAMPLESIZE; j++){
//		sensorLeft += lcbuf[j];
//		sensorRight += rcbuf[j];
//	}
//
//	sensorLeft = sensorLeft / SAMPLESIZE;
//	sensorRight = sensorRight /SAMPLESIZE;

//	static S32 sensorLeft = 0;
//	static S32 sensorRight = 0;
//
//	static U8 lcount = 0;
//	static U8 rcount = 0;
//
//	if(sensorL < WALL){
//			sensorLeft = sensorL;
//			lcount = 2;
//		}
//		else if(lcount == 0)
//			sensorLeft = sensorL;
//		else
//			lcount = lcount-1;
//
//	if(sensorR < WALL){
//			sensorRight = sensorR;
//			rcount = 2;
//		}
//		else if(rcount == 0)
//			sensorRight = sensorR;
//		else
//			rcount = rcount-1;


	return (sensorLeft >= WALL && sensorRight >= WALL) ? TARGET_NO :
			(sensorLeft < WALL && sensorRight >= WALL) ? TARGET_LEFT :
			(sensorLeft >= WALL && sensorRight < WALL) ? TARGET_RIGHT : TARGET_CENTER;
}


    TASK(OSEK_Task_Background)
    {
      display_clear(1);
      while(1){

    	  S32 sensor1 = ecrobot_get_sonar_sensor(NXT_PORT_S1);
    	  S32 sensor2 = ecrobot_get_sonar_sensor(NXT_PORT_S2);

    	  U8 direction = directionCheck(sensor1, sensor2);


    	  if(direction == TARGET_NO)
    		  sweep();
    	  else if(direction == TARGET_LEFT && nxt_motor_get_count(NXT_PORT_A) > 0)
    		  ecrobot_set_motor_speed(NXT_PORT_A,-80);
    	  else if(direction == TARGET_RIGHT && nxt_motor_get_count(NXT_PORT_A) < 90)
    		  ecrobot_set_motor_speed(NXT_PORT_A,80);
    	  else
    		  ecrobot_set_motor_mode_speed(NXT_PORT_A,1,0);


    	  display_goto_xy(0, 0);
    	  display_int(sensor1,3);

    	  display_goto_xy(0,1);
    	  display_int(sensor2,3);

    	  display_goto_xy(0,2);
    	  display_int(direction, 1);

    	  display_update();

    	  systick_wait_ms(30);
      }
    }




