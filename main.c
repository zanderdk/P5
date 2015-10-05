#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_base.h"
#include "ecrobot_private.h"
#include "ecrobot_interface.h"
#include "dist_nx.h"
#include "PIDTarget.h"

#define ANGLE 60
#define SAMPLESIZE 3


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



/*void sweep(){

	static U8 t = 0;

	if(t == 0)
		MotorPID((ANGLE+5),NXT_PORT_A);
	else if(t == 1)
		MotorPID(-(ANGLE+5),NXT_PORT_A);

	if(nxt_motor_get_count(NXT_PORT_A) > ANGLE)
		t = 1;
	else if(nxt_motor_get_count(NXT_PORT_A) < -ANGLE)
		t = 0;
}*/


    TASK(OSEK_Task_Background)
    {
      display_clear(1);
      while(1){

    	  S8 direction = directionCheck(NXT_PORT_S1, NXT_PORT_S2);
    	  S32 speed = PIDTarget(direction);

    	  if(direction == TARGET_NO)
    	  {
    		  //sweep();
    		  nxt_motor_set_speed(NXT_PORT_A, 0,0);
    	  }
    	  else if (speed > 0 && nxt_motor_get_count(NXT_PORT_A) < 90)
    	  {
    		  nxt_motor_set_speed(NXT_PORT_A, speed, 0);
    	  }
    	  else if (speed < 0 && nxt_motor_get_count(NXT_PORT_A) > -90)
    	  {
    		  nxt_motor_set_speed(NXT_PORT_A, speed, 0);
    	  }
    	  else
    	  {
    		  nxt_motor_set_speed(NXT_PORT_A, 0, 1);
    	  }

    	  display_goto_xy(0, 0);
    	  	  	  display_string("Left:");
    	  	  	  display_goto_xy(5,0);
    	      	  display_int(ecrobot_get_sonar_sensor(NXT_PORT_S1),3);

    	      	  display_goto_xy(0,1);
    	      	  display_string("Right:");
    	      	  display_goto_xy(5,1);
    	      	  display_int(ecrobot_get_sonar_sensor(NXT_PORT_S2),3);

    	      	  display_goto_xy(0,2);
    	      	  display_string("Speed:");
    	      	  display_goto_xy(5,2);
    	      	  display_int(PIDTarget(direction), 3);

    	      	  display_goto_xy(0,3);
    	      	  display_string("Direc:");
    	      	  display_goto_xy(5,3);
    	      	  display_int(direction, 2);

    	  display_update();

    	  systick_wait_ms(30);
      }
    }




