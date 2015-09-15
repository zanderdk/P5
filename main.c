#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_base.h"
#include "ecrobot_private.h"
#include "ecrobot_interface.h"
#include "dist_nx.h"
S32 Integrale = 0;
S32 LastError = 0;
S16 PID(S32, S32);

    /* nxtOSEK hook to be invoked from an ISR in category 2 */
    void user_1ms_isr_type2(void){ /* do nothing */ }

    void ecrobot_device_initialize(void)
    {
        nxt_motor_set_speed(NXT_PORT_A, 0, 1);
    }
    
    void ecrobot_device_terminate(void)
    {
    	nxt_motor_set_speed(NXT_PORT_A, 0, 1);
    }


    TASK(OSEK_Task_Background)
    {
      while(1){

    	  display_goto_xy(0, 0);
    	  S32 Current = nxt_motor_get_count(NXT_PORT_A);
    	  S16 getSpeed = PID(270, Current);
    	  if(getSpeed < 50 && getSpeed > 0)
    	  {
    		  getSpeed = 50;
    	  }
    	  else if(getSpeed > -50 && getSpeed < 0)
    	  {
    		  getSpeed = -50;
    	  }
    	  nxt_motor_set_speed(NXT_PORT_A, getSpeed, 0);
    	  display_int(getSpeed, 10);

    	  display_update();

    	  systick_wait_ms(200);
      }
    }

    S16 PID(S32 target, S32 current)
    {
    	S32 Kp = 1;
    	S32 Ki = 0;
    	S32 Kd = 0;
    	S32 Error = target - current;
    	display_int(Error, 4);
    	systick_wait_ms(10000);
    	S32 Derivative = Error - LastError;
    	Integrale = Integrale + Error;
    	S32 Speed = Error * Kp + Ki*Integrale + Kd*Derivative;
    	Speed = Speed / 1000;
    	LastError = Error;
    	if(Speed > 100)
    	{
    		return 100;
    	}
    	else if(Speed < -100)
    	{
    		return -100;
    	}
    	else
    	{
    		return Speed;
    	}
    }
