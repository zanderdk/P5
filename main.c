#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_base.h"
#include "ecrobot_private.h"
#include "ecrobot_interface.h"
#include "dist_nx.h"

/* #define Kp 0.05292  // Kp */
/* #define Kd 0.0375   //Kd */
/* #define Ki 0.0000000075   // Ki */

#define Kp 0.075            // Kp
#define Kd 0.0375           // Kd
#define Ki 0.0000000075     // Ki


S32 PID(S32, S32);
S32 MotorPID(S32, U8);

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

    	  MotorPID(NXT_PORT_A, 360);

    	  display_update();

    	  systick_wait_ms(200);
      }
    }


    S32 MotorPID(S32 target, U8 motor)
    {
    	S32 Current = nxt_motor_get_count(motor);
    	S32 getSpeed = PID(target, Current);
    	nxt_motor_set_speed(motor, getSpeed, 0);
    	return getSpeed
    }

    S32 PID(S32 target, S32 current)
    {
    	static S32 Integrale = 0;
    	static S32 LastError = 0;


    	S32 Error = target - current;
    	S32 Derivative = Error - LastError;
    	Integrale = Integrale + Error;
    	S32 Speed = (S32)(Error * Kp + Ki*Integrale + Kd*Derivative);
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
    		if(Speed < 50 && Speed > 0)
    		{
    			  Speed = 50 + Speed;
    		}
    		else if(Speed > -50 && Speed < 0)
    		{
    		  Speed = (Speed - 50);
    		}
    		return Speed;
    	}
    }
