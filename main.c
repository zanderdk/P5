#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_base.h"
#include "ecrobot_private.h"
#include "ecrobot_interface.h"
#include "dist_nx.h"
#include "PID.h"

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

    	  MotorPID(360, NXT_PORT_A);

    	  display_update();

    	  systick_wait_ms(200);
      }
    }


