#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_base.h"
#include "ecrobot_private.h"
#include "ecrobot_interface.h"
#include "math.h"
#include "dist_nx.h"
#include "PIDTarget.h"

#define ANGLE 60
#define SAMPLESIZE 3


    /* nxtOSEK hook to be invoked from an ISR in category 2 */
    void user_1ms_isr_type2(void){ /* do nothing */ }

    void ecrobot_device_initialize(void)
    {
        ecrobot_init_dist_sensor(NXT_PORT_S1, RANGE_MEDIUM, 1);
        ecrobot_init_dist_sensor(NXT_PORT_S2, RANGE_MEDIUM, 0);
    }
    
    void ecrobot_device_terminate(void)
    {
    	ecrobot_term_dist_sensor(NXT_PORT_S1);
    	ecrobot_term_dist_sensor(NXT_PORT_S2);
    }

    TASK(OSEK_Task_Background)
    {
      while(1){

    	  S8 target = directionCheck(NXT_PORT_S1, NXT_PORT_S2);
    	  S32 speed = PIDTarget(target);

    	  nxt_motor_set_speed(NXT_PORT_A, speed, 0);

    	  display_goto_xy(0, 0);
    	  display_int(target,1);

    	  display_update();

    	  systick_wait_ms(50);
      }
    }




