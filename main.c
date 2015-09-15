#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_base.h"
#include "ecrobot_private.h"
#include "ecrobot_interface.h"
#include "dist_nx.h"
    /* nxtOSEK hook to be invoked from an ISR in category 2 */
    void user_1ms_isr_type2(void){ /* do nothing */ }

    void ecrobot_device_initialize(void)
    {
        ecrobot_init_dist_sensor(NXT_PORT_S1, RANGE_MEDIUM);
        ecrobot_init_dist_sensor(NXT_PORT_S2, RANGE_MEDIUM);
    }
    
    void ecrobot_device_terminate(void)
    {
        ecrobot_term_dist_sensor(NXT_PORT_S1);
        ecrobot_term_dist_sensor(NXT_PORT_S2);
    }


    TASK(OSEK_Task_Background)
    {
      while(1){
    	  display_goto_xy(0, 0);
    	  display_string("Distance 1:");
    	  display_goto_xy(0, 1);
        U32 dis = ecrobot_get_dist_sensor(NXT_PORT_S1);
        display_goto_xy(0,0);
    	  display_string("Distance 2:");
    	  display_goto_xy(0,4);
    	  display_int(ecrobot_get_dist_sensor(NXT_PORT_S2), 5);
    	  ecrobot_set_motor_mode_speed(NXT_PORT_A, 0, 5);
    	  display_update();

    	  systick_wait_ms(50);
      }
    }
