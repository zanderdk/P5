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
        ecrobot_init_dist_sensor(NXT_PORT_S1, RANGE_MEDIUM);
    }
    
    void ecrobot_device_terminate(void)
    {
    	ecrobot_term_dist_sensor(NXT_PORT_S1);
    }

    TASK(OSEK_Task_Background)
    {
      while(1){
    	  S32 sensor1 = (S32)ecrobot_get_dist_sensor(NXT_PORT_S1);
          S32 val = (S32)sensor1;

    	  display_goto_xy(0, 0);
    	  display_int(val,6);

    	  display_update();

    	  systick_wait_ms(50);
      }
    }




