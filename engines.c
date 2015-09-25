#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_base.h"
#include "ecrobot_private.h"
#include "ecrobot_interface.h"

/* nxtOSEK hook to be invoked from an ISR in category 2 */
    void user_1ms_isr_type2(void){ /* do nothing */ }

    void ecrobot_device_initialize(void)
    {

    }

    void ecrobot_device_terminate(void)
    {

    }


    TASK(OSEK_Task_Background)
    {
    	while(1)
    	{
    		ecrobot_set_motor_mode_speed(NXT_PORT_A, 0, 80);
    		ecrobot_set_motor_mode_speed(NXT_PORT_B, 0, 80);
    	}
    }
