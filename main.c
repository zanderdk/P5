#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_base.h"
#include "ecrobot_private.h"
#include "ecrobot_interface.h"
#include "dist_nx.h"
#include "PID.h"

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
        
    }
    
    void ecrobot_device_terminate(void)
    {
    	ecrobot_term_sonar_sensor(NXT_PORT_S1);
    	ecrobot_term_sonar_sensor(NXT_PORT_S2);
    }


U8 directionCheck(S32 sensor1, S32 sensor2){

	static S32 lcbuf[5] = {{0}};
	static S32 rcbuf[5] = {{0}};
	static S32 i = 0;

	lcbuf[i] = sensor1;
	rcbuf[i] = sensor2;

	i = (i == 5) ? 0 : ++i;

	S32 sensorLeft = 0;
	S32 sensorRight = 0;

	U8 j;
	for(j = 0; j < 5; j++){
		sensorLeft += lcbuf[j];
		sensorRight += rcbuf[j];
	}

	sensorLeft = sensorLeft / 5;
	sensorRight = sensorRight /5;

	return (sensorLeft >= WALL && sensorRight >= WALL) ? TARGET_NO :
				(sensorLeft < WALL && sensorRight >= WALL) ? TARGET_LEFT :
						(sensorLeft >= WALL && sensorRight < WALL) ? TARGET_RIGHT : TARGET_CENTER;
}


    TASK(OSEK_Task_Background)
    {
      display_clear(1);
      while(1){

    	  display_goto_xy(0, 0);

    	  //display_int(directionCheck(ecrobot_get_sonar_sensor(NXT_PORT_S1), ecrobot_get_sonar_sensor(NXT_PORT_S2)), 1);
    	  display_int(1, 1);
    	  display_update();

    	  systick_wait_ms(200);
      }
    }




