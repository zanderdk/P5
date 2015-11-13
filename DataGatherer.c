/*
 * DataGatherer.c
 *
 *  Created on: 28/10/2015
 *      Author: Abstrakten
 */

#include "DataGatherer.h"

#define WALL 600
#define TARGET_NO 0
#define TARGET_LEFT 1
#define TARGET_RIGHT 2
#define TARGET_CENTER 3

void ecrobot_device_initialize()
{
  ecrobot_init_bt_slave("1234");
  ecrobot_init_dist_sensor(NXT_PORT_S1, RANGE_MEDIUM, 1);
  ecrobot_init_dist_sensor(NXT_PORT_S2, RANGE_MEDIUM, 0);
  nxt_motor_set_speed(NXT_PORT_A, 0, 1);
}

void ecrobot_device_terminate()
{
  ecrobot_term_bt_connection();
  ecrobot_term_sonar_sensor(NXT_PORT_S1);
  /* ecrobot_term_sonar_sensor(NXT_PORT_S2); */
  ecrobot_term_dist_sensor(NXT_PORT_S2);
  nxt_motor_set_speed(NXT_PORT_A, 0, 1);
}

U8 directionCheck(S32 sensorLeft, S32 sensorRight){
	return (sensorLeft >= WALL && sensorRight >= WALL) ? TARGET_NO :
			(sensorLeft < WALL && sensorRight >= WALL) ? TARGET_LEFT :
			(sensorLeft >= WALL && sensorRight < WALL) ? TARGET_RIGHT : TARGET_CENTER;
}

void gatherData(U8 sensor1, U8 sensor2, U8 motor){

	for(S32 j = 0; j < 40; j++){
		for(S32 k = 0; k < 3; k++){

			const U8 Log[30] = {{0}};
			U8 i = 0;
			S32 x = 0;

			while(directionCheck(sensor1,sensor2) == 0){
				;
			}

			ecrobot_set_motor_mode_speed(motor,0,60+j);

			while(x = nxt_motor_get_count(motor) < 90){
				if((x >= i*3) == 0){
					Log[i] = directionCheck(sensor1,sensor2);
					i = i + 1;
				}
			}

			U32 epoch = systick_get_ms();

			while(systick_get_ms() - epoch < 5000){
				MotorPID(0,motor);
				systick_wait_ms(20);
			}

			ecrobot_send_bt(Log,0,30);
		}
	}
}
