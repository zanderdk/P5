/* datalogging.c */
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "PID.h"
#include "dist_nx.h"

#define ANGLE 60
#define SAMPLESIZE 3
#define WALL 100
#define TARGET_NO 0
#define TARGET_LEFT 1
#define TARGET_RIGHT 2
#define TARGET_CENTER 3

/* OSEK declarations */
DeclareCounter(SysTimerCnt);
DeclareTask(Task1);
DeclareTask(Task2);

void bt_data_logger(void);

/* nxtOSEK hooks */
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
  ecrobot_term_dist_sensor(NXT_PORT_S1);
  ecrobot_term_dist_sensor(NXT_PORT_S2);
  nxt_motor_set_speed(NXT_PORT_A, 0, 1);
}

/* nxtOSEK hook to be invoked from an ISR in category 2 */
void user_1ms_isr_type2(void)
{
  StatusType ercd;

  ercd = SignalCounter(SysTimerCnt); /* Increment OSEK Alarm Counter */
  if (ercd != E_OK)
  {
    ShutdownOS(ercd);
  }
}

void sweep(){

	static U8 t = 0;

	if(t == 0)
		MotorPID((ANGLE+15),NXT_PORT_A);
	else if(t == 1)
		MotorPID(-(ANGLE+15),NXT_PORT_A);

	if(nxt_motor_get_count(NXT_PORT_A) > ANGLE)
		t = 1;
	else if(nxt_motor_get_count(NXT_PORT_A) < -ANGLE)
		t = 0;
}


U8 directionCheck(S32 sensorLeft, S32 sensorRight){
	return (sensorLeft >= WALL && sensorRight >= WALL) ? TARGET_NO :
			(sensorLeft < WALL && sensorRight >= WALL) ? TARGET_LEFT :
			(sensorLeft >= WALL && sensorRight < WALL) ? TARGET_RIGHT : TARGET_CENTER;
}



/* Task1 executed every 10msec */
TASK(Task1)
{
  bt_data_logger();
/*
  if(nxt_motor_get_count(NXT_PORT_A) > 75)
	  nxt_motor_set_speed(NXT_PORT_A, -50, 0);
  else if(nxt_motor_get_count(NXT_PORT_A) < -75)
  {
	  nxt_motor_set_speed(NXT_PORT_A, 50, 1);
  }
*/
  /* display Sensors/Motors/NXT internal status */
  ecrobot_status_monitor("Data Logging");
  TerminateTask();
}

/* Task2 executed every 2000msec */
TASK(Task2)
{
      //sweep();
      TerminateTask();
}

void bt_data_logger(void)
{
	static U8 data_log_buffer[16];
    S32 sensor1 = ecrobot_get_dist_sensor(NXT_PORT_S1);
    S32 sensor2 = ecrobot_get_dist_sensor(NXT_PORT_S2);


	*((U32 *)(&data_log_buffer[0]))  = (U32)systick_get_ms();
	*((S32 *)(&data_log_buffer[4]))  = (S32)nxt_motor_get_count(NXT_PORT_A);
	*((S32 *)(&data_log_buffer[8]))  = (S32)sensor1;
	*((S32 *)(&data_log_buffer[12]))  = (S32)sensor2;
		
	ecrobot_send_bt(data_log_buffer, 0, 16);
}

