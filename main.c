#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_base.h"
#include "ecrobot_private.h"
#include "ecrobot_interface.h"
#include "dist_nx.h"
#include "matrix_algebra.h"
#include "PID.h"
#include "weapon_system.h"

#define WSMOTOR1 NXT_PORT_B
#define WSMOTOR2 NXT_PORT_C


#define ANGLE 60
#define SAMPLESIZE 3

#define LEFT_4 -4
#define LEFT_3 -3
#define LEFT_2 -2
#define LEFT_1 -1
#define CENTER  0
#define RIGHT_1 1
#define RIGHT_2 2
#define RIGHT_3 3
#define RIGHT_4 4
#define UNKNOWN 5

#define RANGE_FAR 300
#define RANGE_CLOSE 700

#define MOTOR_FAST 80
#define MOTOR_MED  75
#define MOTOR_SLOW 70
#define MOTOR_STOP  0

#define LEFT_SENSOR NXT_PORT_S1
#define RIGHT_SENSOR NXT_PORT_S2

#define VK 8.0

DeclareCounter(SysTimerCnt);
DeclareTask(Task1);
DeclareTask(Task2);
DeclareTask(Task3);

U32 WSRotation = 0;
S8 resetRot = 0;
double kalmanReading = 0;
double x[2][1] = {{-40.0},{50.0}};
S8 flag3 = 0;
S8 flag2 = 0;
S8 prev = UNKNOWN;
S8 counter = 0;
S32 shots = 0;
U8 enableTask2 = 0;
long double determinant = 0;
double P[2][2] = {{0.4,0.0},{0.0,30.0}};
U8 resetCounter = 0;

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

void reset()
{
    kalmanReading = 0;
    x[0][0] = -40.0;
    x[1][0] = 50.0;
    flag3 = 0;
    flag2 = 0;
    prev = UNKNOWN;
    counter = 0;
    enableTask2 = 0;
    determinant = 0;
    shots = 0;
    P[0][0] = 0.4;
    P[0][1] = 0.0;
    P[1][0] = 0.0;
    P[1][1] = 30.0;
    nxt_motor_set_count(NXT_PORT_A, 0);
}

void resetTowerTo(S32 pos)
{
    U32 start = systick_get_ms();
    while(start + 1500 > systick_get_ms()){
        MotorPID(pos, NXT_PORT_A, 0);
    }
    nxt_motor_set_speed(NXT_PORT_A, 0, 1);
    systick_wait_ms(500);
}

void ecrobot_device_initialize(void)
{
    ecrobot_init_bt_slave("1234");
    ecrobot_init_dist_sensor(NXT_PORT_S1, RANGE_MEDIUM, 1);
    ecrobot_init_dist_sensor(NXT_PORT_S2, RANGE_MEDIUM, 0);
}

void ecrobot_device_terminate(void)
{
    ecrobot_term_bt_connection();
	ecrobot_term_dist_sensor(NXT_PORT_S1);
	ecrobot_term_dist_sensor(NXT_PORT_S2);
    nxt_motor_set_speed(NXT_PORT_A,0,1);
    nxt_motor_set_speed(NXT_PORT_B,0,1);
    nxt_motor_set_speed(NXT_PORT_C,0,1);

}

void kalman(double zn)
{
    /* Constants and matrices used for kalman calculations */
    static double R = VK;
    static double I[2][2] = {{1.0,0.0},{0.0,1.0}}; 
    double t = 0.01;
    double h[1][2] = {{1.0, 0.0}};
    double hT[2][1] = {{1.0},{0.0}};
    double a[2][2] = {{1.0, t},{0.0, 1.0}};
    double aT[2][2] = {{1.0,0.0},{t,1.0}}; 
	double K[2][1] = {{0.0},{0.0}};
    double y[2][1] = {{0.0},{0.0}};
    double temp[1][2] = {{0.0, 0.0}};
    double temp2v = 0.0;
    double (*temp2)[1] = (double (*)[1])(&temp2v);
    double temp3[2][2] = {{0.0, 0.0},{0.0, 0.0}};

    /* Calculate kalman gain in K */
    matrixMultiply(1,2,2,2, h, P, temp);
	matrixMultiply(1,2,2,1, temp, hT, temp2);
	temp2v += R;
	temp2v = 1.0 / temp2v;
	matrixMultiply(2,2,2,1, P, hT, K);
	matrixScale(2,1, K, temp2v, K);

	/* Update state estimate */
	matrixMultiply(1,2,2,1, h, x, temp2);
	zn -= temp2v;
	matrixScale(2,1, K, zn, y);
	matrixAdd(2,1, x, y, x);

	/* Update state covariance */
	matrixMultiply(2,1,1,2, K, h, temp3);
	matrixSubtract(2,2, I, temp3, temp3);
	matrixMultiply(2,2,2,2, temp3, P, P);

	/* Project into timestep k+1 */
	matrixMultiply(2,2,2,1, a, x, x);
	matrixMultiply(2,2,2,2, a, P, P);
	matrixMultiply(2,2,2,2, P, aT, P);
    determinant = matrixDeterminant(2, 2, P) * 10000.0;
}

TASK(Task3)
{
	MotorPID(WSRotation,WSMOTOR1, 0);
	MotorPID(WSRotation,WSMOTOR2, 0);
    TerminateTask();
}

TASK(Task2)
{   
    if(enableTask2)
    {
        S8 flag = 1;

        S32 left = (S32)ecrobot_get_dist_sensor(LEFT_SENSOR);
        S32 right = (S32)ecrobot_get_dist_sensor(RIGHT_SENSOR);


        if(left < RANGE_CLOSE && right < RANGE_CLOSE){
            prev = CENTER;
            flag2 = 1;
        }
        else if(left < RANGE_FAR)
            prev = LEFT_2;
        else if(left < RANGE_CLOSE){
            if(prev == UNKNOWN || prev == LEFT_4 || prev == LEFT_3)
                prev = LEFT_3;
            else {
            	prev = LEFT_1;
            	//flag2 = 1;
            }
        }
        else if(right < RANGE_FAR)
            prev = RIGHT_2;
        else if(right < RANGE_CLOSE){
            if(prev == UNKNOWN || prev == RIGHT_4 || prev == RIGHT_3)
                prev = RIGHT_3;
            else { 
                prev = RIGHT_1;
                flag2 = 1;
            }
        }
        else if(prev < 0)
            prev = LEFT_4;
        else if(prev > 0 && prev != UNKNOWN)
            prev = RIGHT_4;
        else {
            prev = UNKNOWN;
            counter = 0;
        }

        if(prev == LEFT_3 || prev == LEFT_2)
            kalmanReading = -13.1705;
        else if(prev == RIGHT_3 || prev == RIGHT_2)
            kalmanReading = 18.94;
        else if(prev == LEFT_4)
            kalmanReading = -13.1705;
        else if(prev == RIGHT_4)
            kalmanReading = 30.0;
        else if(prev == UNKNOWN){
            nxt_motor_set_speed(NXT_PORT_A, 0, 0);
            flag = 0;
        }
        else {
        	kalmanReading = (((double)prev) * 5.418);
        }

        if(counter < 10 && prev != UNKNOWN)
        {
            counter++;
            prev = UNKNOWN;
        }

        else if(flag){
            kalmanReading += (double)nxt_motor_get_count(NXT_PORT_A);

    			kalman(kalmanReading);
				S32 motor_pos = nxt_motor_get_count(NXT_PORT_A);
				if((motor_pos <= 100) && (motor_pos >= -45) ){
					flag3 = 1;
					MotorPID(((U32)x[0][0]) + ((1.0/determinant)*1.0)*(x[1][0]/1.0), NXT_PORT_A, 1);
				}
				else
					nxt_motor_set_speed(NXT_PORT_A, 0, 1);
            }
    }
    TerminateTask();
}

int motor_in_range(int range){
    return (nxt_motor_get_count(NXT_PORT_A) > ((S32)x[0][0]) - range &&
            nxt_motor_get_count(NXT_PORT_A) < ((S32)x[0][0]) + range);
}

void bt_data_logger(void)
{
    static U8 data_log_buffer[16];

    *((U32 *)(&data_log_buffer[0]))  = (U32)systick_get_ms();
    *((S32 *)(&data_log_buffer[4]))  = (S32)nxt_motor_get_count(NXT_PORT_A);
    *((S32 *)(&data_log_buffer[8]))  = (S32)determinant;
    *((S32 *)(&data_log_buffer[12]))  = (S32)x[1][0];
        
    ecrobot_send_bt(data_log_buffer, 0, 16);
}

TASK(Task1)
{   
    resetTowerTo(-30);
    reset();
    enableTask2 = 1;

    while(1){
        if(resetCounter == 6)
        {
            enableTask2 = 0;
            flag3 = 0;
            WSRotation -= 150;
            resetTowerTo(0 - resetRot);
            resetRot += nxt_motor_get_count(NXT_PORT_A);
            
        }
        if(resetCounter == 12)
        {
            resetCounter = 0;
            reset();
            enableTask2 = 1;
        }

        display_clear(1);
        display_goto_xy(0,0);
        display_int((S32)x[0][0], 7);

        display_goto_xy(0,1);
        display_int((S32)x[1][0], 7);
        display_update();
        
        display_goto_xy(0,2);
        display_string("P determinant:");
        display_goto_xy(0,3);
        display_int((S32)determinant, 7);
        bt_data_logger();
        display_update();

        if(flag3){
        	cock();
    		if(motor_in_range(10) && !shots && nxt_motor_get_count(NXT_PORT_A) > 50){
        		shots = fire();
                resetCounter = 1;
    		}
        }
        if(resetCounter > 0)
        {
            resetCounter++;
        }
        systick_wait_ms(100);
    }
  
}

	




