#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_base.h"
#include "ecrobot_private.h"
#include "ecrobot_interface.h"
#include "dist_nx.h"
#include "MatrixAlgebra.h"
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

#define VK 2.0

DeclareCounter(SysTimerCnt);
DeclareTask(Task1);
DeclareTask(Task2);
DeclareTask(Task3);

void reset(void);

S32 shotsfired = 0;
S8 speed = 0;
S8 flag2 = 0;
U32 counter1 = 0;
S8 flag3 = 0;
S8 resetFlag = 0;
S32 shots = 0;
S32 WSRotation = 0;
S32 stopSpeed = 100;
S8 prev = UNKNOWN;
double kalmanReading = 0;
long double deter = 100.0;
double x[2][1] = {{0.0},{0.0}};
double R[2][2] = {{VK*VK, 0.0},{0.0, VK*VK}};
double dt = 0.0;
double I[2][2] = {{1.0,0.0},{0.0,1.0}}; 
double P[2][2] = {{1.0,0.0},{0.0,1.0}};


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

    void ecrobot_device_initialize(void)
    {
        ecrobot_init_dist_sensor(NXT_PORT_S1, RANGE_MEDIUM, 1);
        ecrobot_init_dist_sensor(NXT_PORT_S2, RANGE_MEDIUM, 0);
    }
    
    void ecrobot_device_terminate(void)
    {
        reset();
    	ecrobot_term_dist_sensor(NXT_PORT_S1);
    	ecrobot_term_dist_sensor(NXT_PORT_S2);
    }

    void print4(double kk[2][2])
    {       
        display_clear(1);
        display_goto_xy(0,0);
        display_string("matrix:");
        display_goto_xy(0, 1);
        display_int((U32)(1000*kk[0][0]), 4);
        display_goto_xy(6, 1);
        display_int((U32)(1000*kk[0][1]), 4);

        display_goto_xy(1, 2);
        display_int((U32)(1000*kk[1][0]), 4);
        display_goto_xy(6, 2);
        display_int((U32)(1000*kk[1][1]), 4);

    }

    void print2(double kk[2][1])
    {       
        display_clear(1);
        display_goto_xy(0,0);
        display_string("matrix:");
        display_goto_xy(0, 1);
        display_int((U32)(-100*kk[0][0]), 4);
        display_goto_xy(6, 1);
        display_int((U32)(-100*kk[1][0]), 4);
    }

    void reset(void)
    {
    	nxt_motor_set_speed(NXT_PORT_A,0,1);
        dt = 0.0;
        stopSpeed = 100;
        flag2 = 0;
        flag3 = 0;
        counter1 = 0;
        prev = UNKNOWN;
        P[0][0] = 1.0;
        P[0][1] = 0.0;
        P[1][0] = 0.0;
        P[1][1] = 1.0;
        shots = 0;
        dt= 0;
        x[0][0] = 0.0;
        x[1][0] = 0.0;
        deter = 100.0;
        nxt_motor_set_speed(NXT_PORT_B,0,1);
        nxt_motor_set_speed(NXT_PORT_C,0,1);
    }


    void kalman(double zn[2][1])
    {
        /* Kalman konstanter og */
        int i;
        double *p;
        double t = (dt == 0)? 0.01 : ((double)systick_get_ms()-dt)/1000.0;
        double a[2][2] = {{1.0, t},{0.0, 1.0}};
        double aT[2][2] = {{1.0,0.0},{t,1.0}}; 
        double S[2][2] = {{0.0,0.0},{0.0,0.0}};
        double K[2][2] = {{0.0,0.0},{0.0,0.0}};
        double y[2][1] = {{0.0},{0.0}};

        /* calc Kalman Gain */ 
        matrixMultiplikation(2,2,2,1, a, x, x);
        matrixMultiplikation(2,2,2,2, a, P, P);
        matrixMultiplikation(2,2,2,2, P, aT, P);

        p = (double *)&y[0][0];
        for(i = 0; i < 2; i++)
        	p[i] = -((double *)x)[i];

        matrixAddition(2,1, zn,y,y);


        p = (double *)&S[0][0];
        for(i = 0; i < 4; i++)
        	p[i] = ((double *)P)[i];

        matrixAddition(2,2, S, R, S);
        matrixInvers(2,2,S,S);

        p = (double *)&K[0][0];
        for(i = 0; i < 4; i++)
        	p[i] = ((double *)P)[i];

        matrixMultiplikation(2,2,2,2, K, S, K);

        matrixMultiplikation(2,2,2,1, K, y, y);
        matrixAddition(2,1, x, y, x);        

        deter = 10000000000 * matrixDeterminant(2, 2, K);
        p = (double *)&K[0][0];
        for(i=0; i < 4; i++)
            p[i] = (-p[i]);

        matrixAddition(2,2, I, K, K);
        matrixMultiplikation(2,2,2,2, K, P, P);

        dt = t;
    }

    S8 naive_speed(S8 reading){
    switch(reading){
        case LEFT_3:
            return -MOTOR_FAST;
        case LEFT_2:
            return -MOTOR_MED;
        case LEFT_1:
            return -MOTOR_SLOW;
        case CENTER:
            return MOTOR_STOP;
        case RIGHT_1:
            return MOTOR_SLOW;
        case RIGHT_2:
            return MOTOR_MED;
        case RIGHT_3:
            return MOTOR_FAST;
        default:
            return MOTOR_STOP;
    }
    }

    TASK(Task3)
    {
    	if(flag3 && !resetFlag){
    		MotorPID(WSRotation,WSMOTOR1);
    		MotorPID(WSRotation,WSMOTOR2);
    	}
        TerminateTask();
    }

    TASK(Task2)
    {   
    	if(!resetFlag){
			S8 flag = 0;
			S32 left = (S32)ecrobot_get_dist_sensor(LEFT_SENSOR);
			S32 right = (S32)ecrobot_get_dist_sensor(RIGHT_SENSOR);

			if(left < RANGE_CLOSE && right < RANGE_CLOSE){
				prev = CENTER;
				flag2 = 1;
				counter1++;
			}
			else if(left < RANGE_FAR)
			{
				prev = LEFT_2;
				counter1++;
			}
			else if(left < RANGE_CLOSE){
				if(prev == UNKNOWN || prev == LEFT_4 || prev == LEFT_3)
					prev = LEFT_3;
				else {
					prev = LEFT_1;
					flag2 = 1;
					counter1++;
				}
			}
			else if(right < RANGE_FAR)
				prev = RIGHT_2;
			else if(right < RANGE_CLOSE){
				if(prev == UNKNOWN || prev == RIGHT_4 || prev == RIGHT_3)
					prev = RIGHT_3;
				else prev = RIGHT_1;
			}
			else if(prev < 0){
				prev = LEFT_4;
				counter1 = 0;
			}
			else if(prev > 0 && prev != UNKNOWN)
			{
				prev = RIGHT_4;
				counter1 = 0;
			}
			else prev = UNKNOWN;

			if(prev == LEFT_3)
				kalmanReading = -13.1705;
			else if(prev == RIGHT_3)
				kalmanReading = 18.94;
			else if(prev == LEFT_4)
				kalmanReading = -30;
			else if(prev == RIGHT_4)
				kalmanReading = 30;
			else if(prev == UNKNOWN){
				nxt_motor_set_speed(NXT_PORT_A, 0, 0);
				flag = 1;
				counter1 = 0;
			}
			else kalmanReading = (((double)prev) * 5.418);


			if(!flag && counter1 > 3){
				kalmanReading += (double)nxt_motor_get_count(NXT_PORT_A);
				double zn[2][1] = {{kalmanReading}, {0}};
				kalman(zn);
					S32 motor_pos = nxt_motor_get_count(NXT_PORT_A);
					if((motor_pos <= 200) && (motor_pos >= 45) && flag2){
						flag3 = 1;
						MotorPID(((U32)x[0][0]) + 5, NXT_PORT_A);
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

    TASK(Task1)
    {
       nxt_motor_set_count(NXT_PORT_A, 100);
        
        while(1){
            display_clear(1);
            display_goto_xy(0,0);
            display_int((S32)stopSpeed, 7);
            
            display_goto_xy(0,1);
            display_int((S32)deter, 7);
            display_update();

            if(flag3){
            	cock();
        		if(deter <= 10.0 && motor_in_range(10) && !shots && nxt_motor_get_count(NXT_PORT_A) > 150){
            		shots = fire();
            		systick_wait_ms(1000);
            		resetFlag = 1;
                    nxt_motor_set_speed(NXT_PORT_B,0,1);
                    nxt_motor_set_speed(NXT_PORT_C,0,1);

                    U32 epoch = systick_get_ms();
                    while((epoch+3000) > systick_get_ms()){ //!(nxt_motor_get_count(NXT_PORT_A) < 105 && nxt_motor_get_count(NXT_PORT_A) > 95 && !stopSpeed)
                    	stopSpeed = MotorPID(100, NXT_PORT_A);
                    	systick_wait_ms(20);
                    }
                    reset();
                    systick_wait_ms(3000);
                    resetFlag = 0;
        		}
            }
            systick_wait_ms(50);
        }
    }

