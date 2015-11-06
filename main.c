#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_base.h"
#include "ecrobot_private.h"
#include "ecrobot_interface.h"
#include "dist_nx.h"
#include "dist_nx_v3.h"
#include "PIDTarget.h"
#include "MatrixAlgebra.h"

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

#define VK 11.0
#define R 121.0

DeclareCounter(SysTimerCnt);
DeclareTask(Task1);
DeclareTask(Task2);

S8 kalmanReading = 0;
S8 speed = 0;


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
    	//ecrobot_init_bt_slave("1234");
        ecrobot_init_dist_sensor(NXT_PORT_S1, RANGE_MEDIUM, 1);
        ecrobot_init_dist_sensor(NXT_PORT_S2, RANGE_MEDIUM, 0);
        //ecrobot_init_dist_v3_sensor(NXT_PORT_S3);
    }
    
    void ecrobot_device_terminate(void)
    {
    	//ecrobot_term_bt_connection();
    	ecrobot_term_dist_sensor(NXT_PORT_S1);
    	ecrobot_term_dist_sensor(NXT_PORT_S2);
    	//ecrobot_term_dist_v3_sensor(NXT_PORT_S3);

    }

    void kalman(double zk[1][2])
    {
        /* Kalman konstanter og */

        int i;
        double *p;
        static double dt = 0.0;
        static double I[2][2] = {{1.0,0.0},{0.0,1.0}}; 
        static double Pk[2][2] = {{VK,0.0},{0.0,VK}};
        double t = (dt == 0)? 0.0 : (double)systick_get_ms()-dt;
        double h[2][2] = {{1,t},{0,1}};
        double hT[2][2] = {{1,0},{t,1}};
        double kk[2][2] = {{0,0}, {0,0}};

        /* calc Kalman Gain */

        MatrixMultiplikation(2,2,2,2, Pk, hT, kk);
        double kktemp[2][2] = {{0,0}};
        MatrixMultiplikation(2,2,2,2, h,Pk, kktemp);
        MatrixMultiplikation(2,2,2,2, kktemp, hT, kktemp);
        p = (double *)&kktemp[0][0];
        for(i = 0; i < 4; i++)
            p[i] = p[i] + R;
        MatrixInvers(2, 2, kktemp, kktemp);
        MatrixMultiplikation(2,2,2,2,kk,kktemp,kk);
            
        display_clear(1);

        display_goto_xy(0, 0);
        display_string("Kalman Gain[0][0]:");
        display_goto_xy(1, 1);
        display_int((U32)(100*kk[0][0]), 5);

        display_goto_xy(0, 2);
        display_string("Kalman Gain[1][1]:");
        display_goto_xy(1, 3);
        display_int((U32)(100*kk[1][1]), 5);

        
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

    TASK(Task2)
    {   
        static S8 prev = UNKNOWN;
        S32 left = (S32)ecrobot_get_dist_sensor(LEFT_SENSOR);
        S32 right = (S32)ecrobot_get_dist_sensor(RIGHT_SENSOR);

        if(left < RANGE_CLOSE && right < RANGE_CLOSE)
            prev = CENTER;
        else if(left < RANGE_FAR)
            prev = LEFT_2;
        else if(left < RANGE_CLOSE){
            if(prev == UNKNOWN || prev == LEFT_4 || prev == LEFT_3)
                prev = LEFT_3;
            else prev = LEFT_1;
        }
        else if(right < RANGE_FAR)
            prev = RIGHT_2;
        else if(right < RANGE_CLOSE){
            if(prev == UNKNOWN || prev == RIGHT_4 || prev == RIGHT_3)
                prev = RIGHT_3;
            else prev = RIGHT_1;
        }
        else if(prev < 0)
            prev = LEFT_4;
        else if(prev > 0 && prev != UNKNOWN)
            prev = RIGHT_4;
        else prev = UNKNOWN;

        if(prev == LEFT_3)
            kalmanReading = LEFT_2;
        else if(prev == RIGHT_3)
            kalmanReading = RIGHT_2;
        else if(prev == LEFT_4)
            kalmanReading = LEFT_3;
        else if(prev == RIGHT_4)
            kalmanReading = RIGHT_3;
        else if(prev == UNKNOWN)
            kalmanReading = 4;
        else kalmanReading = prev;

        /* display_clear(1); */
        /*  */
        /* display_goto_xy(0, 0); */
        /* display_string("Kalman reading:"); */
        /* display_goto_xy(1, 1); */
        /* display_int(kalmanReading, 5); */
        /*  */
        /* display_goto_xy(0, 2); */
        /* display_string("Prev:"); */
        /* display_goto_xy(1, 3); */
        /* display_int(prev, 5); */


        systick_wait_ms(15);
       
        TerminateTask();
    }

    TASK(Task1)
    {

      double dummy[1][2] = {{1,1}};
      kalman(dummy);

      while(1){
        /*
          S8 speed = naive_speed(kalmanReading);
          S32 motor_pos = nxt_motor_get_count(NXT_PORT_A);

          if(( speed > 0 && motor_pos <= 50) || ( speed < 0 && motor_pos >= -50)){
            nxt_motor_set_speed(NXT_PORT_A, speed, 0);
            if(speed == 0)
                nxt_motor_set_speed(NXT_PORT_A, 0, 1);
            }
          else
            nxt_motor_set_speed(NXT_PORT_A, 0, 1);
        */

    	  systick_wait_ms(15);
      }
    }

	




