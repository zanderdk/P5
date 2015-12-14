#include "ecrobot_interface.h"
#include "PID.h"

    S32 
    MotorPID(S32 target, U8 motor, U8 flag)
    {
        static S32 getSpeed = 0;
        static S32 integrale[] = {0, 0, 0, 0};
        static S32 lastError[] = {0, 0, 0, 0};
        static S32 lastTarget[] = {0, 0, 0, 0};

        if(lastTarget[motor] != target)
        {
        	integrale[motor] = 0;
        	lastError[motor] = 0;
        	lastTarget[motor] = target;
        }

    	S32 current = nxt_motor_get_count(motor);
        current = (flag)? -current : current;
        if( flag || !((getSpeed <= 80 && getSpeed >= -80) && (current - target) <= 4 && (current - target) >= -4)){
    	   getSpeed = PID(target, current, &integrale[motor], &lastError[motor]);
           getSpeed = (flag)? -getSpeed : getSpeed;
    	   nxt_motor_set_speed(motor, getSpeed, (!getSpeed));
        }
        else 
        {
            nxt_motor_set_speed(motor, 0, 1);
        }
    	return getSpeed;
    }

    S32 
    PID(S32 target, S32 current, S32 *integrale, S32 *lastError)
    {
    	S32 error = target - current;
    	S32 derivative = error - *lastError;
    	*integrale = *integrale + error;
    	double speed = error * Kp + Ki*(*integrale) + Kd*derivative;
    	*lastError = error;

    	return (speed > 0) ?
            (S32)(speed + MotorStartPower) :
            (speed < 0) ?
               (S32)(speed - MotorStartPower) :
                (S32)speed;
    }
