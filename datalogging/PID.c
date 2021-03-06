#include "ecrobot_interface.h"
#include "PID.h"

    S32 
    MotorPID(S32 target, U8 motor)
    {
        static S32 integrale[] = {0, 0, 0, 0};
        static S32 lastError[] = {0, 0, 0, 0};
        static S32 lastTarget[] = {0, 0, 0, 0};

        if(lastTarget[motor] != target)
        {
            integrale[motor] = 0;
            lastError[motor] = 0;
            lastTarget[motor] = target;            
        }

    	register S32 current = nxt_motor_get_count(motor);
    	register S32 getSpeed = PID(target, current, &integrale[motor], &lastError[motor]);
    	nxt_motor_set_speed(motor, getSpeed, 0);
    	return getSpeed;
    }

    S32 
    PID(S32 target, S32 current, S32 *integrale, S32 *lastError)
    {
    	S32 error = target - current;
    	S32 derivative = error - *lastError;
    	*integrale = *integrale + error;
    	S32 speed = (S32)(error * Kp + Ki*(*integrale) + Kd*derivative);
    	*lastError = error;

    	return (speed > 0) ?
            speed + SPEED :
            (speed > -SPEED && speed < 0) ?
                speed - SPEED :
                speed;
    }
