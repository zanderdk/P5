#include "PID.h"
#include "ecrobot_interface.h"

S32 shotsfired = 0;

void cock(U8 motor1, U8 motor2){
	S32 cAngle = shotsfired * 300 + 227;


	MotorPID(cAngle,motor1);
	MotorPID(cAngle,motor2);
}

S32 fire(U8 motor1, U8 motor2){

	S32 fAngle = shotsfired * 300 + 325;

	MotorPID(fAngle,motor1);
	MotorPID(fAngle,motor2);

	shotsfired = (nxt_motor_get_count(motor1) + 100) / 300;
	return shotsfired;
}
