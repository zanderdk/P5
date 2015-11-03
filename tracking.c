#include "ecrobot_types.h"
#include "dist_nx.h"

#define TARGET_FAR_LEFT -3
#define TARGET_MED_LEFT -2
#define TARGET_LEFT     -1
#define TARGET_CENTER    0
#define TARGET_RIGHT     1
#define TARGET_MED_RIGHT 2
#define TARGET_FAR_RIGHT 3
#define TARGET_UNKNOWN   4

#define RANGE_FAR 300
#define RANGE_CLOSE 700

#define MOTOR_FAST 80
#define MOTOR_MED  75
#define MOTOR_SLOW 70
#define MOTOR_STOP  0

S8 position_check(U8 left_sensor, U8 right_sensor){
	static S8 prev = TARGET_UNKNOWN;
	S32 left = (S32)ecrobot_get_dist_sensor(left_sensor);
	S32 right = (S32)ecrobot_get_dist_sensor(right_sensor);

	if(left < RANGE_CLOSE && right < RANGE_CLOSE)
		return prev = TARGET_CENTER;
	if(left < RANGE_FAR)
		return prev = TARGET_MED_LEFT;
	if(left < RANGE_CLOSE)
		return prev = TARGET_LEFT;
	if(right < RANGE_FAR)
		return prev = TARGET_MED_RIGHT;
	if(right < RANGE_CLOSE)
		return prev = TARGET_RIGHT;
	if(prev < 0)
		return prev = TARGET_FAR_LEFT;
	if(prev > 0 && prev != TARGET_UNKNOWN)
		return prev = TARGET_FAR_RIGHT;
	else return prev =TARGET_UNKNOWN;
}

S8 naive_speed(S8 reading){
	switch(reading){
		case TARGET_FAR_LEFT:
			return -MOTOR_FAST;
		case TARGET_MED_LEFT:
			return -MOTOR_MED;
		case TARGET_LEFT:
			return -MOTOR_SLOW;
		case TARGET_CENTER:
			return MOTOR_STOP;
		case TARGET_RIGHT:
			return MOTOR_SLOW;
		case TARGET_MED_RIGHT:
			return MOTOR_MED;
		case TARGET_FAR_RIGHT:
			return MOTOR_FAST;
		default:
			return MOTOR_STOP;
	}
}
