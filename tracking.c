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

S8 position_check(U8 left_sensor, U8 right_sensor, S8 prev){
	S32 left = (S32)ecrobot_get_dist_sensor(left_sensor);
	S32 right = (S32)ecrobot_get_dist_sensor(right_sensor);

	if(left < RANGE_CLOSE && right < RANGE_CLOSE)
		return TARGET_CENTER;
	if(left < RANGE_FAR)
		return TARGET_MED_LEFT;
	if(left < RANGE_CLOSE)
		if(prev == TARGET_UNKNOWN)
			return TARGET_FAR_LEFT;
		return TARGET_LEFT;
	if(right < RANGE_FAR)
		return TARGET_MED_RIGHT;
	if(right < RANGE_CLOSE)
		if(prev == TARGET_UNKNOWN)
			return TARGET_FAR_RIGHT;
		return TARGET_RIGHT;
	else return TARGET_UNKNOWN;
}

S8 naive_speed(S8 reading){
	switch(reading){
		case -3:
			return -100
		case -2:
			return -80;
		case -1:
			return -60;
		case 0:
			return 0;
		case 1:
			return 60;
		case 2:
			return 80;
		case 3:
			return 100;
		default:
			return 0;
	}
	
	/*
	S8 sign = 0; //((U8)reading) >> 7;

	if(reading > 0)
		sign = -1;
	else sign = 1;


	
	S8 speed = reading * 20 + 40 * sign;

	// (40 * !!reading) * sign + (reading * 20) * sign;
	return (speed == 120) ? 0 : speed;
	*/
}
