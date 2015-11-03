#include "ecrobot_types.h"
#include "dist_nx.h"

#define TARGET_NOT_LEFT -3
#define TARGET_FAR_LEFT -2
#define TARGET_LEFT     -1
#define TARGET_CENTER    0
#define TARGET_RIGHT     1
#define TARGET_FAR_RIGHT 2
#define TARGET_NOT_RIGHT 3
#define TARGET_UNKNOWN   4

#define RANGE_FAR 300
#define RANGE_CLOSE 700

S8 position_check(U8 left_sensor, U8 right_sensor){
	S32 left = (S32)ecrobot_get_dist_sensor(left_sensor);
	S32 right = (S32)ecrobot_get_dist_sensor(right_sensor);

	if(left < RANGE_CLOSE && right < RANGE_CLOSE)
		return TARGET_CENTER;
	if(left < RANGE_FAR)
		return TARGET_FAR_LEFT;
	if(left < RANGE_CLOSE)
		return TARGET_LEFT;
	if(right < RANGE_FAR)
		return TARGET_FAR_RIGHT;
	if(right < RANGE_CLOSE)
		return TARGET_RIGHT;
	else return TARGET_UNKNOWN;
}
