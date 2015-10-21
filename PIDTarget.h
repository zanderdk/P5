#include "ecrobot_types.h"

#define Kp 50
#define Ki 5
#define Kd 25

#define TARGET_NO 2
#define TARGET_LEFT -1
#define TARGET_CENTER 0
#define TARGET_RIGHT 1

#define WALL 500

S8 directionCheck(S8, S8);
S32 PIDTarget(S8);
