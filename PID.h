#include "ecrobot_types.h"

//#define Kp 0.075            // Kp
//#define Kd 0.0375           // Kd
#define Kp 0.100            // Kp
#define Kd 0.400           // Kd
#define Ki 0.0000000075     // Ki
#define MotorStartPower 80

extern S32 PID(S32, S32, S32*, S32*);
extern S32 MotorPID(S32, U8);
