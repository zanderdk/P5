#include "PID.h"
#include "ecrobot_interface.h"

static U32 shotsfired = 0;
extern U32 WSRotation;

void cock() {
    WSRotation = shotsfired * 300 + 150;
}


S32 fire() {
    ++shotsfired;
    return shotsfired;
}
