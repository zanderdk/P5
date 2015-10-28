#include <stddef.h>
#include <string.h>

#include "ecrobot_base.h"
#include "ecrobot_private.h"
#include "ecrobot_interface.h"
#include "dist_nx_v3.h"

S32 distance[4] = {-1, -1, -1, -1};

void ecrobot_init_dist_v3_sensor(U8 port_id)
{
    ecrobot_init_i2c(port_id, LOWSPEED);
}

void ecrobot_term_dist_v3_sensor(U8 port_id)
{
    ecrobot_term_i2c(port_id);
    U8 i;
    for(i = 0; i < 4; i++)
        distance[i] = -1;
}

S32 ecrobot_get_dist_v3_sensor(U8 port_id)
{
    static U8 data[4][2] = {{0, 0}};
       
    if(i2c_busy(port_id) != 0)
        return -1;
    
    distance[port_id] = (S32)((data[port_id][1] << 8) | data[port_id][0]);
    ecrobot_read_i2c(port_id, 0x03, 0x42, data[port_id], 2);

    return distance[port_id];
}

