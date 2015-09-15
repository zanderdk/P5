#include <stddef.h>
#include <string.h>

#include "ecrobot_base.h"
#include "ecrobot_private.h"
#include "ecrobot_interface.h"
#include "dist_nx.h"

S32 dist[4] = {-1, -1, -1, -1};

void ecrobot_init_obstical_detection_sensor(U8 port_id, U8 range)
{
    if(range == RANGE_SHORT)
    {
        set_digi0(port_id);
    }
    if(range == RANGE_LONG)
    {
	    unset_digi0(port_id);
    }
}

U16 ecrobot_get_obstical_detection_sensor(U8 port_id)
{
	return (U16)sensor_adc(port_id);
}

void ecrobot_init_dist_sensor(U8 port_id, U8 range)
{
    U8 buf = 0x31 + range;
    ecrobot_init_i2c(port_id, LOWSPEED);
    ecrobot_send_i2c(port_id, 0x03, 0x41, &buf, 1);
}

void ecrobot_term_dist_sensor(U8 port_id)
{
    ecrobot_term_i2c(port_id);
    U8 i;
    for(i = 0; i < 4; i++)
    {
        dist[i] = -1;
    }
}

S32 ecrobot_get_dist_sensor(U8 port_id)
{
    static U8 data[4][2] = {0, 0};
       
    if(i2c_busy(port_id) != 0)
        return -1;
    
    dist[port_id] = (S32)((data[port_id][1] << 8) | data[port_id][0]);
    ecrobot_read_i2c(port_id, 0x03, 0x42, data[port_id], 2);

    return dist[port_id];
}

