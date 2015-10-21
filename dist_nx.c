#include <stddef.h>
#include <string.h>
#include <math.h>

#include "ecrobot_base.h"
#include "ecrobot_private.h"
#include "ecrobot_interface.h"
#include "dist_nx.h"

double dist[4] = {-1, -1, -1, -1};

double func(long double x, long double a, long double b){
    return a/(pow(x, b));
}

double calc(long double x) {
    if(x > 1193.3)
        return func(x, 775273, 1.152850108742073); 
    if(x > 777.504) 
        return func(x, 862622, 1.169099132247283); 
    if(x > 667.248)
        return func(x, 320247000, 2.055922469360928); 
    if(x > 643.784)
        return func(x, 1236340000000000000, 5.442282287396021);
    
    return func(x, 10000700000000, 3.632572817825240); 
}

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

double ecrobot_get_dist_sensor(U8 port_id)
{
    static U8 data[2] = {0, 0};
       
    if(i2c_busy(port_id) != 0)
        return -1;
    
    dist[port_id] = (double)((data[1] << 8) | data[0]);
    dist[port_id] = calc(dist[port_id]);

    ecrobot_read_i2c(port_id, 0x03, 0x44, data, 2);

    return dist[port_id];
}

