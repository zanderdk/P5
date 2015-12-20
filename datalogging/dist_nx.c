#include <stddef.h>
#include <string.h>
#include <math.h>

#include "ecrobot_base.h"
#include "ecrobot_private.h"
#include "ecrobot_interface.h"
#include "dist_nx.h"

double dist[4] = {-1, -1, -1, -1};
U8 sen[4] = {0,0,0,0};

double func(long double x, long double a, long double b){
    return a/(pow(x, b));
}

double calc2(long double x) {
    if(x > 1226.87)
        return func(x, 581127.6081355726, 1.1078247500891030605522820291876226945); 
    if(x > 835.973) 
        return func(x, 1656333.5741040043, 1.25345796169767853357774826542185982966); 
    
    long double val = func(x, 382502193234270160000000.0, 7.1927674628603778680252272435543044909);
    return (val > 800)? 800 : val;
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
    
    long double val = func(x, 10000700000000, 3.632572817825240);
    return (val > 900)? 900 : val;
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

void ecrobot_init_dist_sensor(U8 port_id, U8 range, U8 sensor)
{
    sen[port_id] = sensor;
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
    static U8 data[4][2] = {{0, 0}};
       
    if(i2c_busy(port_id) != 0)
        return -1;
    
    dist[port_id] = (double)((data[port_id][1] << 8) | data[port_id][0]);

    if(sen[port_id])
        dist[port_id] = calc(dist[port_id]);

    ecrobot_read_i2c(port_id, 0x03, 0x42 + sen[port_id]*2 , data[port_id], 2);

    return dist[port_id];
}

