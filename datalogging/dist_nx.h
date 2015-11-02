#include "ecrobot_types.h"

#define RANGE_NORMAL     0
#define RANGE_SHORT      1
#define RANGE_MEDIUM     2
#define RANGE_LONG       4

/* NXT medium range distance sensor API */
extern void ecrobot_init_dist_sensor(U8 port_id, U8 range, U8 sen);
extern double ecrobot_get_dist_sensor(U8 port_id);
extern void ecrobot_term_dist_sensor(U8 port_id);
extern U16 ecrobot_get_obstical_detection_sensor(U8 port_id);
extern void ecrobot_init_obstical_detection_sensor(U8 port_id, U8 range);
extern double calc(long double x);
