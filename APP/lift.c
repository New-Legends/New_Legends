#include "lift.h"


void Lifting_task(void const * argument)
{   
    strt.init   =   lift_init;
    strt.init();
    while(1)
    {
        strt.measure();
        strt.sensor();
        strt.lift_set_mode();
        strt.ore_set_mode();
        strt.lift_control();
        strt.ore_control();
        strt.can_send();
    }
}
