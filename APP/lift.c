#include "lift.h"


void Lifting_task(void const * argument)
{   
    strt.init   =   lift_init;
    strt.init();
    while(1)
    {
        strt.sensor();
        strt.lift_set_mode();
        strt.ore_set_mode();
        strt.lift_control();
        strt.ore_control();
        vTaskDelay(1);
        strt.can_send();
        vTaskDelay(3);
    }
}
