#include "lift.h"


void Lifting_task(void const * argument)
{   
    strt.init   =   lift_init;
    strt.init();
    while(1)
    {
        strt.lift_lenth = 1.0*(strt.motor_measure[0]->round*360)/19+1.0*(strt.motor_measure[0]->ecd*360)/19/8192;
        // strt.sensor();
        // if(strt.auto_behave->auto_mode == 0)
        // {
            strt.lift_set_mode();
            strt.lift_control();
        // }else{

        // }
        
        strt.ore_set_mode();
        strt.ore_control();
        vTaskDelay(1);
        strt.can_send();
        vTaskDelay(3);
    }
}
