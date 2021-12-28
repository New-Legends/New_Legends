#include "chassis.h"
#include "main.h"

void chassis_task(void const * argument)
{   
    chassis.init    =   chassis_init;
    chassis.init();
    while(1)
    {
        chassis.set_mode();
        chassis.control();
        chassis.can_send();
        vTaskDelay(10);
    }
}

