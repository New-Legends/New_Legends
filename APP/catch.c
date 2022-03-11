#include "catch.h"


void catch_task(void const *argument)
{
    catch.init = catch_init;
    catch.init();             //初始化函数指针及参数
    while(1)
    {
        catch.sensor();       //空接传感器读取
        catch.set_mode();     //更改遥控器控制模式
        catch.control();      //更改电机控制模式
        catch.can_send();     //CAN协议发送
    }
}
