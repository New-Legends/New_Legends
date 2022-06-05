#include "catch.h"


void catch_task(void const *argument)
{
    catch.init = catch_init;
    catch.init();             //初始化函数指针及参数
    while(1)
    {
        electromagnet_control();
        oreflip_servo();
        catch.sensor();       //空接传感器读取
        // if(catch.auto_behave->auto_mode == 0)               //手动模式
        // {
            catch.set_mode();     //更改遥控器控制模式
            catch.control();      //更改电机控制模式
        // }else{                                              //自动模式
        //     catch_auto_control();
        // }
        
        catch.can_send();     //CAN协议发送
    }
}
 