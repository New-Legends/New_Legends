#include "catch.h"


void catch_task(void const *argument)
{
    catch.init = catch_init;
    catch.init();             //鍒濆�嬪寲鍑芥暟鎸囬拡鍙婂弬鏁�
    while(1)
    {
        catch.sensor();       //绌烘帴浼犳劅鍣ㄨ�诲彇
        catch.set_mode();     //鏇存敼閬ユ帶鍣ㄦ帶鍒舵ā寮�
        catch.control();      //鏇存敼鐢垫満鎺у埗妯″紡
        catch.can_send();     //CAN鍗忚��鍙戦€�
    }
}
