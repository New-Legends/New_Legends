#include "catch_auto.h"
#include "gpio.h"

auto_t auto_ctrl;

void catch_auto_init(void)
{
    auto_ctrl.rc_data = get_remote_control_point();

    auto_ctrl.press_flag = 1;
    auto_ctrl.auto_mode = 0;

    auto_ctrl.a_catch_mode = 0;
    auto_ctrl.a_takein_mode = 0;
    auto_ctrl.a_takeout_mode = 0;
    auto_ctrl.a_exchange_mode = 0;

    auto_ctrl.a_lift_target = 0;
    auto_ctrl.a_stretch_target = 0;
    // auto_ctrl.a_flip_target = 0.0f;
    auto_ctrl.a_catch_target = 0;

    for (int i = 0; i < 8; i++)
    {
        auto_ctrl.motor[i] = get_motor_measure_point(i);
    }

}

void sensor(void)
{
    if (HAL_GPIO_ReadPin(Photogate_GPIO_Port, Photogate_Pin) == GPIO_PIN_RESET)
    {
        auto_ctrl.photogate_1  =   0;
    }
    if (HAL_GPIO_ReadPin(Photogate_GPIO_Port, Photogate_Pin) == GPIO_PIN_SET)
    {
        auto_ctrl.photogate_1  =   1;
    }
}

void measure(void)
{   
    auto_ctrl.a_flip_angle = 1.0*(auto_ctrl.motor[0]->round*360)/19+1.0*(auto_ctrl.motor[0]->ecd*360)/19/8192;
    auto_ctrl.a_stretch_angle = 1.0*(auto_ctrl.motor[2]->round*360)/19+1.0*(auto_ctrl.motor[2]->ecd*360)/19/8192;
    auto_ctrl.a_lift_angle = 1.0*(auto_ctrl.motor[4]->round*360)/19+1.0*(auto_ctrl.motor[4]->ecd*360)/19/8192;
}

void auto_set_mode(void)
{
    auto_ctrl.last_press_flag = auto_ctrl.press_flag;
    if(auto_ctrl.rc_data->key.v == KEY_PRESSED_OFFSET_R)
    {
        auto_ctrl.press_flag = 1;
    }else{
        auto_ctrl.press_flag = 0;
    }

    if(auto_ctrl.last_press_flag != auto_ctrl.press_flag && auto_ctrl.press_flag == 1)
    {
        if(auto_ctrl.auto_mode == 1)
        {
            auto_ctrl.auto_mode = 0;
        }else{
            auto_ctrl.auto_mode = 1;
        }
    }

    if(auto_ctrl.auto_mode == 1)
    {
        // 自动取矿
        if(auto_ctrl.rc_data->key.v == KEY_PRESSED_OFFSET_Z)
        {
            if(auto_ctrl.a_catch_mode == 0 && auto_ctrl.a_takein_mode == 0 && auto_ctrl.a_takeout_mode == 0 && auto_ctrl.a_exchange_mode == 0)
            {
                auto_ctrl.a_catch_mode = 1;
            }
        }
        // 自动收纳
        if(auto_ctrl.rc_data->key.v == KEY_PRESSED_OFFSET_X)
        {
            if(auto_ctrl.a_catch_mode == 0 && auto_ctrl.a_takein_mode == 0 && auto_ctrl.a_takeout_mode == 0 && auto_ctrl.a_exchange_mode == 0)
            {
                auto_ctrl.a_takein_mode = 1;
            }
        }
        // 自动取出
        if(auto_ctrl.rc_data->key.v == KEY_PRESSED_OFFSET_C)
        {
            if(auto_ctrl.a_catch_mode == 0 && auto_ctrl.a_takein_mode == 0 && auto_ctrl.a_takeout_mode == 0 && auto_ctrl.a_exchange_mode == 0)
            {
                auto_ctrl.a_takeout_mode = 1;
            }
        }
        // 自动兑换
        if(auto_ctrl.rc_data->key.v == KEY_PRESSED_OFFSET_B)
        {
            if(auto_ctrl.a_catch_mode == 0 && auto_ctrl.a_takein_mode == 0 && auto_ctrl.a_takeout_mode == 0 && auto_ctrl.a_exchange_mode == 0)
            {
                auto_ctrl.a_exchange_mode = 1;
            }
        }
    }else{

        auto_ctrl.a_catch_mode = 0;
        auto_ctrl.a_takein_mode = 0;
        auto_ctrl.a_takeout_mode = 0;
        auto_ctrl.a_exchange_mode = 0;
    }
}

void auto_control(void)
{
    // 自动取矿
    if(auto_ctrl.a_catch_mode == 1)
    {
        auto_ctrl.a_lift_target = -200.0f;
        auto_ctrl.a_stretch_target = 700.0f;
        auto_ctrl.a_flip_target = 95;
        if(auto_ctrl.photogate_1 == 0)
        {
            auto_ctrl.a_catch_target = 1;
        }else{
            auto_ctrl.a_catch_target = 0;
        }
    }
    // 自动收纳
    if(auto_ctrl.a_takein_mode == 1)
    {
        // auto_ctrl.a_lift_target = 0;
        // auto_ctrl.a_stretch_target = 0;
        // auto_ctrl.a_flip_target = 0;
        // auto_ctrl.a_catch_target = 0;
    }
    // 自动取出
    if(auto_ctrl.a_takeout_mode == 1)
    {
        // auto_ctrl.a_lift_target = 0;
        // auto_ctrl.a_stretch_target = 0;
        // auto_ctrl.a_flip_target = 0;
        // auto_ctrl.a_catch_target = 0;
    }
    // 自动兑换
    if(auto_ctrl.a_exchange_mode == 1)
    {
        // auto_ctrl.a_lift_target = 0;
        // auto_ctrl.a_stretch_target = 0;
        // auto_ctrl.a_flip_target = 0;
        // auto_ctrl.a_catch_target = 0;
    }
}

void auto_set_angle(void)
{
    // if(auto_ctrl.a_catch_mode == 1)
    // {
    //     auto_ctrl.a_lift_target = 0;
    //     auto_ctrl.a_stretch_target = 0;
    //     auto_ctrl.a_flip_target = 0;
    //     auto_ctrl.a_catch_target = 0;
    //     if(auto_ctrl.photogate_1 == 0)
    //     {
    //         auto_ctrl.a_catch_target = 1;
    //     }
    // }

    // if(auto_ctrl.a_takein_mode == 1)
    // {
    //     auto_ctrl.a_lift_target = 0;
    //     auto_ctrl.a_stretch_target = 0;
    //     auto_ctrl.a_flip_target = 0;
    //     auto_ctrl.a_catch_target = 0;
    // }

    // if(auto_ctrl.a_takeout_mode == 1)
    // {
    //     auto_ctrl.a_lift_target = 0;
    //     auto_ctrl.a_stretch_target = 0;
    //     auto_ctrl.a_flip_target = 0;
    //     auto_ctrl.a_catch_target = 0;
    // }

    // if(auto_ctrl.a_exchange_mode == 1)
    // {
    //     auto_ctrl.a_lift_target = 0;
    //     auto_ctrl.a_stretch_target = 0;
    //     auto_ctrl.a_flip_target = 0;
    //     auto_ctrl.a_catch_target = 0;
    // }
}

const auto_t *get_auto_control_point(void)
{
    return &auto_ctrl;
}

void StartDefaultTask(void const * argument)
{
    catch_auto_init();
    while (1)
    {
        sensor();
        measure();
        auto_control();
        auto_set_mode();
    }
}
