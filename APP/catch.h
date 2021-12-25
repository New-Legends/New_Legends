/*      宁波工程学院    电气201         张超
        2021.12.6      工程机器人      夹爪
        使用函数指针及模块化编程
        
        1.大结构体，用于存储除枚举类型数据，以便其他文件读取
        2.框架函数，仿照官方C型开发板步兵代码编写，以便后期添加功能及更改
        3.功能函数，由基本功能函数修改而来，完成串口收发、PID算法等工作
*/
#ifndef CATCH_H
#define CATCH_H

//头文件引用
#include "remote_control.h"
#include "struct_typedef.h"
#include "can.h"

//大结构体
typedef struct
{
    //遥控器指针
    const RC_ctrl_t *rc_data;

    //函数指针定义
    void (*init)();
    void (*set_mode)();
    void (*control)();
    void (*can_send)();

    //电机电流、电机状态存储
    struct
    {
        int16_t left;
        int16_t right;
        int16_t stretch;
        int16_t catch;
        int     flip_state;
        int     stretch_state;
        int     catch_state;
    }can;
    

    //遥控器状态命名
    #define left_switch_is_up           (catch.rc_data->rc.s[1] == 1)
    #define left_switch_is_mid          (catch.rc_data->rc.s[1] == 3)
    #define left_switch_is_down         (catch.rc_data->rc.s[1] == 2)
    #define right_switch_is_up          (catch.rc_data->rc.s[0] == 1)
    #define right_switch_is_mid         (catch.rc_data->rc.s[0] == 3)
    #define right_switch_is_down        (catch.rc_data->rc.s[0] == 2)

    #define left_rocker_up              (catch.rc_data->rc.ch[3] > 0)
    #define left_rocker_down            (catch.rc_data->rc.ch[3] < 0)
    #define left_rocker_mid             (catch.rc_data->rc.ch[3] == 0)


    //电机状态命名
    #define flip_state_is_stop               (catch.can.flip_state == stop)
    #define flip_state_is_forward            (catch.can.flip_state == forward)
    #define flip_state_is_reverse            (catch.can.flip_state == reverse)

    #define stretch_state_is_stop       (catch.can.stretch_state == stop)
    #define stretch_state_is_out        (catch.can.stretch_state == stretch_out)
    #define stretch_state_is_back       (catch.can.stretch_state == stretch_back)

    #define catch_state_is_stop         (catch.can.catch_state == stop)
    #define catch_state_is_open         (catch.can.catch_state == close)
    #define catch_state_is_close        (catch.can.catch_state == open)

}catch_ctrl_t;


typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} catch_measure_t;


enum
{
    motor_left_ID = 0x201,
    motor_right_ID = 0x202,
}catch_ID;


enum
{
    stop    =   0,
    forward,
    reverse,
    stretch_out,
    stretch_back,
    close,
    open,
}catch_state;


catch_ctrl_t catch;

//框架函数

//更改遥控器控制模式
void catch_set_mode(void)
{

    if (left_switch_is_down)
    {

        if (left_rocker_up)
        {
            catch.can.flip_state =   forward;
        }

        if (left_rocker_down)
        {
            catch.can.flip_state =   reverse;
        }

        if (left_rocker_mid)
        {
            catch.can.flip_state =   stop;
        }

    }
    else
    {
        catch.can.flip_state =   stop;
    }


    if (right_switch_is_up)
    {
        if (left_rocker_up)
        {
            catch.can.stretch_state =   stretch_out;
        }

        if (left_rocker_down)
        {
            catch.can.stretch_state =   stretch_back;
        }
        
        if (left_rocker_mid)
        {
            catch.can.stretch_state =   stop;
        }
        
        
    }
    else
    {
        catch.can.stretch_state =   stop;
    }

    if (right_switch_is_mid)
    {
        if (left_rocker_up)
        {
            catch.can.catch_state =   close;
        }

        if (left_rocker_down)
        {
            catch.can.catch_state =   open;
        }
        
        if (left_rocker_mid)
        {
            catch.can.catch_state =   stop;
        }
        
        
    }
    else
    {
        catch.can.catch_state =   stop;
    }
    
}

//更改电机控制模式
void catch_control(void)
{

    if (flip_state_is_stop)
    {
        catch.can.left  =   0;
        catch.can.right =   0;
    }

    if (flip_state_is_forward)
    {
        catch.can.left  =   1000;
        catch.can.right =   -1000;
    }

    if (flip_state_is_reverse)
    {
        catch.can.left  =   -1000;
        catch.can.right =   1000;
    }


    if (stretch_state_is_stop)
    {
        catch.can.stretch   =   0;
    }

    if (stretch_state_is_out)
    {
        catch.can.stretch   =   1000;
    }

    if (stretch_state_is_back)
    {
        catch.can.stretch   =   -1000;
    }


    if (catch_state_is_stop)
    {
        catch.can.catch   =   0;
    }

    if (catch_state_is_close)
    {
        catch.can.catch   =   1000;
    }

    if (catch_state_is_open)
    {
        catch.can.catch    =   -1000;
    }
}

//CAN协议发送
static CAN_TxHeaderTypeDef  can_tx_message;
static uint8_t              catch_can_send_data[8];
void catch_can_send(void)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = 0x200;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    catch_can_send_data[0] = catch.can.left >> 8;
    catch_can_send_data[1] = catch.can.left;
    catch_can_send_data[2] = catch.can.right >> 8;
    catch_can_send_data[3] = catch.can.right;
    catch_can_send_data[4] = catch.can.stretch >> 8;
    catch_can_send_data[5] = catch.can.stretch;
    catch_can_send_data[6] = catch.can.catch >> 8;
    catch_can_send_data[7] = catch.can.catch;

    HAL_CAN_AddTxMessage(&hcan2, &can_tx_message, catch_can_send_data, &send_mail_box);
}

//初始化函数指针及参数
void catch_init(void)
{
    catch.set_mode  =   catch_set_mode;
    catch.control   =   catch_control;
    catch.can_send  =   catch_can_send;

    catch.rc_data   =   get_remote_control_point();

    catch.can.flip_state =   stop;

}



//功能函数

catch_measure_t motor_catch[2];

#define get_motor_measure(ptr, data)                                   \
{                                                                      \
    (ptr)->last_ecd = (ptr)->ecd;                                      \
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);               \
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);         \
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);     \
    (ptr)->temperate = (data)[6];                                      \
}

void get_catch_measure(void)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx_header, rx_data);
    switch (rx_header.StdId)
    {
        case motor_left_ID:
        case motor_right_ID:
        {
            static uint8_t i = 0;
            //get motor id
            i = rx_header.StdId - motor_left_ID;
            get_motor_measure(&motor_catch[i], rx_data);
            break;
        }
        default:
        {
            break;
        }
    }
 }



#endif
