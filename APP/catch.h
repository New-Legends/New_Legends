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
#include "CAN_receive.h"

//大结构体
typedef struct
{
    //遥控器指针
    const RC_ctrl_t *rc_data;
    const motor_measure_t *motor_measure[4];

    //函数指针定义
    void (*init)();
    void (*sensor)();
    void (*set_mode)();
    void (*control)();
    void (*can_send)();
    fp32 (*PID_calc)();

    int16_t  catch_sensor;

    //电机电流、电机状态存储
    struct
    {
        int16_t left;   //翻转左
        int16_t right;  //翻转右
        int16_t stretch;//出矿
        int16_t catch;  //夹紧
        int16_t left_target;   
        int16_t right_target;  
        int16_t stretch_target;
        int16_t catch_target;
        int16_t left_speed;   
        int16_t right_speed;  
        int16_t stretch_speed;
        int16_t catch_speed;  
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
    #define catch_state_is_shut         (catch.can.catch_state == shut)

    #define catch_sensor_0              (catch.catch_sensor == 0)
    #define catch_sensor_1              (catch.catch_sensor == 1)
}catch_ctrl_t;

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

enum
{
    stop    =   0,
    forward,
    reverse,
    stretch_out,
    stretch_back,
    close,
    open,
    shut
}catch_state;

catch_ctrl_t catch;

typedef struct
{
    uint8_t mode;
    //PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次

} catch_pid_strt;
catch_pid_strt catch_PID[4];

//一号电机PID
float CATCH_LEFT_KP     =   23.0f;
float CATCH_LEFT_KI     =   0.0f;
float CATCH_LEFT_KD     =   0.0f;
float CATCH_LEFT_MOUT   =   16000.0f;
float CATCH_LEFT_MIOUT  =   1.0f;
//二号电机PID
float CATCH_RIGHT_KP     =   23.0f;
float CATCH_RIGHT_KI     =   0.0f;
float CATCH_RIGHT_KD     =   0.0f;
float CATCH_RIGHT_MOUT   =   16000.0f;
float CATCH_RIGHT_MIOUT  =   1.0f;
//三号电机PID
float CATCH_STRETCH_KP     =   10.0f;
float CATCH_STRETCH_KI     =   0.0f;
float CATCH_STRETCH_KD     =   0.0f;
float CATCH_STRETCH_MOUT   =   16000.0f;
float CATCH_STRETCH_MIOUT  =   1.0f;
//四号电机PID
float CATCH_CATCH_KP     =   10.0f;
float CATCH_CATCH_KI     =   0.0f;
float CATCH_CATCH_KD     =   0.0f;
float CATCH_CATCH_MOUT   =   10000.0f;
float CATCH_CATCH_MIOUT  =   1.0f;    

//框架函数

//更改遥控器控制模式
void catch_set_mode(void)
{
    //翻转
    if (left_switch_is_up && right_switch_is_down)
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
        catch.can.flip_state =   shut;
        
    }

    //出爪
    if (left_switch_is_up && right_switch_is_mid)
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

    //夹紧
    if (left_switch_is_up && right_switch_is_up)
    {
        if (left_rocker_up)
        {
            catch.can.catch_state =   close;

        }
        
        if (left_rocker_mid)
        {
            catch.can.catch_state =   stop;

        }
        
        if (catch.catch_sensor == 0)
        {
            catch.can.catch_state =   close;

        }

        if (left_rocker_down)
        {
            catch.can.catch_state =   open;
            catch.catch_sensor == 1;

        }

    }
    //else
    //{
        //catch.can.catch_state =   stop;

    //}
    
}

//更改电机控制模式
void catch_control(void)
{
    catch.can.left_speed = catch.motor_measure[0]->speed_rpm;   
    catch.can.right_speed = catch.motor_measure[1]->speed_rpm;  
    catch.can.stretch_speed = catch.motor_measure[2]->speed_rpm;
    catch.can.catch_speed = catch.motor_measure[3]->speed_rpm; 
    
    if (flip_state_is_stop)
    {
        catch.can.left_target  =   0;
        catch.can.right_target =   0;
    }

    if (flip_state_is_forward)
    {
        catch.can.left_target  =   -20 * 19;
        catch.can.right_target =   20 * 19;
    }

    if (flip_state_is_reverse)
    {
        catch.can.left_target  =   20 * 19;
        catch.can.right_target =   -20 * 19;
    }
    catch.can.left  = (int16_t)catch.PID_calc(&catch_PID[0],catch.can.left_speed,catch.can.left_target);
    catch.can.right = (int16_t)catch.PID_calc(&catch_PID[1],catch.can.right_speed,catch.can.right_target);

    if (stretch_state_is_stop)
    {
        catch.can.stretch_target   =   0;
    }

    if (stretch_state_is_out)
    {
        catch.can.stretch_target   =   -40 * 19;
    }

    if (stretch_state_is_back)
    {
        catch.can.stretch_target   =   40 * 19;
    }
    catch.can.stretch = (int16_t)catch.PID_calc(&catch_PID[2],catch.can.stretch_speed,catch.can.stretch_target);

    if (catch_state_is_stop)
    {
        catch.can.catch_target   =   0;
    }

    if (catch_state_is_close)
    {
        catch.can.catch_target   =   -360 * 19;
    }

    if (catch_state_is_open)
    {
        catch.can.catch_target    =   360 * 19;
    }
    catch.can.catch = (int16_t)catch.PID_calc(&catch_PID[3],catch.can.catch_speed,catch.can.catch_target);
    
    //if (catch_state_is_shut)
    //{
    //    catch.can.catch    =   0;
    //}
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

//夹爪空接传感器
void catch_sensor(void)
{
    if (HAL_GPIO_ReadPin(Photogate_GPIO_Port, Photogate_Pin) == GPIO_PIN_RESET)
    {
        catch.catch_sensor  =   0;
    }

    if (HAL_GPIO_ReadPin(Photogate_GPIO_Port, Photogate_Pin) == GPIO_PIN_SET)
    {
        catch.catch_sensor  =   1;
    }
    
}

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

fp32 catch_PID_calc(catch_pid_strt *pid, int16_t ref, int16_t set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

void catch_PID_init(void)
{
    catch_PID[0].mode = PID_POSITION;
    catch_PID[0].Kp = CATCH_LEFT_KP;
    catch_PID[0].Ki = CATCH_LEFT_KI;
    catch_PID[0].Kd = CATCH_LEFT_KD;
    catch_PID[0].max_out = CATCH_LEFT_MOUT;
    catch_PID[0].max_iout = CATCH_LEFT_MIOUT;
    catch_PID[0].Dbuf[0] = catch_PID[0].Dbuf[1] = catch_PID[0].Dbuf[2] = 0.0f;
    catch_PID[0].error[0] = catch_PID[0].error[1] = catch_PID[0].error[2] = catch_PID[0].Pout = catch_PID[0].Iout = catch_PID[0].Dout = catch_PID[0].out = 0.0f;

    catch_PID[1].mode = PID_POSITION;
    catch_PID[1].Kp = CATCH_RIGHT_KP;
    catch_PID[1].Ki = CATCH_RIGHT_KI;
    catch_PID[1].Kd = CATCH_RIGHT_KD;
    catch_PID[1].max_out = CATCH_RIGHT_MOUT;
    catch_PID[1].max_iout = CATCH_RIGHT_MIOUT;
    catch_PID[1].Dbuf[0] = catch_PID[1].Dbuf[1] = catch_PID[1].Dbuf[2] = 0.0f;
    catch_PID[1].error[0] = catch_PID[1].error[1] = catch_PID[1].error[2] = catch_PID[1].Pout = catch_PID[1].Iout = catch_PID[1].Dout = catch_PID[1].out = 0.0f;

    catch_PID[2].mode = PID_POSITION;
    catch_PID[2].Kp = CATCH_STRETCH_KP;
    catch_PID[2].Ki = CATCH_STRETCH_KI;
    catch_PID[2].Kd = CATCH_STRETCH_KD;
    catch_PID[2].max_out = CATCH_STRETCH_MOUT;
    catch_PID[2].max_iout = CATCH_STRETCH_MIOUT;
    catch_PID[2].Dbuf[0] = catch_PID[2].Dbuf[1] = catch_PID[2].Dbuf[2] = 0.0f;
    catch_PID[2].error[0] = catch_PID[2].error[1] = catch_PID[2].error[2] = catch_PID[2].Pout = catch_PID[2].Iout = catch_PID[2].Dout = catch_PID[2].out = 0.0f;

    catch_PID[3].mode = PID_POSITION;
    catch_PID[3].Kp = CATCH_CATCH_KP;
    catch_PID[3].Ki = CATCH_CATCH_KI;
    catch_PID[3].Kd = CATCH_CATCH_KD;
    catch_PID[3].max_out = CATCH_CATCH_MOUT;
    catch_PID[3].max_iout = CATCH_CATCH_MIOUT;
    catch_PID[3].Dbuf[0] = catch_PID[3].Dbuf[1] = catch_PID[3].Dbuf[2] = 0.0f;
    catch_PID[3].error[0] = catch_PID[3].error[1] = catch_PID[3].error[2] = catch_PID[3].Pout = catch_PID[3].Iout = catch_PID[3].Dout = catch_PID[3].out = 0.0f;
}


//初始化函数指针及参数
void catch_init(void)
{
    catch.sensor    =   catch_sensor;
    catch.set_mode  =   catch_set_mode;
    catch.control   =   catch_control;
    catch.can_send  =   catch_can_send;
    catch.PID_calc  =   catch_PID_calc;

    catch.rc_data   =   get_remote_control_point();
    catch_PID_init();

    for (uint8_t i = 0; i < 4; i++)
    {
        catch.motor_measure[i] = get_motor_measure_point(i);
    }

    catch.can.flip_state =   stop;

}

#endif
