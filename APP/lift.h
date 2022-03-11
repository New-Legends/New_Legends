#ifndef LIFT_H
#define LIFT_H

#include "remote_control.h"
#include "can.h"
#include "CAN_receive.h"
#include "struct_typedef.h"

//大结构体
typedef struct
{

    const RC_ctrl_t *rc_data;
    const motor_measure_t *motor_measure[4];

    //函数指针定义
    void (*init)();
    void (*pid_init)();
    void (*lift_set_mode)();
    void (*lift_control)();  
    void (*ore_set_mode)();
    void (*ore_control)(); 
    fp32 (*PID_calc)();

    void (*can_send)();
    void (*sensor)();

    //电机电流、电机状态存储
    struct
    {   
        struct
        {
            int16_t left;
            int16_t right;
            int16_t left_target;
            int16_t right_target;
            int16_t left_speed;
            int16_t right_speed;
            int     state;
        }lift;
        
        struct
        {
            int16_t left;
            int16_t right;
            int16_t left_target;
            int16_t right_target;
            int     state;
        }ore;

    }can;

    struct
    {
        
        int8_t    photogate_1;
        int8_t    photogate_2;

    }sensor_data;
    

    //遥控器状态命名
    #define left_switch_is_up           (strt.rc_data->rc.s[1] == 1)
    #define left_switch_is_mid          (strt.rc_data->rc.s[1] == 3)
    #define left_switch_is_down         (strt.rc_data->rc.s[1] == 2)
    #define right_switch_is_up          (strt.rc_data->rc.s[0] == 1)
    #define right_switch_is_mid         (strt.rc_data->rc.s[0] == 3)
    #define right_switch_is_down        (strt.rc_data->rc.s[0] == 2)

    #define left_rocker_up              (strt.rc_data->rc.ch[3] > 0)
    #define left_rocker_down            (strt.rc_data->rc.ch[3] < 0)
    #define left_rocker_mid             (strt.rc_data->rc.ch[3] == 0)

    //电机状态命名
    #define state_is_stop               (strt.can.lift.state == stop)
    #define state_is_up                 (strt.can.lift.state == up)
    #define state_is_down               (strt.can.lift.state == down)
    #define state_is_shut               (strt.can.lift.state == shut)
    
    #define ore_state_is_stop           (strt.can.ore.state == stop)
    #define ore_state_is_in             (strt.can.ore.state == in)
    #define ore_state_is_out            (strt.can.ore.state == out)

}strt_t;

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

//PID算法结构体
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

} lift_pid_strt;
lift_pid_strt lift_PID[4];

//一号电机PID
float LIFT_LEFT_KP     =   11.0f;
float LIFT_LEFT_KI     =   0.0f;
float LIFT_LEFT_KD     =   0.1f;
float LIFT_LEFT_MOUT   =   16000.0f;
float LIFT_LEFT_MIOUT  =   1.0f;
//二号电机PID
float LIFT_RIGHT_KP     =   11.0f;
float LIFT_RIGHT_KI     =   0.0f;
float LIFT_RIGHT_KD     =   0.1f;
float LIFT_RIGHT_MOUT   =   16000.0f;
float LIFT_RIGHT_MIOUT  =   1.0f;
//三号电机PID
float ORE_LEFT_KP     =   100.0f;
float ORE_LEFT_KI     =   0.0f;
float ORE_LEFT_KD     =   0.0f;
float ORE_LEFT_MOUT   =   16000.0f;
float ORE_LEFT_MIOUT  =   1.0f;
//四号电机PID
float ORE_RIGHT_KP     =   100.0f;
float ORE_RIGHT_KI     =   0.0f;
float ORE_RIGHT_KD     =   0.0f;
float ORE_RIGHT_MOUT   =   16000.0f;
float ORE_RIGHT_MIOUT  =   1.0f;    


enum
{
    motor_left_ID = 0x201,
    motor_right_ID = 0x202,
    ore_motor_left_ID = 0x203,
    ore_motor_right_ID = 0x204,
}motor_ID;

enum
{
    stop    =   0,
    up,
    down,
    in,
    out,
    shut
}state_type;

strt_t    strt;

/************函数开始*************/
void lift_set_mode(void)
{
    if(left_switch_is_mid && right_switch_is_mid)     
    {
        if (left_rocker_up)
        {
            strt.can.lift.state = up;

        }
        else if(left_rocker_down)
        {
            strt.can.lift.state = down;

        }
        else
        {
            strt.can.lift.state = stop;

        }

    }else
    {
        strt.can.lift.state = shut;

    }

}

void ore_set_mode(void)
{
    if(left_switch_is_mid && right_switch_is_up)     
    {
       if (left_rocker_up)//strt.sensor_data.photogate_1
       {
           strt.can.ore.state   =   in;

       }else if(left_rocker_down)//strt.sensor_data.photogate_1 == 0
       {
           strt.can.ore.state   =   out;

       }else{
           strt.can.ore.state   =   stop;

       }

    }
    else
    {
        strt.can.ore.state   =  stop;
    }

}

void lift_control(void)
{   
    strt.can.lift.left_speed    = strt.motor_measure[0]->speed_rpm;
    strt.can.lift.right_speed   = strt.motor_measure[1]->speed_rpm;

    strt.can.lift.left  = 0;
    strt.can.lift.right = 0;

    if (state_is_stop)
    {
        strt.can.lift.left_target   =   -13 * 19;
        strt.can.lift.right_target  =   13 * 19;
    }
    if (state_is_up)
    {
        strt.can.lift.left_target   =   -40 * 19;
        strt.can.lift.right_target  =   40 * 19;
        
    }
    if (state_is_down)
    {
        strt.can.lift.left_target   =   40 * 19;
        strt.can.lift.right_target  =   -40 * 19;
        
    }
    if (state_is_shut)
    {
        strt.can.lift.left_target   =   -13 * 19;
        strt.can.lift.right_target  =   13 * 19;
    }

    strt.can.lift.left = (int16_t)strt.PID_calc(&lift_PID[0],strt.can.lift.left_speed,strt.can.lift.left_target);
    strt.can.lift.right = (int16_t)strt.PID_calc(&lift_PID[1],strt.can.lift.right_speed,strt.can.lift.right_target);
    
}

void ore_control(void)
{
    if (ore_state_is_stop)
    {
        strt.can.ore.left_target   =   0;
        strt.can.ore.right_target  =   0;
    }

    if (ore_state_is_in)
    {
        strt.can.ore.left_target   =   1000;
        strt.can.ore.right_target  =   -1000;
    }

    if (ore_state_is_out)
    {
        strt.can.ore.left_target   =   -1000;
        strt.can.ore.right_target  =   1000;
    }

}

void sensor(void)
{
    if (HAL_GPIO_ReadPin(Photogate_GPIO_Port, Photogate_Pin) == GPIO_PIN_RESET)
    {
        strt.sensor_data.photogate_1  =   0;
    }

    if (HAL_GPIO_ReadPin(Photogate_GPIO_Port, Photogate_Pin) == GPIO_PIN_SET)
    {
        strt.sensor_data.photogate_1  =   1;
    }
    
}


static CAN_TxHeaderTypeDef  can_tx_message;
static uint8_t              lift_can_send_data[8];
void can_send(void)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = 0x200;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    lift_can_send_data[0] = strt.can.lift.left >> 8;
    lift_can_send_data[1] = strt.can.lift.left;
    lift_can_send_data[2] = strt.can.lift.right >> 8;
    lift_can_send_data[3] = strt.can.lift.right;
    lift_can_send_data[4] = strt.can.ore.left >> 8;
    lift_can_send_data[5] = strt.can.ore.left;
    lift_can_send_data[6] = strt.can.ore.right >> 8;
    lift_can_send_data[7] = strt.can.ore.right;

    HAL_CAN_AddTxMessage(&hcan1, &can_tx_message, lift_can_send_data, &send_mail_box);
}

void lift_PID_init(void)
{
    lift_PID[0].mode = PID_POSITION;
    lift_PID[0].Kp = LIFT_LEFT_KP;
    lift_PID[0].Ki = LIFT_LEFT_KI;
    lift_PID[0].Kd = LIFT_LEFT_KD;
    lift_PID[0].max_out = LIFT_LEFT_MOUT;
    lift_PID[0].max_iout = LIFT_LEFT_MIOUT;
    lift_PID[0].Dbuf[0] = lift_PID[0].Dbuf[1] = lift_PID[0].Dbuf[2] = 0.0f;
    lift_PID[0].error[0] = lift_PID[0].error[1] = lift_PID[0].error[2] = lift_PID[0].Pout = lift_PID[0].Iout = lift_PID[0].Dout = lift_PID[0].out = 0.0f;

    lift_PID[1].mode = PID_POSITION;
    lift_PID[1].Kp = LIFT_RIGHT_KP;
    lift_PID[1].Ki = LIFT_RIGHT_KI;
    lift_PID[1].Kd = LIFT_RIGHT_KD;
    lift_PID[1].max_out = LIFT_RIGHT_MOUT;
    lift_PID[1].max_iout = LIFT_RIGHT_MIOUT;
    lift_PID[1].Dbuf[0] = lift_PID[1].Dbuf[1] = lift_PID[1].Dbuf[2] = 0.0f;
    lift_PID[1].error[0] = lift_PID[1].error[1] = lift_PID[1].error[2] = lift_PID[1].Pout = lift_PID[1].Iout = lift_PID[1].Dout = lift_PID[1].out = 0.0f;

    lift_PID[2].mode = PID_POSITION;
    lift_PID[2].Kp = ORE_LEFT_KP;
    lift_PID[2].Ki = ORE_LEFT_KI;
    lift_PID[2].Kd = ORE_LEFT_KD;
    lift_PID[2].max_out = ORE_LEFT_MOUT;
    lift_PID[2].max_iout = ORE_LEFT_MIOUT;
    lift_PID[2].Dbuf[0] = lift_PID[2].Dbuf[1] = lift_PID[2].Dbuf[2] = 0.0f;
    lift_PID[2].error[0] = lift_PID[2].error[1] = lift_PID[2].error[2] = lift_PID[2].Pout = lift_PID[2].Iout = lift_PID[2].Dout = lift_PID[2].out = 0.0f;

    lift_PID[3].mode = PID_POSITION;
    lift_PID[3].Kp = ORE_RIGHT_KP;
    lift_PID[3].Ki = ORE_RIGHT_KI;
    lift_PID[3].Kd = ORE_RIGHT_KD;
    lift_PID[3].max_out = ORE_RIGHT_MOUT;
    lift_PID[3].max_iout = ORE_RIGHT_MIOUT;
    lift_PID[3].Dbuf[0] = lift_PID[3].Dbuf[1] = lift_PID[3].Dbuf[2] = 0.0f;
    lift_PID[3].error[0] = lift_PID[3].error[1] = lift_PID[3].error[2] = lift_PID[3].Pout = lift_PID[3].Iout = lift_PID[3].Dout = lift_PID[3].out = 0.0f;
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

fp32 lift_PID_calc(lift_pid_strt *pid, int16_t ref, int16_t set)
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

//万恶的PID至此结束
void lift_init(void)
{
    strt.rc_data    =   get_remote_control_point();

    strt.lift_set_mode  =   lift_set_mode;
    strt.lift_control   =   lift_control;
    
    strt.ore_set_mode   =   ore_set_mode;
    strt.ore_control    =   ore_control;
    strt.can_send       =   can_send;
    strt.sensor         =   sensor;
    strt.can.lift.state = stop;
    strt.can.ore.state = stop;

    for (uint8_t i = 0; i < 4; i++)
    {
        strt.motor_measure[i] = get_motor_measure_point(i+4);
    }


    strt.pid_init   =   lift_PID_init;
    strt.pid_init();
    strt.PID_calc   =   lift_PID_calc;

    strt.sensor_data.photogate_1  =   0;
}

#endif

