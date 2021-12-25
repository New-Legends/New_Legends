#ifndef LIFT_H
#define LIFT_H

#include "remote_control.h"
#include "can.h"


//大结构体
typedef struct
{

    const RC_ctrl_t *rc_data;


    //函数指针定义
    void (*init)();
    void (*pid_init)();
    void (*lift_set_mode)();
    void (*lift_control)();  
    void (*measure)();
    void (*ore_set_mode)();
    void (*ore_control)(); 
    fp32 (*pid_calc)();

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
    
    #define ore_state_is_stop           (strt.can.ore.state == stop)
    #define ore_state_is_in             (strt.can.ore.state == in)
    #define ore_state_is_out            (strt.can.ore.state == out)

}strt_t;

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

//PID参数
#define     lift_left_speed_KP  0.0
#define     lift_left_speed_KI  0.0
#define     lift_left_speed_KD  0.0

#define     lift_left_speed_max_out   0.0
#define     lift_left_speed_max_iout  0.0

#define     lift_left_location_KP  0.0
#define     lift_left_location_KI  0.0
#define     lift_left_location_KD  0.0

#define     lift_left_location_max_out   0.0
#define     lift_left_location_max_iout  0.0

#define     lift_right_speed_KP  0.0
#define     lift_right_speed_KI  0.0
#define     lift_right_speed_KD  0.0

#define     lift_right_speed_max_out   0.0
#define     lift_right_speed_max_iout  0.0

#define     lift_right_location_KP  0.0
#define     lift_right_location_KI  0.0
#define     lift_right_location_KD  0.0

#define     lift_right_location_max_out   0.0
#define     lift_right_location_max_iout  0.0


#define     ore_left_speed_KP  0.0
#define     ore_left_speed_KI  0.0
#define     ore_left_speed_KD  0.0

#define     ore_left_speed_max_out   0.0
#define     ore_left_speed_max_iout  0.0

#define     ore_right_speed_KP  0.0
#define     ore_right_speed_KI  0.0
#define     ore_right_speed_KD  0.0

#define     ore_right_speed_max_out   0.0
#define     ore_right_speed_max_iout  0.0

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

} pid_lift_def;
pid_lift_def    pid_speed[4];
pid_lift_def    pid_location[2];

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} lift_measure_t;


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
}state_type;

strt_t    strt;

/************函数开始*************/
void lift_set_mode(void)
{
    if(left_switch_is_up)     
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
        strt.can.lift.state = stop;
    }

}

void ore_set_mode(void)
{
    if(left_switch_is_mid)     
    {
       if (strt.sensor_data.photogate_1)
       {
           strt.can.ore.state   =   in;
       }
       if (strt.sensor_data.photogate_1 == 0)
       {
           strt.can.ore.state   =   out;
       }
       
    }
    else
    {
        strt.can.ore.state   =  stop;
    }

}

void lift_control(void)
{
    if (state_is_stop)
    {
        strt.can.lift.left_target   =   0;
        strt.can.lift.right_target  =   strt.can.lift.left_target;
    }
    if (state_is_up)
    {
        strt.can.lift.left_target   =   0;
        strt.can.lift.right_target  =   strt.can.lift.left_target;
    }
    if (state_is_down)
    {
        strt.can.lift.left_target   =   0;
        strt.can.lift.right_target  =   strt.can.lift.left_target;
    }
    strt.can.lift.left  =   strt.pid_calc(pid_location[0],strt.rc_data->ranging_data,strt.can.lift.left_target);
    strt.can.lift.left  =   strt.pid_calc(pid_location[0],motor_lift[0].speed_rpm,strt.can.lift.left);
    
    strt.can.lift.right  =   strt.pid_calc(pid_location[1],strt.rc_data->ranging_data,strt.can.lift.right_target);
    strt.can.lift.right  =   strt.pid_calc(pid_location[1],motor_lift[1].speed_rpm,strt.can.lift.right);
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

//万恶的PID从此开始
void lift_PID_init(void)
{
    pid_lift_def *lift_left_speed   =   &pid_speed[0];
    lift_left_speed->mode = PID_DELTA;
    lift_left_speed->Kp = lift_left_speed_KP;
    lift_left_speed->Ki = lift_left_speed_KI;
    lift_left_speed->Kd = lift_left_speed_KD;
    lift_left_speed->max_out = lift_left_speed_max_out;
    lift_left_speed->max_iout = lift_left_speed_max_iout;
    lift_left_speed->Dbuf[0] = lift_left_speed->Dbuf[1] = lift_left_speed->Dbuf[2] = 0.0f;
    lift_left_speed->error[0] = lift_left_speed->error[1] = lift_left_speed->error[2] = lift_left_speed->Pout = lift_left_speed->Iout = lift_left_speed->Dout = lift_left_speed->out = 0.0f;

    pid_lift_def *lift_right_speed   =   &pid_speed[1];
    lift_right_speed->mode = PID_DELTA;
    lift_right_speed->Kp = lift_right_speed_KP;
    lift_right_speed->Ki = lift_right_speed_KI;
    lift_right_speed->Kd = lift_right_speed_KD;
    lift_right_speed->max_out = lift_right_speed_max_out;
    lift_right_speed->max_iout = lift_right_speed_max_iout;
    lift_right_speed->Dbuf[0] = lift_right_speed->Dbuf[1] = lift_right_speed->Dbuf[2] = 0.0f;
    lift_right_speed->error[0] = lift_right_speed->error[1] = lift_right_speed->error[2] = lift_right_speed->Pout = lift_right_speed->Iout = lift_right_speed->Dout = lift_right_speed->out = 0.0f;

    pid_lift_def *ore_left_speed   =   &pid_speed[2];
    ore_left_speed->mode = PID_DELTA;
    ore_left_speed->Kp = ore_left_speed_KP;
    ore_left_speed->Ki = ore_left_speed_KI;
    ore_left_speed->Kd = ore_left_speed_KD;
    ore_left_speed->max_out = ore_left_speed_max_out;
    ore_left_speed->max_iout = ore_left_speed_max_iout;
    ore_left_speed->Dbuf[0] = ore_left_speed->Dbuf[1] = ore_left_speed->Dbuf[2] = 0.0f;
    ore_left_speed->error[0] = ore_left_speed->error[1] = ore_left_speed->error[2] = ore_left_speed->Pout = ore_left_speed->Iout = ore_left_speed->Dout = ore_left_speed->out = 0.0f;

    pid_lift_def *ore_right_speed   =   &pid_speed[3];
    ore_right_speed->mode = PID_DELTA;
    ore_right_speed->Kp = ore_right_speed_KP;
    ore_right_speed->Ki = ore_right_speed_KI;
    ore_right_speed->Kd = ore_right_speed_KD;
    ore_right_speed->max_out = ore_right_speed_max_out;
    ore_right_speed->max_iout = ore_right_speed_max_iout;
    ore_right_speed->Dbuf[0] = ore_right_speed->Dbuf[1] = ore_right_speed->Dbuf[2] = 0.0f;
    ore_right_speed->error[0] = ore_right_speed->error[1] = ore_right_speed->error[2] = ore_right_speed->Pout = ore_right_speed->Iout = ore_right_speed->Dout = ore_right_speed->out = 0.0f;

    pid_lift_def *lift_left_location   =   &pid_location[0];
    lift_left_location->mode = PID_POSITION;
    lift_left_location->Kp = lift_left_location_KP;
    lift_left_location->Ki = lift_left_location_KI;
    lift_left_location->Kd = lift_left_location_KD;
    lift_left_location->max_out = lift_left_location_max_out;
    lift_left_location->max_iout = lift_left_location_max_iout;
    lift_left_location->Dbuf[0] = lift_left_location->Dbuf[1] = lift_left_location->Dbuf[2] = 0.0f;
    lift_left_location->error[0] = lift_left_location->error[1] = lift_left_location->error[2] = lift_left_location->Pout = lift_left_location->Iout = lift_left_location->Dout = lift_left_location->out = 0.0f;

    pid_lift_def *lift_right_location   =   &pid_location[1];
    lift_right_location->mode = PID_POSITION;
    lift_right_location->Kp = lift_right_location_KP;
    lift_right_location->Ki = lift_right_location_KI;
    lift_right_location->Kd = lift_right_location_KD;
    lift_right_location->max_out = lift_right_location_max_out;
    lift_right_location->max_iout = lift_right_location_max_iout;
    lift_right_location->Dbuf[0] = lift_right_location->Dbuf[1] = lift_right_location->Dbuf[2] = 0.0f;
    lift_right_location->error[0] = lift_right_location->error[1] = lift_right_location->error[2] = lift_right_location->Pout = lift_right_location->Iout = lift_right_location->Dout = lift_right_location->out = 0.0f;

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

fp32 lift_PID_calc(pid_lift_def *pid, fp32 ref, fp32 set)
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

#define get_motor_measure(ptr, data)                                   \
{                                                                      \
    (ptr)->last_ecd = (ptr)->ecd;                                      \
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);               \
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);         \
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);     \
    (ptr)->temperate = (data)[6];                                      \
}

lift_measure_t  motor_lift[4];
void lift_measure(void)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx_header, rx_data);
    switch (rx_header.StdId)
    {
        case  motor_left_ID:
        case  motor_right_ID:
        case  ore_motor_left_ID:
        case  ore_motor_right_ID:
        {
            static uint8_t i = 0;
            //get motor id
            i = rx_header.StdId - motor_left_ID;
            get_motor_measure(&motor_lift[i], rx_data);
            break;
        }
        default:
        {
            break;
        }
    }
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
    strt.measure        =   lift_measure;
    strt.can.lift.state = stop;
    strt.can.ore.state = stop;
    
    strt.pid_init   =   lift_PID_init;
    strt.pid_init();
    strt.pid_calc   =   lift_PID_calc;

    strt.sensor_data.photogate_1  =   0;
}

#endif

