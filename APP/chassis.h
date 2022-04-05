#ifndef CHASSIS_H
#define CHASSIS_H

#include "remote_control.h"
#include "struct_typedef.h"
#include "can.h"
#include "CAN_receive.h"

typedef struct 
{
    const RC_ctrl_t *rc_data;
    const motor_measure_t *motor_measure[5];

    void (*init)();
    void (*set_mode)();
    void (*control)();
    void (*can_send)();
    void (*measure)();
    fp32 (*PID_calc)();

    struct 
    {
        int16_t motor1;//从左前轮开始顺时针电机1-4
        int16_t motor2;
        int16_t motor3;
        int16_t motor4;
        int16_t motor1_target;
        int16_t motor2_target;
        int16_t motor3_target;
        int16_t motor4_target;
        int16_t motor1_speed;
        int16_t motor2_speed;
        int16_t motor3_speed;
        int16_t motor4_speed;
        int16_t chassis_state;
        float   translation_ratio;
        float   rotating_ratio;
        int16_t vx;
        int16_t vy;
        int16_t vz;
    }can;

    #define     RIGHT_SWITCH_UP         (chassis.rc_data->rc.s[0] == 1)
    #define     RIGHT_SWITCH_MID        (chassis.rc_data->rc.s[0] == 3)
    #define     RIGHT_SWITCH_DOWN       (chassis.rc_data->rc.s[0] == 2)
    #define     LEFT_SWITCH_UP          (chassis.rc_data->rc.s[1] == 1)
    #define     LEFT_SWITCH_MID         (chassis.rc_data->rc.s[1] == 3)
    #define     LEFT_SWITCH_DOWN        (chassis.rc_data->rc.s[1] == 2)
    
    #define     SHUT_DOWN               (chassis.can.chassis_state == shut)
    #define     STANDARD                (chassis.can.chassis_state == standard)
    #define     FAST                    (chassis.can.chassis_state == fast)

}chassis_ctrl_t;
chassis_ctrl_t chassis;

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

} chassis_pid_strt;
chassis_pid_strt chassis_PID[4];

int16_t transverse;           
int16_t lengthways;            
int16_t roating;               

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

enum
{
    shut    =   0,
    standard,
    fast,
    crazy
}chassis_state_type;
//一号电机PID
float MOTOR1_KP     =   100.0f;
float MOTOR1_KI     =   0.0f;
float MOTOR1_KD     =   0.0f;
float MOTOR1_MOUT   =   16000.0f;
float MOTOR1_MIOUT  =   1.0f;
//二号电机PID
float MOTOR2_KP     =   100.0f;
float MOTOR2_KI     =   0.0f;
float MOTOR2_KD     =   0.0f;
float MOTOR2_MOUT   =   16000.0f;
float MOTOR2_MIOUT  =   1.0f;
//三号电机PID
float MOTOR3_KP     =   100.0f;
float MOTOR3_KI     =   0.0f;
float MOTOR3_KD     =   0.0f;
float MOTOR3_MOUT   =   16000.0f;
float MOTOR3_MIOUT  =   1.0f;
//四号电机PID
float MOTOR4_KP     =   100.0f;
float MOTOR4_KI     =   0.0f;
float MOTOR4_KD     =   0.0f;
float MOTOR4_MOUT   =   16000.0f;
float MOTOR4_MIOUT  =   1.0f;    

//速度模式设置
void chassis_set_mode(void)
{
    if(LEFT_SWITCH_DOWN && RIGHT_SWITCH_DOWN)
    {
        chassis.can.chassis_state   =   shut;

    }else if(LEFT_SWITCH_DOWN && RIGHT_SWITCH_UP)
    {
        chassis.can.chassis_state   =   fast;
        
    }else{
        chassis.can.chassis_state   =   standard;
    }

}

void chassis_control(void)
{   
    //初始化
    chassis.can.motor1 = 0;
    chassis.can.motor2 = 0;
    chassis.can.motor3 = 0;
    chassis.can.motor4 = 0;

    //灵敏度设置
    if(STANDARD)
    {
        chassis.can.translation_ratio = 0.1;
        chassis.can.rotating_ratio = 0.1;
    }
    if(FAST)
    {
        chassis.can.translation_ratio = 0.3;
        chassis.can.rotating_ratio = 0.3;
    }

    //遥控器死区
    transverse = chassis.rc_data->rc.ch[0];
    lengthways = chassis.rc_data->rc.ch[1];
    roating    = chassis.rc_data->rc.ch[2];

    if (lengthways > -100 && lengthways <100)
    {
        lengthways = 0;
    }
    if (transverse >= -100 && transverse <= 100)
    {
        transverse = 0;
    }
    if (roating >= -100 && roating <= 100)
    {
        roating = 0;
    }

    //速度倍率计算
    chassis.can.vx = chassis.can.translation_ratio * lengthways;
    chassis.can.vy = chassis.can.translation_ratio * transverse;
    chassis.can.vz = chassis.can.rotating_ratio * roating;
    //平移运动解算
    chassis.can.motor1_target = chassis.can.vy + chassis.can.vx;
    chassis.can.motor2_target = chassis.can.vy - chassis.can.vx;
    chassis.can.motor3_target = -1*chassis.can.motor1_target;
    chassis.can.motor4_target = -1*chassis.can.motor2_target;
    //旋转角度解算
    chassis.can.motor1_target += chassis.can.vz;
    chassis.can.motor2_target += chassis.can.vz;
    chassis.can.motor3_target += chassis.can.vz;
    chassis.can.motor4_target += chassis.can.vz;
    //读取值减速比计算
    chassis.can.motor1_speed = chassis.motor_measure[0]->speed_rpm/19;
    chassis.can.motor2_speed = chassis.motor_measure[1]->speed_rpm/19;
    chassis.can.motor3_speed = chassis.motor_measure[2]->speed_rpm/19;
    chassis.can.motor4_speed = chassis.motor_measure[3]->speed_rpm/19;

    if(SHUT_DOWN)
    {
        chassis.can.motor1_target = 0;
        chassis.can.motor2_target = 0;
        chassis.can.motor3_target = 0;
        chassis.can.motor4_target = 0;

        chassis.can.motor1 = 0;
        chassis.can.motor2 = 0;
        chassis.can.motor3 = 0;
        chassis.can.motor4 = 0;

    }
    else{
        chassis.can.motor1 = (int16_t)chassis.PID_calc(&chassis_PID[0],chassis.can.motor1_speed,chassis.can.motor1_target);
        chassis.can.motor2 = (int16_t)chassis.PID_calc(&chassis_PID[1],chassis.can.motor2_speed,chassis.can.motor2_target);
        chassis.can.motor3 = (int16_t)chassis.PID_calc(&chassis_PID[2],chassis.can.motor3_speed,chassis.can.motor3_target);
        chassis.can.motor4 = (int16_t)chassis.PID_calc(&chassis_PID[3],chassis.can.motor4_speed,chassis.can.motor4_target);
    
    }
}

static CAN_TxHeaderTypeDef  can_tx_message;
static int8_t               chassis_can_send_data[8];
void chassis_can_send(void)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = 0x200;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = chassis.can.motor1 >> 8;
    chassis_can_send_data[1] = chassis.can.motor1;
    chassis_can_send_data[2] = chassis.can.motor2 >> 8;
    chassis_can_send_data[3] = chassis.can.motor2;
    chassis_can_send_data[4] = chassis.can.motor3 >> 8;
    chassis_can_send_data[5] = chassis.can.motor3;
    chassis_can_send_data[6] = chassis.can.motor4 >> 8;
    chassis_can_send_data[7] = chassis.can.motor4;

    HAL_CAN_AddTxMessage(&hcan2, &can_tx_message, chassis_can_send_data, &send_mail_box);
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

fp32 chassis_PID_calc(chassis_pid_strt *pid, int16_t ref, int16_t set)
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

void chassis_PID_init(void)
{
    chassis_PID[0].mode = PID_POSITION;
    chassis_PID[0].Kp = MOTOR1_KP;
    chassis_PID[0].Ki = MOTOR1_KI;
    chassis_PID[0].Kd = MOTOR1_KD;
    chassis_PID[0].max_out = MOTOR1_MOUT;
    chassis_PID[0].max_iout = MOTOR1_MIOUT;
    chassis_PID[0].Dbuf[0] = chassis_PID[0].Dbuf[1] = chassis_PID[0].Dbuf[2] = 0.0f;
    chassis_PID[0].error[0] = chassis_PID[0].error[1] = chassis_PID[0].error[2] = chassis_PID[0].Pout = chassis_PID[0].Iout = chassis_PID[0].Dout = chassis_PID[0].out = 0.0f;

    chassis_PID[1].mode = PID_POSITION;
    chassis_PID[1].Kp = MOTOR2_KP;
    chassis_PID[1].Ki = MOTOR2_KI;
    chassis_PID[1].Kd = MOTOR2_KD;
    chassis_PID[1].max_out = MOTOR2_MOUT;
    chassis_PID[1].max_iout = MOTOR2_MIOUT;
    chassis_PID[1].Dbuf[0] = chassis_PID[1].Dbuf[1] = chassis_PID[1].Dbuf[2] = 0.0f;
    chassis_PID[1].error[0] = chassis_PID[1].error[1] = chassis_PID[1].error[2] = chassis_PID[1].Pout = chassis_PID[1].Iout = chassis_PID[1].Dout = chassis_PID[1].out = 0.0f;

    chassis_PID[2].mode = PID_POSITION;
    chassis_PID[2].Kp = MOTOR3_KP;
    chassis_PID[2].Ki = MOTOR3_KI;
    chassis_PID[2].Kd = MOTOR3_KD;
    chassis_PID[2].max_out = MOTOR3_MOUT;
    chassis_PID[2].max_iout = MOTOR3_MIOUT;
    chassis_PID[2].Dbuf[0] = chassis_PID[2].Dbuf[1] = chassis_PID[2].Dbuf[2] = 0.0f;
    chassis_PID[2].error[0] = chassis_PID[2].error[1] = chassis_PID[2].error[2] = chassis_PID[2].Pout = chassis_PID[2].Iout = chassis_PID[2].Dout = chassis_PID[2].out = 0.0f;

    chassis_PID[3].mode = PID_POSITION;
    chassis_PID[3].Kp = MOTOR4_KP;
    chassis_PID[3].Ki = MOTOR4_KI;
    chassis_PID[3].Kd = MOTOR4_KD;
    chassis_PID[3].max_out = MOTOR4_MOUT;
    chassis_PID[3].max_iout = MOTOR4_MIOUT;
    chassis_PID[3].Dbuf[0] = chassis_PID[3].Dbuf[1] = chassis_PID[3].Dbuf[2] = 0.0f;
    chassis_PID[3].error[0] = chassis_PID[3].error[1] = chassis_PID[3].error[2] = chassis_PID[3].Pout = chassis_PID[3].Iout = chassis_PID[3].Dout = chassis_PID[3].out = 0.0f;
}

void chassis_init(void)
{
    chassis.rc_data =   get_remote_control_point();
    for (uint8_t i = 0; i < 5; i++)
    {
        chassis.motor_measure[i] = get_motor_measure_point(i);
    }
    
    chassis.set_mode    =   chassis_set_mode;
    chassis.control     =   chassis_control;
    chassis.PID_calc    =   chassis_PID_calc;
    chassis.can_send    =   chassis_can_send;
    chassis_PID_init();
    chassis.can.chassis_state   =   shut;
}
//PID计算

#endif
