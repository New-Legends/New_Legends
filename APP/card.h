#ifndef CARD_H
#define CARD_H
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
    fp32 (*PID_calc)();

    struct 
    {
        int16_t card;
        int16_t card_speed;
        int16_t card_target;
        uint8_t state;
    }can;
    

    #define     RIGHT_SWITCH_UP         (card.rc_data->rc.s[0] == 1)
    #define     RIGHT_SWITCH_MID        (card.rc_data->rc.s[0] == 3)
    #define     RIGHT_SWITCH_DOWN       (card.rc_data->rc.s[0] == 2)
    #define     LEFT_SWITCH_UP          (card.rc_data->rc.s[1] == 1)
    #define     LEFT_SWITCH_MID         (card.rc_data->rc.s[1] == 3)
    #define     LEFT_SWITCH_DOWN        (card.rc_data->rc.s[1] == 2)

    #define     LEFT_CH_UP              (card.rc_data->rc.ch[3] > 100)
    #define     LEFT_CH_DOWN            (card.rc_data->rc.ch[3] < -100)
    #define     LEFT_CH_MID             (card.rc_data->rc.ch[3] > -100 && card.rc_data->rc.ch[1] < 100)
    #define     card_state_stop         (card.can.state  ==  stop)
    #define     card_state_forward      (card.can.state  ==  forward)
    #define     card_state_reverse      (card.can.state  ==  reverse)
    #define     card_state_shut         (card.can.state  ==  shut)

}card_ctrl_t;

enum state_enum
{
    stop = 0,
    forward,
    reverse,
    shut
};

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

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

} card_pid_strt;
card_pid_strt card_pid;

//一号电机PID
float CARD_KP     =   11.0f;
float CARD_KI     =   0.0f;
float CARD_KD     =   0.1f;
float CARD_MOUT   =   10000.0f;
float CARD_MIOUT  =   1.0f;

card_ctrl_t card;
void card_set_mode(void)
{
    if(LEFT_SWITCH_MID && RIGHT_SWITCH_DOWN)
    {
        if(LEFT_CH_MID)
        {
            card.can.state  =   stop;
        }
        if(LEFT_CH_UP)
        {
            card.can.state  =   forward;
        }
        if(LEFT_CH_DOWN)
        {
            card.can.state  =   reverse;
        }
    }else{
        card.can.state  =   shut;
    }

}

void card_control(void)
{
    card.can.card_speed = card.motor_measure[4]->speed_rpm;

    if(card_state_stop)
    {
        card.can.card_target   =   0;
    }
    if(card_state_forward)
    {
        card.can.card_target   =   20 * 36;
    }
    if(card_state_reverse)
    {
        card.can.card_target   =   -20 * 36;
    }
    card.can.card = (int16_t)card.PID_calc(&card_pid,card.can.card_speed,card.can.card_target) - 1000;

    if(card_state_shut)
    {
        card.can.card = 0;
    }
}

static CAN_TxHeaderTypeDef  can_tx_message;
static uint8_t              card_can_send_data[8];

void card_can_send(void)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = 0x200;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    card_can_send_data[0] = card.can.card >> 8;
    card_can_send_data[1] = card.can.card;

    HAL_CAN_AddTxMessage(&hcan1, &can_tx_message, card_can_send_data, &send_mail_box);
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

fp32 card_PID_calc(card_pid_strt *pid, int16_t ref, int16_t set)
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

void card_PID_init(void)
{
    card_pid.mode = PID_POSITION;
    card_pid.Kp = CARD_KP;
    card_pid.Ki = CARD_KI;
    card_pid.Kd = CARD_KD;
    card_pid.max_out = CARD_MOUT;
    card_pid.max_iout = CARD_MIOUT;
    card_pid.Dbuf[0] = card_pid.Dbuf[1] = card_pid.Dbuf[2] = 0.0f;
    card_pid.error[0] = card_pid.error[1] = card_pid.error[2] = card_pid.Pout = card_pid.Iout = card_pid.Dout = card_pid.out = 0.0f;
}

void card_init(void)
{
    card.rc_data   =   get_remote_control_point();
    card.set_mode  =   card_set_mode;
    card.control   =   card_control;
    card.can_send  =   card_can_send;
    card.PID_calc  =   card_PID_calc;
    for (uint8_t i = 0; i < 5; i++)
    {
        card.motor_measure[i] = get_motor_measure_point(i);
    }

    card_PID_init();

}

#endif
