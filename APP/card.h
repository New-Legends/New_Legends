#ifndef CARD_H
#define CARD_H
#include "remote_control.h"
#include "struct_typedef.h"
#include "can.h"

typedef struct 
{
    const RC_ctrl_t *rc_data;


    void (*init)();
    void (*set_mode)();
    void (*control)();
    void (*can_send)();
    void (*measure)();

    struct 
    {
        int16_t card;
        uint8_t state;
    }can;
    

    #define     RIGHT_SWITCH_DOWN       (card.rc_data->rc.s[0] == 2)
    #define     RIGHT_CH_UP             (card.rc_data->rc.ch[1] > 100)
    #define     RIGHT_CH_DOWN           (card.rc_data->rc.ch[1] < -100)
    #define     RIGHT_CH_MID            (card.rc_data->rc.ch[1] > -100 && card.rc_data->rc.ch[1] < 100)
    #define     card_state_stop         (card.can.state  ==  stop)
    #define     card_state_forward      (card.can.state  ==  forward)
    #define     card_state_reverse      (card.can.state  ==  reverse)


    #define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

}card_ctrl_t;

enum state_enum
{
    stop = 0,
    forward,
    reverse
};

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
    int16_t real_target;
} card_measure_t;

card_ctrl_t card;
void card_set_mode(void)
{
    if(RIGHT_SWITCH_DOWN)
    {
        if(RIGHT_CH_MID)
        {
            card.can.state  =   stop;
        }
        if(RIGHT_CH_UP)
        {
            card.can.state  =   forward;
        }
        if(RIGHT_CH_DOWN)
        {
            card.can.state  =   reverse;
        }
    }else{
        card.can.state  =   stop;
    }
}

void card_control(void)
{
    if(card_state_stop)
    {
        card.can.card   =   0;
    }
    if(card_state_forward)
    {
        card.can.card   =   1000;
    }
    if(card_state_reverse)
    {
        card.can.card   =   -1000;
    }
  /* USER CODE END card_task */
}   

card_measure_t  motor_card;

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

void card_init(void)
{
    card.rc_data   =   get_remote_control_point();
    card.set_mode  =   card_set_mode;
    card.control   =   card_control;
    card.can_send  =   card_can_send;
}

#endif
