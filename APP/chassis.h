#ifndef CHASSIS_H
#define CHASSIS_H

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
        uint16_t motor1;//从左前轮开始顺时针电机1-4
        uint16_t motor2;
        uint16_t motor3;
        uint16_t motor4;
        uint16_t motor1_target;
        uint16_t motor2_target;
        uint16_t motor3_target;
        uint16_t motor4_target;
        uint8_t  chassis_state;
        uint8_t  translation_ratio;
        uint8_t  rotating_ratio;
        uint8_t  vx;
        uint8_t  vy;
        uint8_t  vz;
    }can;
    
    #define     RIGHT_SWITCH_UP         (chassis.rc_data->rc.s[0] == 1)
    #define     RIGHT_SWITCH_MID        (chassis.rc_data->rc.s[0] == 3)
    #define     RIGHT_SWITCH_DOWN       (chassis.rc_data->rc.s[0] == 2)
    #define     transverse              (chassis.rc_data->rc.ch[0])
    #define     lengthways              (chassis.rc_data->rc.ch[1])
    #define     roating                 (chassis.rc_data->rc.ch[2])
    
    #define     SHUT_DOWN               (chassis.can.chassis_state == standard)
    #define     STANDARD                (chassis.can.chassis_state == fast)
    #define     FAST                    (chassis.can.chassis_state == shut)

}chassis_ctrl_t;
chassis_ctrl_t chassis;

enum
{
    shut    =   0,
    standard,
    fast,
    crazy
}chassis_state_type;

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
}chassis_measure_t;
chassis_measure_t motor_chassis[4];

//速度模式设置
void chassis_set_mode(void)
{
    if(RIGHT_SWITCH_MID)
    {
        chassis.can.chassis_state   =   shut;
    }else{
        chassis.can.chassis_state   =   standard;
    }
    
}

void chassis_control(void)
{   
    //灵敏度设置
    if(SHUT_DOWN)
    {
        chassis.can.translation_ratio = 0;
    }
    if(STANDARD)
    {
        chassis.can.translation_ratio = 1;
    }
    if(FAST)
    {
        chassis.can.translation_ratio = 1;
    }
    //速度倍率计算
    chassis.can.vx = chassis.can.translation_ratio * lengthways;
    chassis.can.vy = chassis.can.translation_ratio * transverse;
    chassis.can.vz = chassis.can.rotating_ratio * roating;
    //平移运动解算
    chassis.can.motor1 = chassis.can.vy + chassis.can.vx;
    chassis.can.motor2 = chassis.can.vy - chassis.can.vx;
    chassis.can.motor3 = -chassis.can.motor1;
    chassis.can.motor4 = -chassis.can.motor2;
    //旋转角度解算
    chassis.can.motor1 += chassis.can.vz;
    chassis.can.motor2 += chassis.can.vz;
    chassis.can.motor3 += chassis.can.vz;
    chassis.can.motor4 += chassis.can.vz;
}

static CAN_TxHeaderTypeDef  can_tx_message;
static uint8_t              chassis_can_send_data[8];
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

#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

void chassis_motor_measure(void)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8]; 
    HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx_header, rx_data);
    switch (rx_header.StdId)
    {
        case 513:
        case 514:
        case 515:
        case 516:
        {
            uint8_t i;
            i = rx_header.StdId - 513;
            get_motor_measure(&motor_chassis[i], rx_data);
            break;
        }
        default:
        {
            break;
        }
    }
}

void chassis_init(void)
{
    chassis.rc_data =   get_remote_control_point();

    chassis.measure     =   chassis_motor_measure;
    chassis.set_mode    =   chassis_set_mode;
    chassis.control     =   chassis_control;
    chassis.can_send    =   chassis_can_send;

    chassis.can.chassis_state   =   shut;
}

#endif
