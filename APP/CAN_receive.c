#include "CAN_receive.h"
#include "can.h"
#include "remote_control.h"

motor_measure_t motor[9];
sensor_measure_t lift_sensor[2];
const RC_ctrl_t *c_rc_data;
int16_t rc_flag = 0;
reset_t Reset_s;

#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (int16_t)((data)[2] << 8 | (data)[3]);       \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

#define get_sensor_measure(ptr, data)                                   \
    {                                                                   \
        (ptr)->signal_strength = (uint16_t)((data)[4] << 8 | (data)[5]);\
        (ptr)->dis_status = (uint8_t)(data)[3];                         \
        (ptr)->dis2 = (uint16_t)((data)[2] << 8 | (data)[1]);           \
        (ptr)->dis1 = (uint8_t)(data)[0];                               \
        (ptr)->dis0 = (ptr)->dis2 * 256 + (ptr)->dis1;                  \
        (ptr)->dis = (ptr)->dis0*1.000/1000;                            \
    }

const motor_measure_t *get_motor_measure_point(uint8_t i)
{
    return &motor[i];
}

const reset_t *get_reset_point(void)
{
    return &Reset_s;
}

const sensor_measure_t *get_sensor_measure_point(uint8_t i)
{
    return &lift_sensor[i];
}

void can_receive_init(void)
{
    for(int i=0;i<8;i++)
    {
        motor[i].round = 0;
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if(rc_flag == 0)
    {
        c_rc_data = get_remote_control_point();
        rc_flag = 1;
        Reset_s.Reset_key = 0;
    }

    //重新计圈
    Reset_s.Reset_last_flag = Reset_s.Reset_flag;
    if(c_rc_data->key.v == KEY_PRESSED_OFFSET_CTRL)
    {
        Reset_s.Reset_flag = 1;
    }else{
        Reset_s.Reset_flag = 0;
    }
    Reset_s.Reset_last_key = Reset_s.Reset_key;
    if(Reset_s.Reset_flag != Reset_s.Reset_last_flag && Reset_s.Reset_flag == 1)
    {
        if(Reset_s.Reset_key == 0)
        {
            Reset_s.Reset_key = 1;
        }
    }else{
        if(Reset_s.Reset_key == 1)
        {
            Reset_s.Reset_key = 0;
        }
    }

    if(Reset_s.Reset_last_key != Reset_s.Reset_key && Reset_s.Reset_last_key == 1)
    {
        for(int i=0;i<8;i++)
        {
            motor[i].round = 0;
        }
    }

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    if(hcan == &hcan1)
    {
        switch (rx_header.StdId)
        {
            case CAN_LIFT_LEFT_ID:
            {
                static uint8_t i = 0;
                //get motor id
                i = rx_header.StdId - CAN_LIFT_LEFT_ID + 4;
                get_motor_measure(&motor[i], rx_data);
                if(motor[i].ecd - motor[i].last_ecd > 5000)
                {
                    motor[i].round--;
                }
                if(motor[i].ecd - motor[i].last_ecd < -5000)
                {
                    motor[i].round++;
                }
                break;
            }
            case CAN_LIFT_RIGHT_ID:
            case CAN_ORE_LEFT_ID:
            case CAN_ORE_RIGHT_ID:
            {
                static uint8_t i = 0;
                //get motor id
                i = rx_header.StdId - CAN_LIFT_LEFT_ID + 4;
                get_motor_measure(&motor[i], rx_data);
                break;
            }


            default:
            {
                break;
            }
        }
    }
    if(hcan == &hcan2)
    {
        switch (rx_header.StdId)
        {
            case CAN_FLIP_LEFT_ID:
            {
                static uint8_t i = 0;
                //get motor id
                i = rx_header.StdId - CAN_FLIP_LEFT_ID;
                get_motor_measure(&motor[i], rx_data);
                if(motor[i].ecd - motor[i].last_ecd > 5000)
                {
                    motor[i].round--;
                }
                if(motor[i].ecd - motor[i].last_ecd < -5000)
                {
                    motor[i].round++;
                }
            }
            case CAN_FLIP_RIGHT_ID:
            case CAN_STRETCH_ID:
            {
                static uint8_t i = 0;
                //get motor id
                i = rx_header.StdId - CAN_FLIP_LEFT_ID;
                get_motor_measure(&motor[i], rx_data);
                if(motor[i].ecd - motor[i].last_ecd > 5000)
                {
                    motor[i].round--;
                }
                if(motor[i].ecd - motor[i].last_ecd < -5000)
                {
                    motor[i].round++;
                }
            }
            case CAN_CATCH_ID:
            {
                static uint8_t i = 0;
                //get motor id
                i = rx_header.StdId - CAN_FLIP_LEFT_ID;
                get_motor_measure(&motor[i], rx_data);
                break;
            }

            case LIFT_SENSOR_ID:
            {
                get_sensor_measure(&lift_sensor[0],rx_data);
                break;
            }
            case STRETCH_SENSOR_ID:
            {
                get_sensor_measure(&lift_sensor[1],rx_data);
                break;
            }
            default:
            {
                break;
            }
        }
    }
}
