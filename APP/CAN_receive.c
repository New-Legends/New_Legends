#include "CAN_receive.h"
#include "can.h"

motor_measure_t motor[8];

#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (int16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

const motor_measure_t *get_motor_measure_point(uint8_t i)
{
    return &motor[i];
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    if(hcan == &hcan1)
    {
        switch (rx_header.StdId)
        {
            case CAN_LIFT_LEFT_ID:
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
            case CAN_FLIP_RIGHT_ID:
            case CAN_STRETCH_ID:
            case CAN_CATCH_ID:
            {
                static uint8_t i = 0;
                //get motor id
                i = rx_header.StdId - CAN_FLIP_LEFT_ID;
                get_motor_measure(&motor[i], rx_data);
                break;
            }

            default:
            {
                break;
            }
        }
    }
}
