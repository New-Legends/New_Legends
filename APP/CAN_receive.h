#ifndef CAN_receive_H
#define CAN_receive_H

#include "struct_typedef.h"

typedef enum
{
    CAN_FLIP_LEFT_ID = 0x201,
    CAN_FLIP_RIGHT_ID = 0x202,
    CAN_STRETCH_ID = 0x203,
    CAN_CATCH_ID = 0x204,

} can_msg_id_can2_e;

typedef enum
{
    CAN_LIFT_LEFT_ID = 0x201,
    CAN_LIFT_RIGHT_ID = 0x202,
    CAN_ORE_LEFT_ID = 0x203,
    CAN_ORE_RIGHT_ID = 0x204,

} can_msg_id_can1_e;

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

extern const motor_measure_t *get_motor_measure_point(uint8_t i);

#endif
