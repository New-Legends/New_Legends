#ifndef CAN_receive_H
#define CAN_receive_H

#include "struct_typedef.h"

typedef struct 
{
    int16_t Reset_key;
    int16_t Reset_last_key;
    int16_t Reset_flag;
    int16_t Reset_last_flag;
}reset_t;


typedef enum
{
    CAN_FLIP_LEFT_ID = 0x201,//[0]
    CAN_FLIP_RIGHT_ID = 0x202,//[1]
    CAN_STRETCH_ID = 0x203,//[2]
    CAN_CATCH_ID = 0x204,//[3]
    
    STRETCH_SENSOR_ID = 0x205,
    LIFT_SENSOR_ID = 0x206,
} can_msg_id_can2_e;

typedef enum
{
    CAN_LIFT_LEFT_ID = 0x201,//[4]
    CAN_LIFT_RIGHT_ID = 0x202,//[5]
    CAN_ORE_LEFT_ID = 0x203,//[6]
    CAN_ORE_RIGHT_ID = 0x204,//[7]

} can_msg_id_can1_e;

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
    int16_t round;
} motor_measure_t;

typedef struct
{
    uint8_t dis1;
    uint16_t dis2;
    uint8_t dis_status;
    uint16_t signal_strength;
    int16_t dis0;
    float dis;
} sensor_measure_t;

extern const motor_measure_t *get_motor_measure_point(uint8_t i);
extern const reset_t *get_reset_point(void);
extern const sensor_measure_t *get_sensor_measure_point(uint8_t i);

extern void can_receive_init(void);
#endif
