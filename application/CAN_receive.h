/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

//此处暂时为了调试，把CAN的顺序调整了一下
#define GIMBAL_CAN hcan2
#define SHOOT_CAN hcan2
#define CHASSIS_CAN hcan1
#define SUPER_CAP_CAN hcan2

//云台机构电机编号
enum gimbal_motor_id_e
{
  YAW_MOTOR = 0,
  PITCH_MOTOR,
};

//发射机构电机编号
enum shoot_motor_id_e
{
  LEFT_FRIC_MOTOR = 0,
  RIGHT_FRIC_MOTOR,
  TRIGGER_MOTOR,
  MAGAZINE_MOTOR,
};

//底盘动力电机编号
enum motive_chassis_motor_id_e
{
  //底盘动力电机接收
  MOTIVE_FR_MOTOR = 0,
  MOTIVE_FL_MOTOR,
  MOTIVE_BL_MOTOR,
  MOTIVE_BR_MOTOR,

};

//底盘舵向电机编号
enum rudde_chassisr_motor_id_e
{
  //底盘舵向电机
  RUDDER_FL_MOTOR = 0,
  RUDDER_FR_MOTOR,
  RUDDER_BL_MOTOR,
  RUDDER_BR_MOTOR,
};

/* CAN send and receive ID */
typedef enum
{

  //发射机构电机接受ID CAN1
  CAN_LEFT_FRIC_MOTOR_ID = 0x201,
  CAN_RIGHT_FRIC_MOTOR_ID = 0x202,
  CAN_TRIGGER_MOTOR_ID = 0x203,
  CAN_MAGAZINE_MOTOR_ID = 0X204,
  CAN_SHOOT_ALL_ID = 0x200,

  //云台电机接收ID CAN1
  CAN_YAW_MOTOR_ID = 0x205,
  CAN_PITCH_MOTOR_ID = 0x206,
  CAN_GIMBAL_ALL_ID = 0x1FF,

  //底盘动力电机接收ID  CAN2
  CAN_MOTIVE_FR_MOTOR_ID = 0x201,
  CAN_MOTIVE_FL_MOTOR_ID = 0x202,
  CAN_MOTIVE_BL_MOTOR_ID = 0x203,
  CAN_MOTIVE_BR_MOTOR_ID = 0x204,
  CAN_CHASSIS_MOTIVE_ALL_ID = 0x200,

  //底盘舵向电机ID CAN2
  CAN_RUDDER_FL_MOTOR_ID = 0x205,
  CAN_RUDDER_FR_MOTOR_ID = 0x206,
  CAN_RUDDER_BL_MOTOR_ID = 0x207, 
  CAN_RUDDER_BR_MOTOR_ID = 0X208,
  CAN_CHASSIS_RUDDER_ALL_ID = 0x1FF,

  //超级电容接收ID CAN2
  CAN_SUPER_CAP_ID = 0x211,
  CAN_SUPER_CAP_ALL_ID = 0x210,
} can_msg_id_e;

//rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;



/**
  * @brief          发送电机控制电流(0x209,0x20A,0x20B,0x20C)
  * @param[in]      yaw: (0x209) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x20A) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      rev1: (0x20B)  保留，电机控制电流
  * @param[in]      rev2: (0x20C) 保留，电机控制电流
  * @retval         none
  */
extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t rev1, int16_t rev2);


/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
extern void CAN_cmd_chassis_reset_ID(void);


/**
  * @brief          发送动力电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_chassis_motive(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);


/**
  * @brief          发送舵向电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      motor1: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      motor2: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      motor3: (0x207) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      motor4: (0x208) 6020电机控制电流, 范围 [-30000,30000]
  * @retval         none
  */
extern void CAN_cmd_chassis_rudder(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);


/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      left_fric: (0x205) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      right_fric: (0x206) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      trigger: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      保留: (0x208) 保留，电机控制电流
  * @retval         none
  */
extern void CAN_cmd_shoot(int16_t left_fric, int16_t right_fric, int16_t trigger, int16_t magazine);

/**
  * @brief          超级电容发送功率输出
  * @param[in]      0x211 超级电容功率
  * @retval         none
  */
extern void CAN_cmd_super_cap(int16_t temPower);

/**
  * @brief          返回发射机构电机 3508电机数据指针
  * @param[in]       i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_shoot_motor_measure_point(uint8_t i);

/**
  * @brief          返回云台机构电机数据指针
  * @param[in]       i: 电机编号,范围[0,1]
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_gimbal_motor_measure_point(uint8_t i);

/**
  * @brief          返回底盘动力电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_chassis_motive_motor_measure_point(uint8_t i);

/**
  * @brief          返回底盘舵向电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_chassis_rudder_motor_measure_point(uint8_t i);

#endif
