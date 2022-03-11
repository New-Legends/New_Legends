/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
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

#include "CAN_receive.h"

#include "cmsis_os.h"

#include "main.h"
#include "bsp_rng.h"


#include "detect_task.h"
#include "super_cap_task.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
    


/*
�������, 
0:���̵��1 3508���,  1:���̵��2 3508���,2:���̵��3 3508���,3:���̵��4 3508���;
4:��Ħ���ֵ�� 3508���, 5��Ħ���ֵ�� 3508���, 6������� 3508���, 7�޵�� ��ʱ����
8:yaw��̨��� 6020���; 9:pitch��̨��� 6020���;
*/
static motor_measure_t motor_chassis[10];

static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];
static CAN_TxHeaderTypeDef  shoot_tx_message;
static uint8_t              shoot_can_send_data[8];
static CAN_TxHeaderTypeDef  super_cap_tx_message;
static uint8_t              super_cap_can_send_data[8];      




/**
  * @brief          hal��CAN�ص�����,���յ������
  * @param[in]      hcan:CAN���ָ��
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId)
    {
        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
        case CAN_LEFT_FRIC_MOTOR_ID:
        case CAN_RIGHT_FRIC_MOTOR_ID:
        case CAN_TRIGGER_MOTOR_ID:
        case CAN_YAW_MOTOR_ID:
        case CAN_PIT_MOTOR_ID:
        case CAN_SUPER_CAP_ID:
        {
            if(rx_header.StdId == CAN_SUPER_CAP_ID)  //����
            {                                  
              cap_update_cap_inputvot((float)((int16_t)(rx_data)[1] << 8 | (rx_data)[0]) / 100.0f) ;            
              cap_update_cap_capvot((float)((int16_t)(rx_data)[3] << 8 | (rx_data)[2]) / 100.0f);      
              cap_update_cap_test_current((float)((int16_t)(rx_data)[5] << 8 | (rx_data)[4]) / 100.0f) ;  
              cap_update_cap_target_power((float)((int16_t)(rx_data)[7] << 8 | (rx_data)[6]) / 100.0f) ;       
            }
            else
            {
              static uint8_t i = 0;
              //get motor id
              i = rx_header.StdId - CAN_3508_M1_ID;
              get_motor_measure(&motor_chassis[i], rx_data);
              detect_hook(CHASSIS_MOTOR1_TOE + i);
            }
              break;
        }

        default:
        {
            break;
        }
    }


}


 /**
  * @brief          ���͵�����Ƶ���(0x209,0x20A,0x20B,0x20C)
  * @param[in]      yaw: (0x209) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      pitch: (0x20A) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      rev1: (0x20B)  ������������Ƶ���
  * @param[in]      rev2: (0x20C) ������������Ƶ���
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t rev1, int16_t rev2)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (rev1 >> 8);
    gimbal_can_send_data[5] = rev1;
    gimbal_can_send_data[6] = (rev2 >> 8);
    gimbal_can_send_data[7] = rev2;
	

	
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}


/**
  * @brief          ����IDΪ0x700��CAN��,��������3508��������������ID
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;
		

	
    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


/**
  * @brief          ���͵�����Ƶ���(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor2: (0x202) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor3: (0x203) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor4: (0x204) 3508������Ƶ���, ��Χ [-16384,16384]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;



    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


/**
  * @brief          ���͵�����Ƶ���(0x205,0x206,0x207,0x208)
  * @param[in]      left_fric: (0x205) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      right_fric: (0x206) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      trigger: (0x207) 2006������Ƶ���, ��Χ [-10000,10000]
  * @param[in]      ����: (0x208) ������������Ƶ���
  * @retval         none
  */
void CAN_cmd_shoot(int16_t left_fric, int16_t right_fric, int16_t trigger, int16_t rev)
{
    uint32_t send_mail_box;
    shoot_tx_message.StdId = CAN_SHOOT_ALL_ID;
    shoot_tx_message.IDE = CAN_ID_STD;
    shoot_tx_message.RTR = CAN_RTR_DATA;
    shoot_tx_message.DLC = 0x08;
    shoot_can_send_data[0] = (left_fric >> 8);
    shoot_can_send_data[1] = left_fric;
    shoot_can_send_data[2] = (right_fric >> 8);
    shoot_can_send_data[3] = right_fric;
    shoot_can_send_data[4] = (trigger >> 8);
    shoot_can_send_data[5] = trigger;
    shoot_can_send_data[6] = (rev >> 8);
    shoot_can_send_data[7] = rev;
	

	
    HAL_CAN_AddTxMessage(&SHOOT_CAN, &shoot_tx_message, shoot_can_send_data, &send_mail_box);

}

/**
  * @brief          �������ݷ��͹������
  * @param[in]      0x211 �������ݹ���
  * @retval         none
  */
void CAN_cmd_super_cap(int16_t temPower)
{	
   uint32_t send_mail_box;
    super_cap_tx_message.StdId = CAN_SUPER_CAP_ALL_ID;
    super_cap_tx_message.IDE = CAN_ID_STD;
    super_cap_tx_message.RTR = CAN_RTR_DATA;
    super_cap_tx_message.DLC = 0x08;
    super_cap_can_send_data[0] = (temPower >> 8);
    super_cap_can_send_data[1] = temPower;
    super_cap_can_send_data[2] = 0;
    super_cap_can_send_data[3] = 0;
    super_cap_can_send_data[4] = 0;
    super_cap_can_send_data[5] = 0;
    super_cap_can_send_data[6] = 0;
    super_cap_can_send_data[7] = 0;

	
    HAL_CAN_AddTxMessage(&SUPER_CAP_CAN, &super_cap_tx_message, super_cap_can_send_data, &send_mail_box);

}

/**
  * @brief          ����yaw 6020�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_chassis[8];
}


/**
  * @brief          ����pitch 6020�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_chassis[9];
}


/**
  * @brief          ���ز������ 3508�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_chassis[6];
}

/**
  * @brief          ����Ħ���� 3508�������ָ��
  * @param[in]      i: ������,��Χ[0,1]
  * @retval         �������ָ��
  */
const motor_measure_t *get_fric_motor_measure_point(uint8_t i)
{
    return &motor_chassis[i + 4];
}



/**
  * @brief          ���ص��̵�� 3508�������ָ��
  * @param[in]      i: ������,��Χ[0,3]
  * @retval         �������ָ��
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}
