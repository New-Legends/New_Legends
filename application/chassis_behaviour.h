/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_behaviour.c/h
  * @brief      according to remote control, change the chassis behaviour.
  *             ����ң������ֵ������������Ϊ��
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================
    ���Ҫ���һ���µ���Ϊģʽ
    1.���ȣ���chassis_behaviour.h�ļ��У� ���һ������Ϊ������ chassis_behaviour_e
    erum
    {  
        ...
        ...
        CHASSIS_XXX_XXX, // ����ӵ�
    }chassis_behaviour_e,

    2. ʵ��һ���µĺ��� chassis_xxx_xxx_control(fp32 *vx, fp32 *vy, fp32 *wz, chassis_move_t * chassis )
        "vx,vy,wz" �����ǵ����˶�����������
        ��һ������: 'vx' ͨ�����������ƶ�,��ֵ ǰ���� ��ֵ ����
        �ڶ�������: 'vy' ͨ�����ƺ����ƶ�,��ֵ ����, ��ֵ ����
        ����������: 'wz' �����ǽǶȿ��ƻ�����ת�ٶȿ���
        ������µĺ���, ���ܸ� "vx","vy",and "wz" ��ֵ��Ҫ���ٶȲ���
    3.  ��"chassis_behaviour_mode_set"��������У�����µ��߼��жϣ���chassis_behaviour_mode��ֵ��CHASSIS_XXX_XXX
        �ں���������"else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)" ,Ȼ��ѡ��һ�ֵ��̿���ģʽ
        4��:
        CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW : 'vx' and 'vy'���ٶȿ��ƣ� 'wz'�ǽǶȿ��� ��̨�͵��̵���ԽǶ�
        �����������"xxx_angle_set"������'wz'
        CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW : 'vx' and 'vy'���ٶȿ��ƣ� 'wz'�ǽǶȿ��� ���̵������Ǽ�����ľ��ԽǶ�
        �����������"xxx_angle_set"
        CHASSIS_VECTOR_NO_FOLLOW_YAW : 'vx' and 'vy'���ٶȿ��ƣ� 'wz'����ת�ٶȿ���
        CHASSIS_VECTOR_RAW : ʹ��'vx' 'vy' and 'wz'ֱ�����Լ�������ֵĵ���ֵ������ֵ��ֱ�ӷ��͵�can ������
    4.  ��"chassis_behaviour_control_set" ������������
        else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)
        {
            chassis_xxx_xxx_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
        }
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H
#include "struct_typedef.h"
#include "chassis_task.h"

typedef enum
{
  CHASSIS_ZERO_FORCE,                   //chassis will be like no power,��������, ��û�ϵ�����
  CHASSIS_NO_MOVE,                      //chassis will be stop,���̱��ֲ���
  CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW,   //chassis will follow gimbal, usually in infantry,�����������̸�����̨
  CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW,  //chassis will follow chassis yaw angle, usually in engineer,
                                        //because chassis does have gyro sensor, its yaw angle is calculed by gyro in gimbal and gimbal motor angle,
                                        //if you have a gyro sensor in chassis, please updata yaw, pitch, roll angle in "chassis_feedback_update"  function
                                        //���̵��̽Ƕȿ��Ƶ��̣����ڵ���δ�������ǣ��ʶ��Ƕ��Ǽ�ȥ��̨�Ƕȶ��õ���
                                        //����е�������������µ��̵�yaw��pitch��roll�Ƕ� ��chassis_feedback_update������
  CHASSIS_NO_FOLLOW_YAW,                //chassis does not follow angle, angle is open-loop,but wheels have closed-loop speed
                                        //���̲�����Ƕȣ��Ƕ��ǿ����ģ������������ٶȻ�
  CHASSIS_OPEN,                          //the value of remote control will mulitiply a value, get current value that will be sent to can bus
                                        // ң������ֵ���Ա����ɵ���ֵ ֱ�ӷ��͵�can������
} chassis_behaviour_e;

//���⣬���������Ϊģʽ����
extern chassis_behaviour_e chassis_behaviour_mode ;
extern chassis_behaviour_e last_chassis_behaviour_mode;

//Ť����������
extern bool_t swing_switch;  

//С���ݶ�������
extern bool_t top_switch; 
//45�ȽǶԵж�������
extern bool_t pisa_switch;  
extern uint16_t pisa_delay_time; 
#define MISS_CLOSE 0
#define MISS_BEGIN 1
#define MISS_OVER  2


#define SWING_KEY ((chassis_move.chassis_RC->key.v  & KEY_PRESSED_OFFSET_C) && !(chassis_move.chassis_last_key_v & KEY_PRESSED_OFFSET_C))
#define PISA_KEY ((chassis_move.chassis_RC->key.v  & KEY_PRESSED_OFFSET_X) && !(chassis_move.chassis_last_key_v & KEY_PRESSED_OFFSET_X))
#define TOP_KEY ((chassis_move.chassis_RC->key.v  & KEY_PRESSED_OFFSET_F) && !(chassis_move.chassis_last_key_v & KEY_PRESSED_OFFSET_F))


#define PISA_DELAY_TIME 500
#define CHASSIS_OPEN_RC_SCALE 10 // in CHASSIS_OPEN mode, multiply the value. ��chassis_open ģ���£�ң�������Ըñ������͵�can��




/**
  * @brief          ͨ���߼��жϣ���ֵ"chassis_behaviour_mode"������ģʽ
  * @param[in]      chassis_move_mode: ��������
  * @retval         none
  */
extern void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode);


/**
  * @brief          ���ÿ�����.���ݲ�ͬ���̿���ģʽ��������������Ʋ�ͬ�˶�.������������棬����ò�ͬ�Ŀ��ƺ���.
  * @param[out]     vx_set, ͨ�����������ƶ�.
  * @param[out]     vy_set, ͨ�����ƺ����ƶ�.
  * @param[out]     wz_set, ͨ��������ת�˶�.
  * @param[in]      chassis_move_rc_to_vector,  ��������������Ϣ.
  * @retval         none
  */

extern void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

#endif
