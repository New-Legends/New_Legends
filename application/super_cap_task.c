/**
  * @file       super_cap.c/h
  * @brief      �������ݿ�������
  * @note       �ɴӳ�����ݿ��ư��ж�ȡ���ݵ�ѹ������ϵͳ�������������ϵͳ�����ѹ���趨����
								��Ҫע�⣬����Ҫ����������ʱ��Ӧ��ע���ݵ�ѹ��Ҫ����12V�������C620����ĵ�ѹ������ʹ����ϵ�
								ʹ�÷�ʽ�������̵�������أ������趨����ʱ������ϵͳ���벻�ᳬ���趨���ʣ����㲿���ɷ������ݲ���
								���ڷ��������������ޣ�����ʱ��Ҳ���ޣ����Գ�����ʱ��Ҳ����̫�á���������8F(100F,12��)
								ͨ��canͨѶ�趨���ʣ���30W-130W�ɵ���Ĭ�Ϲ���35W������ֵ3000-13000�����۸�����α仯����ϵͳ�������ʼ�ջ����130W
								���ƽ��飺
								���������趨���������Ӳ���ϵͳ��ȡ�Ĺ�������ֵ�Զ��ı䣬�����˶�����������ʵ�ֶ��ݳ�����
								
								�ر�ע�⣺ȷ������ģ��İ汾���Լ����Ӧ�Ĳ���ϵͳ����Э��汾������ᵼ���޷�����ȷ�Ĳ�����Ϣ�����³�������ʧЧ
  *
  @verbatim
  ==============================================================================
   һ����������ӽ�CAN_receive.c��
	 
	 #include "super_cap.h"
	 
	 int16_t Cap_Inputvot,Cap_Capvot,Cap_Test_current,Cap_Target_Power;
	
	 if(RxMessage.StdId == 0x211)//�������ݿ��ư�		
	{
		Cap_Inputvot  = (fp32)((int16_t)(RxMessage.Data[1]<<8|RxMessage.Data[0]))/100.0f;  //�����ѹ
		Cap_Update_Cap_Inputvot(Cap_Inputvot);
		
		Cap_Capvot = (fp32)((int16_t)(RxMessage.Data[3]<<8|RxMessage.Data[2]))/100.0f;  //���ݵ�ѹ
		Cap_Update_Cap_Capvot(Cap_Capvot);
		
		Cap_Test_current = (fp32)((int16_t)(RxMessage.Data[5]<<8|RxMessage.Data[4]))/100.0f;	  //�������
		Cap_Update_Cap_Test_current(Cap_Test_current);
		
		Cap_Target_Power = (fp32)((int16_t)(RxMessage.Data[7]<<8|RxMessage.Data[6]))/100.0f;	 //���빦��
		Cap_Update_Cap_Target_Power(Cap_Target_Power);
	}	
  ==============================================================================
  @endverbatim
  */
#include "super_cap_task.h"
#include "can.h"
#include "remote_control.h"
#include "CAN_receive.h"

#include "referee.h"

fp32 input_vot;
fp32 supercap_vot;
fp32 input_current;
fp32 target_power;

extern ext_game_robot_state_t robot_state; //0x0201     ����������״̬
uint8_t cap_change = FALSE; //�����ѹ���ͱ�ʶ��


void cap_update_cap_inputvot(int16_t inputvot)
{
	input_vot = inputvot; //��ȡ�����ѹ
}

void cap_update_cap_capvot(int16_t capvot)
{
	supercap_vot = capvot; //��ȡ���ݵ�ѹ
}

void cap_update_cap_test_current(int16_t current)
{
	input_current = current; //��ȡ�������
}

void cap_update_cap_target_power(int16_t power)
{
	target_power = power; //��ȡ���빦��
}

void cap_read_cap_buff(float *_cap_buff)
{
	/*����������ʽ E= 1/2*C*U*U
    CΪ������ֵ U�ǵ������˵ĵ�ѹ
    */
   //�ӵ�������
	int cap_son_num = 10;
	*_cap_buff = 0.5*50*(supercap_vot/cap_son_num)*(supercap_vot/cap_son_num) * cap_son_num;
}
void cap_init()
{
    for (uint8_t i=0; i<5; i++){
        vTaskDelay(1);
        CAN_cmd_super_cap(4000);
    }
}
//������
//void super_cap_task(void const *pvParameters)
//{

//	//�ȴ������������������������
//	vTaskDelay(SUPER_CAP_TASK_INIT_TIME);

//	for (;;)
//	{

//					if( supercap_vot <= 14 || cap_change == TRUE) //���ݵ�ѹ��Ҫ����12V������ܻ����C620��ѹ����,���ֱ�Ӷϵ�
//					{
//						 if(supercap_vot >= 20)
//						{
//								if (game_state.chassis_power_limit <= 40 )//��ǰ�������������<40  Ŀ�깦��Ϊ39w
//								{
//									 CAN_cmd_super_cap(3900);
//								}
//								else if(game_state.chassis_power_limit > 40 && game_state.chassis_power_limit <= 50)//��ǰ�������������40-50w  Ŀ�깦��Ϊ49w
//								{
//									 CAN_cmd_super_cap(4900);
//								}	
//								else if(game_state.chassis_power_limit > 50 && game_state.chassis_power_limit <= 60)//��ǰ�������������50-60w  Ŀ�깦��Ϊ59w
//								{
//									CAN_cmd_super_cap(5900);
//								}
//								else if(game_state.chassis_power_limit > 60 && game_state.chassis_power_limit <= 70)//��ǰ�������������60-70w  Ŀ�깦��Ϊ79w
//								{
//									CAN_cmd_super_cap(6900);
//								}
//								else if(game_state.chassis_power_limit > 70 && game_state.chassis_power_limit <= 80)//��ǰ�������������70-80w  Ŀ�깦��Ϊ79w
//								{
//									CAN_cmd_super_cap(7900);
//								}
//								else if(game_state.chassis_power_limit > 80 && game_state.chassis_power_limit <= 100)//��ǰ�������������80-100w  Ŀ�깦��Ϊ99w
//								{
//									CAN_cmd_super_cap(9900);
//								}
//								else if(game_state.chassis_power_limit > 100 && game_state.chassis_power_limit < 120)//��ǰ�������������100-120w  Ŀ�깦��Ϊ119w
//								{
//									CAN_cmd_super_cap(11900);
//								}
//								cap_change = FALSE;
//							}
//						else
//						{
//							cap_change = TRUE;
//							CAN_cmd_super_cap(13000);
//						}
//					}
//					else
//					{
//						if (game_state.chassis_power_limit <= 40 )//��ǰ�������������<40  Ŀ�깦��Ϊ39w
//						{
//							 CAN_cmd_super_cap(3900);
//						}
//						else if(game_state.chassis_power_limit > 40 && game_state.chassis_power_limit <= 50)//��ǰ�������������40-50w  Ŀ�깦��Ϊ49w
//						{
//							 CAN_cmd_super_cap(4900);
//						}	
//						else if(game_state.chassis_power_limit > 50 && game_state.chassis_power_limit <= 60)//��ǰ�������������50-60w  Ŀ�깦��Ϊ59w
//						{
//							CAN_cmd_super_cap(5900);
//						}
//						else if(game_state.chassis_power_limit > 60 && game_state.chassis_power_limit <= 70)//��ǰ�������������60-70w  Ŀ�깦��Ϊ79w
//						{
//							CAN_cmd_super_cap(6900);
//						}
//						else if(game_state.chassis_power_limit > 70 && game_state.chassis_power_limit <= 80)//��ǰ�������������70-80w  Ŀ�깦��Ϊ79w
//						{
//							CAN_cmd_super_cap(7900);
//						}
//						else if(game_state.chassis_power_limit > 80 && game_state.chassis_power_limit <= 100)//��ǰ�������������80-100w  Ŀ�깦��Ϊ99w
//						{
//							CAN_cmd_super_cap(9900);
//						}
//						else if(game_state.chassis_power_limit > 100 && game_state.chassis_power_limit < 120)//��ǰ�������������100-120w  Ŀ�깦��Ϊ119w
//						{
//							CAN_cmd_super_cap(11900);
//						}
//					}
//				}

//		vTaskDelay(SUPER_CONTROL_TIME);
//}






