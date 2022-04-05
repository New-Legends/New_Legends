#include "save.h"
#include "tim.h"
#include "remote_control.h"



static void save_init(void)
{
	save.rc_data = get_remote_control_point();
}

void gimbal_init(void)
{
	save.gimbal_angle = 1500;
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, save.gimbal_angle);
}

void gimbal_control(void)
{
	if(save.rc_data->rc.ch[3] > 100)
	{
		save.gimbal_angle = 2500;
	}

	if(save.rc_data->rc.ch[3] < -100)
	{
		save.gimbal_angle = 500;
	}
	if(save.rc_data->rc.ch[3] > -100 && save.rc_data->rc.ch[3] < 100)
	{
		save.gimbal_angle = 1500;
	}

	if(save.gimbal_angle>=2500)
	{
		save.gimbal_angle = 2500;
	}
	if(save.gimbal_angle<=500)
	{
		save.gimbal_angle = 500;
	}
}

int last_data = 1;

void save_task(void const * argument)
{
	save_init();
	gimbal_init();
	while (1)
	{
		if (LEFT_SWITCH_DOWN && RIGHT_SWITCH_MID)
		{
			if (LEFT_CH_UP)
			{
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1200);
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 2500);
				last_data = 1;
				//HAL_GPIO_WritePin(SERVO_GPIO_Port, SERVO_Pin, GPIO_PIN_RESET);
			}
			else if(LEFT_CH_DOWN)
			{
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 2500);
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1200);
				last_data = 2;
				//HAL_GPIO_WritePin(SERVO_GPIO_Port, SERVO_Pin, GPIO_PIN_SET);
			}
		}
		if(GIMBAL_CTRL)
		{
			gimbal_control();
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, save.gimbal_angle);
		}
	}
}

