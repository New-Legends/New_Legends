#include "save.h"
#include "tim.h"
#include "remote_control.h"

save_control_t save_control;

static void save_init(save_control_t *init)
{
	init->rc_data = get_remote_control_point();
}

int last_data = 1;

void save_task(void const * argument)
{
	save_init(&save_control);
  while (1)
  {
		if (last_data != save_control.rc_data->rc.s[1] && save_control.rc_data->rc.s[1] != 3)
		{
			if (save_control.rc_data->rc.s[1] == 1)
			{
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1500);
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 2500);
				last_data = save_control.rc_data->rc.s[1];
				//HAL_GPIO_WritePin(SERVO_GPIO_Port, SERVO_Pin, GPIO_PIN_RESET);
			}
			else if(save_control.rc_data->rc.s[1] == 2)
			{
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 2500);
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1500);
				last_data = save_control.rc_data->rc.s[1];
				//HAL_GPIO_WritePin(SERVO_GPIO_Port, SERVO_Pin, GPIO_PIN_SET);
			}
		}
	}
}

