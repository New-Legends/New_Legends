/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       oled_task.c/h
  * @brief      OLED show error value.oled???????
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "oled_task.h"
#include "main.h"
#include "oled.h"

#include "cmsis_os.h"
#include "detect_task.h"
#include "voltage_task.h"

#define OLED_CONTROL_TIME 10
#define REFRESH_RATE 10

const error_t *error_list_local;


uint8_t shoot_toe_name[4][4] = {"FR\0", "RR\0", "TRI\0", "COV\0"};
uint8_t other_toe_name[4][4] = {"IMU\0", "REF\0", "CAP\0", "   \0"};

uint8_t last_oled_error = 0;
uint8_t now_oled_errror = 0;
static uint8_t refresh_tick = 0;

/**
  * @brief          oled task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          oled??
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void oled_task(void const *argument)
{
    uint8_t i;
    uint8_t show_col, show_row;
    error_list_local = get_error_list_point();
    osDelay(1000);
    OLED_init();
    OLED_LOGO();
    i = 100;
    while (i--)
    {
        if (OLED_check_ack())
        {
            detect_hook(OLED_TOE);
        }
        osDelay(10);
    }
    while (1)
    {
        //use i2c ack to check the oled
        if (OLED_check_ack())
        {
            detect_hook(OLED_TOE);
        }

        now_oled_errror = toe_is_error(OLED_TOE);
        //oled init
        if (last_oled_error == 1 && now_oled_errror == 0)
        {
            OLED_init();
        }

        if (now_oled_errror == 0)
        {
            refresh_tick++;
            //10Hz refresh
            if (refresh_tick > configTICK_RATE_HZ / (OLED_CONTROL_TIME * REFRESH_RATE))
            {
                refresh_tick = 0;
                OLED_operate_gram(PEN_CLEAR);
                OLED_show_graphic(0, 1, &battery_box);

                if (get_battery_percentage() < 10)
                {
                    OLED_printf(9, 2, "%d", get_battery_percentage());
                }
                else if (get_battery_percentage() < 100)
                {
                    OLED_printf(6, 2, "%d", get_battery_percentage());
                }
                else
                {
                    OLED_printf(3, 2, "%d", get_battery_percentage());
                }

                /*
                    OLED显示：
                    电池电量 DBUS YAW PIT:  遥控器， yaw轴电机， pitch轴电机
                    FR RR TRI COV:          发射机构电机： 左摩擦轮，右摩擦轮，拨盘，弹仓盖
                    M0 M1 M2 M3:            底盘动力电机： 右前， 左前， 左后， 右后
                    R1 R2 R3 R4:            底盘舵向电机： 右前， 左前， 左后， 右后
                    IMU REF CAP:            陀螺仪， 裁判通信， 超级电容

                */
                OLED_show_string(32, 2, "DBUS");
                OLED_show_graphic(48, 2, &check_box[error_list_local[DBUS_TOE].error_exist]);

                OLED_show_string(65, 2, "YAW");
                OLED_show_graphic(83, 2, &check_box[error_list_local[GIMBAL_YAW_MOTOR_TOE].error_exist]);

                OLED_show_string(97, 2, "PIT");
                OLED_show_graphic(115, 2, &check_box[error_list_local[GIMBAL_PITCH_MOTOR_TOE].error_exist]);

                for (i = SHOOT_LEFT_FRIC_MOTOR_ID; i < SHOOT_COVER_MOTOR_TOE + 1; i++)
                {
                    uint8_t j = i - SHOOT_LEFT_FRIC_MOTOR_ID;
                    show_col = (j * 32) % 128;
                    show_row = 13 ;
                    OLED_show_string(show_col , show_row, shoot_toe_name[j]);
                    OLED_show_graphic(show_col + 18, show_row, &check_box[error_list_local[i].error_exist]);
                }

                for (i = CHASSIS_MOTIVE_FR_MOTOR_TOE; i < CHASSIS_MOTIVE_BR_MOTOR_TOE + 1; i++)
                {
                    uint8_t j = i - CHASSIS_MOTIVE_FR_MOTOR_TOE;
                    show_col = (j * 32) % 128;
                    show_row = 26 ;
                    OLED_show_char(show_col, show_row, 'M');
                    OLED_show_char(show_col + 6, show_row, '0' + j);
                    OLED_show_graphic(show_col + 12, show_row, &check_box[error_list_local[i].error_exist]);
                }

                for (i = CHASSIS_RUDDER_FR_MOTOR_TOE; i < CHASSIS_RUDDER_BR_MOTOR_TOE + 1; i++)
                {
                    uint8_t j = i - CHASSIS_RUDDER_FR_MOTOR_TOE;
                    show_col = (j * 32) % 128;
                    show_row = 39 ;
                    OLED_show_char(show_col, show_row, 'R');
                    OLED_show_char(show_col + 6, show_row, '0' + j);
                    OLED_show_graphic(show_col + 12, show_row, &check_box[error_list_local[i].error_exist]);
                }

                for (i = RM_IMU_TOE; i < SUPER_CAP_TOE + 1; i++)
                {
                    uint8_t j = i - RM_IMU_TOE;
                    show_col = (i * 32) % 128;
                    show_row = 50 ;
                    OLED_show_string(show_col , show_row, other_toe_name[j]);
                    OLED_show_graphic(show_col + 18, show_row, &check_box[error_list_local[i].error_exist]);
                }

                OLED_refresh_gram();
            }
        }

        last_oled_error = now_oled_errror;
        osDelay(OLED_CONTROL_TIME);
    }
}

/*

                // OLED_show_string(65, 5, "YAW");
                // OLED_show_graphic(83, 5, &check_box[error_list_local[GIMBAL_YAW_MOTOR_TOE].error_exist]);

                // OLED_show_string(97, 5, "PIT");
                // OLED_show_graphic(115, 5, &check_box[error_list_local[GIMBAL_PITCH_MOTOR_TOE].error_exist]);

                // for (i = SHOOT_LEFT_FRIC_MOTOR_ID; i < SHOOT_COVER_MOTOR_TOE + 1; i++)
                // {
                //     uint8_t j = i - SHOOT_LEFT_FRIC_MOTOR_ID;
                //     show_col = (j * 32) % 128;
                //     show_row = 16 + j / 4 * 12;
                //     OLED_show_string(show_col + 6, show_row, shoot_toe_name[j]);
                //     OLED_show_graphic(show_col + 12, show_row, &check_box[error_list_local[i].error_exist]);
                // }

                // for (i = CHASSIS_MOTIVE_FR_MOTOR_TOE; i < CHASSIS_MOTIVE_BR_MOTOR_TOE + 1; i++)
                // {
                //     uint8_t j = i - CHASSIS_MOTIVE_FR_MOTOR_TOE;
                //     show_col = (j * 32) % 128;
                //     show_row = 32 + j / 4 * 12;
                //     OLED_show_char(show_col, show_row, 'M');
                //     OLED_show_char(show_col + 6, show_row, '0' + j);
                //     OLED_show_graphic(show_col + 12, show_row, &check_box[error_list_local[i].error_exist]);
                // }

                // for (i = CHASSIS_RUDDER_FR_MOTOR_TOE; i < CHASSIS_RUDDER_BR_MOTOR_TOE + 1; i++)
                // {
                //     uint8_t j = i - CHASSIS_RUDDER_FR_MOTOR_TOE;
                //     show_col = (j * 32) % 128;
                //     show_row = 48 + j / 4 * 12;
                //     OLED_show_char(show_col, show_row, 'R');
                //     OLED_show_char(show_col + 6, show_row, '0' + j);
                //     OLED_show_graphic(show_col + 12, show_row, &check_box[error_list_local[i].error_exist]);
                // }

                // for (i = RM_IMU_TOE; i < SUPER_CAP_TOE + 1; i++)
                // {
                //     show_col = (i * 32) % 128;
                //     show_row = 15 + i / 4 * 12;
                //     OLED_show_string(show_col, show_row, other_toe_name[i - RM_IMU_TOE]);
                //     OLED_show_graphic(show_col + 18, show_row, &check_box[error_list_local[i].error_exist]);
                // }
*/