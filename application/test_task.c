/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       test_task.c/h
  * @brief      buzzer warning task.ˇäĂůĆ÷ą¨žŻČÎÎń
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

#include "test_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_buzzer.h"
#include "detect_task.h"

static void buzzer_warn_error(uint8_t num);

const error_t *error_list_test_local;

uint8_t exit_flag = 0;
uint8_t rising_falling_flag;
uint8_t buzzer_close_flag = 0;
/**
  * @brief          test task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          testČÎÎń
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void test_task(void const * argument)
{
    static uint8_t error, last_error;
    static uint8_t error_num;
    error_list_test_local = get_error_list_point();

    while(1)
    {
        error = 0;

        //find error
        //ˇ˘ĎÖ´íÎó
        for(error_num = 0; error_num < REFEREE_TOE; error_num++)
        {
            if(error_list_test_local[error_num].error_exist)
            {
                error = 1;
                break;
            }
        }

        if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET && exit_flag  == 0)
        {
            exit_flag = 1;
            rising_falling_flag = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
        }
            

        //前台程序
        if (exit_flag == 1)
        {
            exit_flag = 2;
            if (rising_falling_flag == GPIO_PIN_RESET)
            {
                //debouce
                //消抖
                HAL_Delay(20);
                if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
                {
                    if (buzzer_close_flag == 0)
                        buzzer_close_flag = 1;
                    else if (buzzer_close_flag == 1)
                        buzzer_close_flag = 0;

                    exit_flag = 0;
                }
                else
                {
                    exit_flag = 0;
                }
            }
            else if (rising_falling_flag == GPIO_PIN_SET)
            {
                //debouce
                //消抖
                HAL_Delay(20);
                if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET)
                {
                    exit_flag = 0;
                }
                else
                {
                    exit_flag = 0;
                }
            }
        }

        //关闭蜂鸣器
        if (buzzer_close_flag == 1)
        {
            buzzer_off();
        }
        else //打开蜂鸣器
        {
            //no error, stop buzzer
            //没有错误, 停止蜂鸣器
            if (error == 0 && last_error != 0)
            {
                buzzer_off();
            }
            //have error
            //有错误
            if (error)
            {
                buzzer_warn_error(error_num + 1);
            }
        }


        last_error = error;
        osDelay(10);
    }
}



/**
  * @brief          ĘšľĂˇäĂůĆ÷Ďě
  * @param[in]      num:ĎěÉů´ÎĘý
  * @retval         none
  */
static void buzzer_warn_error(uint8_t num)
{
    static uint8_t show_num = 0;
    static uint8_t stop_num = 100;
    if(show_num == 0 && stop_num == 0)
    {
        show_num = num;
        stop_num = 100;
    }
    else if(show_num == 0)
    {
        stop_num--;
        buzzer_off();
    }
    else
    {
        static uint8_t tick = 0;
        tick++;
        if(tick < 50)
        {
            buzzer_off();
        }
        else if(tick < 100)
        {
            buzzer_on(1, 30000);
        }
        else
        {
            tick = 0;
            show_num--;
        }
    }
}


