#include "bsp_board.h"
#include "main.h"
#include "tim.h"

GPIO_PinState BSP_Key_Read(void)
{
    return HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
}

void BSP_Servo_Start(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void BSP_Servo_Set_Pulse(uint16_t pulse_us)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_us);
}

void BSP_Red_Start(void)
{
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
}

void BSP_Red_Set(uint16_t brightness)
{
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, brightness);
}

void BSP_Green_Set(uint8_t on)
{
    if (on == 0)
    {
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
    }
}
