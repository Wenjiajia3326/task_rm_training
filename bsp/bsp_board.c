#include "bsp_board.h"
#include "main.h"
#include "tim.h"
#include "bsp_config.h"

GPIO_PinState BSP_Key_Read(void)
{
    return HAL_GPIO_ReadPin(BSP_KEY_PORT, BSP_KEY_PIN);
}

void BSP_Servo_Start(void)
{
    HAL_TIM_PWM_Start(&BSP_SERVO_TIMER, BSP_SERVO_CHANNEL);
}

void BSP_Servo_Set_Pulse(uint16_t pulse_us)
{
    __HAL_TIM_SET_COMPARE(&BSP_SERVO_TIMER, BSP_SERVO_CHANNEL, pulse_us);
}

void BSP_Red_Start(void)
{
    HAL_TIM_PWM_Start(&BSP_RED_TIMER, BSP_RED_CHANNEL);
}

void BSP_Red_Set(uint16_t brightness)
{
    __HAL_TIM_SET_COMPARE(&BSP_RED_TIMER, BSP_RED_CHANNEL, brightness);
}

void BSP_Green_Set(uint8_t on)
{
    if (on == 0)
    {
        HAL_GPIO_WritePin(BSP_GREEN_PORT, BSP_GREEN_PIN, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(BSP_GREEN_PORT, BSP_GREEN_PIN, GPIO_PIN_SET);
    }
}
