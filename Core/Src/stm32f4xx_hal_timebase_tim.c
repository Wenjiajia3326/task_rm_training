#include "stm32f4xx_hal.h"
TIM_HandleTypeDef htim6;

// TIM6负责HAL的1ms计时。SysTick由FreeRTOS使用。
HAL_StatusTypeDef HAL_InitTick(uint32_t priority)
{
    RCC_ClkInitTypeDef clocks;
    uint32_t latency;
    HAL_RCC_GetClockConfig(&clocks, &latency);
    uint32_t clock = HAL_RCC_GetPCLK1Freq();
    if (clocks.APB1CLKDivider != RCC_HCLK_DIV1) clock *= 2;
    __HAL_RCC_TIM6_CLK_ENABLE();
    htim6.Instance = TIM6;
    htim6.Init.Prescaler = clock / 1000000U - 1U;
    htim6.Init.Period = 999;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (priority >= (1UL << __NVIC_PRIO_BITS)) return HAL_ERROR;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK) return HAL_ERROR;
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, priority, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    uwTickPrio = priority;
    return HAL_TIM_Base_Start_IT(&htim6);
}
void HAL_SuspendTick(void) { __HAL_TIM_DISABLE_IT(&htim6, TIM_IT_UPDATE); }
void HAL_ResumeTick(void) { __HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE); }
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) HAL_IncTick();
}
