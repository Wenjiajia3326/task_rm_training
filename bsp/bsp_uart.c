#include "bsp_uart.h"
#include "usart.h"

HAL_StatusTypeDef BSP_UART_Start_Receive(uint8_t *buffer, uint16_t size)
{
    HAL_StatusTypeDef result;

    result = HAL_UARTEx_ReceiveToIdle_DMA(&huart6, buffer, size);
    // 保留空闲和接满事件 不处理接收一半的事件
    __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
    return result;
}

HAL_StatusTypeDef BSP_UART_Send(uint8_t *data, uint16_t length)
{
    return HAL_UART_Transmit(&huart6, data, length, 100);
}

/// @brief HAL接收事件入口 只转交USART6的事件
/// @param huart 产生接收事件的串口句柄
/// @param size 本次接收长度 单位字节
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart != &huart6)
    {
        return;
    }

    BSP_UART_Rx_Callback(size);
}

__weak void BSP_UART_Rx_Callback(uint16_t length)
{
    (void)length;
}
