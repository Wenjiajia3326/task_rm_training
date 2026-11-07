#include "bsp_uart.h"
#include "usart.h"
#include "bsp_config.h"

HAL_StatusTypeDef BSP_UART_Start_Receive(uint8_t *buffer, uint16_t size)
{
    HAL_StatusTypeDef result;

    result = HAL_UARTEx_ReceiveToIdle_DMA(&BSP_UART_HANDLE, buffer, size);
    // 保留空闲和接满事件 不处理接收一半的事件
    __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
    return result;
}

HAL_StatusTypeDef BSP_UART_Send(uint8_t *data, uint16_t length)
{
    return HAL_UART_Transmit(&BSP_UART_HANDLE, data, length, BSP_UART_TX_TIMEOUT_MS);
}

/// @brief HAL接收事件入口 只转交USART6的事件
/// @param huart 产生接收事件的串口句柄
/// @param size 本次接收长度 单位字节
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart != &BSP_UART_HANDLE)
    {
        return;
    }

    BSP_UART_Rx_Callback(size);
}

__weak void BSP_UART_Rx_Callback(uint16_t length)
{
    (void)length;
}
