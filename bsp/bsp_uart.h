#ifndef BSP_UART_H
#define BSP_UART_H

#include "stm32f4xx_hal.h"

/// @brief 启动USART6普通模式DMA接收并关闭半满中断
/// @param buffer 接收数组 不能为NULL 接收和上层处理期间必须保持有效
/// @param size 接收数组容量 单位字节 必须大于0
/// @return HAL接收启动结果
/// @note 上层处理完本批数据后再调用 启动接收前需完成CubeMX的DMA初始化
HAL_StatusTypeDef BSP_UART_Start_Receive(uint8_t *buffer, uint16_t size);

/// @brief 使用USART6发送数据 超时保持为100ms
/// @param data 待发送数组 不能为NULL
/// @param length 发送长度 单位字节
/// @return HAL发送结果
/// @note 阻塞调用 只在串口任务中使用
HAL_StatusTypeDef BSP_UART_Send(uint8_t *data, uint16_t length);

/// @brief 接收事件交给上层 默认不处理
/// @param length 本次接收长度 单位字节
/// @note 由上层提供同名实现 在中断中调用 仅交接长度和就绪标志
void BSP_UART_Rx_Callback(uint16_t length);

#endif
