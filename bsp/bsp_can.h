#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "stm32f4xx_hal.h"

/// @brief 配置CAN1过滤器并启动FIFO0接收中断
/// @note 在任务启动时调用一次 失败进入Error_Handler
void BSP_CAN_Init(void);

/// @brief 从CAN1发送8字节标准数据帧
/// @param id 标准帧ID 范围0到0x7FF
/// @param data 待发送的8字节数组 不能为NULL
/// @return HAL_OK表示提交成功 HAL_BUSY表示无空闲邮箱 其他值为HAL错误
HAL_StatusTypeDef BSP_CAN_Send(uint16_t id, uint8_t data[8]);

/// @brief 将收到的一帧交给上层 默认不处理
/// @param header 本次报文的帧头 仅回调期间有效
/// @param data 本次报文的8字节接收数组 仅前DLC个字节有效
/// @note 由上层提供同名实现 在中断内调用 不可阻塞或保存传入指针
void BSP_CAN_Rx_Callback(const CAN_RxHeaderTypeDef *header,
                         const uint8_t data[8]);

#endif
