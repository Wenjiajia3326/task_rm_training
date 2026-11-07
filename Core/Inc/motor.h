#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

/// @brief 电机CAN反馈数据
typedef struct
{
    uint16_t encoder;    ///< 编码器位置 范围0到8191
    int16_t speed;       ///< 实际转速 单位rpm
    int16_t current;     ///< 电流反馈原始值
    uint8_t temperature; ///< 电机温度 单位摄氏度
} MotorData;

/// @brief CAN接收中断更新的数据 供任务读取
/// @note volatile不保证同时读取多个字段时数据属于同一帧
extern volatile MotorData motor;

/// @brief 配置CAN过滤器并启动电机反馈接收
/// @note 启动电机任务时调用一次 配置失败进入Error_Handler
void Motor_Init(void);

/// @brief 判断电机反馈是否超时
/// @return 1表示未收到反馈或反馈超时 0表示在线
uint8_t Motor_Is_Offline(void);

/// @brief 发送GM6020电压控制值
/// @param value 电压控制值 范围为-25000到25000
/// @return HAL_OK表示已提交发送 HAL_BUSY表示无空闲邮箱
///         HAL_ERROR表示参数错误或HAL发送失败
/// @note 当前控制电机ID为1 HAL_OK不代表电机已执行
HAL_StatusTypeDef Motor_Send(int16_t value);

#endif
