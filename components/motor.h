#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f4xx_hal.h"

/// @brief 电机CAN反馈数据
typedef struct
{
    uint16_t encoder;    ///< 编码器位置 范围0到8191
    int16_t speed;       ///< 实际转速 单位rpm
    int16_t current;     ///< 电流反馈原始值
    uint8_t temperature; ///< 电机温度 单位摄氏度
} MotorData;

#ifdef __cplusplus
extern "C" {
#endif

/// @brief 保存离线时间并启动电机反馈接收
/// @param offline_ms 反馈超时时间 单位ms 由应用层传入
/// @note 启动电机任务时调用一次 配置失败进入Error_Handler
void Motor_Init(uint32_t offline_ms);

/// @brief 判断电机反馈是否超时
/// @return 1表示未收到反馈或反馈超时 0表示在线
uint8_t Motor_Is_Offline(void);

/// @brief 发送GM6020电压控制值
/// @param value 电压控制值 范围为-25000到25000
/// @return HAL_OK表示已提交发送 HAL_BUSY表示无空闲邮箱
///         HAL_ERROR表示参数错误或HAL发送失败
/// @note 当前控制电机ID为1 HAL_OK不代表电机已执行
HAL_StatusTypeDef Motor_Send(int16_t value);

/// @brief 读取最新的电机反馈转速
/// @return 实际转速 单位rpm 尚未收到反馈时为0
int16_t Motor_Get_Speed(void);

#ifdef __cplusplus
}
#endif

#endif
