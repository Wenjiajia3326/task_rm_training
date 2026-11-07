#ifndef BSP_BOARD_H
#define BSP_BOARD_H

#include "stm32f4xx_hal.h"

/// @brief 读取板载按键的原始电平 不进行消抖
/// @return GPIO_PIN_RESET表示按下 GPIO_PIN_SET表示松开
GPIO_PinState BSP_Key_Read(void);

/// @brief 启动TIM1通道1的舵机PWM 需先设置初始脉宽
void BSP_Servo_Start(void);

/// @brief 设置舵机PWM脉宽 不负责角度计算
/// @param pulse_us 脉宽 单位us 上层按舵机要求限制范围
/// @note 当前TIM1计数周期为1us
void BSP_Servo_Set_Pulse(uint16_t pulse_us);

/// @brief 启动TIM5通道3的红灯PWM
void BSP_Red_Start(void);

/// @brief 设置红灯PWM比较值
/// @param brightness 0为关闭 1000为全亮 对应当前ARR为999
void BSP_Red_Set(uint16_t brightness);

/// @brief 设置绿灯亮灭
/// @param on 0表示关闭 非0表示点亮
void BSP_Green_Set(uint8_t on);

#endif
