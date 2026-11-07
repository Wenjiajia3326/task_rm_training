#ifndef KEY_H
#define KEY_H

#include <stdint.h>

/// @brief 按键消抖的四个阶段
typedef enum
{
    KEY_RELEASED,     ///< 已松开 等待按下
    KEY_PRESS_CHECK,  ///< 等待确认按下
    KEY_PRESSED,      ///< 已按下 等待松开
    KEY_RELEASE_CHECK ///< 等待确认松开
} KeyState;

/// @brief 更新按键消抖状态
/// @param now 当前时间 单位ms
/// @return 1表示产生一次有效按下 0表示没有新的按下事件
/// @note 只由舵机任务周期调用 长按不重复触发
/// @note 间隔20ms后再次确认电平 不要求期间每次采样都相同
uint8_t Key_Update(uint32_t now);

#endif
