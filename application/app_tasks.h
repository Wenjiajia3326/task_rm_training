#ifndef APP_TASKS_H
#define APP_TASKS_H

/// @brief 运行电机控制任务 初始化后按固定周期循环
void App_Motor_Task(void);

/// @brief 运行串口任务 处理命令并反馈转速
void App_UART_Task(void);

/// @brief 运行舵机任务 有效按下时增加目标角度
void App_Servo_Task(void);

/// @brief 运行灯光任务 根据电机在线状态更新输出
/// @note 以上函数均包含无限循环 每个函数只由对应任务调用一次
void App_LED_Task(void);

#endif
