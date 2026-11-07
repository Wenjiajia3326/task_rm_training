#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/// @brief 电机控制周期 单位ms
#define MOTOR_PERIOD_MS 10

/// @brief PID计算间隔 单位s
#define MOTOR_PID_DT (MOTOR_PERIOD_MS / 1000.0f)

/// @brief 速度环比例系数
#define MOTOR_SPEED_KP 25.0f

/// @brief 速度环积分系数
#define MOTOR_SPEED_KI 13.0f

/// @brief 速度环微分系数
#define MOTOR_SPEED_KD 0.0f

/// @brief 积分累计值的限制
#define MOTOR_INTEGRAL_LIMIT 650.0f

/// @brief PID输出的限制
#define MOTOR_OUTPUT_LIMIT 12000.0f

/// @brief 电机反馈超时时间 单位ms
#define MOTOR_OFFLINE_MS 100

/// @brief 串口接收缓冲区大小 单位字节
#define UART_RX_SIZE 64

/// @brief 串口发送实际转速的间隔 单位ms
#define UART_REPORT_PERIOD_MS 2000

/// @brief 舵机每次按键增加的角度 单位度
#define SERVO_STEP_DEG 20

/// @brief 舵机最大目标角度 单位度
#define SERVO_MAX_DEG 180

/// @brief 舵机零度对应的PWM脉宽 单位us
#define SERVO_MIN_PULSE_US 500

/// @brief 舵机从零度到最大角度的脉宽变化量 单位us
#define SERVO_PULSE_RANGE_US 2000

/// @brief 红灯完整呼吸周期 单位ms
#define LED_BREATH_PERIOD_MS 2000

#endif
