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

#endif