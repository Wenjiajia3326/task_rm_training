#ifndef BSP_CONFIG_H
#define BSP_CONFIG_H

/// @brief 电机通信使用的CAN句柄和外设
#define BSP_CAN_HANDLE hcan1
#define BSP_CAN_INSTANCE CAN1

/// @brief CAN1过滤器组号和CAN2过滤器起始组号
#define BSP_CAN_FILTER_BANK 0
#define BSP_CAN_SLAVE_START_BANK 14

/// @brief 命令通信使用的串口句柄
#define BSP_UART_HANDLE huart6

/// @brief 串口阻塞发送超时 单位ms
#define BSP_UART_TX_TIMEOUT_MS 100

/// @brief 舵机PWM定时器和通道 当前计数周期为1us
#define BSP_SERVO_TIMER htim1
#define BSP_SERVO_CHANNEL TIM_CHANNEL_1

/// @brief 红灯PWM定时器和通道
#define BSP_RED_TIMER htim5
#define BSP_RED_CHANNEL TIM_CHANNEL_3

/// @brief 红灯全亮比较值 必须与当前TIM5的ARR为999对应
#define BSP_LED_PWM_MAX 1000

/// @brief 板载按键引脚 沿用CubeMX生成的引脚定义
#define BSP_KEY_PORT KEY_GPIO_Port
#define BSP_KEY_PIN KEY_Pin

/// @brief 绿灯引脚 沿用CubeMX生成的引脚定义
#define BSP_GREEN_PORT LED_G_GPIO_Port
#define BSP_GREEN_PIN LED_G_Pin

#endif
