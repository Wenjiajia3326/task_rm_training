#include "motor.h"
#include "bsp_can.h"

// 中断更新反馈 任务每次读取内存中的最新字段
volatile MotorData motor = {0};

/// @brief 最近一次有效反馈的接收时间 单位ms
static volatile uint32_t motor_rx_tick = 0;
/// @brief 是否收到过有效反馈 由CAN中断更新
static volatile uint8_t motor_received = 0;

// 由应用层在启动CAN之前设置 运行期间只读取
static uint32_t motor_offline_ms = 0;

void Motor_Init(uint32_t offline_ms)
{
    motor_offline_ms = offline_ms;
    BSP_CAN_Init();
}

/// @brief 检查电机反馈帧并解析数据
/// @param header BSP传入的帧头
/// @param data BSP传入的原始数据
/// @note 在CAN接收中断内执行
void BSP_CAN_Rx_Callback(const CAN_RxHeaderTypeDef *header,
                         const uint8_t data[8])
{
    // 只处理标准数据帧
    if (header->IDE != CAN_ID_STD ||
        header->RTR != CAN_RTR_DATA)
    {
        return;
    }

    // GM6020反馈包含8个数据字节
    if (header->DLC != 8)
    {
        return;
    }

    // 按电机ID为1处理 
    if (header->StdId != 0x205)
    {
        return;
    }

    // 按反馈协议合并高低字节
    motor.encoder = (uint16_t)((data[0] << 8) | data[1]);
    motor.speed = (int16_t)((data[2] << 8) | data[3]);
    motor.current = (int16_t)((data[4] << 8) | data[5]);
    motor.temperature = data[6];

    // 收到有效反馈后更新时间
    motor_rx_tick = HAL_GetTick();
    motor_received = 1;
}

uint8_t Motor_Is_Offline(void)
{
    if (motor_received == 0)
    {
        return 1;
    }

    if (HAL_GetTick() - motor_rx_tick > motor_offline_ms)
    {
        return 1;
    }

    return 0;
}

HAL_StatusTypeDef Motor_Send(int16_t value)
{
    uint8_t data[8] = {0};

    // 电压控制值范围为-25000到25000
    if (value > 25000 || value < -25000)
    {
        return HAL_ERROR;
    }

    uint16_t raw = (uint16_t)value;

    // 电机ID为1 使用前两个字节
    data[0] = (uint8_t)(raw >> 8);
    data[1] = (uint8_t)(raw & 0xFF);

    return BSP_CAN_Send(0x1FF, data);
}
