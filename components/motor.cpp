#include "motor.hpp"
#include "bsp_can.h"

// 静态对象在启动时清零 不使用动态内存
static Motor motor;

void Motor::Init(uint32_t offline_ms)
{
    this->offline_ms = offline_ms;
    BSP_CAN_Init();
}

void Motor::Parse_Feedback(const CAN_RxHeaderTypeDef *header,
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
    feedback.encoder = (uint16_t)((data[0] << 8) | data[1]);
    feedback.speed = (int16_t)((data[2] << 8) | data[3]);
    feedback.current = (int16_t)((data[4] << 8) | data[5]);
    feedback.temperature = data[6];

    // 收到有效反馈后更新时间
    rx_tick = HAL_GetTick();
    received = 1;
}

uint8_t Motor::Is_Offline(void) const
{
    if (received == 0)
    {
        return 1;
    }

    if (HAL_GetTick() - rx_tick > offline_ms)
    {
        return 1;
    }

    return 0;
}

HAL_StatusTypeDef Motor::Send(int16_t value)
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

int16_t Motor::Get_Speed(void) const
{
    return feedback.speed;
}

// freertos.c仍按C语言编译 通过以下接口使用电机对象
extern "C" void Motor_Init(uint32_t offline_ms)
{
    motor.Init(offline_ms);
}

extern "C" uint8_t Motor_Is_Offline(void)
{
    return motor.Is_Offline();
}

extern "C" HAL_StatusTypeDef Motor_Send(int16_t value)
{
    return motor.Send(value);
}

extern "C" int16_t Motor_Get_Speed(void)
{
    return motor.Get_Speed();
}

/// @brief BSP收到报文后交给当前电机对象检查和解析
/// @param header 本次报文帧头 不能为NULL
/// @param data 本次报文数据 不能为NULL
/// @note 保留原有中断处理路径 这里不计算PID
extern "C" void BSP_CAN_Rx_Callback(const CAN_RxHeaderTypeDef *header,
                                    const uint8_t data[8])
{
    motor.Parse_Feedback(header, data);
}
