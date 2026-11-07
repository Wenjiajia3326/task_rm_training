#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "motor.h"

/// @brief 管理一台GM6020的反馈和电压控制 当前电机ID为1
/// @note 反馈由中断更新 任务只读取单个字段 不保证多字段快照一致
class Motor
{
public:
    /// @brief 保存反馈超时时间并启动CAN接收 任务启动时调用一次
    /// @param offline_ms 应用层传入的反馈超时时间 单位ms
    void Init(uint32_t offline_ms);

    /// @brief 检查帧格式和ID 合法时更新电机反馈
    /// @param header 收到的CAN帧头 不能为NULL
    /// @param data 收到的8字节数组 不能为NULL
    /// @note 在接收中断内调用 不保存传入指针
    void Parse_Feedback(const CAN_RxHeaderTypeDef *header,
                        const uint8_t data[8]);

    /// @brief 判断反馈是否超时
    /// @return 1表示离线 0表示在线
    uint8_t Is_Offline(void) const;

    /// @brief 读取最新反馈转速
    /// @return 实际转速 单位rpm
    int16_t Get_Speed(void) const;

    /// @brief 发送电压控制值
    /// @param value 控制量 范围为-25000到25000
    /// @return BSP的发送结果 参数越界返回HAL_ERROR
    HAL_StatusTypeDef Send(int16_t value);

private:
    volatile MotorData feedback; ///< CAN中断更新的编码器 转速 电流和温度
    volatile uint32_t rx_tick;   ///< 最近一次有效反馈时间 单位ms
    volatile uint8_t received;   ///< 1表示已收到过有效反馈
    uint32_t offline_ms;        ///< 应用层初始化时传入的超时时间
};

#endif
