#include "motor.h"
#include "can.h"
#include "app_config.h"
// 中断更新反馈 任务每次读取内存中的最新字段
volatile MotorData motor = {0};

static volatile uint32_t motor_rx_tick = 0;
static volatile uint8_t motor_received = 0;


void Motor_Init(void) // 配置过滤器并启动CAN接收
{
    CAN_FilterTypeDef filter = {0}; // 先把过滤器配置结构体清零

    filter.FilterBank = 0;
    filter.SlaveStartFilterBank = 14; // 从第14组开始分给CAN2

    filter.FilterMode = CAN_FILTERMODE_IDMASK;// 过滤器模式为掩码模式方便接收所有报文
    filter.FilterScale = CAN_FILTERSCALE_32BIT;

    filter.FilterIdHigh = 0;
    filter.FilterIdLow = 0;
    filter.FilterMaskIdHigh = 0;
    filter.FilterMaskIdLow = 0;//全部掩码为0先接收全部数据

    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;//将过滤器分配给FIFO0
    filter.FilterActivation = ENABLE;//最后再将过滤器激活

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
    {
        Error_Handler();
    }
    // 启动CAN1
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }

    // FIFO0收到报文时产生接收中断
    if (HAL_CAN_ActivateNotification(
            &hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//CAN1 收到报文以后 这个函数把接收队列 FIFO0 里面的报文取出来。
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    if (hcan->Instance != CAN1)
    {
        return;
    }

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0,
                            &header, data) != HAL_OK)
    {
        return;
    }

    // 只处理标准数据帧
    if (header.IDE != CAN_ID_STD ||
        header.RTR != CAN_RTR_DATA)
    {
        return;
    }

    // GM6020反馈包含8个数据字节
    if (header.DLC != 8)
    {
        return;
    }

    // 按电机ID为1处理 
    if (header.StdId != 0x205)
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
uint8_t Motor_IsOffline(void) // 判断电机是否掉线
{
    if (motor_received == 0)
    {
        return 1;
    }

    if (HAL_GetTick() - motor_rx_tick > MOTOR_OFFLINE_MS)
    {
        return 1;
    }

    return 0;
}
HAL_StatusTypeDef Motor_Send(int16_t value) // 发送电压控制值 电机ID为1
{
    CAN_TxHeaderTypeDef header = {0};
    uint8_t data[8] = {0};
    uint32_t mailbox;

    // 电压控制值范围为-25000到25000
    if (value > 25000 || value < -25000)
    {
        return HAL_ERROR;
    }

    uint16_t raw = (uint16_t)value;

    header.StdId = 0x1FF;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8;
    header.TransmitGlobalTime = DISABLE;

    // 电机ID为1 使用前两个字节
    data[0] = (uint8_t)(raw >> 8);
    data[1] = (uint8_t)(raw & 0xFF);

    // 没有空闲邮箱时 本次不提交
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
    {
        return HAL_BUSY;
    }

    return HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox);
}