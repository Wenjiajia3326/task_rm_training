#include "bsp_can.h"
#include "can.h"

void BSP_CAN_Init(void)
{
    CAN_FilterTypeDef filter = {0};

    filter.FilterBank = 0;
    filter.SlaveStartFilterBank = 14; // 从第14组开始分给CAN2

    filter.FilterMode = CAN_FILTERMODE_IDMASK; // 掩码全为0时接收全部报文
    filter.FilterScale = CAN_FILTERSCALE_32BIT;

    filter.FilterIdHigh = 0;
    filter.FilterIdLow = 0;
    filter.FilterMaskIdHigh = 0;
    filter.FilterMaskIdLow = 0;

    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;

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

HAL_StatusTypeDef BSP_CAN_Send(uint16_t id, uint8_t data[8])
{
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox;

    header.StdId = id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8;
    header.TransmitGlobalTime = DISABLE;

    // 邮箱满时直接返回 不阻塞当前任务
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
    {
        return HAL_BUSY;
    }

    return HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox);
}

/// @brief HAL接收入口 只取出CAN1报文并交给上层
/// @param hcan 产生接收中断的CAN句柄
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    if (hcan->Instance != CAN1)
    {
        return;
    }

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
    {
        return;
    }

    BSP_CAN_Rx_Callback(&header, data);
}

// weak表示默认实现 上层的同名普通函数会替代它
__weak void BSP_CAN_Rx_Callback(const CAN_RxHeaderTypeDef *header,
                                const uint8_t data[8])
{
    (void)header;
    (void)data;
}
