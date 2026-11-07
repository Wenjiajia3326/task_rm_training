#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
typedef struct
{
    uint16_t encoder;
    int16_t speed;
    int16_t current;
    uint8_t temperature;
} MotorData;

// volatile用于中断与任务共享 单次读取字段 不保证多字段快照一致
extern volatile MotorData motor;

void Motor_Init(void);
uint8_t Motor_IsOffline(void);
HAL_StatusTypeDef Motor_Send(int16_t value);

#endif