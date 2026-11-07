/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "pid.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"
#include "tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_SIZE 64
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
PID speed_pid;
// 串口任务修改 电机任务读取 初始为0表示不驱动
volatile float target_speed = 0.0f;
int16_t motor_value = 0;
HAL_StatusTypeDef send_result = HAL_OK;


// 串口收到的数据
uint8_t rx_buffer[RX_SIZE];
// 记录这次收到多少个字节
volatile uint16_t rx_length = 0;
// 1表示有数据等待处理
volatile uint8_t rx_flag = 0;
/* USER CODE END Variables */
osThreadId motorTaskHandle;
osThreadId uartTaskHandle;
osThreadId servoTaskHandle;
osThreadId ledTaskHandle;
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void ParseMotorCommand(void);

/* USER CODE END FunctionPrototypes */

void StartMotorTask(void const * argument);
void StartUartTask(void const * argument);
void StartServoTask(void const * argument);
void StartLedTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */

  /* definition and creation of motorTask */
  osThreadDef(motorTask, StartMotorTask, osPriorityAboveNormal, 0, 256);
  motorTaskHandle = osThreadCreate(osThread(motorTask), NULL);

  /* definition and creation of uartTask */
  osThreadDef(uartTask, StartUartTask, osPriorityNormal, 0, 512);
  uartTaskHandle = osThreadCreate(osThread(uartTask), NULL);

  /* definition and creation of servoTask */
  osThreadDef(servoTask, StartServoTask, osPriorityNormal, 0, 256);
  servoTaskHandle = osThreadCreate(osThread(servoTask), NULL);

  /* definition and creation of ledTask */
  osThreadDef(ledTask, StartLedTask, osPriorityBelowNormal, 0, 256);
  ledTaskHandle = osThreadCreate(osThread(ledTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}


/* USER CODE BEGIN Header_StartMotorTask */
/**
* @brief Function implementing the motorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void const * argument)
{
  /* USER CODE BEGIN StartMotorTask */
  (void)argument;

  PID_Init(&speed_pid, 25.0f, 13.0f, 0.0f,
           0.01f, 650.0f, 12000.0f);
  Motor_Init();
  // 在循环外记录起点 后面每次按10ms周期唤醒
  TickType_t last_tick = xTaskGetTickCount();
  for (;;)
  {
    // 读一次共享目标 本轮判断和计算使用同一个值
    float speed = target_speed;

    // 离线或目标为0时 清除积分并发送零控制量
    if (Motor_IsOffline() == 1 || speed == 0.0f)
    {
      PID_Reset(&speed_pid);
      motor_value = 0;
    }
    else
    {
      motor_value = (int16_t)PID_Calc(
          &speed_pid, speed, motor.speed);
    }

    send_result = Motor_Send(motor_value);
    vTaskDelayUntil(&last_tick, pdMS_TO_TICKS(10));
  }
  /* USER CODE END StartMotorTask */
}


/* USER CODE BEGIN Header_StartUartTask */
/**
* @brief Function implementing the uartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUartTask */
void StartUartTask(void const * argument)
{
  /* USER CODE BEGIN StartUartTask */
  (void)argument;
  // 记录上次发送的时间
  uint32_t send_tick = HAL_GetTick();

// 保存准备发送的文字及其长度
  char text[32];
  int length;

  
  HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rx_buffer, RX_SIZE);
  
  __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);

  for (;;)
{
  // 收到数据后才处理
  if (rx_flag == 1)
  {
    // 检查命令 合法时修改目标转速
    ParseMotorCommand();

    // 这批数据已经处理完
    rx_length = 0;
    rx_flag = 0;

    // 开始接收下一批数据
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rx_buffer, RX_SIZE);
    __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
  }
  // 距离上次发送达到2秒 就反馈实际转速
if (HAL_GetTick() - send_tick >= 2000)
{
  send_tick = HAL_GetTick();

  length = sprintf(text, "speed: %d rpm\r\n", (int)motor.speed);

  HAL_UART_Transmit(&huart6, (uint8_t *)text, length, 100);
}

  osDelay(1);
}
  /* USER CODE END StartUartTask */
}

/* USER CODE BEGIN Header_StartServoTask */
/**
* @brief Function implementing the servoTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartServoTask */
void StartServoTask(void const * argument)
{
  /* USER CODE BEGIN StartServoTask */
  (void)argument;

  // 保存舵机目标角度 初始为0度
  uint16_t angle = 0;

  // 下一步用它记录按下是否已处理 避免按住时重复增加角度
  uint8_t pressed = 0;

  // TIM1每计数一次是1us 标称0到180度映射为500到2500us
  // 先设置初始脉宽
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
                        500 + angle * 2000 / 180);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

 for (;;)
{
  // 按下是低电平 等20ms后再确认
  if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
  {
    osDelay(20);

    if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET &&
        pressed == 0)
    {
      // 本次按下已经处理 按住不再重复执行
      pressed = 1;

      // 每次增加20度 到180度后保持
      if (angle < 180)
      {
        angle += 20;
      }

      // 根据新角度更新PWM脉宽
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
                            500 + angle * 2000 / 180);
    }
  }
  else
  {
    // 松开也确认20ms 再允许下一次按下
    osDelay(20);

    if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET)
    {
      pressed = 0;
    }
  }

  osDelay(10);
}
  /* USER CODE END StartServoTask */
}

/* USER CODE BEGIN Header_StartLedTask */
/**
* @brief Function implementing the ledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedTask */
void StartLedTask(void const * argument)
{
  /* USER CODE BEGIN StartLedTask */
  (void)argument;

  // phase是2秒周期中的位置 brightness是PWM比较值
  uint32_t phase;
  uint16_t brightness;

  // 红灯PH12使用TIM5通道3 PWM只需启动一次
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);

  for (;;)
  {
    if (Motor_IsOffline() == 1)
    {
      // 没有有效电机反馈时 关闭绿灯
      HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);

      // 余数在0到1999之间循环 对应2秒的呼吸周期
      phase = HAL_GetTick() % 2000;
      if (phase < 1000)
      {
        // 前1秒比较值逐渐增加 红灯逐渐变亮
        brightness = phase;
      }
      else
      {
        // 后1秒比较值逐渐减小 红灯逐渐变暗
        brightness = 2000 - phase;
      }

      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, brightness);
    }
    else
    {
      // 电机在线 红灯关闭 绿灯常亮
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
      HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
    }

    // 每隔约10ms更新亮度 等待时其他任务可以运行
    osDelay(10);
  }
  /* USER CODE END StartLedTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
// 串口空闲或DMA接满时 HAL会调用这个函数
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  // 只处理USART6收到的数据
  if (huart != &huart6)
  {
    return;
  }

  // 先记录长度 再标记有数据需要处理
  rx_length = size;
  rx_flag = 1;
}
// 每次发送一条完整命令 例如motor+100或motor-100
// 错误时直接退出 保留原目标 不在回调里计算或控制电机
// 当前按空闲分批 不拼接分批命令 也不拆分连续的多条命令
// 检查转速命令 合法时修改目标转速
void ParseMotorCommand(void)
{
  uint16_t length = rx_length;
  int32_t speed = 0;

  // 长度不能超过接收数组 接满时整批丢弃 避免执行截断命令
  if (length == 0 || length >= RX_SIZE)
  {
    return;
  }

  // 去掉末尾的回车和换行
  while (length > 0)
  {
    if (rx_buffer[length - 1] != '\r' &&
        rx_buffer[length - 1] != '\n')
    {
      break;
    }

    length--;
  }

  // 最短命令是motor+0 共7个字符
  if (length < 7)
  {
    return;
  }

  // 前5个字符必须是motor
  if (strncmp((char *)rx_buffer, "motor", 5) != 0)
  {
    return;
  }

  // 第6个字符必须是正号或负号
  if (rx_buffer[5] != '+' && rx_buffer[5] != '-')
  {
    return;
  }

  // 把后面的数字字符转换成转速
  for (uint16_t i = 6; i < length; i++)
  {
    if (rx_buffer[i] < '0' || rx_buffer[i] > '9')
    {
      return;
    }

    speed = speed * 10 + (rx_buffer[i] - '0');

    // 防止数值过大 这里按反馈转速的数据范围检查
    if (speed > 32767)
    {
      return;
    }
  }

  if (rx_buffer[5] == '-')
  {
    speed = -speed;
  }

  // 全部检查通过后才修改目标
  target_speed = speed;
}
/* USER CODE END Application */
