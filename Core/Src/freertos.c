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
#include "app_tasks.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId motorTaskHandle;
osThreadId uartTaskHandle;
osThreadId servoTaskHandle;
osThreadId ledTaskHandle;
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

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
/// @brief 按固定周期计算速度环PID并发送电机控制量
/// @param argument 未使用的任务参数
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void const * argument)
{
  /* USER CODE BEGIN StartMotorTask */
  (void)argument;
  App_Motor_Task();
  /* USER CODE END StartMotorTask */
}

/* USER CODE BEGIN Header_StartUartTask */
/// @brief 处理串口命令并定时反馈实际转速
/// @param argument 未使用的任务参数
/* USER CODE END Header_StartUartTask */
void StartUartTask(void const * argument)
{
  /* USER CODE BEGIN StartUartTask */
  (void)argument;
  App_UART_Task();
  /* USER CODE END StartUartTask */
}

/* USER CODE BEGIN Header_StartServoTask */
/// @brief 检查按键并更新舵机目标角度
/// @param argument 未使用的任务参数
/* USER CODE END Header_StartServoTask */
void StartServoTask(void const * argument)
{
  /* USER CODE BEGIN StartServoTask */
  (void)argument;
  App_Servo_Task();
  /* USER CODE END StartServoTask */
}

/* USER CODE BEGIN Header_StartLedTask */
/// @brief 根据电机在线状态更新红绿灯
/// @param argument 未使用的任务参数
/* USER CODE END Header_StartLedTask */
void StartLedTask(void const * argument)
{
  /* USER CODE BEGIN StartLedTask */
  (void)argument;
  App_LED_Task();
  /* USER CODE END StartLedTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
