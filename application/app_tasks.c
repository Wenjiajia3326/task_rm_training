#include "app_tasks.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "motor.h"
#include "pid.h"
#include "key.h"
#include "bsp_uart.h"
#include "string.h"
#include "stdio.h"
#include "bsp_board.h"
#include "app_config.h"

/// @brief 速度环控制器 只在电机任务中计算和重置
PID speed_pid;
// 串口任务修改 电机任务读取 初始为0表示不驱动
volatile float target_speed = 0.0f;
int16_t motor_value = 0;
HAL_StatusTypeDef send_result = HAL_OK;

// 串口收到的数据
uint8_t rx_buffer[UART_RX_SIZE];
// 记录这次收到多少个字节
volatile uint16_t rx_length = 0;
// 1表示有数据等待处理
volatile uint8_t rx_flag = 0;

static void Motor_Parse_Command(void);

void App_Motor_Task(void)
{
  PID_Init(&speed_pid,
    MOTOR_SPEED_KP,
    MOTOR_SPEED_KI,
    MOTOR_SPEED_KD,
    MOTOR_PID_DT,
    MOTOR_INTEGRAL_LIMIT,
    MOTOR_OUTPUT_LIMIT);
  Motor_Init(MOTOR_OFFLINE_MS);
  // 在循环外记录起点 后面每次按10ms周期唤醒
  TickType_t last_tick = xTaskGetTickCount();
  for (;;)
  {
    // 读一次共享目标 本轮判断和计算使用同一个值
    float speed = target_speed;

    // 离线或目标为0时 清除积分并发送零控制量
    if (Motor_Is_Offline() == 1 || speed == 0.0f)
    {
      PID_Reset(&speed_pid);
      motor_value = 0;
    }
    else
    {
      motor_value = (int16_t)PID_Calc(
        &speed_pid, speed, Motor_Get_Speed());
    }

    send_result = Motor_Send(motor_value);
    vTaskDelayUntil(&last_tick, pdMS_TO_TICKS(MOTOR_PERIOD_MS));
  }
}

void App_UART_Task(void)
{
  // 记录上次发送的时间
  uint32_t send_tick = HAL_GetTick();

  // 保存准备发送的文字及其长度
  char text[32];
  int length;

  BSP_UART_Start_Receive(rx_buffer, UART_RX_SIZE);

  for (;;)
  {
    // 收到数据后才处理
    if (rx_flag == 1)
    {
      // 检查命令 合法时修改目标转速
      Motor_Parse_Command();

      // 这批数据已经处理完
      rx_length = 0;
      rx_flag = 0;

      // 开始接收下一批数据
      BSP_UART_Start_Receive(rx_buffer, UART_RX_SIZE);
    }
    // 距离上次发送达到2秒 就反馈实际转速
    if (HAL_GetTick() - send_tick >= UART_REPORT_PERIOD_MS)
    {
      send_tick = HAL_GetTick();

      length = sprintf(text, "speed: %d rpm\r\n", (int)Motor_Get_Speed());

      BSP_UART_Send((uint8_t *)text, length);
    }

    osDelay(1);
  }
}

void App_Servo_Task(void)
{
  // 保存舵机目标角度 初始为0度
  uint16_t angle = 0;

  // TIM1每计数一次是1us 标称0到180度映射为500到2500us
  // 先设置初始脉宽
  BSP_Servo_Set_Pulse(SERVO_MIN_PULSE_US + angle * SERVO_PULSE_RANGE_US / SERVO_MAX_DEG);
  BSP_Servo_Start();

  for (;;)
  {
    // 消抖组件只报告按下事件 舵机动作由任务决定
    if (Key_Update(HAL_GetTick()) == 1)
    {
      // 每次增加20度 到180度后保持
      if (angle < SERVO_MAX_DEG)
      {
        angle += SERVO_STEP_DEG;
      }

      BSP_Servo_Set_Pulse(
          SERVO_MIN_PULSE_US + angle * SERVO_PULSE_RANGE_US / SERVO_MAX_DEG);
    }

    osDelay(10);
  }
}

void App_LED_Task(void)
{
  // phase是呼吸周期中的位置 brightness是PWM比较值
  uint32_t phase;
  uint16_t brightness;

  // 红灯PH12使用TIM5通道3 PWM只需启动一次
  BSP_Red_Start();

  for (;;)
  {
    if (Motor_Is_Offline() == 1)
    {
      // 没有有效电机反馈时 关闭绿灯
      BSP_Green_Set(0);

      // 用余数得到当前处于呼吸周期的哪个位置
      phase = HAL_GetTick() % LED_BREATH_PERIOD_MS;
      if (phase < LED_BREATH_PERIOD_MS / 2)
      {
        // 前半周期逐渐变亮 比较值换算到PWM范围
        brightness = phase * BSP_LED_PWM_MAX / (LED_BREATH_PERIOD_MS / 2);
      }
      else
      {
        // 后半周期逐渐变暗
        brightness = (LED_BREATH_PERIOD_MS - phase) * BSP_LED_PWM_MAX
        / (LED_BREATH_PERIOD_MS / 2);
      }

      BSP_Red_Set(brightness);
    }
    else
    {
      // 电机在线 红灯关闭 绿灯常亮
      BSP_Red_Set(0);
      BSP_Green_Set(1);
    }

    // 每隔约10ms更新亮度 等待时其他任务可以运行
    osDelay(10);
  }
}

/// @brief 将接收长度和就绪标志交给串口任务
/// @param length 本次接收的字节数
/// @note 在中断中执行 处理完成前不重新启动接收
void BSP_UART_Rx_Callback(uint16_t length)
{
  // 先记录长度 再标记有数据需要处理
  rx_length = length;
  rx_flag = 1;
}

/// @brief 检查一批串口数据 合法时更新目标转速
/// @note 由串口任务调用 错误命令保留原目标
/// @note 每批仅支持一条完整命令 例如motor+100或motor-100
/// @note 不拼接分批命令 也不拆分连续的多条命令
static void Motor_Parse_Command(void)
{
  uint16_t length = rx_length;
  int32_t speed = 0;

  // 长度不能超过接收数组 接满时整批丢弃 避免执行截断命令
  if (length == 0 || length >= UART_RX_SIZE)
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
