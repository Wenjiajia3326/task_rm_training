#include "pid.h"

// 保存参数后清除上一轮控制的积分和误差
void PID_Init(PID *pid, float kp, float ki, float kd,
              float dt, float integral_limit, float output_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->dt = dt;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;

    PID_Reset(pid);
}

void PID_Reset(PID *pid)
{
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
}

float PID_Calc(PID *pid, float target, float feedback)
{
    float error = target - feedback;

    // 累积误差积分
    pid->integral += error * pid->dt;

    if (pid->integral > pid->integral_limit)
    {
        pid->integral = pid->integral_limit;
    }
    else if (pid->integral < -pid->integral_limit)
    {
        pid->integral = -pid->integral_limit;
    }

    // 根据前后两次误差计算变化速度
    float derivative = (error - pid->last_error) / pid->dt;

    float output = pid->kp * error
                 + pid->ki * pid->integral
                 + pid->kd * derivative;

    // 限制最终输出
    if (output > pid->output_limit)
    {
        output = pid->output_limit;
    }
    else if (output < -pid->output_limit)
    {
        output = -pid->output_limit;
    }

    pid->last_error = error;

    return output;
}
