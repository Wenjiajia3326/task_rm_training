#include "pid.h"

void PID_Init(PID *pid, float kp, float ki, float kd,float dt, float integral_limit, float output_limit)//初始设置
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->dt = dt;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;

    PID_Reset(pid);
}
void PID_Reset(PID *pid)//重置积分和误差
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

    // 计算pid误差变化速度
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