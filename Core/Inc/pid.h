#ifndef PID_H
#define PID_H

typedef struct
{
    float kp;
    float ki;
    float kd;

    float dt;
    float integral;
    float last_error;

    float integral_limit;
    float output_limit;
} PID;

void PID_Init(PID *pid, float kp, float ki, float kd,
              float dt, float integral_limit, float output_limit);
void PID_Reset(PID *pid);
float PID_Calc(PID *pid, float target, float feedback);

#endif