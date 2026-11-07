#ifndef PID_H
#define PID_H

/// @brief 单个PID控制器的参数和计算记录
typedef struct
{
    float kp;             ///< 比例系数
    float ki;             ///< 积分系数
    float kd;             ///< 微分系数

    float dt;             ///< PID计算间隔 单位s
    float integral;       ///< 误差随时间累计的积分值
    float last_error;     ///< 上一次计算的误差

    float integral_limit; ///< 积分累计值的限制
    float output_limit;   ///< 最终输出的限制
} PID;

/// @brief 保存PID参数并清除之前的计算记录
/// @param pid 要初始化的PID控制器 不能为NULL
/// @param kp 比例系数
/// @param ki 积分系数
/// @param kd 微分系数
/// @param dt 计算间隔 单位s 必须大于0 并与实际控制周期匹配
/// @param integral_limit 积分累计值的限制 必须大于等于0
/// @param output_limit 最终输出的限制 必须大于等于0
void PID_Init(PID *pid, float kp, float ki, float kd,
              float dt, float integral_limit, float output_limit);

/// @brief 清除累计积分和上一次误差 保留PID参数
/// @param pid 要重置的PID控制器 不能为NULL
void PID_Reset(PID *pid);

/// @brief 根据目标值和反馈值计算一次PID输出
/// @param pid 要使用的PID控制器 不能为NULL
/// @param target 目标值 与feedback使用相同单位
/// @param feedback 实际反馈值
/// @return 经过输出限幅的控制量
/// @note 同一个PID控制器由同一个任务负责计算和重置
float PID_Calc(PID *pid, float target, float feedback);

#endif
