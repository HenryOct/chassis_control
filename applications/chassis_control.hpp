#ifndef CHASSIS_CONTROL_HPP
#define CHASSIS_CONTROL_HPP

#include "uart/uart.hpp"
#include "tools/mecanum/mecanum.hpp"
#include "tools/pid/pid.hpp"
#include "motor/rm_motor/rm_motor.hpp"

// 底盘电机实例化
inline sp::RM_Motor chassis_rf(1, sp::RM_Motors::M3508);
inline sp::RM_Motor chassis_lf(2, sp::RM_Motors::M3508); 
inline sp::RM_Motor chassis_lr(3, sp::RM_Motors::M3508);
inline sp::RM_Motor chassis_rr(4, sp::RM_Motors::M3508);

// 定义一个麦轮底盘
// 参数：轮子半径0.076m，前后轮距一半0.15m，左右轮距一半0.15m
inline sp::Mecanum mecanum_chassis(0.076f, 0.15f, 0.15f);

// PID参数定义 (极保守设置，确保稳定)
constexpr float PID_DT = 0.001f;    // 1kHz控制频率
constexpr float PID_KP = 3.0f;      // 进一步降低比例增益
constexpr float PID_KI = 0.1f;      // 极小的积分增益
constexpr float PID_KD = 0.0f;      // 暂时关闭微分项
constexpr float PID_MO = 2.0f;      // 进一步降低最大输出
constexpr float PID_MIO = 0.5f;     // 极小的积分限制
constexpr float PID_ALPHA = 0.5f;   // 更强的D项滤波

//// PID控制器 - 每个轮子一个速度环PID
//                                    dt     kp    ki    kd    mo   mio   alpha
inline sp::PID chassis_lf_pid(PID_DT, PID_KP, PID_KI, PID_KD, PID_MO, PID_MIO, PID_ALPHA);
inline sp::PID chassis_lr_pid(PID_DT, PID_KP, PID_KI, PID_KD, PID_MO, PID_MIO, PID_ALPHA);
inline sp::PID chassis_rf_pid(PID_DT, PID_KP, PID_KI, PID_KD, PID_MO, PID_MIO, PID_ALPHA);
inline sp::PID chassis_rr_pid(PID_DT, PID_KP, PID_KI, PID_KD, PID_MO, PID_MIO, PID_ALPHA);

// 底盘控制结构体
struct ChassisData
{
    float vx_set;      // 前后移动速度设定值 m/s
    float vy_set;      // 左右移动速度设定值 m/s  
    float wz_set;      // 旋转角速度设定值 rad/s
    
    float speed_lf_set; // 左前轮速度设定值 rad/s
    float speed_lr_set; // 左后轮速度设定值 rad/s
    float speed_rf_set; // 右前轮速度设定值 rad/s
    float speed_rr_set; // 右后轮速度设定值 rad/s
    
    float torque_lf;   // 左前轮输出力矩 N·m
    float torque_lr;   // 左后轮输出力矩 N·m
    float torque_rf;   // 右前轮输出力矩 N·m
    float torque_rr;   // 右后轮输出力矩 N·m
};

#endif // CHASSIS_CONTROL_HPP
