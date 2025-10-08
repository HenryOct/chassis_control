#ifndef CHASSIS_CONTROL_HPP
#define CHASSIS_CONTROL_HPP

// 必要的包含
#include "io/dbus/dbus.hpp"
#include "io/can/can.hpp"
#include "tools/mecanum/mecanum.hpp"
#include "tools/pid/pid.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "referee/pm02/pm02.hpp"
#include "motor/super_cap/super_cap.hpp"

// 任务函数声明
extern "C" void can_task(void const * argument);
extern "C" void uart_task(void const * argument);

// 底盘电机实例化（RM3508电机，减速比14.9，配合C620电调）
inline sp::RM_Motor chassis_rf(1, sp::RM_Motors::M3508, 14.9f);
inline sp::RM_Motor chassis_lf(2, sp::RM_Motors::M3508, 14.9f); 
inline sp::RM_Motor chassis_lr(3, sp::RM_Motors::M3508, 14.9f);
inline sp::RM_Motor chassis_rr(4, sp::RM_Motors::M3508, 14.9f);

// 定义一个麦轮底盘
// 参数：轮子半径77mm(直径154mm)，纵向间距330mm(半距165mm)，横向间距370mm(半距185mm)
inline sp::Mecanum mecanum_chassis(0.077f, 0.165f, 0.185f);

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
    
    // 功率控制相关数据
    uint16_t chassis_power_limit;  // 底盘功率限制 W
    float power_scale_factor;      // 功率缩放因子 (0.0-1.0)
    bool power_limit_active;       // 功率限制是否激活
    
    // 超级电容功率数据
    float power_in;                // 电池输入功率 W (需要限制的)
    float power_out;               // 电容输出功率 W
    float chassis_actual_power;    // 底盘实际功率 W (power_out - power_in)
    float predicted_power;         // 预测输入功率 W (基于功率模型)
};

// 外部声明，在对应任务中实例化
extern sp::DBus remote;     // uart_task.cpp中实例化
extern sp::PM02 pm02;       // uart_task.cpp中实例化
extern sp::CAN can2;        // can_task.cpp中实例化

// 底盘数据实例
extern ChassisData chassis_data;

// 超级电容实例化 (自动模式)
inline sp::SuperCap super_cap(sp::SuperCapMode::AUTOMODE);

// 当前电容工作模式 (由左拨杆控制)
extern sp::SuperCapMode current_supercap_mode;

// PID参数定义 (简化版本，移除复杂滤波)
constexpr float PID_DT = 0.001f;    // 1000Hz控制频率
constexpr float PID_KP = 0.5f;      // 比例增益
constexpr float PID_KI = 0.05f;      // 积分增益
constexpr float PID_KD = 0.01f;      // 微分增益
constexpr float PID_MO = 2.5f;     // 最大输出限制 (N·m) - 保护机械结构
constexpr float PID_MIO = 1.0f;     // 积分输出限制 (N·m)
constexpr float PID_ALPHA = 0.0f;   // D项滤波系数 (不使用滤波)

// 功率模型参数（需要根据实际测试调整）
constexpr float K1_TORQUE_LOSS = 2.0f;        // 转矩损耗系数
constexpr float K2_SPEED_LOSS = 0.005f;        // 角速度损耗系数  
constexpr float K3_STATIC_POWER = 6.2f;       // 静态待机功耗 W
constexpr float K4_TORQUE_RATE = 0.007f;         // 转矩变化率系数
constexpr float K5_SPEED_RATE = 0.0f;         // 速度变化率系数

// PID控制器 - 每个轮子一个速度环PID
//                                    dt     kp    ki    kd    mo   mio   alpha
inline sp::PID chassis_lf_pid(PID_DT, PID_KP, PID_KI, PID_KD, PID_MO, PID_MIO, PID_ALPHA);
inline sp::PID chassis_lr_pid(PID_DT, PID_KP, PID_KI, PID_KD, PID_MO, PID_MIO, PID_ALPHA);
inline sp::PID chassis_rf_pid(PID_DT, PID_KP, PID_KI, PID_KD, PID_MO, PID_MIO, PID_ALPHA);
inline sp::PID chassis_rr_pid(PID_DT, PID_KP, PID_KI, PID_KD, PID_MO, PID_MIO, PID_ALPHA);

// 功率控制函数声明
void update_power_data();
void apply_power_limit();
float calculate_torque_scale_factor();
float predict_power_consumption();

#endif // CHASSIS_CONTROL_HPP
