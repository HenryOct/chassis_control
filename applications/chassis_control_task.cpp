#include "cmsis_os.h"
#include "chassis_control.hpp"
#include "buzzer_control.hpp"
#include <cmath>
#include <cstdlib>
#include <algorithm>  


// 右拨杆状态监控变量
static sp::DBusSwitchMode last_sw_r = sp::DBusSwitchMode::MID;  // 上次右拨杆状态

// 功率控制相关常量
constexpr uint16_t DEFAULT_POWER_LIMIT = 60;   // 默认功率限制 60W
constexpr float POWER_SCALE_MIN = 0.1f;        // 最小功率缩放因子（安全下限）

// 功率模型参数在 chassis_control.hpp 中定义

// 更新功率数据从裁判系统和超级电容
void update_power_data()
{
    // 从PM02裁判系统获取功率限制
    chassis_data.chassis_power_limit = pm02.robot_status.chassis_power_limit;
    
    // 如果裁判系统离线，使用默认值
    if (chassis_data.chassis_power_limit == 0) {
        chassis_data.chassis_power_limit = DEFAULT_POWER_LIMIT;
    }
    
    // 从超级电容获取实际功率数据
    chassis_data.power_in = super_cap.power_in;           // 电池输入功率
    chassis_data.power_out = super_cap.power_out;         // 电容输出功率
    chassis_data.chassis_actual_power = chassis_data.power_out - chassis_data.power_in;  // 底盘实际功率
    chassis_data.predicted_power = predict_power_consumption();  // 预测功率
}

// 根据功率模型预测输入功率
float predict_power_consumption()
{
    // P_in = Σ(τ·ω) + K₁·Σ(τ²) + K₂·Σ(ω²) + K₃
    
    // 获取各电机的转矩和角速度
    float torque_lf = chassis_data.torque_lf;
    float torque_lr = chassis_data.torque_lr; 
    float torque_rf = chassis_data.torque_rf;
    float torque_rr = chassis_data.torque_rr;
    
    float speed_lf = chassis_lf.speed;
    float speed_lr = chassis_lr.speed;
    float speed_rf = chassis_rf.speed;
    float speed_rr = chassis_rr.speed;
    
    // 计算各项功率分量
    // 1. 轴功率项：Σ(τ·ω)
    float shaft_power = torque_lf * speed_lf + torque_lr * speed_lr + 
                       torque_rf * speed_rf + torque_rr * speed_rr;
    
    // 2. 转矩损耗项：K₁·Σ(τ²)
    float torque_loss = K1_TORQUE_LOSS * (torque_lf * torque_lf + torque_lr * torque_lr +
                                         torque_rf * torque_rf + torque_rr * torque_rr);
    
    // 3. 角速度损耗项：K₂·Σ(ω²)
    float speed_loss = K2_SPEED_LOSS * (speed_lf * speed_lf + speed_lr * speed_lr +
                                       speed_rf * speed_rf + speed_rr * speed_rr);
    
    // 4. 静态功耗项：K₃
    float static_power = K3_STATIC_POWER;
    
    // 总预测功率
    float predicted_power = std::abs(shaft_power) + torque_loss + speed_loss + static_power;
    
    return predicted_power;
}

// 计算转矩缩放系数（基于二次方程求解）
float calculate_torque_scale_factor()
{
    float power_limit = static_cast<float>(chassis_data.chassis_power_limit);
    
    // 使用超级电容的实际功率进行判断
    float current_power_in = chassis_data.power_in;
    
    // 如果实际功率不超限，不需要缩放
    if (current_power_in <= power_limit) {
        return 1.0f;
    }
    
    // 计算二次方程的系数
    // 方程：K₁·K²·Σ(τ²) + K·Σ(τ·ω) + (K₂·Σ(ω²) + K₃ - P_max) = 0
    
    // 获取各电机数据
    float torque_lf = chassis_data.torque_lf;
    float torque_lr = chassis_data.torque_lr; 
    float torque_rf = chassis_data.torque_rf;
    float torque_rr = chassis_data.torque_rr;
    
    float speed_lf = chassis_lf.speed;
    float speed_lr = chassis_lr.speed;
    float speed_rf = chassis_rf.speed;
    float speed_rr = chassis_rr.speed;
    
    // 计算求和项
    float sum_tau_omega = torque_lf * speed_lf + torque_lr * speed_lr + 
                         torque_rf * speed_rf + torque_rr * speed_rr;
    
    float sum_tau_squared = torque_lf * torque_lf + torque_lr * torque_lr +
                           torque_rf * torque_rf + torque_rr * torque_rr;
    
    float sum_omega_squared = speed_lf * speed_lf + speed_lr * speed_lr +
                             speed_rf * speed_rf + speed_rr * speed_rr;
    
    // 二次方程系数
    float a = K1_TORQUE_LOSS * sum_tau_squared;
    float b = sum_tau_omega;
    float c = K2_SPEED_LOSS * sum_omega_squared + K3_STATIC_POWER - power_limit;
    
    // 计算判别式
    float discriminant = b * b - 4 * a * c;
    
    // 判别式小于0，无实数解
    if (discriminant < 0) {
        return 1.0f;  // 使用最小缩放作为安全值
    }
    
    // 求解二次方程（取较小的正根）
    float sqrt_discriminant = std::sqrt(discriminant);
    float k = (-b + sqrt_discriminant) / (2 * a);
    return k;
}

// 应用功率限制（严格按照PDF逻辑）
void apply_power_limit()
{
    // 计算转矩缩放系数
    chassis_data.power_scale_factor = calculate_torque_scale_factor();
    
    // 判断是否需要激活功率限制
    chassis_data.power_limit_active = (chassis_data.power_scale_factor < 1.0f);
    
    // 按照PDF：τ' = K × τ，对所有电机扭矩进行缩放
    chassis_data.torque_lf *= chassis_data.power_scale_factor;
    chassis_data.torque_lr *= chassis_data.power_scale_factor;
    chassis_data.torque_rf *= chassis_data.power_scale_factor;
    chassis_data.torque_rr *= chassis_data.power_scale_factor;
    
    // 额外安全限幅 - 保护机械结构
    constexpr float MAX_SAFE_TORQUE = 8.0f;
    chassis_data.torque_lf = std::max(std::min(chassis_data.torque_lf, MAX_SAFE_TORQUE), -MAX_SAFE_TORQUE);
    chassis_data.torque_lr = std::max(std::min(chassis_data.torque_lr, MAX_SAFE_TORQUE), -MAX_SAFE_TORQUE);
    chassis_data.torque_rf = std::max(std::min(chassis_data.torque_rf, MAX_SAFE_TORQUE), -MAX_SAFE_TORQUE);
    chassis_data.torque_rr = std::max(std::min(chassis_data.torque_rr, MAX_SAFE_TORQUE), -MAX_SAFE_TORQUE);
}

// 禁用所有电机，保证底盘和摩擦轮都不会移动
void disable_all_motors()
{
    // 清零底盘数据
    chassis_data.torque_lf = 0.0f;
    chassis_data.torque_lr = 0.0f;
    chassis_data.torque_rf = 0.0f;
    chassis_data.torque_rr = 0.0f;
    
    // 停止底盘电机（CAN发送在can_task中处理）
    chassis_lf.cmd(0.0f);
    chassis_lr.cmd(0.0f);
    chassis_rf.cmd(0.0f);
    chassis_rr.cmd(0.0f);
    
    // 注意：PID类没有reset方法，积分项会在下次calc时自然清零
}

// 底盘移动控制
void chassis_move_control(float vx, float vy, float wz)
{
    // 保存目标速度到底盘数据
    chassis_data.vx_set = vx;
    chassis_data.vy_set = vy;
    chassis_data.wz_set = wz;
    
    // 使用麦轮运动学计算各轮速度
    mecanum_chassis.calc(vx, vy, wz);
    
    // 设置各轮目标速度
    chassis_data.speed_lf_set = mecanum_chassis.speed_lf;
    chassis_data.speed_lr_set = mecanum_chassis.speed_lr;
    chassis_data.speed_rf_set = mecanum_chassis.speed_rf;
    chassis_data.speed_rr_set = mecanum_chassis.speed_rr;
    
    // 死区控制：当目标速度很小时，直接设为0，避免抖动
    constexpr float SPEED_DEADZONE = 0.1f;  // 速度死区 rad/s
    
    float target_lf = (std::abs(chassis_data.speed_lf_set) < SPEED_DEADZONE) ? 0.0f : chassis_data.speed_lf_set;
    float target_lr = (std::abs(chassis_data.speed_lr_set) < SPEED_DEADZONE) ? 0.0f : chassis_data.speed_lr_set;
    float target_rf = (std::abs(chassis_data.speed_rf_set) < SPEED_DEADZONE) ? 0.0f : chassis_data.speed_rf_set;
    float target_rr = (std::abs(chassis_data.speed_rr_set) < SPEED_DEADZONE) ? 0.0f : chassis_data.speed_rr_set;
    
    // 完整的PID速度闭环控制
    chassis_lf_pid.calc(target_lf, chassis_lf.speed);
    chassis_lr_pid.calc(target_lr, chassis_lr.speed);
    chassis_rf_pid.calc(target_rf, chassis_rf.speed);
    chassis_rr_pid.calc(target_rr, chassis_rr.speed);
    
    // 使用PID输出值
    chassis_data.torque_lf = chassis_lf_pid.out;
    chassis_data.torque_lr = chassis_lr_pid.out;
    chassis_data.torque_rf = chassis_rf_pid.out;
    chassis_data.torque_rr = chassis_rr_pid.out;
    
    // 输出死区：当转矩很小时，直接设为0
    constexpr float TORQUE_DEADZONE = 0.2f;  // 转矩死区 N·m
    if (std::abs(chassis_data.torque_lf) < TORQUE_DEADZONE) chassis_data.torque_lf = 0.0f;
    if (std::abs(chassis_data.torque_lr) < TORQUE_DEADZONE) chassis_data.torque_lr = 0.0f;
    if (std::abs(chassis_data.torque_rf) < TORQUE_DEADZONE) chassis_data.torque_rf = 0.0f;
    if (std::abs(chassis_data.torque_rr) < TORQUE_DEADZONE) chassis_data.torque_rr = 0.0f;
    
    // 应用功率限制
    //apply_power_limit();
    
    // 写入控制指令到电机对象（CAN发送在can_task中处理）
    chassis_lf.cmd(chassis_data.torque_lf);
    chassis_lr.cmd(chassis_data.torque_lr);
    chassis_rf.cmd(chassis_data.torque_rf);
    chassis_rr.cmd(chassis_data.torque_rr);
}

extern "C" void chassis_control_task()
{
    // 初始化底盘数据
    chassis_data.vx_set = 0.0f;
    chassis_data.vy_set = 0.0f;
    chassis_data.wz_set = 0.0f;
    
    // 初始化功率控制数据
    chassis_data.chassis_power_limit = DEFAULT_POWER_LIMIT;
    chassis_data.power_scale_factor = 1.0f;
    chassis_data.power_limit_active = false;
    
    // 初始化超级电容功率数据
    chassis_data.power_in = 0.0f;
    chassis_data.power_out = 0.0f;
    chassis_data.chassis_actual_power = 0.0f;
    chassis_data.predicted_power = 0.0f;

    while (true) {
        // 更新功率数据（从裁判系统获取最新数据）
        //update_power_data();
        
        // 循环检查遥控器是否在线
        if (!remote.is_alive(HAL_GetTick())) 
        {
            // 遥控器离线，禁用所有电机
            disable_all_motors();
            osDelay(10);
            continue;
        }
        
        // 检测右拨杆状态变化并播放音效
        if (remote.sw_r != last_sw_r) 
        {
            if (remote.sw_r == sp::DBusSwitchMode::MID && last_sw_r == sp::DBusSwitchMode::DOWN) {
                request_sound_effect(SoundEffect::SWITCH_UP);
            }
            else if (remote.sw_r == sp::DBusSwitchMode::DOWN && last_sw_r == sp::DBusSwitchMode::MID) {
                request_sound_effect(SoundEffect::SWITCH_DOWN);
            }
            last_sw_r = remote.sw_r;  // 更新右拨杆状态
        }
        
        // 右拨杆中档：底盘控制模式  
        if (remote.sw_r == sp::DBusSwitchMode::MID) 
        {
            
            // 完整的PID控制底盘
            const float MAX_LINEAR_SPEED = 2.0f;   // 最大线速度 m/s，前后左右各2m/s
            const float ROTATION_SPEED = 5.0f;     // 固定转向速度 2rad/s
            
            // 直接获取摇杆输入值
            float raw_vx = remote.ch_lv;
            float raw_vy = remote.ch_lh;
            float raw_right_v = remote.ch_rv;
            
            // 遥控器输入死区，避免微小输入导致抖动
            constexpr float RC_DEADZONE = 0.05f;  // 遥控器死区 (5%)
            if (std::abs(raw_vx) < RC_DEADZONE) raw_vx = 0.0f;
            if (std::abs(raw_vy) < RC_DEADZONE) raw_vy = 0.0f;
            if (std::abs(raw_right_v) < RC_DEADZONE) raw_right_v = 0.0f;
            
            // 直接线性映射
            float vx = raw_vx * MAX_LINEAR_SPEED;   // 前后移动
            float vy = raw_vy * MAX_LINEAR_SPEED;   // 左右移动
            
            // 获取右摇杆左右值
            float raw_right_h = remote.ch_rh;
            
            // 右摇杆也应用死区
            if (std::abs(raw_right_h) < RC_DEADZONE) raw_right_h = 0.0f;
            
            // 计算旋转速度：右摇杆前后控制向左转，左右控制向右转
            float wz = 0.0f;
            if (std::abs(raw_right_v) > 0.0f) {
                wz = raw_right_v * ROTATION_SPEED;   // 右摇杆前后：向左转
            }
            else if (std::abs(raw_right_h) > 0.0f) {
                wz = -raw_right_h * ROTATION_SPEED;  // 右摇杆左右：向右转
            }
            
            // 使用PID控制底盘移动
            chassis_move_control(vx, vy, wz);
        }
        // 右拨杆下档时全部电机失能
        else if (remote.sw_r == sp::DBusSwitchMode::DOWN) 
        {
            disable_all_motors();
        }
        // bó gan bu zai zhi禁用所有电机
        else 
        {
            disable_all_motors();
        }
        
        osDelay(1);  //1000Hz控制频率
    }
}
