#include "cmsis_os.h"
#include "chassis_control.hpp"
#include "buzzer_control.hpp"
#include <cmath>
#include <cstdlib>
#include <algorithm>  

// 参数常量
constexpr uint16_t DEFAULT_POWER_LIMIT = 100;
constexpr float POWER_SCALE_MIN = 0.1f;
constexpr float MAX_LINEAR_SPEED = 2.0f;
constexpr float ROTATION_SPEED = 5.0f;
constexpr float MAX_SAFE_TORQUE = 8.0f;
constexpr uint32_t CONTROL_PERIOD_MS = 1;
constexpr uint32_t OFFLINE_DELAY_MS = 10;

static sp::DBusSwitchMode last_sw_r = sp::DBusSwitchMode::MID;

// 更新功率数据，从裁判系统和超级电容获取最新数据
void update_power_data()
{
    chassis_data.chassis_power_limit = pm02.robot_status.chassis_power_limit;
    
    if (chassis_data.chassis_power_limit == 0) 
    {
        chassis_data.chassis_power_limit = DEFAULT_POWER_LIMIT;
    }
    
    chassis_data.power_in = super_cap.power_in;
    chassis_data.power_out = super_cap.power_out;
    chassis_data.chassis_actual_power = chassis_data.power_out - chassis_data.power_in;
    chassis_data.predicted_power = predict_power_consumption();
}

// 根据功率模型预测输入功率消耗
float predict_power_consumption()
{
    float torque_lf = chassis_data.torque_lf;
    float torque_lr = chassis_data.torque_lr; 
    float torque_rf = chassis_data.torque_rf;
    float torque_rr = chassis_data.torque_rr;
    
    float speed_lf = chassis_lf.speed;
    float speed_lr = chassis_lr.speed;
    float speed_rf = chassis_rf.speed;
    float speed_rr = chassis_rr.speed;
    
    float shaft_power = torque_lf * speed_lf + torque_lr * speed_lr + 
                       torque_rf * speed_rf + torque_rr * speed_rr;
    
    float torque_loss = K1_TORQUE_LOSS * (torque_lf * torque_lf + torque_lr * torque_lr +
                                         torque_rf * torque_rf + torque_rr * torque_rr);
    
    float speed_loss = K2_SPEED_LOSS * (speed_lf * speed_lf + speed_lr * speed_lr +
                                       speed_rf * speed_rf + speed_rr * speed_rr);
    
    float static_power = K3_STATIC_POWER;
    
    float predicted_power = shaft_power + torque_loss + speed_loss + static_power;
    
    return predicted_power;
}

// 计算转矩缩放系数，基于二次方程求解最优缩放因子
float calculate_torque_scale_factor()
{
    float power_limit = static_cast<float>(chassis_data.chassis_power_limit);
    float current_power_in = chassis_data.power_in;
    
    if (current_power_in <= power_limit) {
        return 1.0f;
    }
    
    // 获取各电机转矩和速度数据
    float torque_lf = chassis_data.torque_lf;
    float torque_lr = chassis_data.torque_lr; 
    float torque_rf = chassis_data.torque_rf;
    float torque_rr = chassis_data.torque_rr;
    
    float speed_lf = chassis_lf.speed;
    float speed_lr = chassis_lr.speed;
    float speed_rf = chassis_rf.speed;
    float speed_rr = chassis_rr.speed;
    
    float sum_tau_omega = torque_lf * speed_lf + torque_lr * speed_lr + 
                         torque_rf * speed_rf + torque_rr * speed_rr;
    
    float sum_tau_squared = torque_lf * torque_lf + torque_lr * torque_lr +
                           torque_rf * torque_rf + torque_rr * torque_rr;
    
    float sum_omega_squared = speed_lf * speed_lf + speed_lr * speed_lr +
                             speed_rf * speed_rf + speed_rr * speed_rr;
    
    float a = K1_TORQUE_LOSS * sum_tau_squared;
    float b = sum_tau_omega;
    float c = K2_SPEED_LOSS * sum_omega_squared + K3_STATIC_POWER - power_limit;
    
    float discriminant = b * b - 4 * a * c;
    
    if (discriminant < 0) {
        return POWER_SCALE_MIN;
    }
    
    float sqrt_discriminant = std::sqrt(discriminant);
    float k = (-b + sqrt_discriminant) / (2 * a);
    return k;
}

// 应用功率限制，对所有电机转矩进行缩放
void apply_power_limit()
{
    chassis_data.power_scale_factor = calculate_torque_scale_factor();
    chassis_data.power_limit_active = (chassis_data.power_scale_factor < 1.0f);
    
    chassis_data.torque_lf *= chassis_data.power_scale_factor;
    chassis_data.torque_lr *= chassis_data.power_scale_factor;
    chassis_data.torque_rf *= chassis_data.power_scale_factor;
    chassis_data.torque_rr *= chassis_data.power_scale_factor;
    
    chassis_data.torque_lf = std::max(std::min(chassis_data.torque_lf, MAX_SAFE_TORQUE), -MAX_SAFE_TORQUE);
    chassis_data.torque_lr = std::max(std::min(chassis_data.torque_lr, MAX_SAFE_TORQUE), -MAX_SAFE_TORQUE);
    chassis_data.torque_rf = std::max(std::min(chassis_data.torque_rf, MAX_SAFE_TORQUE), -MAX_SAFE_TORQUE);
    chassis_data.torque_rr = std::max(std::min(chassis_data.torque_rr, MAX_SAFE_TORQUE), -MAX_SAFE_TORQUE);
}

// 禁用所有电机，清零转矩并停止电机
void disable_all_motors()
{
    chassis_data.torque_lf = 0.0f;
    chassis_data.torque_lr = 0.0f;
    chassis_data.torque_rf = 0.0f;
    chassis_data.torque_rr = 0.0f;
    
    chassis_lf.cmd(0.0f);
    chassis_lr.cmd(0.0f);
    chassis_rf.cmd(0.0f);
    chassis_rr.cmd(0.0f);
}

// 底盘移动控制，将期望速度转换为电机转矩输出
void chassis_move_control(float vx, float vy, float wz)
{
    chassis_data.vx_set = vx;
    chassis_data.vy_set = vy;
    chassis_data.wz_set = wz;
    
    mecanum_chassis.calc(vx, vy, wz);
    
    // 设置各轮目标速度
    chassis_data.speed_lf_set = mecanum_chassis.speed_lf;
    chassis_data.speed_lr_set = mecanum_chassis.speed_lr;
    chassis_data.speed_rf_set = mecanum_chassis.speed_rf;
    chassis_data.speed_rr_set = mecanum_chassis.speed_rr;
    
    chassis_lf_pid.calc(chassis_data.speed_lf_set, chassis_lf.speed);
    chassis_lr_pid.calc(chassis_data.speed_lr_set, chassis_lr.speed);
    chassis_rf_pid.calc(chassis_data.speed_rf_set, chassis_rf.speed);
    chassis_rr_pid.calc(chassis_data.speed_rr_set, chassis_rr.speed);
    
    // 获取PID输出转矩
    chassis_data.torque_lf = chassis_lf_pid.out;
    chassis_data.torque_lr = chassis_lr_pid.out;
    chassis_data.torque_rf = chassis_rf_pid.out;
    chassis_data.torque_rr = chassis_rr_pid.out;
    
    chassis_lf.cmd(chassis_data.torque_lf);
    chassis_lr.cmd(chassis_data.torque_lr);
    chassis_rf.cmd(chassis_data.torque_rf);
    chassis_rr.cmd(chassis_data.torque_rr);
}

// 主控制任务，处理遥控器输入和底盘控制
extern "C" void chassis_control_task()
{
    chassis_data.chassis_power_limit = DEFAULT_POWER_LIMIT;

    while (true) {
        update_power_data();
        
        if (!remote.is_alive(HAL_GetTick())) {
            disable_all_motors();
            osDelay(OFFLINE_DELAY_MS);
            continue;
        }
        
        if (remote.sw_r != last_sw_r) {
            if (remote.sw_r == sp::DBusSwitchMode::MID && last_sw_r == sp::DBusSwitchMode::DOWN) {
                request_sound_effect(SoundEffect::SWITCH_UP);
            }
            else if (remote.sw_r == sp::DBusSwitchMode::DOWN && last_sw_r == sp::DBusSwitchMode::MID) {
                request_sound_effect(SoundEffect::SWITCH_DOWN);
            }
            last_sw_r = remote.sw_r;
        }
        
        if (remote.sw_r == sp::DBusSwitchMode::MID) {
            // 获取摇杆输入值
            float raw_vx = remote.ch_lv;
            float raw_vy = remote.ch_lh;
            float raw_right_v = remote.ch_rv;
            
            // 转换为底盘速度
            float vx = raw_vx * MAX_LINEAR_SPEED;
            float vy = (-raw_vy) * MAX_LINEAR_SPEED;
            
            float raw_right_h = remote.ch_rh;
            
            // 计算旋转速度
            float wz = 0.0f;
            if (std::abs(raw_right_v) > 0.0f) {
                wz = raw_right_v * ROTATION_SPEED;
            }
            else if (std::abs(raw_right_h) > 0.0f) {
                wz = -raw_right_h * ROTATION_SPEED;
            }
            
            chassis_move_control(vx, vy, wz);
        }
        else if (remote.sw_r == sp::DBusSwitchMode::DOWN) {
            disable_all_motors();
        }
        else {
            disable_all_motors();
        }
        
        osDelay(CONTROL_PERIOD_MS);
    }
}
