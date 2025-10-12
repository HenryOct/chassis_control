#include "cmsis_os.h"
#include "chassis_control.hpp"
#include "buzzer_control.hpp"
#include <cmath>
#include <cstdlib>
#include <algorithm>  

// 参数常量
constexpr uint16_t DEFAULT_POWER_LIMIT = 80;
constexpr float POWER_SCALE_MIN = 0.1f;
constexpr float MAX_LINEAR_SPEED = 2.0f;
constexpr float ROTATION_SPEED = 10.0f;
constexpr float MAX_SAFE_TORQUE = 8.0f;
constexpr uint32_t CONTROL_PERIOD_MS = 1;
constexpr uint32_t OFFLINE_DELAY_MS = 10;

// 当前电容工作模式实例化
sp::SuperCapMode current_supercap_mode = sp::SuperCapMode::AUTOMODE;

static sp::DBusSwitchMode last_sw_r = sp::DBusSwitchMode::MID;
static sp::DBusSwitchMode last_sw_l = sp::DBusSwitchMode::MID;

// 更新功率数据，从裁判系统和超级电容获取最新数据
void update_power_data()
{
    chassis_data.chassis_power_limit = pm02.robot_status.chassis_power_limit;
    
    if (chassis_data.chassis_power_limit == 0) 
    {
        chassis_data.chassis_power_limit = (DEFAULT_POWER_LIMIT-5.0f);
    }
    
    chassis_data.power_in = super_cap.power_in;
    chassis_data.power_out = super_cap.power_out;
    chassis_data.chassis_actual_power = chassis_data.power_in - chassis_data.power_out;
    chassis_data.predicted_power = predict_power_consumption();
}

// 功率预测模型，包含静态和动态功率项
float predict_power_consumption()
{
    static float filtered_power = 0.0f;
    constexpr float FILTER_ALPHA = 0.05f;
    
    // 历史数据用于计算变化率
    static float last_torque_lf = 0.0f, last_torque_lr = 0.0f;
    static float last_torque_rf = 0.0f, last_torque_rr = 0.0f;
    static float last_speed_lf = 0.0f, last_speed_lr = 0.0f;
    static float last_speed_rf = 0.0f, last_speed_rr = 0.0f;
    
    float torque_lf = chassis_data.torque_lf;
    float torque_lr = chassis_data.torque_lr; 
    float torque_rf = chassis_data.torque_rf;
    float torque_rr = chassis_data.torque_rr;
    
    float speed_lf = chassis_lf.speed;
    float speed_lr = chassis_lr.speed;
    float speed_rf = chassis_rf.speed;
    float speed_rr = chassis_rr.speed;
    
    // 计算转矩和速度变化率
    constexpr float CONTROL_FREQ = 1000.0f;
    float torque_rate_lf = std::abs(torque_lf - last_torque_lf) * CONTROL_FREQ;
    float torque_rate_lr = std::abs(torque_lr - last_torque_lr) * CONTROL_FREQ;
    float torque_rate_rf = std::abs(torque_rf - last_torque_rf) * CONTROL_FREQ;
    float torque_rate_rr = std::abs(torque_rr - last_torque_rr) * CONTROL_FREQ;
    
    float speed_rate_lf = std::abs(speed_lf - last_speed_lf) * CONTROL_FREQ;
    float speed_rate_lr = std::abs(speed_lr - last_speed_lr) * CONTROL_FREQ;
    float speed_rate_rf = std::abs(speed_rf - last_speed_rf) * CONTROL_FREQ;
    float speed_rate_rr = std::abs(speed_rr - last_speed_rr) * CONTROL_FREQ;
    
    // 静态功率项计算
    float shaft_power = torque_lf * speed_lf + torque_lr * speed_lr + 
                       torque_rf * speed_rf + torque_rr * speed_rr;
    
    float torque_loss = K1_TORQUE_LOSS * (torque_lf * torque_lf + torque_lr * torque_lr +
                                         torque_rf * torque_rf + torque_rr * torque_rr);
    
    float speed_loss = K2_SPEED_LOSS * (speed_lf * speed_lf + speed_lr * speed_lr +
                                       speed_rf * speed_rf + speed_rr * speed_rr);
    
    float static_power = K3_STATIC_POWER;
    
    // 动态功率项
    float dynamic_torque_power = K4_TORQUE_RATE * (torque_rate_lf + torque_rate_lr + 
                                                  torque_rate_rf + torque_rate_rr);
    float dynamic_speed_power = K5_SPEED_RATE * (speed_rate_lf + speed_rate_lr + 
                                                speed_rate_rf + speed_rate_rr);
    
    // 总功率预测
    float raw_predicted_power = shaft_power + torque_loss + speed_loss + static_power + 
                               dynamic_torque_power + dynamic_speed_power;
    
    // 低通滤波平滑
    filtered_power = FILTER_ALPHA * raw_predicted_power + (1.0f - FILTER_ALPHA) * filtered_power;
    
    // 更新历史数据
    last_torque_lf = torque_lf;
    last_torque_lr = torque_lr;
    last_torque_rf = torque_rf;
    last_torque_rr = torque_rr;
    last_speed_lf = speed_lf;
    last_speed_lr = speed_lr;
    last_speed_rf = speed_rf;
    last_speed_rr = speed_rr;
    
    return filtered_power;
}

// 功率限制控制，计算转矩缩放系数
float calculate_torque_scale_factor()
{
    float power_limit = static_cast<float>((chassis_data.chassis_power_limit-5.0f));
    float predicted_power = predict_power_consumption();
    
    if (predicted_power <= (power_limit-5.0f))
        return 1.0f;
    
    // 获取电机数据用于二次方程求解
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
    
    // 安全限制缩放因子范围
    k = std::max(k, POWER_SCALE_MIN);
    k = std::min(k, 1.0f);
    
    return k;
}

// 应用功率限制和安全转矩限制
void apply_power_limit()
{
    chassis_data.power_scale_factor = calculate_torque_scale_factor();
    chassis_data.power_limit_active = (chassis_data.power_scale_factor < 1.0f);
    
    // 按比例缩放所有电机转矩
    chassis_data.torque_lf *= chassis_data.power_scale_factor;
    chassis_data.torque_lr *= chassis_data.power_scale_factor;
    chassis_data.torque_rf *= chassis_data.power_scale_factor;
    chassis_data.torque_rr *= chassis_data.power_scale_factor;
    
    // 安全转矩限幅
    chassis_data.torque_lf = std::max(std::min(chassis_data.torque_lf, MAX_SAFE_TORQUE), -MAX_SAFE_TORQUE);
    chassis_data.torque_lr = std::max(std::min(chassis_data.torque_lr, MAX_SAFE_TORQUE), -MAX_SAFE_TORQUE);
    chassis_data.torque_rf = std::max(std::min(chassis_data.torque_rf, MAX_SAFE_TORQUE), -MAX_SAFE_TORQUE);
    chassis_data.torque_rr = std::max(std::min(chassis_data.torque_rr, MAX_SAFE_TORQUE), -MAX_SAFE_TORQUE);
}

// 停止所有电机
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

// 底盘运动控制主函数
void chassis_move_control(float vx, float vy, float wz)
{
    chassis_data.vx_set = vx;
    chassis_data.vy_set = vy;
    chassis_data.wz_set = wz;
    
    // 麦轮运动学解算
    mecanum_chassis.calc(vx, vy, wz);
    chassis_data.speed_lf_set = mecanum_chassis.speed_lf;
    chassis_data.speed_lr_set = mecanum_chassis.speed_lr;
    chassis_data.speed_rf_set = mecanum_chassis.speed_rf;
    chassis_data.speed_rr_set = mecanum_chassis.speed_rr;
    
    // PID速度闭环控制
    chassis_lf_pid.calc(chassis_data.speed_lf_set, chassis_lf.speed);
    chassis_lr_pid.calc(chassis_data.speed_lr_set, chassis_lr.speed);
    chassis_rf_pid.calc(chassis_data.speed_rf_set, chassis_rf.speed);
    chassis_rr_pid.calc(chassis_data.speed_rr_set, chassis_rr.speed);
    
    chassis_data.torque_lf = chassis_lf_pid.out;
    chassis_data.torque_lr = chassis_lr_pid.out;
    chassis_data.torque_rf = chassis_rf_pid.out;
    chassis_data.torque_rr = chassis_rr_pid.out;

    // 功率管理
    update_power_data();
    apply_power_limit();
    
    // 发送电机指令
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
        // 遥控器离线检测
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
        
        // 左拨杆音效和电容模式控制
        if (remote.sw_l != last_sw_l) {
            if (remote.sw_l == sp::DBusSwitchMode::UP && last_sw_l != sp::DBusSwitchMode::UP) {
                request_sound_effect(SoundEffect::LEFT_SWITCH_UP);
            }
            else if (remote.sw_l == sp::DBusSwitchMode::DOWN && last_sw_l != sp::DBusSwitchMode::DOWN) {
                request_sound_effect(SoundEffect::LEFT_SWITCH_DOWN);
            }
            else if (remote.sw_l == sp::DBusSwitchMode::MID && last_sw_l != sp::DBusSwitchMode::MID) {
                request_sound_effect(SoundEffect::LEFT_SWITCH_UP);
            }
            last_sw_l = remote.sw_l;
        }
        
        // 电容模式设置
        if (remote.sw_l == sp::DBusSwitchMode::MID) {
            current_supercap_mode = sp::SuperCapMode::DISCHARGE;  // 只放不充模式
        } else {
            current_supercap_mode = sp::SuperCapMode::AUTOMODE;   // 自动模式
        }
        
        // 底盘控制逻辑
        if (remote.sw_r == sp::DBusSwitchMode::MID) {
            float raw_vx = remote.ch_lv;
            float raw_vy = remote.ch_lh;
            float raw_right_v = remote.ch_rv;
            float raw_right_h = remote.ch_rh;
            
            float vx = raw_vx * MAX_LINEAR_SPEED;
            float vy = (-raw_vy) * MAX_LINEAR_SPEED;
            
            float wz = 0.0f;
            if (std::abs(raw_right_v) > 0.0f) {
                wz = raw_right_v * ROTATION_SPEED;
            }
            else if (std::abs(raw_right_h) > 0.0f) {
                wz = -raw_right_h * ROTATION_SPEED;
            }
            
            chassis_move_control(vx, vy, wz);
        }
        else {
            disable_all_motors();
        }
        
        osDelay(CONTROL_PERIOD_MS);
    }
}
