#include "cmsis_os.h"
#include "chassis_control.hpp"
#include "can/can.hpp"
#include "uart/uart.hpp"
#include "buzzer_control.hpp"
#include <cmath>
#include <cstdlib>  

ChassisData chassis_data;

// 右拨杆状态监控变量
static sp::DBusSwitchMode last_sw_r = sp::DBusSwitchMode::MID;  // 上次右拨杆状态

// 功率控制相关常量
constexpr uint16_t DEFAULT_POWER_LIMIT = 40;   // 默认功率限制 40W
constexpr uint16_t POWER_BUFFER_THRESHOLD = 20; // 缓冲能量阈值 20J
constexpr float POWER_SCALE_MIN = 0.3f;        // 最小功率缩放因子
constexpr float POWER_SCALE_MAX = 1.0f;        // 最大功率缩放因子

// 更新功率数据从裁判系统
void update_power_data()
{
    // 从PM02裁判系统获取功率限制和缓冲能量
    chassis_data.chassis_power_limit = pm02.robot_status.chassis_power_limit;
    chassis_data.buffer_energy = pm02.power_heat.buffer_energy;
    
    // 如果裁判系统离线，使用默认值
    if (chassis_data.chassis_power_limit == 0) {
        chassis_data.chassis_power_limit = DEFAULT_POWER_LIMIT;
    }
}

// 计算功率缩放因子
float calculate_power_scale_factor()
{
    // 基于缓冲能量计算缩放因子
    if (chassis_data.buffer_energy >= POWER_BUFFER_THRESHOLD) {
        // 缓冲能量充足，不限制功率
        return POWER_SCALE_MAX;
    } else if (chassis_data.buffer_energy <= 5) {
        // 缓冲能量极低，严格限制功率
        return POWER_SCALE_MIN;
    } else {
        // 线性插值计算缩放因子
        float scale = POWER_SCALE_MIN + 
                     (POWER_SCALE_MAX - POWER_SCALE_MIN) * 
                     (chassis_data.buffer_energy - 5.0f) / (POWER_BUFFER_THRESHOLD - 5.0f);
        return scale;
    }
}

// 应用功率限制
void apply_power_limit()
{
    // 计算当前功率缩放因子
    chassis_data.power_scale_factor = calculate_power_scale_factor();
    
    // 判断是否需要激活功率限制
    chassis_data.power_limit_active = (chassis_data.power_scale_factor < POWER_SCALE_MAX);
    
    // 如果功率限制激活，对所有电机扭矩进行缩放
    if (chassis_data.power_limit_active) {
        chassis_data.torque_lf *= chassis_data.power_scale_factor;
        chassis_data.torque_lr *= chassis_data.power_scale_factor;
        chassis_data.torque_rf *= chassis_data.power_scale_factor;
        chassis_data.torque_rr *= chassis_data.power_scale_factor;
    }
}

// 禁用所有电机，保证底盘和摩擦轮都不会移动
void disable_all_motors()
{
    // 清零底盘数据
    chassis_data.torque_lf = 0.0f;
    chassis_data.torque_lr = 0.0f;
    chassis_data.torque_rf = 0.0f;
    chassis_data.torque_rr = 0.0f;
    
    // 停止底盘电机
    chassis_lf.cmd(0.0f);
    chassis_lr.cmd(0.0f);
    chassis_rf.cmd(0.0f);
    chassis_rr.cmd(0.0f);
    
    // 发送底盘电机控制指令 (0x200)
    send_chassis_motors();
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
    
    // 完整的PID速度闭环控制
    chassis_lf_pid.calc(chassis_data.speed_lf_set, chassis_lf.speed);
    chassis_lr_pid.calc(chassis_data.speed_lr_set, chassis_lr.speed);
    chassis_rf_pid.calc(chassis_data.speed_rf_set, chassis_rf.speed);
    chassis_rr_pid.calc(chassis_data.speed_rr_set, chassis_rr.speed);
    
    // 使用PID输出值
    chassis_data.torque_lf = chassis_lf_pid.out;
    chassis_data.torque_lr = chassis_lr_pid.out;
    chassis_data.torque_rf = chassis_rf_pid.out;
    chassis_data.torque_rr = chassis_rr_pid.out;
    
    // 应用功率限制
    apply_power_limit();
    
    // 写入控制指令
    chassis_lf.cmd(chassis_data.torque_lf);
    chassis_lr.cmd(chassis_data.torque_lr);
    chassis_rf.cmd(chassis_data.torque_rf);
    chassis_rr.cmd(chassis_data.torque_rr);
    
    // 发送控制指令
    send_chassis_motors();
}

extern "C" void chassis_control_task()
{
    // 初始化遥控器
    remote.request();
    
    // 初始化CAN2总线
    can2.config();
    can2.start();
    
    // 初始化PM02裁判系统
    pm02.request();
    
    // 初始化底盘数据
    chassis_data.vx_set = 0.0f;
    chassis_data.vy_set = 0.0f;
    chassis_data.wz_set = 0.0f;
    
    // 初始化功率控制数据
    chassis_data.chassis_power_limit = DEFAULT_POWER_LIMIT;
    chassis_data.buffer_energy = 60;  // 初始缓冲能量设为60J
    chassis_data.power_scale_factor = 1.0f;
    chassis_data.power_limit_active = false;

    while (true) {
        // 更新功率数据（从裁判系统获取最新数据）
        update_power_data();
        
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
            const float ROTATION_SPEED = 2.0f;     // 固定转向速度 2rad/s
            
            // 直接获取摇杆输入值
            float raw_vx = remote.ch_lv;
            float raw_vy = remote.ch_lh;
            float raw_right_v = remote.ch_rv;
            
            // 直接线性映射
            float vx = raw_vx * MAX_LINEAR_SPEED;   // 前后移动
            float vy = raw_vy * MAX_LINEAR_SPEED;   // 左右移动
            
            // 获取右摇杆左右值
            float raw_right_h = remote.ch_rh;
            
            // 计算旋转速度：右摇杆前后控制向左转，左右控制向右转
            float wz = 0.0f;
            if (abs(raw_right_v) > 0.0f) {
                wz = raw_right_v * ROTATION_SPEED;   // 右摇杆前后：向左转
            }
            else if (abs(raw_right_h) > 0.0f) {
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
        
        // 向超级电容发送控制数据
        uint8_t super_cap_tx_data[8];
        super_cap.write(super_cap_tx_data, 
                       chassis_data.chassis_power_limit, 
                       chassis_data.buffer_energy,
                       pm02.robot_status.power_management_chassis_output);
        
        // 发送超级电容控制指令到CAN总线 (假设使用CAN2，ID 0x300)
        can2.tx(super_cap.tx_id, super_cap_tx_data);
        
        osDelay(1);  //1000Hz控制频率
    }
}
