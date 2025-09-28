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
    
    // 初始化底盘数据
    chassis_data.vx_set = 0.0f;
    chassis_data.vy_set = 0.0f;
    chassis_data.wz_set = 0.0f;

    while (true) {
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
            
            // 计算旋转速度：右摇杆前后控制向左转，左右控制向右转
            float wz = 0.0f;
            if (abs(raw_right_v) > 0.0f) {
                wz = raw_right_v * ROTATION_SPEED;   // 右摇杆前后：向左转
            }
            else if (abs(raw_right_h) > 0.0f) {
                wz = -raw_right_h * ROTATION_SPEED;  // 右摇杆左右：向右转
            }
            
            // 使用完整的PID控制底盘移动
            chassis_move_control(vx, vy, wz);
        }
        // 右拨杆下档：全部电机失能
        else if (remote.sw_r == sp::DBusSwitchMode::DOWN) 
        {
            // 无论摇杆如何移动，都禁用所有电机
            disable_all_motors();
        }
        // 其他状态：禁用所有电机
        else 
        {
            disable_all_motors();
        }
        
        osDelay(1);  //1000Hz控制频率
    }
}
