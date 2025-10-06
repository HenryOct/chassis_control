#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"
#include "chassis_control.hpp"  // 包含底盘电机定义
#include "usart.h"
#include <cstdio>
#include <cstring>

sp::Plotter plotter(&huart1);

extern "C" void plot_task()
{
  while (true) {
    // 计算功率模型各分量用于调试
    float torque_lf = chassis_data.torque_lf;
    float torque_lr = chassis_data.torque_lr; 
    float torque_rf = chassis_data.torque_rf;
    float torque_rr = chassis_data.torque_rr;
    
    float speed_lf = chassis_lf.speed;
    float speed_lr = chassis_lr.speed;
    float speed_rf = chassis_rf.speed;
    float speed_rr = chassis_rr.speed;
    
    // 计算各项功率分量
    float shaft_power = torque_lf * speed_lf + torque_lr * speed_lr + 
                       torque_rf * speed_rf + torque_rr * speed_rr;
    
    float torque_loss = 2.0f * (torque_lf * torque_lf + torque_lr * torque_lr +
                               torque_rf * torque_rf + torque_rr * torque_rr);
    
    float speed_loss = 0.01f * (speed_lf * speed_lf + speed_lr * speed_lr +
                               speed_rf * speed_rf + speed_rr * speed_rr);
    
    float static_power = 5.0f;
    
    // 使用plotter.plot()发送功率数据到SerialPlot
    plotter.plot(
        chassis_data.power_in,                // 电池输入功率 (实际测量值)
        chassis_data.predicted_power,         // 预测功率 (模型计算值)
        chassis_data.chassis_actual_power,    // 实际功率 (power_out - power_in)
        shaft_power,                          // 轴功率分量 Σ(τ·ω)
        torque_loss,                          // 转矩损耗分量 K₁·Σ(τ²)
        speed_loss,                           // 速度损耗分量 K₂·Σ(ω²)
        static_power,                         // 静态功耗分量 K₃
        chassis_data.power_scale_factor       // 功率缩放系数
    );
    
    osDelay(10);  // 100Hz - SerialPlot适合的采样频率
  }
}