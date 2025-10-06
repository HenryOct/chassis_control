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
    // 简单测试数据，确保串口通信正常
    static float test_counter = 0.0f;
    test_counter += 0.1f;
    
    // 计算功率模型各分量用于调试
    float torque_lf = chassis_data.torque_lf;
    float torque_lr = chassis_data.torque_lr; 
    float torque_rf = chassis_data.torque_rf;
    float torque_rr = chassis_data.torque_rr;
    
    float speed_lf = chassis_lf.speed;
    float speed_lr = chassis_lr.speed;
    float speed_rf = chassis_rf.speed;
    float speed_rr = chassis_rr.speed;
    
    // 计算各项功率分量 (使用头文件中的实际参数)
    float shaft_power = torque_lf * speed_lf + torque_lr * speed_lr + 
                       torque_rf * speed_rf + torque_rr * speed_rr;
    
    float torque_loss = K1_TORQUE_LOSS * (torque_lf * torque_lf + torque_lr * torque_lr +
                                         torque_rf * torque_rf + torque_rr * torque_rr);
    
    float speed_loss = K2_SPEED_LOSS * (speed_lf * speed_lf + speed_lr * speed_lr +
                                       speed_rf * speed_rf + speed_rr * speed_rr);
                                      
    
    // 使用plotter.plot()发送功率数据到SerialPlot (最多10个通道)
    plotter.plot(
        test_counter,                         // 通道1: 测试信号 (应该看到递增的锯齿波)
        (chassis_data.power_out-chassis_data.power_in),                // 通道2: 电池输入功率 (实际测量值)
        chassis_data.predicted_power,         // 通道3: 预测功率 (模型计算值)
        shaft_power,                          // 通道4: 轴功率分量 Σ(τ·ω)
        torque_loss,                          // 通道5: 转矩损耗分量 K₁·Σ(τ²)
        speed_loss,                           // 通道6: 速度损耗分量 K₂·Σ(ω²)
        speed_lf,                             // 通道7: 左前轮速度 (用于调试抖动)
        speed_lr,                             // 通道8: 左后轮速度
        speed_rf,                             // 通道9: 右前轮速度
        speed_rr                              // 通道10: 右后轮速度
    );
    
    osDelay(10);  // 100Hz - SerialPlot适合的采样频率
  }
}