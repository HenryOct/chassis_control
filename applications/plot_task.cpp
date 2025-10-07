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
    
    float speed_lf = chassis_lf.speed;
    float speed_lr = chassis_lr.speed;
    float speed_rf = chassis_rf.speed;
    float speed_rr = chassis_rr.speed;                        
    
    // 使用plotter.plot()发送功率数据到SerialPlot (最多10个通道)
    plotter.plot(
        test_counter,                         // 通道1: 测试信号 (应该看到递增的锯齿波)
        super_cap.power_in,                // 通道2: 电池输入功率 (实际测量值)
        predict_power_consumption(),         // 通道3: 预测功率 (模型计算值)
        speed_lf,                             // 通道4: 左前轮速度 (用于调试抖动)
        speed_lr,                             // 通道5: 左后轮速度
        speed_rf,                             // 通道6: 右前轮速度
        speed_rr                              // 通道7: 右后轮速度
    );
    
    osDelay(10);  // 100Hz - SerialPlot适合的采样频率
  }
}