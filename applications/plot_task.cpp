#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"
#include "chassis_control.hpp"
#include "usart.h"
#include <cstdio>
#include <cstring>

sp::Plotter plotter(&huart1);

// 绘图任务，发送数据到SerialPlot进行可视化
extern "C" void plot_task()
{
  while (true) {
    static float test_counter = 0.0f;
    test_counter += 0.1f;
    
    float speed_lf = chassis_lf.speed;
    float speed_lr = chassis_lr.speed;
    float speed_rf = chassis_rf.speed;
    float speed_rr = chassis_rr.speed;                        
    
    plotter.plot(
        test_counter,
        chassis_data.power_out,
        predict_power_consumption(),
        speed_lf,
        speed_lr,
        speed_rf,
        speed_rr
    );
    
    osDelay(10);
  }
}