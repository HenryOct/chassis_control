#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"
#include "can/can.hpp"
#include "uart/uart.hpp"
#include "chassis_control.hpp"  // 包含底盘电机定义

sp::Plotter plotter(&huart1);

extern "C" void plot_task()
{
  while (true) {
    plotter.plot(
        chassis_lf.speed,   // 左前轮速度
        chassis_lr.speed,   // 左后轮速度  
        chassis_rf.speed,   // 右前轮速度
        chassis_rr.speed    // 右后轮速度
    );
    osDelay(10);  // 100Hz
  }
}