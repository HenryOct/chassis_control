#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"
#include "chassis_control.hpp"
#include "usart.h"
#include <cstdio>
#include <cstring>

sp::Plotter plotter(&huart1);

// 数据可视化任务
extern "C" void plot_task()
{
  while (true) 
  {
    plotter.plot(
        chassis_data.chassis_power_limit,
        chassis_data.power_in,
        predict_power_consumption(),
        chassis_data.power_scale_factor
    );
    
    osDelay(10);
  }
}