#include <cstdio>
#include <cstring>

#include "chassis_control.hpp"
#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"
#include "usart.h"

sp::Plotter plotter(&huart1);
extern sp::PM02 pm02;  // uart_task.cpp中实例化

// 数据可视化任务
extern "C" void plot_task()
{
  while (true) {
    plotter.plot(
      chassis_data.chassis_power_limit, chassis_data.power_in, predict_power_consumption(),
      // super_cap.cap_energy
      pm02.power_heat.buffer_energy);

    osDelay(10);
  }
}