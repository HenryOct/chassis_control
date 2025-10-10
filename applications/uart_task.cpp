#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "referee/pm02/pm02.hpp"
#include "usart.h"

extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

sp::DBus remote(&huart3);
sp::PM02 pm02(&huart6);

// 串口通信任务
extern "C" void uart_task(void const * argument)
{
    remote.request();
    pm02.request();
    
    while (true) {
        osDelay(1);
    }
}

// 串口接收中断处理
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
    auto stamp_ms = osKernelSysTick();

    if (huart == &huart3) {
        remote.update(Size, stamp_ms);
        remote.request();
    }
    
    if (huart == &huart6) {
        pm02.update(Size);
        pm02.request();
    }
}

// 串口错误处理
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
    if (huart == &huart3) {
        remote.request();
    }
    
    if (huart == &huart6) {
        pm02.request();
    }
}
