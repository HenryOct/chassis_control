#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "referee/pm02/pm02.hpp"
#include "usart.h"

extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

sp::DBus remote(&huart3);
sp::PM02 pm02(&huart6);

// UART任务，处理遥控器和裁判系统通信
extern "C" void uart_task(void const * argument)
{
    remote.request();
    pm02.request();
    
    while (true) {
        osDelay(1);
    }
}

// UART接收完成中断回调
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
    auto stamp_ms = osKernelSysTick();

    if (huart == &huart3) {
        remote.update(Size, stamp_ms);
        remote.request();
    }
    
    if (huart == pm02.huart) {
        pm02.update(Size);
        pm02.request();
    }
}

// UART错误处理回调
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
    if (huart == &huart3) {
        remote.request();
    }
    
    if (huart == pm02.huart) {
        pm02.request();
    }
}
