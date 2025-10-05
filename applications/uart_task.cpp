#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "referee/pm02/pm02.hpp"
#include "usart.h"

// 按照sp_middleware标准，外部声明UART句柄
extern UART_HandleTypeDef huart3; // 遥控器
extern UART_HandleTypeDef huart6; // PM02裁判系统

// 遥控器实例
sp::DBus remote(&huart3);

// PM02裁判系统实例 (C板使用huart6)
sp::PM02 pm02(&huart6);

extern "C" void uart_task(void const * argument)
{
    // 初始化遥控器
    remote.request();
    
    // 初始化PM02裁判系统
    pm02.request();
    
    while (true) {
        // UART任务主要负责初始化，数据处理在中断中完成
        osDelay(10);
    }
}

// UART接收完成中断回调
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
    auto stamp_ms = osKernelSysTick();

    // 遥控器数据接收处理 (UART3)
    if (huart == &huart3) {
        remote.update(Size, stamp_ms);
        remote.request();
    }
    
    // PM02裁判系统数据接收处理 (UART6)
    if (huart == pm02.huart) {
        pm02.update(Size);
        pm02.request();
    }
}

// UART错误处理回调
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
    // 遥控器错误处理
    if (huart == &huart3) {
        remote.request();
    }
    
    // PM02裁判系统错误处理
    if (huart == pm02.huart) {
        pm02.request();
    }
}
