#include "cmsis_os.h"
#include "can.hpp"
#include "chassis_control.hpp"

void can2_recv(uint32_t stamp_ms)
{
    can2.recv();
    
    // 处理底盘电机反馈 (ID 1-4)
    if (can2.rx_id == chassis_lf.rx_id)  // 0x201
        chassis_lf.read(can2.rx_data, stamp_ms);
    if (can2.rx_id == chassis_lr.rx_id)  // 0x202
        chassis_lr.read(can2.rx_data, stamp_ms);
    if (can2.rx_id == chassis_rf.rx_id)  // 0x203
        chassis_rf.read(can2.rx_data, stamp_ms);
    if (can2.rx_id == chassis_rr.rx_id)  // 0x204
        chassis_rr.read(can2.rx_data, stamp_ms);
    
    // 处理超级电容反馈 (ID 0x301)
    if (can2.rx_id == super_cap.rx_id)
        super_cap.read(can2.rx_data, stamp_ms);
}


extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
    auto stamp_ms = osKernelSysTick();

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
        if (hcan == &hcan2)
            can2_recv(stamp_ms);
    }
}