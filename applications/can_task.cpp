#include "cmsis_os.h"
#include "can.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "motor/super_cap/super_cap.hpp"
#include "referee/pm02/pm02.hpp"
#include "chassis_control.hpp"

extern CAN_HandleTypeDef hcan2;

sp::CAN can2(&hcan2);
ChassisData chassis_data;

// CAN通信任务
extern "C" void can_task(void const * argument)
{
    can2.config();
    can2.start();
    
    while (true) {
        // 底盘电机控制
        chassis_lf.write(can2.tx_data);
        chassis_lr.write(can2.tx_data);
        chassis_rf.write(can2.tx_data);
        chassis_rr.write(can2.tx_data);
        can2.send(0x200);
        
        // 超级电容控制
        uint8_t super_cap_tx_data[8];
        super_cap.write(super_cap_tx_data, 
                       chassis_data.chassis_power_limit, 
                       pm02.power_heat.buffer_energy,
                       pm02.robot_status.power_management_chassis_output);
        
        // 根据左拨杆状态覆盖电容模式
        super_cap_tx_data[0] = static_cast<uint8_t>(current_supercap_mode);
        
        for (int i = 0; i < 8; i++) {
            can2.tx_data[i] = super_cap_tx_data[i];
        }
        can2.send(super_cap.tx_id);
        
        osDelay(1);
    }
}

// CAN接收中断处理
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
    auto stamp_ms = osKernelSysTick();

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
        if (hcan == &hcan2) {
            can2.recv();

            // 处理底盘电机反馈
            if (can2.rx_id == chassis_lf.rx_id) chassis_lf.read(can2.rx_data, stamp_ms);
            if (can2.rx_id == chassis_lr.rx_id) chassis_lr.read(can2.rx_data, stamp_ms);
            if (can2.rx_id == chassis_rf.rx_id) chassis_rf.read(can2.rx_data, stamp_ms);
            if (can2.rx_id == chassis_rr.rx_id) chassis_rr.read(can2.rx_data, stamp_ms);
            
            // 处理超级电容反馈
            if (can2.rx_id == super_cap.rx_id) super_cap.read(can2.rx_data, stamp_ms);
        }
    }
}
