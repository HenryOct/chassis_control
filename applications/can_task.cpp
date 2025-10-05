#include "cmsis_os.h"
#include "can.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "motor/super_cap/super_cap.hpp"
#include "referee/pm02/pm02.hpp"
#include "chassis_control.hpp"

// CAN2句柄声明
extern CAN_HandleTypeDef hcan2;

// 定义extern变量
sp::CAN can2(&hcan2);
ChassisData chassis_data;

// 按照sp_middleware标准创建CAN任务
extern "C" void can_task(void const * argument)
{
    
    // 配置和启动CAN2
    can2.config();
    can2.start();
    
    while (true) {
        // 底盘电机控制命令写入CAN2
        chassis_lf.write(can2.tx_data);
        chassis_lr.write(can2.tx_data);
        chassis_rf.write(can2.tx_data);
        chassis_rr.write(can2.tx_data);
        
        // 发送底盘电机控制命令 (0x200 = ID 1-4)
        can2.send(0x200);
        
        // 超级电容控制
        uint8_t super_cap_tx_data[8];
        super_cap.write(super_cap_tx_data, 
                       chassis_data.chassis_power_limit, 
                       pm02.power_heat.buffer_energy,
                       pm02.robot_status.power_management_chassis_output);
        
        // 发送超级电容控制命令
        can2.tx_data[0] = super_cap_tx_data[0];
        can2.tx_data[1] = super_cap_tx_data[1];
        can2.tx_data[2] = super_cap_tx_data[2];
        can2.tx_data[3] = super_cap_tx_data[3];
        can2.tx_data[4] = super_cap_tx_data[4];
        can2.tx_data[5] = super_cap_tx_data[5];
        can2.tx_data[6] = super_cap_tx_data[6];
        can2.tx_data[7] = super_cap_tx_data[7];
        can2.send(super_cap.tx_id);
        
        osDelay(1); // 1000Hz
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
