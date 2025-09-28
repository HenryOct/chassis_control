#include "cmsis_os.h"
#include "can.hpp"
#include "chassis_control.hpp"

// M3508底盘电机专用函数
void send_chassis_motors() {
    
    chassis_lf.write(can2.tx_data);  // 电机1 → 数据位置 [0:1]
    chassis_lr.write(can2.tx_data);  // 电机2 → 数据位置 [2:3]
    chassis_rf.write(can2.tx_data);  // 电机3 → 数据位置 [4:5]
    chassis_rr.write(can2.tx_data);  // 电机4 → 数据位置 [6:7]
    
    // 所有M3508电机(ID 1-4)使用同一个TX ID: 0x200
    can2.send(0x200);
}