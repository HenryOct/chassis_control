#ifndef CAN_HPP
#define CAN_HPP

#include "io/can/can.hpp"

inline sp::CAN can2(&hcan2);

void send_chassis_motors();  // 底盘电机发送函数

#endif  // CAN_HPP