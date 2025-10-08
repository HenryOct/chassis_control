#include "cmsis_os.h"
#include "io/servo/servo.hpp"

// 舵机实例化（根据您的硬件配置选择）
sp::Servo servo(&htim1, TIM_CHANNEL_1, 168e6f, 180.0f); // C板配置
// sp::Servo servo(&htim1, TIM_CHANNEL_3, 240e6f, 180.0f); // 达妙配置

// 舵机任务，上电后执行一次匀速0-180度旋转
extern "C" void servo_task()
{
    servo.start();
    
    // 从0度开始
    servo.set(0.0f);
    osDelay(1000); // 等待1秒确保舵机到位
    
    // 匀速旋转：每秒45度，总时间4秒，每20ms更新一次角度
    const float total_time = 4.0f; // 总时间4秒
    const float update_interval = 0.02f; // 每20ms更新一次
    const int total_steps = (int)(total_time / update_interval); // 总步数200步
    const float angle_step = 180.0f / total_steps; // 每步角度增量0.9度
    
    for (int i = 0; i <= total_steps; i++) 
    {
        float angle = i * angle_step; // 0, 0.9, 1.8, 2.7, ..., 180度
        servo.set(angle);
        osDelay(20); // 每20ms更新一次
    }
    
    // 旋转完成后保持180度位置
    while (true) osDelay(1000);
}
