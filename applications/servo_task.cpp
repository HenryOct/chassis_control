#include "cmsis_os.h"
#include "io/servo/servo.hpp"

extern TIM_HandleTypeDef htim1;

// 舵机实例化
sp::Servo servo(&htim1, TIM_CHANNEL_1, 168e6f, 180.0f); // C板配置
// sp::Servo servo(&htim1, TIM_CHANNEL_3, 240e6f, 180.0f); // 达妙配置

// 舵机控制任务，上电后匀速旋转0-180度
extern "C" void servo_task()
{
    servo.start();
    servo.set(0.0f);
    osDelay(1000);
    
    // 匀速旋转参数
    const float total_time = 4.0f;
    const float update_interval = 0.02f;
    const int total_steps = (int)(total_time / update_interval);
    const float angle_step = 180.0f / total_steps;
    
    for (int i = 0; i <= total_steps; i++) {
        float angle = i * angle_step;
        servo.set(angle);
        osDelay(20);
    }
    
    while (true) osDelay(1000);
}
