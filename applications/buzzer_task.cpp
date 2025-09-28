#include "cmsis_os.h"
#include "io/buzzer/buzzer.hpp"
#include "buzzer_control.hpp"

sp::Buzzer buzzer(&htim4, TIM_CHANNEL_3, 84e6);

// 全局变量用于任务间通信
volatile SoundEffect sound_request = SoundEffect::NONE;

// 播放升调音效（频率递增）
void play_switch_up_sound() {
    for (int i = 0; i < 3; i++) {
        buzzer.start();
        buzzer.set(800 + 300 * i, 0.01);  // 800Hz → 1100Hz → 1400Hz
        osDelay(80);
        buzzer.stop();
        osDelay(20);
    }
}

// 播放降调音效（频率递减）
void play_switch_down_sound() {
    for (int i = 0; i < 3; i++) {
        buzzer.start();
        buzzer.set(1400 - 300 * i, 0.01);  // 1400Hz → 1100Hz → 800Hz
        osDelay(80);
        buzzer.stop();
        osDelay(20);
    }
}

// 播放启动音效
void play_startup_sound() {
    for (int i = 0; i < 3; i++) {
        buzzer.start();
        buzzer.set(1000 + 500 * i, 0.01);  // 1000Hz → 1500Hz → 2000Hz
        osDelay(100);
        buzzer.stop();
        osDelay(100);
    }
}

// 请求播放音效的函数（供其他任务调用）
void request_sound_effect(SoundEffect effect) {
    sound_request = effect;
}

extern "C" void buzzer_task()
{   
    // 播放启动音
    play_startup_sound();
    
    while (true) {
        // 检查是否有音效请求
        if (sound_request != SoundEffect::NONE) {
            switch (sound_request) {
                case SoundEffect::SWITCH_UP:
                    play_switch_up_sound();
                    break;
                case SoundEffect::SWITCH_DOWN:
                    play_switch_down_sound();
                    break;
                default:
                    break;
            }
            sound_request = SoundEffect::NONE;  // 清除请求
        }
        
        osDelay(50);  // 20Hz检查频率
    }
}