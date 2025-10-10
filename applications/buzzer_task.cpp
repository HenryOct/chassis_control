#include "cmsis_os.h"
#include "io/buzzer/buzzer.hpp"
#include "buzzer_control.hpp"

sp::Buzzer buzzer(&htim4, TIM_CHANNEL_3, 84e6);

volatile SoundEffect sound_request = SoundEffect::NONE;

// 右拨杆升调音效
void play_switch_up_sound() {
    for (int i = 0; i < 3; i++) {
        buzzer.start();
        buzzer.set(800 + 300 * i, 0.01);
        osDelay(80);
        buzzer.stop();
        osDelay(20);
    }
}

// 右拨杆降调音效
void play_switch_down_sound() {
    for (int i = 0; i < 3; i++) {
        buzzer.start();
        buzzer.set(1400 - 300 * i, 0.01);
        osDelay(80);
        buzzer.stop();
        osDelay(20);
    }
}

// 启动音效
void play_startup_sound() {
    for (int i = 0; i < 3; i++) {
        buzzer.start();
        buzzer.set(1000 + 500 * i, 0.01);
        osDelay(100);
        buzzer.stop();
        osDelay(100);
    }
}

// 左拨杆升调音效
void play_left_switch_up_sound() {
    for (int i = 0; i < 3; i++) {
        buzzer.start();
        buzzer.set(1200 + 400 * i, 0.01);
        osDelay(60);
        buzzer.stop();
        osDelay(15);
    }
}

// 左拨杆降调音效
void play_left_switch_down_sound() {
    for (int i = 0; i < 3; i++) {
        buzzer.start();
        buzzer.set(600 - 150 * i, 0.01);
        osDelay(100);
        buzzer.stop();
        osDelay(25);
    }
}

// 请求播放音效的函数
void request_sound_effect(SoundEffect effect) {
    sound_request = effect;
}

// 蜂鸣器音效任务
extern "C" void buzzer_task()
{   
    play_startup_sound();
    
    while (true) {
        if (sound_request != SoundEffect::NONE) {
            switch (sound_request) {
                case SoundEffect::SWITCH_UP:
                    play_switch_up_sound();
                    break;
                case SoundEffect::SWITCH_DOWN:
                    play_switch_down_sound();
                    break;
                case SoundEffect::LEFT_SWITCH_UP:
                    play_left_switch_up_sound();
                    break;
                case SoundEffect::LEFT_SWITCH_DOWN:
                    play_left_switch_down_sound();
                    break;
                default:
                    break;
            }
            sound_request = SoundEffect::NONE;
        }
        
        osDelay(50);
    }
}