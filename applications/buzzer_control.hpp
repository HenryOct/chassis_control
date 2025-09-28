#ifndef BUZZER_CONTROL_HPP
#define BUZZER_CONTROL_HPP

// 音效类型枚举
enum class SoundEffect {
    NONE,
    STARTUP,
    SWITCH_UP,    // 升调：拨杆向上
    SWITCH_DOWN   // 降调：拨杆向下
};

// 请求播放音效的函数声明
void request_sound_effect(SoundEffect effect);

#endif // BUZZER_CONTROL_HPP
