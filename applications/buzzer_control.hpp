#ifndef BUZZER_CONTROL_HPP
#define BUZZER_CONTROL_HPP

// 音效类型枚举
enum class SoundEffect {
    NONE,
    STARTUP,
    SWITCH_UP,        // 升调：右拨杆向上
    SWITCH_DOWN,      // 降调：右拨杆向下
    LEFT_SWITCH_UP,   // 高音调：左拨杆向上
    LEFT_SWITCH_DOWN  // 低音调：左拨杆向下
};

// 请求播放音效的函数声明
void request_sound_effect(SoundEffect effect);

#endif // BUZZER_CONTROL_HPP
