// ModeManager 实现 (2026)

#include "mode_manager.hpp"

#include "plugin/param/runtime_parameter.hpp"

namespace autobuff::predictor {

autobuff::BuffMode ModeManager::update(aimer::AimMode aim_mode, int debounced_lit_count) {
    if (aim_mode == aimer::AimMode::ENERGY_SMALL) {
        reset();
        return autobuff::BuffMode::SMALL_ACTIVE;
    }

    if (aim_mode == aimer::AimMode::ENERGY_LARGE) {
        // 在使用点直接读取运行时参数 (禁止缓存)
        int enter_frames = static_cast<int>(
            runtime_param::get_param<int64_t>("AutoBuff.Mode.enter_large_active_frames"));
        int exit_frames = static_cast<int>(
            runtime_param::get_param<int64_t>("AutoBuff.Mode.exit_large_active_frames"));
        if (enter_frames <= 0) enter_frames = 5;
        if (exit_frames <= 0) exit_frames = 8;

        // 大符激活判断：需要 >= 2 个亮槽 (2026 规则)
        constexpr int LARGE_LIT_THRESHOLD = 2;

        if (debounced_lit_count >= LARGE_LIT_THRESHOLD) {
            active_streak_++;
            inactive_streak_ = 0;

            if (!large_active_ && active_streak_ >= enter_frames) {
                large_active_ = true;
            }
        } else {
            inactive_streak_++;
            active_streak_ = 0;

            if (large_active_ && inactive_streak_ >= exit_frames) {
                large_active_ = false;
            }
        }

        return large_active_ ? autobuff::BuffMode::LARGE_ACTIVE
                             : autobuff::BuffMode::LARGE_INACTIVE;
    }

    // 非能量机关模式
    reset();
    return autobuff::BuffMode::UNKNOWN;
}

void ModeManager::reset() {
    active_streak_ = 0;
    inactive_streak_ = 0;
    large_active_ = false;
}

}  // namespace autobuff::predictor
