// ModeManager: 能量机关模式管理器，带迟滞消抖 (2026)

#ifndef AIMER_AUTOBUFF_PREDICTOR_MODE_MANAGER_HPP
#define AIMER_AUTOBUFF_PREDICTOR_MODE_MANAGER_HPP

#include "aimer/auto_buff/common/types.hpp"
#include "aimer/common/robot_state.hpp"

namespace autobuff::predictor {

class ModeManager {
public:
    ModeManager() = default;

    autobuff::BuffMode update(aimer::AimMode aim_mode, int debounced_lit_count);
    void reset();

private:
    int active_streak_ = 0;    // 连续满足大符激活条件的帧数
    int inactive_streak_ = 0;  // 连续不满足大符激活条件的帧数
    bool large_active_ = false; // 当前是否处于大符激活状态
};

}  // namespace autobuff::predictor

#endif  // AIMER_AUTOBUFF_PREDICTOR_MODE_MANAGER_HPP
