// SlotDebouncer: 每个槽位的 4 状态 FSM 去抖动器 (2026)

#ifndef AIMER_AUTOBUFF_PREDICTOR_SLOT_DEBOUNCER_HPP
#define AIMER_AUTOBUFF_PREDICTOR_SLOT_DEBOUNCER_HPP

#include <array>
#include <cstdint>

#include "aimer/auto_buff/common/types.hpp"

namespace autobuff::predictor {

// 单个槽位稳定输出
struct StableSlot {
    bool valid = false;
    bool is_lit = false;
    bool state_changed = false;
    float confidence = 0.f;
};

class SlotDebouncer {
public:
    struct Output {
        std::array<StableSlot, NUM_SLOTS> slots{};
        uint8_t lit_mask = 0;
        int lit_count = 0;
    };

    SlotDebouncer() = default;

    Output update(const autobuff::BuffDetectionResult& det, double timestamp);
    void reset();

private:
    // 每槽位 4 状态 FSM
    enum class State : uint8_t {
        OFF = 0,
        CANDIDATE_ON = 1,
        ON = 2,
        CANDIDATE_OFF = 3,
    };

    struct SlotFsm {
        State state = State::OFF;
        int count = 0;
        double last_seen = 0.0;
    };

    std::array<SlotFsm, NUM_SLOTS> fsm_{};
};

}  // namespace autobuff::predictor

#endif  // AIMER_AUTOBUFF_PREDICTOR_SLOT_DEBOUNCER_HPP
