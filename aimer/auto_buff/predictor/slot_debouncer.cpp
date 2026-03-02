// SlotDebouncer 实现 (2026)

#include "slot_debouncer.hpp"

#include "plugin/param/runtime_parameter.hpp"

namespace autobuff::predictor {

SlotDebouncer::Output SlotDebouncer::update(
    const autobuff::BuffDetectionResult& det, double timestamp)
{
    // 在使用点直接读取运行时参数 (禁止缓存)
    int on_frames = static_cast<int>(
        runtime_param::get_param<int64_t>("AutoBuff.Predictor.Debounce.on_frames"));
    int off_frames = static_cast<int>(
        runtime_param::get_param<int64_t>("AutoBuff.Predictor.Debounce.off_frames"));
    double missing_timeout =
        runtime_param::get_param<double>("AutoBuff.Predictor.Debounce.missing_timeout");

    if (on_frames <= 0) on_frames = 3;
    if (off_frames <= 0) off_frames = 4;
    if (missing_timeout <= 0.0) missing_timeout = 0.12;

    Output out;

    for (int i = 0; i < NUM_SLOTS; ++i) {
        auto& fsm = fsm_[i];
        const auto& tgt = det.targets[i];

        // 超时强制 OFF
        if (fsm.state != State::OFF && fsm.last_seen > 0.0 &&
            (timestamp - fsm.last_seen) > missing_timeout)
        {
            fsm.state = State::OFF;
            fsm.count = 0;
        }

        bool detected = tgt.valid;
        bool lit = detected && tgt.is_lit;

        State prev = fsm.state;

        if (detected) {
            fsm.last_seen = timestamp;
        }

        switch (fsm.state) {
        case State::OFF:
            if (lit) {
                fsm.state = State::CANDIDATE_ON;
                fsm.count = 1;
            }
            break;
        case State::CANDIDATE_ON:
            if (lit) {
                fsm.count++;
                if (fsm.count >= on_frames) {
                    fsm.state = State::ON;
                }
            } else {
                fsm.state = State::OFF;
                fsm.count = 0;
            }
            break;
        case State::ON:
            if (!detected) {
                fsm.state = State::CANDIDATE_OFF;
                fsm.count = 1;
            } else if (!lit) {
                fsm.state = State::CANDIDATE_OFF;
                fsm.count = 1;
            }
            break;
        case State::CANDIDATE_OFF:
            if (!detected || !lit) {
                fsm.count++;
                if (fsm.count >= off_frames) {
                    fsm.state = State::OFF;
                    fsm.count = 0;
                }
            } else {
                // 重新检测到亮 -> 回到 ON
                fsm.state = State::ON;
                fsm.count = 0;
            }
            break;
        }

        StableSlot& slot = out.slots[i];
        slot.valid = (fsm.state == State::ON || fsm.state == State::CANDIDATE_OFF);
        slot.is_lit = (fsm.state == State::ON);
        slot.state_changed = (fsm.state != prev);
        slot.confidence = detected ? tgt.confidence : 0.f;

        if (slot.is_lit) {
            out.lit_mask |= static_cast<uint8_t>(1u << i);
            out.lit_count++;
        }
    }

    return out;
}

void SlotDebouncer::reset() {
    for (auto& fsm : fsm_) {
        fsm = SlotFsm{};
    }
}

}  // namespace autobuff::predictor
