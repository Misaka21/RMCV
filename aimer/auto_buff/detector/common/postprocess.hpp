#ifndef AIMER_AUTOBUFF_DETECTOR_COMMON_POSTPROCESS_HPP
#define AIMER_AUTOBUFF_DETECTOR_COMMON_POSTPROCESS_HPP

#include <vector>

#include "aimer/auto_buff/common/types.hpp"
#include "aimer/auto_buff/detector/common/raw_types.hpp"
#include "aimer/common/robot_state.hpp"

namespace autobuff::detector {

/**
 * @brief YOLOX 能量机关后处理器
 *
 * 将 RuneDecoder 输出的 RawRuneObject 列表转换为 BuffDetectionResult:
 *   - R 标中心: 所有检测的 r_center 置信度加权平均
 *   - 装甲板角度: atan2(-(center.y - r.y), center.x - r.x)
 *   - 槽位分配: round(angle / 72°) mod 5, 置信度高者胜
 *   - is_lit: RuneType::INACTIVATED → true (当前目标)
 */
class Postprocessor {
public:
    autobuff::BuffDetectionResult build_result(
        const std::vector<RawRuneObject>& raw_objects,
        const LetterboxMeta& meta,
        const aimer::RobotState& robot_state,
        int frame_id,
        double timestamp_s,
        autobuff::DetectorBackend backend,
        autobuff::EnemyColor filter_color = autobuff::EnemyColor::UNKNOWN
    ) const;
};

}  // namespace autobuff::detector

#endif  // AIMER_AUTOBUFF_DETECTOR_COMMON_POSTPROCESS_HPP
