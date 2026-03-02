#ifndef AIMER_AUTOBUFF_DETECTOR_COMMON_POSTPROCESS_HPP
#define AIMER_AUTOBUFF_DETECTOR_COMMON_POSTPROCESS_HPP

#include <vector>

#include "aimer/auto_buff/common/types.hpp"
#include "aimer/auto_buff/detector/common/raw_types.hpp"
#include "aimer/common/robot_state.hpp"

namespace autobuff::detector {

/**
 * @brief YOLO 后处理器
 *
 * 将 Sp25Decoder 输出的 RawBuffObject 列表转换为 BuffDetectionResult：
 *   - R 标中心：由所有检测中 kpt[5] (内侧尖端) 外推
 *   - 扇叶角度：atan2(-(center.y - r.y), center.x - r.x)（Y 轴翻转像素坐标系）
 *   - 槽位分配：round(angle / (2π/5)) mod 5，置信度高者获胜
 *   - 所有检测均设 is_lit=true（YOLO 只看到点亮扇叶）
 */
class Postprocessor {
public:
    /**
     * @brief 构建最终检测结果
     * @param raw_objects  解码器输出（已还原到原图坐标）
     * @param meta         Letterbox 元数据（本函数不再使用，坐标已还原）
     * @param robot_state  当前串口自身状态
     * @param frame_id     帧号
     * @param timestamp_s  时间戳（秒）
     * @param backend      后端类型标识
     * @return BuffDetectionResult
     */
    autobuff::BuffDetectionResult build_result(
        const std::vector<RawBuffObject>& raw_objects,
        const LetterboxMeta& meta,
        const aimer::RobotState& robot_state,
        int frame_id,
        double timestamp_s,
        autobuff::DetectorBackend backend
    ) const;
};

}  // namespace autobuff::detector

#endif  // AIMER_AUTOBUFF_DETECTOR_COMMON_POSTPROCESS_HPP
