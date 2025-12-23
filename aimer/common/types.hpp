//
// 公共类型定义 - 数据流结构
// 定义各层级间传递的消息类型
//

#ifndef AIMER_COMMON_TYPES_HPP
#define AIMER_COMMON_TYPES_HPP

#include "aimer/common/robot_state.hpp"
#include "aimer/auto_aim/common/types.hpp"  // DetectedArmor, DetectionResult 等

namespace aimer {

// 导出常用类型到 aimer 命名空间 (方便使用)
using autoaim::DetectedArmor;
using autoaim::DetectionResult;
using autoaim::EnemyColor;
using autoaim::ArmorType;
using autoaim::ArmorNumber;

}  // namespace aimer

#endif  // AIMER_COMMON_TYPES_HPP
