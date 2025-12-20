//
// 检测器类型定义 - 转发到 auto_aim/common/types.hpp
//

#ifndef AIMER_AUTOAIM_DETECTOR_TYPES_HPP
#define AIMER_AUTOAIM_DETECTOR_TYPES_HPP

#include "aimer/auto_aim/common/types.hpp"

namespace autoaim::detector {

// 从 autoaim 命名空间导入类型
using autoaim::EnemyColor;
using autoaim::ArmorType;
using autoaim::ArmorNumber;
using autoaim::DetectedArmor;

using autoaim::armor_type_to_string;
using autoaim::armor_number_to_string;
using autoaim::string_to_armor_number;
using autoaim::correct_armor_type;

using autoaim::SMALL_ARMOR_WIDTH;
using autoaim::SMALL_ARMOR_HEIGHT;
using autoaim::LARGE_ARMOR_WIDTH;
using autoaim::LARGE_ARMOR_HEIGHT;

}  // namespace autoaim::detector

#endif  // AIMER_AUTOAIM_DETECTOR_TYPES_HPP
