/**
 * @file enemy_model_factory.cpp
 * @brief 模型工厂实现
 */

#include "enemy_model.hpp"
#include "vehicle/vehicle_model.hpp"
#include "outpost/outpost_model.hpp"
#include "base/base_model.hpp"

namespace autoaim::predictor {

EnemyModelFactory::EnemyModelFactory() {
    // 注册模型创建函数
    model_map_ = {
        { ModelType::VEHICLE,
          [](int target_id, EnemyType enemy_type) {
              return std::make_unique<VehicleModel>(target_id, enemy_type);
          }
        },
        { ModelType::OUTPOST,
          [](int target_id, EnemyType enemy_type) {
              return std::make_unique<OutpostModel>(target_id, enemy_type);
          }
        },
        { ModelType::BASE,
          [](int target_id, EnemyType enemy_type) {
              return std::make_unique<BaseModel>(target_id, enemy_type);
          }
        },
    };
}

std::unique_ptr<EnemyModelInterface> EnemyModelFactory::create(int target_id, EnemyType enemy_type) {
    ModelType model_type = get_model_type(enemy_type);

    auto it = model_map_.find(model_type);
    if (it != model_map_.end()) {
        return it->second(target_id, enemy_type);
    }

    // 默认返回 VehicleModel
    return std::make_unique<VehicleModel>(target_id, enemy_type);
}

}  // namespace autoaim::predictor
