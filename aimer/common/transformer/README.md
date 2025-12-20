  完整数据流

  下位机数据 (Imu坐标系)          相机数据 (Camera坐标系)
  ├── vx, vy (速度)               ├── 装甲板像素位置
  ├── q_imu (姿态)                └── PnP → p_camera
  │
  └──────────────┬────────────────────────┘
                 │
                 ▼
           变换到 World 坐标系
           ├── v_world = transform_vector<Imu, World>(v_imu, q)
           └── p_world = transform_point<Camera, World>(p_cam, q)
                 │
           大地坐标系下预测
           └── 考虑双方速度、重力、弹道
                 │
                 ▼
           变换回 Barrel 坐标系
           └── p_barrel = transform_point<World, Barrel>(p_predict, q)
                 │
                 ▼
           计算云台角度 (yaw, pitch)

  TF树最终结构

  World (大地坐标系)
    │   - 预测在这里做
    │   - 速度、加速度都在这个坐标系下
    │
    └── Imu (云台坐标系)
         │   - 下位机数据的原始坐标系
         │   - vx, vy, q_imu
         │
         ├── Camera (相机坐标系)
         │       - PnP求解结果
         │
         └── Barrel (枪口坐标系)
                 - 弹道计算起点
                 - 最终输出yaw/pitch
