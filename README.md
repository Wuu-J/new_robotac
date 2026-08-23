# ROBOTAC 四足机甲挑战赛（钢镚 L1 / xgb）

第二十五届全国大学生机器人大赛 ROBOTAC 四足机甲挑战赛参赛代码仓库。
平台：MATRiX 仿真（MuJoCo + UE5 + CARLA），Ubuntu 22.04 + ROS2 Humble。

## 目录结构

```
├── matrix_robotac_first/user_code/   # 本队全部代码
│   ├── teleop_keyboard.py            # 键盘遥控（调试用）
│   ├── slam_odom_mapper.py           # 建图节点：纯 odom 配准 + z过滤 + 限距
│   │                                 #   → /slam_map（PCD）、/map_2d（占据栅格）、TF
│   ├── auto_explore.py               # 自主遍历 v2：左手规则墙跟随
│   │                                 #   （窄前扇区+多帧确认+PD阻尼+弧线转弯）
│   ├── frontier_explore.py           # Frontier 前沿探索（Yamauchi 1997）
│   │                                 #   （/map_2d + A* + 纯追踪，系统化覆盖全场）
│   ├── slam_view.rviz                # rviz 配置（Fixed Frame=world）
│   └── *.pcd                         # 测试建图产物
├── 注意事项.md                        # ⭐ 踩坑汇总手册（红线：改代码前必读）
├── CLAUDE.md                          # 项目与工作约定
├── SLAM算法升级方案.md
├── ROS2_bag录制回放指南.md
├── ROBOTAC_对话总结_20260820-21.md
├── 双系统安装指南_*.md
├── Ubuntu装好后完整步骤_准备与代码传入.md
└── rules.pdf                          # 赛题说明（PDF 全 25 页）
```

## 快速开始

1. 部署环境与启动仿真：见 `注意事项.md` 2.1/2.9/2.11（含双显卡 PRIME offload、SDK 三件套）。
2. 启动建图节点：
   ```bash
   cd matrix_robotac_first/user_code
   source /opt/ros/humble/setup.bash
   python3 slam_odom_mapper.py --out map.pcd --save-after 1800
   ```
3. 自主遍历（左手规则）或前沿探索（系统覆盖）：
   ```bash
   python3 auto_explore.py       # 墙跟随 v2
   python3 frontier_explore.py   # frontier 探索（需建图节点运行）
   ```

⚠️ 开发红线：每次修改代码前先读 `注意事项.md`；PCD 必须为算法原生输出，禁止任何后处理。

> 本仓库只包含本队代码与文档；官方 MATRiX 仿真平台本体（`matrix_robotac_first/src/`、`deps/` 等）
> 体积约 7.6GB 未上传，请从官方渠道获取。
