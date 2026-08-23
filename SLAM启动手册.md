# FAST-LIO2 SLAM 启动手册（ROBOTAC 中期检查建图）

> 2026-08-24 实测版。平台：MATRiX 仿真（UeSim + mc_ctrl），Ubuntu 22.04 + ROS2 Humble，钢镚 L1（xgb）。

## 目录结构

| 路径 | 内容 |
|---|---|
| `slam_ws_src/FASTLIO2_ROS2/` | FAST-LIO2 + PGO 源码（SadCream fork，已适配仿真话题与外参） |
| `slam_ws_src/livox_ros_driver2/` | Livox ROS2 驱动（编译依赖，仅用其 CustomMsg 消息类型，不连真雷达） |
| `matrix_robotac_first/user_code/livox_bridge.py` | 桥接：PointCloud2→CustomMsg + IMU 加速度 m/s²→g |
| `matrix_robotac_first/user_code/lio_map_builder.py` | 累积地图（/lio_map 持久显示 + PCD 输出） |
| `matrix_robotac_first/user_code/merge_pgo_maps.py` | 合并 PGO 分块地图为提交用 PCD（纯 numpy） |
| `matrix_robotac_first/user_code/slam_lio.rviz` | rviz 配置（Fixed Frame=world，绿色累积地图+红色轨迹） |

## 一次性环境准备（本机已完成，换机重做）

```bash
# 1. apt 依赖
sudo apt install -y libceres-dev libgflags-dev libatlas-base-dev ros-humble-pcl-ros ros-humble-gtsam
# 2. Livox-SDK2（livox_ros_driver2 编译需要 liblivox_lidar_sdk_static.a）
git clone https://github.com/Livox-SDK/Livox-SDK2.git && cd Livox-SDK2
mkdir -p build && cd build && cmake .. && make -j$(nproc) && sudo make install
# 3. Sophus 1.22.10（装到本地前缀 ~/fastlio2_deps/sophus）
git clone --depth 1 --branch 1.22.10 https://github.com/strasdat/Sophus.git && cd Sophus
mkdir -p build && cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON -DCMAKE_INSTALL_PREFIX=~/fastlio2_deps/sophus
make -j$(nproc) && make install
# 4. livox_ros_driver2 编译（本仓库已含源码 slam_ws_src/livox_ros_driver2）
mkdir -p ~/livox_ws/src && cp -r slam_ws_src/livox_ros_driver2 ~/livox_ws/src/
cd ~/livox_ws/src/livox_ros_driver2 && source /opt/ros/humble/setup.bash && ./build.sh humble
# 5. FASTLIO2_ROS2 编译（本仓库已含源码与配置修改 slam_ws_src/FASTLIO2_ROS2）
mkdir -p ~/fastlio2_ws/src && cp -r slam_ws_src/FASTLIO2_ROS2 ~/fastlio2_ws/src/
cd ~/fastlio2_ws && source /opt/ros/humble/setup.bash && source ~/livox_ws/install/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/home/stylerobotac/fastlio2_deps/sophus \
  -DCMAKE_CXX_FLAGS=-I/home/stylerobotac/fastlio2_deps/sophus/include
```

## 每次开机启动流程（6 个终端）

**终端 1 — 仿真**（笔记本双显卡必须带 PRIME offload 前缀）：
```bash
cd ~/matrix_robotac_workspace/matrix_robotac_first && __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia __VK_LAYER_NV_optimus=NVIDIA_only ./run_sim.sh xgb 1 0 0 0
```
另开终端确认就绪（看到 /front_lidar 和 /imu 两行）：
```bash
source /opt/ros/humble/setup.bash && ros2 topic list | grep -E 'front_lidar|/imu'
```

**终端 2 — 桥接**：
```bash
source /opt/ros/humble/setup.bash && source ~/livox_ws/install/setup.bash && cd ~/matrix_robotac_workspace/matrix_robotac_first/user_code && python3 livox_bridge.py
```

**终端 3 — PGO 完整 SLAM**（LIO + 回环，自带回环图 rviz）：
```bash
source /opt/ros/humble/setup.bash && source ~/fastlio2_ws/install/setup.bash && ros2 launch pgo pgo_launch.py
```

**终端 4 — 累积地图**：
```bash
source /opt/ros/humble/setup.bash && cd ~/matrix_robotac_workspace/matrix_robotac_first/user_code && python3 lio_map_builder.py --out ~/robotac_maps/lio_map.pcd
```

**终端 5（可选）— 绿色地图 rviz**：
```bash
source /opt/ros/humble/setup.bash && rviz2 -d /home/stylerobotac/matrix_robotac_workspace/matrix_robotac_first/user_code/slam_lio.rviz
```

**终端 6 — 遥控**（自动站立，WASD 动 / Space 急停 / L 趴下 / Q 退出）：
```bash
source /opt/ros/humble/setup.bash && cd ~/matrix_robotac_workspace/matrix_robotac_first/user_code && python3 teleop_keyboard.py
```

## 回环验证要点

- 走**闭合路线**（如沿外围一圈）回到出生点 **1m 以内**，距上次路过 >60 秒
- 转向 ≤0.4 rad/s（快转圈航向欠转，压力测试见下）
- 回环成功信号：终端 3 rviz 出现绿色边；或 `ros2 topic echo /pgo/loop_markers --once`

## 保存与合并（建图完成后）

```bash
source /opt/ros/humble/setup.bash && source ~/fastlio2_ws/install/setup.bash
mkdir -p ~/robotac_maps/final_map
ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: '/home/stylerobotac/robotac_maps/final_map', save_patches: true}"
python3 ~/matrix_robotac_workspace/matrix_robotac_first/user_code/merge_pgo_maps.py /home/stylerobotac/robotac_maps/final_map /home/stylerobotac/robotac_maps/final_map/merged.pcd
```

## ⚠️ 实测踩坑速查（2026-08-24）

| 坑 | 处理 |
|---|---|
| 仿真重启后 LIO 发散（位置跳到几千） | 终端 3 **Ctrl+C 重跑**（LIO 无数据期间状态漂坏；桥接无状态不用重启） |
| odom 实际 ~460Hz 非文档 10Hz | 缓冲区/TF 节流相关设计已按高频处理 |
| lidar 头戳滞后 odom ~11-20ms（采集管线延迟） | 语义正常，按头戳对齐即可，无需补偿 |
| 仿真 lidar 帧为"整帧冻结"快照 | 桥接 offset_time=0（勿开 --sweep-ms，实测开假扫掠反而引入假畸变） |
| 快转圈（1 rad/s+）LIO 航向欠转 1-11% | 状态相关退化（无回环 LIO 弱点）；任务速度 ≤0.4 rad/s 影响小 |
| fork 只订阅 livox CustomMsg、IMU 加速度 ×10（按 g 单位设计） | livox_bridge.py 已处理这两个兼容问题 |
| pgo.yaml 改动后需同步到 install/ | `cp src/.../pgo.yaml install/.../pgo.yaml` |
| 长命令粘贴断行报错 | 整行一次粘贴 |
| 一次只跑一个 SDK 客户端（43988 互斥） | teleop / 自研控制脚本二选一 |

## 比赛红线提醒

- PCD 必须为算法原生输出：PGO patches + 优化位姿合并 = 算法管线内输出，禁止对已保存 PCD 事后加工
- 视频一镜到底禁剪辑（mp4 ≤15min ≤500MB）
