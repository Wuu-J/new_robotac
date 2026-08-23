# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

第二十五届全国大学生机器人大赛 **ROBOTAC 四足机甲挑战赛** 参赛项目。核心代码在 `matrix_robotac_first/`，基于官方 MATRiX 仿真平台（MuJoCo + Unreal Engine 5 + CARLA），环境为 Ubuntu 22.04 + ROS2 Humble，机器人固定使用「钢镚 L1」（xgb 型号）。

- 三大任务：任务一 SLAM 迷宫建图（导出**原生 PCD**，30 分）、任务二 路径规划夺宝箱（30 分，联动任务三弹药）、任务三 实机 AprilTag 自动打击（40 分）。
- 中期检查硬 deadline：**2026-09-12 前**提交（PCD + 一镜到底演示视频），决赛 10 月中旬（天津）。
- 本工作区为 git 仓库（远程 `github.com/Wuu-J/robotac`，私有）：仅跟踪本队代码（`user_code/`）、文档与 PCD；官方平台（7.6GB）被 `.gitignore` 排除。无测试/CI/lint，修改前自行备份。
- 工作区根目录的其他内容（中文 md 手册、`rules.pdf` 赛题、`bags/`、`.workbuddy/`）是项目文档与另一工具（WorkBuddy）的产物，不是 Claude Code 的。

## 红线工作约定（用户明确要求，最高优先级）

1. **每次生成或修改任何代码前，先 Read `注意事项.md`**（工作区根目录）。它是用户整理的踩坑汇总手册（比赛规则、建图评分细则、SDK 部署/API 陷阱、Top 20 速查表），以它为准。
2. **不确定就先问用户，不要自己假设**：SDK API 用法、参数范围、Topic 名称、IP/端口、比赛规则、评分阈值、配置文件含义——任何不确定点都要先向用户确认再写代码。

背景文档（按需阅读）：`ROBOTAC_对话总结_20260820-21.md`（踩坑与决策史）、`ROS2_bag录制回放指南.md`（云端 bag 回放链路）、`Ubuntu装好后完整步骤_准备与代码传入.md`（双系统装好后的操作步骤）。

## 常用命令

全部在 `matrix_robotac_first/` 下执行，并需先 `source /opt/ros/humble/setup.bash`（否则报 `libfastcdr.so.1` 缺失）。

### 启动仿真（主入口）

```bash
./run_sim.sh [robot] [scene_id] [offscreen] [pixelstream] [mujoco_running]
# 本地带窗口：./run_sim.sh xgb 1 0 0 0
# 云端/无头：  ./run_sim.sh xgb 1 1 0 0
```

- 笔记本双显卡（Optimus）必须用 PRIME offload 走独显：
  `__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia __VK_LAYER_NV_optimus=NVIDIA_only ./run_sim.sh xgb 1 0 0 0`
- 一次只启动一个实例（有 flock 启动锁，且脚本启动前会 pkill 旧进程）。
- `MUJOCORUNNING=1` 才启用 MuJoCo 物理（需 license），默认关闭。

### 安装依赖

```bash
./scripts/build.sh          # 只装系统依赖 + dpkg -i deps/*.deb；不下载 UeSim、不编译 MC/MuJoCo
./scripts/download_uesim.sh # 从 Google Drive 下载 UeSim（国内网络可能不通）
```

README 与 Dockerfile 中提到的 `./build.sh`、`./open_sim_launcher` **不存在**（文档错误，docker build 会失败），直接裸机部署即可。

### SDK 控制示例（验证控制链路）

```bash
python3 deps/zsibot_sdk/demo/zsl-1/python/examples/highlevel_demo_Interactive.py
```

### bag 录制 / 回放（云端无头仿真时看数据的方式）

```bash
ros2 bag record /front_lidar /front_camera/image/compressed /front_depth/image/compressed /imu /odom/mujoco_odom -o ~/bags/xxx
ros2 bag play ~/bags/xxx -l     # -l 循环；示例 bag 在 bags/matrix_30s/
rviz2                            # 用 rviz/matrix.rviz（Fixed Frame 已设 lidar，bag 里没有 TF）
```

## 架构

`run_sim.sh` 编排启动三个组件（缺一不可的运行时结构）：

1. `src/UeSim/Linux/UeSim.sh` — UE5 仿真（渲染 + 传感器），把传感器数据发布到 ROS2 Topic。UE 启动后脚本固定 sleep 5 再起下一组件。
2. `src/robot_mc/run_mc.sh` → `build/export/mc/bin/mc_ctrl r` — 运动控制进程（taskset 绑 CPU7），即 SDK 服务端（仿真 IP 127.0.0.1，端口 43988）。
3. `src/robot_mujoco/simulate/build/robot_mujoco` — MuJoCo 物理引擎（可选，默认关闭）。

关键数据流与配置：

- `config/config.json` 定义传感器（RGB 1080p、深度 640×480、Mid360 激光雷达）与 Topic。`run_sim.sh` 用 sed **就地改写**多个配置（config.json、simulate/config.yaml、run_mc.sh、xg-user-parameters.yaml），并把 config.json 拷进 UeSim 的 Content/model/config/ 目录。反复运行会累积脏配置。
- 用户控制代码通过 `deps/zsibot_sdk`（C++/Python，库在 `lib/zsl-1/{arch}`）连接 43988 端口控制机器狗。**`include/zsl-1/highlevel.h` 是 API 函数名的唯一权威来源**，SDK 文档（`docs/`）部分函数名是错的。
- 机器人/场景模型 XML：`src/robot_mujoco/zsibot_robots/xgb/`（含 unreal.xml，出生点 z 硬编码 0.65）。
- 仿真内通信用 Zenoh 协议；切真机需改机器人端 `src/robot_mc/build/export/config/sdk_config.yaml` 的 `target_ip` 并重启设备（真机 WiFi 192.168.234.1 / 有线 192.168.168.168）。

仿真传感器 Topic（均 10Hz，除 IMU）：`/front_camera/image/compressed`、`/front_depth/image/compressed`（CompressedImage，**深度只是压缩图像不是点云**，cloudmode:false）、`/front_lidar`（PointCloud2，frame_id=`lidar`，**无 TF 发布**）、`/imu`、`/odom/mujoco_odom`。

## 关键约束速查（完整版见 注意事项.md）

- SDK 函数名以 `highlevel.h` 为准：`standUp` / `checkConnect` / `cancelTwoLegStand`（照文档写 `standUP` 等会编译失败）。
- move 速度死区：vx ±0.05~3 m/s、vy ±0.1~1 m/s、yaw ±0.02~3 rad/s；停止必须传 0，超限返回错误码 0x3013。
- 状态机：移动中不可直接切 standUp/lieDown/jump 等动作，先 `move(0,0,0)` 停稳。
- **3 秒无 SDK 数据 → 机器人自动趴下**：控制循环间隔必须 < 3s。
- 仿真 initRobot 固定 `("127.0.0.1", 43988, "127.0.0.1")`（本地 IP、本地端口、机器人 IP）。
- `run_sim.sh` 只支持 xgb（其他参数直接 exit 1）；SCENE_ID 0/1/2 全部映射 RobotAC_Level_2。
- GPU：`DefaultEngine.ini` 写死 Vulkan 1.3 + SM6（`SF_VULKAN_SM6`）——VMware 虚拟机跑不了，需 NVIDIA 独显物理机或云 GPU；PixelStreaming 已证实不可用；缺 `libmujoco.so.3.3.0` 时在 MuJoCoUE 插件目录建符号链接。
- 比赛规则红线：PCD 禁止任何后处理（直接交算法原始输出）；视频一镜到底禁剪辑（mp4 ≤15min ≤500MB）；任务三禁止人工瞄准/控制。
- 实机：电量低会直接断电；SDK 运行中手柄失效；程序/运控更新后 sdk_config 配置会重置。
