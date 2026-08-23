# ROS2 bag 录制回放指南（AutoDL 云端仿真）

> 2026-08-20 生成。云端仿真无头运行（PixelStreaming 有底层兼容问题），改用「服务器录制 bag → 本地回放可视化」的方式看数据。

## 一、已完成的云端录制

- **位置**：服务器 `/home/ubuntu/bags/matrix_30s`（已下载到本地 `E:\ROBOTAC\matrix_robotac_worksapce\bags\matrix_30s\`）
- **时长**：30 秒，225MB
- **内容**：

| Topic | 类型 | 帧数 |
|-------|------|------|
| /front_lidar | sensor_msgs/msg/PointCloud2 | 296（10Hz）|
| /front_camera/image/compressed | sensor_msgs/msg/CompressedImage | 296 |
| /front_depth/image/compressed | sensor_msgs/msg/CompressedImage | 296 |
| /imu | sensor_msgs/msg/Imu | 14766 |
| /odom/mujoco_odom | nav_msgs/msg/Odometry | 14760 |

## 二、服务器端录制命令（以后想录新数据用）

```bash
# 登录服务器
ssh ubuntu@117.50.221.86

# 录制 30 秒核心传感器数据
source /opt/ros/humble/setup.bash
mkdir -p ~/bags
timeout 30 ros2 bag record /front_lidar /front_camera/image/compressed \
  /front_depth/image/compressed /imu /odom/mujoco_odom \
  -o ~/bags/matrix_30s
```

> 提示：想让机器狗动起来再录，先运行 SDK demo 控制机器狗移动，再开录制。录制时长建议 30-120 秒（控制文件大小）。

## 三、本地回放步骤（VMware Ubuntu 虚拟机）

### 1. 装 ROS2 Humble（如果虚拟机里还没装）

```bash
sudo apt update
sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop   # 含 rviz2 + rosbag2
source /opt/ros/humble/setup.bash
```

### 2. 访问 bag 文件

bag 已下载到 `E:\ROBOTAC\matrix_robotac_worksapce\bags\matrix_30s\`
虚拟机里通过共享文件夹访问（**挂载路径以实际为准，本机实测是 `/mnt/hgfs/matrix_ws`**）：

```bash
ls /mnt/hgfs/matrix_ws/bags/matrix_30s/
# 应看到 matrix_30s_0.db3 和 metadata.yaml
```

> 如果共享文件夹还没挂载：VMware 菜单「虚拟机 → 设置 → 选项 → 共享文件夹」添加 `E:\ROBOTAC\matrix_robotac_worksapce`。

### 3. 回放数据

```bash
source /opt/ros/humble/setup.bash

# 先看 bag 信息（注意用实际挂载路径）
ros2 bag info /mnt/hgfs/matrix_ws/bags/matrix_30s

# 回放（-r 0.5 是半速，方便观察；去掉 -r 为正常速度）
ros2 bag play /mnt/hgfs/matrix_ws/bags/matrix_30s -r 0.5
```

### 4. rviz2 可视化（另开一个终端）

```bash
source /opt/ros/humble/setup.bash
rviz2
```

在 rviz2 里：

1. **Global Options → Fixed Frame**：这个 bag **没有 /tf 数据**，把 Fixed Frame 设成点云的 frame_id（先用下面命令查）：
   ```bash
   ros2 topic echo /front_lidar --once | grep frame_id
   ```
   一般是 `base_link` 或 `front_lidar`，直接试这两个即可（点云是 Lidar 自身坐标系，选它自己的 frame 不需要 TF）。

2. 添加显示器：

| 显示类型 | Topic | 说明 |
|---------|-------|------|
| PointCloud2 | /front_lidar | 激光点云（点大小调到 2-3 更明显）|
| Image | /front_camera/image/compressed | 前视 RGB 图像，**Transport Hint 选 compressed** |
| Image | /front_depth/image/compressed | 深度图像（伪彩色）|

> 提示：点云如果显示为单色直线，说明只看到部分扫描线，把 PointCloud2 显示的 Size 调大、Decay Time 设 0；图像空白就把 Image 显示的 Transport Hint 从 raw 改成 compressed。

### 5. 常见问题

| 问题 | 解决 |
|------|------|
| rviz2 里点云不显示 | 把 Global Options → Fixed Frame 改成 `map` 或 `base_link` |
| Image 显示空白 | 选 Image 显示器后，Transport Hint 选 `compressed` |
| 回放速度太慢/太快 | `ros2 bag play -r 0.5`（半速）/ `-r 2`（双倍速）|
| bag 里没有 TF | 如果显示坐标系错误，先 `ros2 bag play` 完整 bag，或手动添加 TF |

## 四、替代方案（实时连接，暂不推荐）

- **本地 ROS2 直连云端**：两端设置相同 `ROS_DOMAIN_ID` + SSH 隧道转发 DDS 端口（7777 UDP 等）。配置繁琐、延迟高，**先用 bag 回放即可**。
- **服务器直接跑 rviz2**：服务器无显示器，需 xvfb + X 转发（`ssh -X`），图像传输慢，不推荐。

## 五、录视频建议（中期检查用）

中期检查要交演示视频。方案：
1. 服务器录制 bag（含机器狗运动）→ 下载 → 本地 rviz2 回放时**录屏**（OBS/系统录屏）
2. 或者后续尝试修好 PixelStreaming 直接浏览器录屏（当前不可用）
3. 视频要求：mp4/H.264、≤15 分钟、≤500MB、**一镜到底禁剪辑**
