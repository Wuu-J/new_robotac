# Ubuntu 双系统装好后 — 完整操作步骤

> **适用**：ROBOTAC 四足机甲挑战赛 MATRiX 仿真（UE5.7 + MuJoCo + ROS2 Humble）
> **场景**：Ubuntu 22.04 已装到 Acer 2TB 盘的 150GB 分区（见《双系统安装指南_E盘2TB专用版.md》）
> **目标**：让 Ubuntu 能跑起仿真窗口、控制机器人、录 bag、回放看数据

---

## 一、你需要准备什么（清单）

### 1. 硬件与系统
| 项目 | 要求 | 说明 |
|------|------|------|
| Ubuntu 22.04 | 已安装 | 桌面版（带 GUI），装到 Acer 盘 150GB 分区 |
| NVIDIA 驱动 | **≥ 550** | 决定仿真能否运行的关键，Vulkan 必须 ≥ 1.3 |
| 网络 | 能上网 | 装依赖、下载包需要 |
| 磁盘空间 | Ubuntu 分区内 ≥ 20GB 空闲 | 项目代码约 8GB + 依赖 + bag 文件 |

### 2. 文件资料（从 Windows E 盘带过去）
| 文件 | 位置（Windows） | 大小 | 是否必需 |
|------|----------------|------|---------|
| **项目代码** | `E:\ROBOTAC\matrix_robotac_worksapce\matrix_robotac_first\` | 约 7.5GB | ✅ 必需 |
| **注意事项.md** | `E:\ROBOTAC\matrix_robotac_worksapce\注意事项.md` | 小 | ✅ 必需（红线文档，开发前必读）|
| **对话总结文档** | `E:\ROBOTAC\matrix_robotac_worksapce\ROBOTAC_对话总结_20260820-21.md` | 小 | ✅ 建议（Ubuntu 的 WorkBuddy 靠它恢复上下文）|
| 双系统安装指南 | `E:\ROBOTAC\matrix_robotac_worksapce\双系统安装指南_E盘2TB专用版.md` | 小 | ✅ 建议 |
| bag 文件 | `E:\ROBOTAC\matrix_robotac_worksapce\bags\matrix_30s\` | 235MB | ⭕ 可选（回放练习用）|
| SDK 文档 | `matrix_robotac_first\deps\zsibot_sdk\docs\` | 小 | ✅ 已含在项目代码里 |

### 3. 软件（进入 Ubuntu 后安装）
- ROS2 Humble（含 rviz2）
- python3-pip、build-essential、cmake 等编译工具
- NVIDIA 驱动 550+

---

## 二、代码怎么传进 Ubuntu（三种方式对比）

| 方式 | 速度 | 适合场景 | 复杂度 |
|------|------|---------|--------|
| **A. U 盘拷贝** ⭐推荐 | 快 | 一次性全量传输 | ⭐ 最简单 |
| **B. SCP / SFTP 局域网传** | 中 | 后续增量更新 | 中 |
| **C. 网盘/云盘下载** | 慢 | 应急 | 中 |

> 建议组合拳：**第一次用 U 盘全量拷**（7.5GB 一次到位），**之后改代码用 SCP 增量同步**。

### 方式 A：U 盘拷贝（推荐，最简单）

```
1. Windows 端：
   把 U 盘插入电脑
   将 matrix_robotac_first 整个文件夹复制到 U 盘
   （也可以先右键压缩成 zip，拷起来更快，7.5GB→约3GB）

2. Ubuntu 端：
   插入 U 盘 → 文件管理器左侧会出现 U 盘图标，点击打开
   将 matrix_robotac_first 复制到主目录（/home/你的用户名/）
```

> ⚠️ 注意：U 盘如果是 FAT32 格式，单个文件不能超过 4GB，压缩包需分卷或用 exFAT 格式 U 盘。

**推荐目录结构**（和 Windows 保持一致）：

```bash
mkdir -p ~/matrix_robotac_worksapce
# 把 U 盘里的 matrix_robotac_first 放进这个目录
# 最终结构：
# ~/matrix_robotac_worksapce/matrix_robotac_first/
# ~/matrix_robotac_worksapce/注意事项.md
# ~/matrix_robotac_worksapce/ROBOTAC_对话总结_20260820-21.md
```

### 方式 B：SCP 局域网传输（后续增量更新）

```
1. 确保 Windows 和 Ubuntu 连同一个 WiFi/局域网
2. Ubuntu 端开 SSH：
   sudo apt install openssh-server
   sudo systemctl enable --now ssh
   ip addr show | grep inet        # 记下 Ubuntu 的 IP，如 192.168.1.100

3. Windows 端 PowerShell 传输：
   # 整个目录
   scp -r E:\ROBOTAC\matrix_robotac_worksapce\matrix_robotac_first 用户名@192.168.1.100:~/matrix_robotac_worksapce/
   # 单个文件
   scp E:\ROBOTAC\matrix_robotac_worksapce\注意事项.md 用户名@192.168.1.100:~/matrix_robotac_worksapce/
```

> 更友好的图形化工具：**FileZilla**（Windows 装客户端，图形界面拖拽上传）。

### 方式 C：直接读 Windows 分区（应急，不推荐长期用）

```
双系统下，Ubuntu 可以直接挂载读 E 盘 NTFS 分区：
lsblk                          # 找到 Acer 盘的 NTFS 分区（E 盘）
sudo mkdir -p /mnt/windows_e
sudo mount -t ntfs-3g /dev/nvmeXn1pX /mnt/windows_e    # 换成实际设备名
# 然后直接 cp 到 Ubuntu 目录
cp -r /mnt/windows_e/ROBOTAC/matrix_robotac_worksapce/matrix_robotac_first ~/matrix_robotac_worksapce/
```

> ⚠️ 只读挂载最安全，不要在 Ubuntu 里写 E 盘 NTFS（有损坏风险）。

---

## 三、Ubuntu 里要做的配置（按顺序执行）

### 第 1 步：系统更新 + 常用工具

```bash
sudo apt update && sudo apt upgrade -y

sudo apt install -y vim git curl wget htop tree build-essential \
    cmake net-tools openssh-server gnome-tweaks \
    software-properties-common apt-transport-https \
    ca-certificates gnupg lsb-release python3-pip

sudo timedatectl set-timezone Asia/Shanghai
# 双系统时间同步（必做，否则 Windows/Ubuntu 时间会错乱）
sudo timedatectl set-local-rtc 1 --adjust-system-clock
```

### 第 2 步：安装 NVIDIA 驱动（关键！）

```bash
# 1. 看显卡是否识别
lspci | grep -i vga        # 应看到 NVIDIA RTX 4050

# 2. 安装 550+ 驱动
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt update
sudo ubuntu-drivers autoinstall
# 或指定版本：sudo apt install -y nvidia-driver-550

# 3. 重启
sudo reboot

# 4. 验证（重启后）
nvidia-smi                                   # 驱动版本 ≥ 550
vulkaninfo --summary | grep -E "apiVersion"  # 必须 ≥ 1.3（SM6 要求）
```

> ❌ 如果 vulkaninfo 低于 1.3 或 UeSim 又报 Vulkan 错误 → 驱动没装好，回到这一步。
> ✅ 这是之前 VMware 虚拟机失败的原因（虚拟显卡不满足 SM6），物理机 RTX 4050 没问题。

### 第 3 步：安装 ROS2 Humble

```bash
# 1. locale
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# 2. 添加 ROS2 源
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. 安装（含 rviz2）
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep

# 4. 环境变量
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 5. 验证
ros2 --version    # 应显示 humble
```

### 第 4 步：安装 MATRiX 仿真依赖

```bash
sudo apt install -y libeigen3-dev libpcl-dev libopencv-dev \
    libsuitesparse-dev libyaml-cpp-dev \
    ros-humble-pcl-conversions ros-humble-cv-bridge ros-humble-sensor-msgs
```

---

## 四、验证：启动仿真（有窗口！）

```bash
cd ~/matrix_robotac_worksapce/matrix_robotac_first

# 启动仿真 —— 窗口模式（第3个参数 0 = 不用离屏；第4个 0 = 不开 PixelStreaming）
./run_sim.sh xgb 1 0 0 0
```

**预期**：会弹出一个 UE5 窗口，看到机器狗和场地！

> 对比：之前在 AutoDL 服务器上用 `xgb 1 1 0 0`（离屏无窗口），本地物理机有显卡，用 `xgb 1 0 0 0` 就能直接看画面。

### 验证传感器

```bash
# 另开一个终端
source /opt/ros/humble/setup.bash
ros2 topic list
# 应看到：
#   /front_lidar                (激光雷达点云 10Hz)
#   /front_camera/image/compressed  (RGB 相机)
#   /front_depth/image/compressed   (深度相机)
#   /imu /odom/mujoco_odom /fire

ros2 topic hz /front_lidar    # 应约 10Hz
```

### 可选：rviz2 实时查看

```bash
rviz2
# Add → By topic → 加 /front_lidar (PointCloud2) 和 /front_camera/image/compressed (Image)
# Fixed Frame 填：lidar（点云 frame_id）
```

---

## 五、日常使用流程速查

| 目的 | 命令 |
|------|------|
| 启动仿真（窗口） | `cd ~/matrix_robotac_worksapce/matrix_robotac_first && ./run_sim.sh xgb 1 0 0 0` |
| 停止仿真 | 仿真窗口按 Q 或终端 Ctrl+C（注意 run_sim.sh 有启动锁，重启前先杀旧进程）|
| 录 bag | `ros2 bag record -a -o ~/bags/test1` → Ctrl+C 停止 |
| 回放 bag | `ros2 bag play ~/bags/test1 -r 0.5 -l`（-l 循环播放）|
| 查看 bag 内容 | `ros2 bag info ~/bags/test1` |
| 运行 SDK 控制 demo | `cd ~/matrix_robotac_worksapce/matrix_robotac_first/deps/zsibot_sdk/demo/zsl-1/python/examples && python3 highlevel_demo_Interactive.py` |
| 同步 Windows 代码 | Windows PowerShell: `scp -r E:\ROBOTAC\... 用户名@UbuntuIP:~/matrix_robotac_worksapce/` |

---

## 六、装完后检查清单（逐条打钩）

```bash
echo "=== 系统 ==="; grep PRETTY_NAME /etc/os-release
echo "=== GPU ===";  nvidia-smi | head -5
echo "=== Vulkan ==="; vulkaninfo --summary 2>&1 | grep -E "apiVersion|GPU id"
echo "=== ROS2 ===";  ros2 --version
echo "=== 项目 ===";  ls ~/matrix_robotac_worksapce/matrix_robotac_first/ | head
```

全部通过 = 仿真环境就绪 ✅

---

## ⚠️ 常见问题

| 问题 | 原因 | 解决 |
|------|------|------|
| UeSim 弹 Vulkan 错误 | 驱动没生效 | 重启 + `nvidia-smi` 验证版本 |
| vulkaninfo < 1.3 | 驱动旧 | 重装 nvidia-driver-550+ |
| 时间错乱 | 双系统 UTC 冲突 | 第 1 步的 set-local-rtc 命令 |
| 找不到 topic | 仿真没起来或没 source | 确认窗口开着 + `source /opt/ros/humble/setup.bash` |
| run_sim 启动失败 | 上次进程没退干净 | `ps -ef \| grep -E "UeSim\|mc_ctrl" \| grep -v grep` 后 kill |

---

> **最后提醒**：每次开发/改代码前，先读 `注意事项.md`（红线文档）。遇到任何报错，把终端输出截图发给我，我帮你逐条排查。
