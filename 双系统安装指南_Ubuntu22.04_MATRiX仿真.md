# Ubuntu 22.04 双系统安装 + 驱动配置完整指南

> **适用场景**：ROBOTAC 四足机甲挑战赛 — MATRiX 仿真平台（UE5.7 + MuJoCo + ROS2 Humble）  
> **目标硬件**：华硕笔记本（AMD Ryzen 7 7735H + NVIDIA RTX 4050 Laptop 6GB）  
> **安装目标**：Windows 11 + Ubuntu 22.04 双系统，Ubuntu 用于运行 MATRiX 仿真  
> **前置条件**：当前 Windows 系统正常运行，硬盘有 ≥100GB 可用空间

---

## 目录

1. [前置准备与备份](#1-前置准备与备份)
2. [分区方案设计](#2-分区方案设计)
3. [制作 Ubuntu 启动盘](#3-制作-ubuntu-启动盘)
4. [BIOS/UEFI 设置](#4-biosuefi-设置)
5. [安装 Ubuntu 22.04](#5-安装-ubuntu-2204)
6. [NVIDIA 驱动安装（关键步骤）](#6-nvidia-驱动安装关键步骤)
7. [ROS2 Humble 安装](#7-ros2-humble-安装)
8. [MATRiX 环境验证](#8-matrix-环境验证)
9. [双系统日常使用](#9-双系统日常使用)
10. [常见问题排查](#10-常见问题排查)

---

## 1. 前置准备与备份

### 1.1 必备工具清单

| 工具                   | 用途            | 获取方式                                      |
| -------------------- | ------------- | ----------------------------------------- |
| U 盘（≥8GB）            | 制作 Ubuntu 启动盘 | 任意品牌，建议 USB 3.0                           |
| Rufus / Ventoy       | 写入 ISO 到 U 盘  | <https://rufus.ie> / <https://ventoy.net> |
| Ubuntu 22.04 LTS ISO | 安装镜像          | <https://ubuntu.com/download/desktop>     |
| 外接硬盘或云存储             | 备份重要数据        | 百度网盘 / OneDrive / 移动硬盘                    |

### 1.2 数据备份（必须执行）

**⚠️ 分区操作有风险，务必先备份以下内容：**

```bash
# 在 Windows 下检查需要备份的内容
# 1. 桌面文件、文档、下载
# 2. 项目代码（E:\ROBOTAC\matrix_robotac_worksapce）
#   → 建议用 Git 推送到远程仓库
# 3. 浏览器书签、配置
# 4. SSH 密钥、开发工具配置
```

### 1.3 检查磁盘空间

```powershell
# 在 PowerShell 中执行（管理员权限）
# 查看磁盘分区情况
diskpart
list disk
list volume
exit

# 确认 C 盘剩余空间 ≥ 150GB（给 Ubuntu 预留）
```

---

## 2. 分区方案设计

### 2.1 推荐分区表（256GB SSD 示例）

| 分区                 | 大小           | 文件系统      | 用途                            |
| ------------------ | ------------ | --------- | ----------------------------- |
| Windows C:         | 120-150GB    | NTFS      | Windows 系统 + 软件               |
| Windows D:（可选）     | 剩余空间         | NTFS      | Windows 数据存储                  |
| EFI 分区（已有）         | 300MB-500MB  | FAT32     | 启动引导（已存在）                     |
| **Ubuntu 根分区 `/`** | **80-100GB** | **ext4**  | **Ubuntu 系统 + ROS2 + MATRiX** |
| **Ubuntu Swap**    | **8-16GB**   | **swap**  | **交换空间（= 内存大小）**              |
| 共享数据区（可选）          | 剩余           | NTFS/ext4 | 双系统共享文件                       |

### 2.2 使用 Windows 自带工具缩容分区

```powershell
# 步骤 1：打开磁盘管理
# 右键「此电脑」→ 管理 → 磁盘管理

# 步骤 2：选择要缩容的分区（通常是 C: 盘）
# 右键 C: → 压缩卷

# 步骤 3：输入压缩空间量（单位 MB）
# 建议：预留 102400 MB（100GB）给 Ubuntu
# 如果空间紧张，最少也要 80GB（81920 MB）

# 步骤 4：压缩后会出现「未分配」黑色区域
# 这块区域后续用于安装 Ubuntu
```

**⚠️ 注意事项：**

- 压缩前先清理 C 盘临时文件（`cleanmgr`），释放更多空间
- 如果压缩后「未分配」区域不连续，可能需要用第三方工具（如 AOMEI Partition Assistant）合并
- **不要删除任何现有分区**，只做压缩

---

## 3. 制作 Ubuntu 启动盘

### 3.1 下载 Ubuntu 22.04 LTS

```bash
# 官方下载地址（推荐国内镜像加速）
# 清华源：
https://mirrors.tuna.tsinghua.edu.cn/ubuntu-releases/22.04.5/
# 选择：ubuntu-22.04.5-desktop-amd64.iso

# 阿里云源：
https://mirrors.aliyun.com/ubuntu-releases/22.04.5/
```

### 3.2 使用 Rufus 制作启动盘（推荐）

```
操作步骤：
1. 插入 U 盘（会格式化，提前备份 U 盘内重要文件）
2. 打开 Rufus
3. 设备：选择你的 U 盘
4. 引导类型选择：点击「选择」，选中下载的 ubuntu-22.04.5-desktop-amd64.iso
5. 分区类型：GPT（如果你的电脑是 UEFI 启动）
   - 如何判断？看磁盘管理里是否有「EFI 系统分区」
   - 现代电脑（2015 年后）基本都是 UEFI + GPT
6. 目标系统类型：UEFI（非 CSM）
7. 文件系统：FAT32（默认）
8. 点击「开始」
9. 弹出警告选「以 ISO 模式写入」（推荐）
10. 等待完成（约 5-10 分钟）
```

### 3.3 使用 Ventoy 制作启动盘（更灵活）

```
Ventoy 优势：U 盘可以放多个 ISO，以后重装系统不用重新制作
1. 下载 Ventoy：https://github.com/ventoy/Ventoy/releases
2. 解压后运行 Ventoy2Disk.exe
3. 选择 U 盘 → 安装（会清空 U 盘）
4. 把 ubuntu-22.04.5-desktop-amd64.iso 直接复制到 U 盘根目录
5. 完成！启动时从 Ventoy 菜单选择 ISO 即可
```

---

## 4. BIOS/UEFI 设置

### 4.1 进入 BIOS 的方法

| 品牌 | 进入方式                                                 |
| -- | ---------------------------------------------------- |
| 华硕 | 开机按 **F2** 或 **Del**（部分型号 F2 进 BIOS，Del 进 Boot Menu） |

### 4.2 关键设置项

```
进入 BIOS 后，找到并修改以下选项：

1. Secure Boot（安全启动）
   → 改为 Disabled（关闭）
   - 原因：Ubuntu 默认签名可能与 Secure Boot 冲突
   - 位置：Security → Secure Boot → Disable

2. Fast Boot / Quick Boot（快速启动）
   → 改为 Disabled（关闭）
   - 原因：避免跳过外设检测导致 U 盘无法识别

3. Launch CSM（兼容性支持模块）
   → 保持 Disabled（如果用 UEFI/GPT 方式安装）
   - 或者 Enabled（如果遇到启动问题）

4. SATA Mode（SATA 模式）
   → 保持 AHCI（默认即可）

5. 显卡设置（如果有独显切换选项）
   → 设置为「强制使用独立显卡」或「混合模式」
   - 原因：确保 Ubuntu 能正确识别 RTX 4050
```

### 4.3 设置启动顺序

```
1. 找到 Boot 菜单（Boot → Boot Option Priorities）
2. 把 USB HDD / U 盘设备移到第一位
3. 保存退出（F10 或 Save & Exit）
```

---

## 5. 安装 Ubuntu 22.04

### 5.1 从 U 盘启动

```
1. 插入制作好的 Ubuntu 启动盘
2. 重启电脑
3. 快速按 F12（华硕）调出 Boot Menu
4. 选择你的 U 盘设备（通常显示为 USB HDD 或 U 盘品牌名）
5. 进入 GRUB 菜单，选择「Try or Install Ubuntu」
```

### 5.2 安装前的准备

```
1. 进入 Live 桌面后，先测试基本功能
   - 能看到桌面吗？（确认显卡基本可用）
   - 键盘鼠标正常吗？
   - 网络连接了吗？（右上角网络图标）

2. （可选但推荐）打开终端测试 GPU 信息
   按 Ctrl+Alt+T 打开终端，输入：
   lspci | grep -i vga
   # 应该能看到 NVIDIA RTX 4050 字样
```

### 5.3 开始正式安装

```
1. 双击桌面上的「Install Ubuntu 22.04 LTS」图标
2. 选择语言：中文（简体）或 English（推荐英文，避免编码问题）
3. 键盘布局：Chinese 或 English (US)

4. 安装类型选择（最关键的一步！）：

   ★★★ 选择「其他选项」（Something else / 手动分区）★★★
   不要选「清除整个磁盘」或「与 Windows 并存」

5. 手动分区操作：

   a) 找到之前在 Windows 下压缩出来的「空闲空间」（free space）
      显示为「空闲」或「free space」

   b) 创建根分区（/）：
      - 点击「空闲空间」→ 「+」号
      - 大小：80000~100000 MB（80-100GB）
      - 类型：主分区（Primary）
      - 位置：空间起始位置
      - 用于：ext4 日志文件系统
      - 挂载点：/

   c) 创建 Swap 分区：
      - 再次点击剩余的「空闲空间」→ 「+」号
      - 大小：16384 MB（16GB，等于你笔记本内存大小）
      - 类型：逻辑分区（Logical）
      - 用于：交换空间（swap area）

   d) （可选）创建共享数据分区：
      - 如果还有剩余空间，可以再建一个 ext4 分区
      - 挂载点：/data 或 /windows_shared
      - 用于存放项目代码等双系统共享文件

   e) 选择安装引导加载器的设备：
      - 选择 EFI 分区（通常是第一个 FAT32 分区，约 300-500MB）
      - 显示为 /dev/nvme0n1p1 或类似名称
      - ⚠️ 不要选错成 Windows 的 C 盘！

6. 点击「现在安装」
7. 时区选择：Shanghai（上海）
8. 创建用户：
   - 你的名字：输入你的名字
   - 计算机名：ubuntu-robotac（或自定义）
   - 用户名：小写字母（如 robotac）
   - 密码：设置一个你能记住的密码
   - ☑️ 自动登录（可选，方便日常使用）
   - ☑️ 要求我加密我的主目录（不勾选，避免性能损耗）

9. 等待安装完成（约 15-30 分钟）
10. 安装完成后提示「重启」→ 点「现在重启」
11. 拔掉 U 盘（否则可能再次进入安装程序）
12. 重启后应该看到 GRUB 引导菜单，可以选择 Ubuntu 或 Windows
```

### 5.4 安装后的首次设置

```bash
# 1. 更新系统（首次开机必做）
sudo apt update && sudo apt upgrade -y

# 2. 安装常用工具
sudo apt install -y \
    vim git curl wget htop tree \
    build-essential cmake \
    net-tools openssh-server \
    gnome-tweaks \
    software-properties-common \
    apt-transport-https ca-certificates \
    gnupg lsb-release

# 3. 配置中文输入法（如果需要）
sudo apt install -y ibus-libpinyin
# 设置 → 区域和语言 → 输入源 → 添加中文（智能拼音）

# 4. 配置时区（如果安装时没选对）
sudo timedatectl set-timezone Asia/Shanghai
```

---

## 6. NVIDIA 驱动安装（关键步骤）

> **⚠️ 这是让 UE5 仿真正常运行的最重要的步骤！**  
> MATRiX 平台要求：NVIDIA 驱动版本 ≥ 535，且支持 Vulkan SM6

### 6.1 检查当前 GPU 状态

```bash
# 查看显卡信息
lspci | grep -i vga
# 预期输出：... NVIDIA Corporation GA107M [GeForce RTX 4050 Laptop GPU] ...

# 查看当前驱动状态
nvidia-smi
# 如果提示 command not found = 还没装驱动
# 如果显示驱动版本 < 535 = 需要升级
```

### 6.2 方法一：使用 Ubuntu 官方驱动仓库（推荐新手）

```bash
# 1. 添加 Graphics PPA（提供更新的驱动）
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt update

# 2. 查看可用的 NVIDIA 驱动版本
ubuntu-drivers devices
# 输出示例：
# == /sys/devices/pci0000:00/0000:01:00.0 ==
# modalias : pci:v000010DEd00002880sv00001043sd00008862bc03sc00i00
# vendor   : NVIDIA Corporation
# model    : GA107M [GeForce RTX 4050 Laptop GPU]
# driver   : nvidia-driver-535 - distro non-free recommended
# driver   : nvidia-driver-550 - third-party free
# driver   : nvidia-driver-560 - third-party free recommended
# driver   : nvidia-driver-565 - third-party free
# driver   : nvidia-driver-570, ... (proprietary, tested)
# driver   : nvidia-open-kernel-source - distro free proprietary

# 3. 安装推荐的驱动（自动选择最新稳定版）
sudo ubuntu-drivers autoinstall

# 或者手动指定版本（推荐 550 或 560，稳定性好）
sudo apt install -y nvidia-driver-550

# 4. 重启电脑使驱动生效
sudo reboot
```

### 6.3 方法二：使用 NVIDIA 官方 Runfile（高级用户，更灵活）

```bash
# 1. 先禁用 Nouveau 开源驱动（否则冲突）
sudo bash -c "echo blacklist nouveau > /etc/modprobe.d/blacklist-nouveau.conf"
sudo bash -c "echo options nouveau modeset=0 >> /etc/modprobe.d/blacklist-nouveau.conf"
sudo update-initramfs -u

# 2. 重启
sudo reboot

# 3. 确认 Nouveau 已禁用
lsmod | grep nouveau
# 无输出 = 成功禁用

# 4. 下载 NVIDIA 官方驱动（去官网或用命令行）
# 推荐 550.xx 或 560.xx 版本（支持 Vulkan SM6，稳定）
wget https://us.download.nvidia.com/XFree86/Linux-x86_64/550.135/NVIDIA-Linux-x86_64-550.135.run

# 5. 给执行权限并安装
chmod +x NVIDIA-Linux-x86_64-550.135.run
sudo ./NVIDIA-Linux-x86_64-550.135.run

# 安装过程中：
# - 接受许可协议（Accept）
# - 如果问是否编译内核模块 → 选 Yes
# - 如果问是否更新 X 配置 → 选 Yes
# - 其他默认即可

# 6. 重启
sudo reboot
```

### 6.4 验证驱动安装成功

```bash
# 重启后执行以下命令逐一验证

# 1. nvidia-smi（最重要）
nvidia-smi
# 应该显示：
# +-----------------------------------------------------------------------------+
# | NVIDIA-SMI 550.135     Driver Version: 550.135     CUDA Version: 12.4       |
# |-------------------------------+----------------------+----------------------+
# | GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
# | Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
# |                               |                      |               MIG M. |
# |===============================+======================+======================|
# |   0  NVIDIA GeForce ...  Off  | 00000000:01:00.0 N/A |                  N/A |
# |  30%    42C    P8    N/A /  W |    6144MiB /  6144MiB |      0%      Default |
# |                               |                      |               N/A |
# +-----------------------------------------------------------------------------+

# 2. 检查 Vulkan 支持
vulkaninfo --summary
# 重点看这两行：
# GPU id = 0 (NVIDIA GeForce RTX 4050 Laptop GPU)
# apiVersion = 1.3.xxx (Vulkan 1.3 = 支持 SM6 ✓)
# driverVersion = 550.xxxx

# 3. 检查 OpenGL
glxinfo | grep "OpenGL version"
# 应该显示：OpenGL version string: 4.6 CUDA 550.xxxx

# 4. 测试 GPU 渲染（简单 GUI 测试）
glmark2-full || sudo apt install -y glmark2-full && glmark2
# 会弹出一个窗口跑 OpenGL benchmark，能跑通说明驱动工作正常
```

### 6.5 如果 Vulkan 版本低于 1.3

```bash
# vulkaninfo 显示 apiVersion = 1.2 或更低
# 说明驱动版本太旧或不完整

# 解决方案：
# 1. 升级到更新的驱动（推荐 550+ 或 560+）
sudo apt purge -y 'nvidia-*'
sudo apt autoremove -y
sudo apt install -y nvidia-driver-560
sudo reboot

# 2. 确保 Vulkan ICD 已安装
sudo apt install -y libvulkan1 mesa-vulkan-drivers vulkan-tools
vulkaninfo --summary  # 再检查一次
```

---

## 7. ROS2 Humble 安装

MATRiX 平台依赖 ROS2 Humble，必须安装。

### 7.1 设置语言环境

```bash
# 确保系统 locale 支持 UTF-8
locale
# 检查是否有 LANG=en_US.UTF-8 或 LANG=zh_CN.UTF-8

# 如果没有，安装 locale 并生成
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 7.2 添加 ROS2 软件源

```bash
# 1. 添加官方 GPG 密钥
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# 2. 添加软件源到 sources.list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 7.3 安装 ROS2 Humble

```bash
# 1. 更新包索引
sudo apt update

# 2. 安装 ROS2 Humble 桌面版（包含 rviz、可视化工具等）
sudo apt install -y ros-humble-desktop

# 3. （可选）安装常用 ROS2 工具包
sudo apt install -y \
    ros-humble-rviz2 \
    ros-humble-ros-base \
    python3-colcon-common-extensions \
    python3-rosdep

# 4. 初始化 rosdep（ROS 包依赖管理器）
sudo rosdep init
rosdep update

# 5. 配置环境变量（每次打开终端都要 source）
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 7.4 验证 ROS2 安装

```bash
# 打开两个终端窗口

# 终端 1：启动发布者节点
ros2 topic pub /chatter std_msgs/String "data: 'Hello from ROS2'"

# 终端 2：订阅话题
ros2 topic echo /chatter
# 应该能看到消息输出：
# data: 'Hello from ROS2'
# ---
# data: 'Hello from ROS2'
# ...

# 验证节点列表
ros2 node list
# 应该看到 /pub_node 等
```

---

## 8. MATRiX 环境验证

### 8.1 同步项目代码到 Ubuntu

```bash
# 方法 A：通过 Git 克隆（推荐）
cd ~
git clone <你的仓库地址> matrix_robotac_worksapce
cd matrix_robotac_worksapce

# 方法 B：从 Windows 分区访问（如果是 NTFS 共享分区）
# 先查看 Windows 分区挂载位置
lsblk
# 找到 Windows 的 NTFS 分区（可能是 /dev/nvme0n1p3 或类似）
sudo mkdir -p /mnt/windows
sudo mount /dev/nvme0n1p3 /mnt/windows  # 替换为实际分区
ls /mnt/windows/Users/<你的用户名>/Desktop/  # 确认能访问

# 方法 C：通过 VMware 共享文件夹（如果还在用虚拟机）
# ls /mnt/hgfs/matrix  # 直接访问
```

### 8.2 安装 MATRiX 依赖

```bash
cd ~/matrix_robotac_worksapce/matrix_robotac_first

# 运行依赖安装脚本（注意事项.md 里提到过这个脚本）
./scripts/build.sh
# 注意：这个脚本只装系统依赖，UeSim 需单独处理

# 如果脚本不存在或报错，手动安装核心依赖
sudo apt install -y \
    python3-pip python3-venv \
    libeigen3-dev \
    libpcl-dev \
    libopencv-dev \
    libsuitesparse-dev \
    libproj-dev \
    libyaml-cpp-dev \
    ros-humble-pcl-conversions \
    ros-humble-cv-bridge \
    ros-humble-sensor-msgs
```

### 8.3 启动仿真测试

```bash
cd ~/matrix_robotac_worksapce/matrix_robotac_first

# 启动仿真（参数含义：机器人=xgb, 场景=Level_2, 无离屏渲染, 无PixelStreaming, 无MuJoCo）
./run_sim.sh xgb 1 0 0 0

# 预期结果：
# 1. 不再弹出 "Vulkan device could not be created" 错误
# 2. 看到 UE5 窗口打开，显示 RobotAC 场景
# 3. 终端输出：
#    [INFO] Starting processes...
#    [INFO] Starting UE
#    [INFO] Starting MC
#    [INFO] All components started.
# 4. 场景中能看到机器狗模型

# 如果成功 → 恭喜！MATRiX 仿真环境搭建完成 ✅
```

### 8.4 验证传感器 Topic

```bash
# 新开一个终端，source ROS2 环境
source /opt/ros/humble/setup.bash

# 查看 Topic 列表
ros2 topic list
# 预期输出：
# /front_camera/image/compressed
# /front_depth/image/compressed
# /front_lidar
# ... 其他 topic

# 测试订阅 RGB 图像
ros2 topic echo /front_camera/image/compressed --once
# 应该能看到一条 CompressedImage 消息
```

---

## 9. 双系统日常使用

### 9.1 GRUB 引导菜单配置

```bash
# 编辑 GRUB 配置
sudo nano /etc/default/grub

# 常用修改项：
GRUB_DEFAULT=0                    # 默认启动项（0=第一个，即 Ubuntu）
GRUB_TIMEOUT=10                   # 菜单等待时间（秒）
GRUB_TIMEOUT_STYLE=menu           # 显示菜单（改为 hidden 可隐藏）
# GRUB_SAVEDEFAULT=true           # 记住上次选择（可选）

# 如果想默认启动 Windows：
# 先查看 Windows 在 GRUB 中的序号
grep -i windows /boot/grub/grub.cfg
# 然后把 GRUB_DEFAULT 改为对应序号（如 "1>3" 或 "Windows Boot Manager..."）

# 应用修改
sudo update-grub
```

### 9.2 时间同步问题修复

```bash
# 问题：双系统时间不一致（Windows 用本地时间，Ubuntu 用 UTC）
# 解决方案：让 Ubuntu 也使用本地时间（推荐）

sudo timedatectl set-local-rtc 1 --adjust-system-clock

# 或者让 Windows 使用 UTC（二选一）
# Windows PowerShell（管理员）：
# Reg add HKLM\SYSTEM\CurrentControlSet\Control\TimeZoneInformation /v RealTimeIsUniversal /t REG_DWORD /d 1
```

### 9.3 文件系统互访

```bash
# Ubuntu 下访问 Windows 分区
# 自动挂载（推荐）：安装 ntfs-3g
sudo apt install -y ntfs-3g
# 重启后 Windows 分区会自动出现在左侧边栏（文件管理器）

# 手动挂载
sudo mkdir -p /mnt/c_drive
sudo mount /dev/nvme0n1p2 /mnt/c_drive  # 替换为实际 Windows 分区

# Windows 下访问 Ubuntu 分区
# 安装 Ext2Fsd 或 DiskInternals Linux Reader（第三方工具）
# 或者直接用 WSL2（Windows Subsystem for Linux）
```

### 9.4 性能优化建议

```bash
# 1. 减少透明效果（提升 UI 流畅度）
gsettings set org.gnome.desktop.interface enable-animations false

# 2. 限制 journal 日志大小
sudo journalctl --vacuum-size=200M

# 3. 安装 preload（预加载常用程序到内存）
sudo apt install -y preload
# 自动后台运行，无需配置

# 4. （可选）安装 TLP 笔记本电源管理
sudo apt install -y tlp tlp-rdw
sudo tlp start
# 可以优化电池续航和散热
```

---

## 10. 常见问题排查

### 10.1 安装阶段问题

| 问题                | 原因                                | 解决方法                                               |
| ----------------- | --------------------------------- | -------------------------------------------------- |
| U 盘无法启动           | BIOS 未设为 UEFI 启动 / Secure Boot 未关 | 重新进 BIOS 检查设置                                      |
| 安装时看不到 Windows 分区 | 分区表损坏 / 磁盘模式错误                    | 用 `gdisk /dev/nvme0n1` 检查分区表                       |
| 安装过程中黑屏           | 显卡驱动兼容问题                          | 安装时选「Safe Graphics」模式（GRUB 菜单按 E 编辑，加 `nomodeset`） |
| GRUB 不出现          | Windows Fast Boot 抢占启动            | 进 Windows → 电源选项 → 关闭快速启动                          |

### 10.2 驱动阶段问题

| 问题                             | 原因                    | 解决方法                                       |
| ------------------------------ | --------------------- | ------------------------------------------ |
| `nvidia-smi` command not found | 驱动未安装或安装失败            | 重新执行第 6 章 `ubuntu-drivers autoinstall`     |
| 登录黑屏 / 循环登录                    | Nouveau 与 NVIDIA 驱动冲突 | 进入 TTY（Ctrl+Alt+F3）卸载驱动重装                  |
| Vulkan 版本 < 1.3                | 驱动版本太旧                | 升级到 550+ 或 560+                            |
| `vulkaninfo` 报错                | 缺少 Vulkan ICD 库       | `sudo apt install libvulkan1 vulkan-tools` |
| 分辨率异常                          | 驱动未正确加载 EDID          | 检查 `/var/log/Xorg.0.log` 错误信息              |

### 10.3 仿真阶段问题

| 问题           | 原因               | 解决方法                                                    |
| ------------ | ---------------- | ------------------------------------------------------- |
| 又弹 Vulkan 错误 | 驱动没装好 / 还是虚拟机    | 回到第 6 章重新验证 `vulkaninfo --summary`                      |
| UE 窗口闪退      | 缺少运行库            | `sudo apt install libegl1 libgl1 libglib2.0-0`          |
| MC 连接失败      | sleep 5 不够（机器慢）  | 编辑 `run_sim.sh` 把 `sleep 5` 改为 `sleep 10`               |
| 机器狗不动        | SDK IP 配置错误      | 检查 `sdk_config.yaml` 和 `initRobot` 参数（见注意事项.md 2.3/3.2） |
| 传感器无数据       | ROS2 Topic 名称不匹配 | `ros2 topic list` 对比 config.json                        |

### 10.4 救急：回滚到纯 Windows

```bash
# 如果 Ubuntu 装坏了想完全删除：
# 1. 用 Windows 安装盘或 PE 启动
# 2. 打开 diskpart
# 3. 删除 Ubuntu 所在分区（ext4 和 swap）
# 4. 修复引导：
#    bootrec /fixmbr
#    bootrec /fixboot
#    bootrec /rebuildbcd
# 5. 重启，恢复纯 Windows
```

---

## 附录：快速检查清单（安装完成后逐条打钩）

```bash
# 复制以下命令到终端一次性执行，检查所有关键组件
echo "=== 系统信息 ==="
uname -a && cat /etc/os-release | grep PRETTY_NAME
echo ""
echo "=== GPU 驱动 ==="
nvidia-smi | head -10
echo ""
echo "=== Vulkan 版本 ==="
vulkaninfo --summary 2>&1 | grep -E "apiVersion|GPU id"
echo ""
echo "=== ROS2 版本 ==="
echo $ROS_DISTRO && ros2 --version
echo ""
echo "=== 磁盘空间 ==="
df -h / | tail -1
echo ""
echo "=== 内存 ==="
free -h | head -2
echo ""
echo "✅ 所有检查通过 = 可以开始跑 MATRiX 仿真了！"
```

---

## 总结

完成以上步骤后，你的电脑将具备：

- ✅ Windows 11（日常办公、写代码）
- ✅ Ubuntu 22.04（运行 MATRiX 仿真）
- ✅ NVIDIA 550+ 驱动（Vulkan SM6 支持）
- ✅ ROS2 Humble（MATRiX 通信框架）
- ✅ GRUB 双系统引导（自由切换）

**下一步行动：**

1. 按本指南完成双系统安装
2. 在 Ubuntu 下克隆/同步项目代码
3. 执行 `./run_sim.sh xgb 1 0 0 0` 验证仿真启动
4. 开始 SLAM 建图 MVP 开发（距离中期检查还剩 ~23 天）

> **遇到问题随时问我，我可以根据具体报错日志帮你定位。**
