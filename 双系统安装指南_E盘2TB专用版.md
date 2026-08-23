# 双系统安装指南 — E 盘（Acer N5000 2TB）专用版

> **适用**：ROBOTAC 四足机甲挑战赛 — MATRiX 仿真平台（UE5.7 + MuJoCo + ROS2 Humble）
> **电脑**：华硕笔记本（AMD Ryzen 7 7735H + NVIDIA RTX 4050 Laptop 6GB）
> **方案**：Windows 11 在 WD 盘，Ubuntu 22.04 装在 Acer 2TB 盘的 150GB 空间
> **状态**：数据已备份 ✅ 磁盘为 NVMe 固态 ✅ 空间充足（E 盘剩余 1.8TB）✅

---

## ⭐ 本方案核心思路

```
磁盘 0: Acer SSD N5000 2TB  (E 盘)  ← 缩容 150GB 给 Ubuntu
磁盘 1: WD SN740 512G       (C/D 盘 + EFI 引导)  ← 完全不动！
磁盘 2: USB U 盘 59G        (启动盘)
```

- **两个系统物理隔离**：Windows 在 WD 盘，Ubuntu 在 Acer 盘，互不影响
- **安全**：只对 E 盘做「压缩卷」（无损操作），WD 盘一个字节都不碰
- **引导**：Ubuntu 的 GRUB 装到已有的 EFI 分区，开机可选 Ubuntu / Windows

---

## 第 1 步：缩容 E 盘（Windows 里操作，5 分钟）

```
1. 右键「此电脑」→ 管理 → 磁盘管理
2. 找到「磁盘 0」（Acer SSD N5000 2TB）→ 右键 E 盘分区 → 「压缩卷」
3. 输入压缩空间量：153600 MB（= 150GB）
   ⚠️ 磁盘管理单位是 MB，150GB = 153600 MB
4. 点击「压缩」
5. 完成后 E 盘后面会出现一个 150GB 的「未分配」黑色区域
```

**⚠️ 检查**：
- 压缩后 E 盘原有文件**不会丢**（无损压缩），但请再进 E 盘确认项目文件都在
- 未分配区域必须是**连续的**（紧跟在 E 盘后面）

---

## 第 2 步：制作 Ubuntu 启动盘（10 分钟）

### 2.1 下载 ISO

```
清华源：https://mirrors.tuna.tsinghua.edu.cn/ubuntu-releases/22.04.5/
阿里源：https://mirrors.aliyun.com/ubuntu-releases/22.04.5/
选择：ubuntu-22.04.5-desktop-amd64.iso
```

### 2.2 Rufus 写入 U 盘

```
1. 下载 Rufus：https://rufus.ie
2. 插入你的 59G U 盘（⚠️ 会被格式化，先确认里面没有要的东西）
3. 打开 Rufus：
   设备       → 选你的 U 盘（59GB USB）
   引导类型   → 选择 ubuntu-22.04.5-desktop-amd64.iso
   分区类型   → GPT
   目标系统   → UEFI（非 CSM）
4. 点「开始」→ 选「以 ISO 模式写入」→ 等待完成
```

**⚠️ 千万别选错盘**：Rufus 设备列表里你的 U 盘是 **59GB USB**，不要选成 2TB 的 Acer 或 512G 的 WD！

---

## 第 3 步：BIOS 设置（华硕，2 分钟）

```
1. 重启电脑，开机快速按 F2 进 BIOS
2. 找到并修改：
   • Secure Boot（安全启动）→ Disabled
   • Fast Boot / Quick Boot → Disabled
3. 按 F10 保存退出（重启）
4. 开机快速按 F12 进 Boot Menu → 选择你的 U 盘启动
```

> 如果启动 U 盘后黑屏：在 GRUB 菜单按 `E` 编辑启动项，在 `linux` 行末尾加 `nomodeset`，按 `F10` 启动

---

## 第 4 步：安装 Ubuntu（核心步骤，20 分钟）

### 4.1 安装前测试

进入 Live 桌面后：
```bash
# 按 Ctrl+Alt+T 打开终端，确认能看到两块硬盘
lsblk
# 预期：nvme0n1（512G WD）和 nvme1n1（2TB Acer）两块盘
# 或者 sda/sdb，具体看显示
```

### 4.2 开始安装

```
1. 双击桌面「Install Ubuntu 22.04 LTS」
2. 语言选 中文 或 English（推荐 English 避免编码问题）
3. 安装类型：★★★ 选「其他选项」（Something else）★★★
   不要选「清除整个磁盘」或「与 Windows 并存」！
```

### 4.3 手动分区（最关键！）

在分区界面找到 **2TB 那块盘（Acer）上 150GB 的「空闲空间」（free space）**：

| 分区 | 大小 | 类型 | 挂载点/用途 |
|------|------|------|------------|
| 根分区 `/` | **134000 MB**（134GB） | ext4 | 挂载点 `/` |
| Swap | **16000 MB**（16GB） | swap | 交换空间 |

操作：
```
a) 选中 Acer 盘（2TB）的 150GB free space → 点「+」
   大小 134000 MB → 类型：主分区 → 用于：ext4 → 挂载点：/ → OK
b) 选中剩余 free space（约16GB）→ 点「+」
   大小 16000 MB → 用于：交换空间 → OK
```

**⚠️ 引导加载器安装位置（千万注意）**：

```
页面底部的「安装启动引导器的设备」：
→ 选择 WD 512G 盘上的 EFI 分区
→ 显示为 /dev/nvmeXn1p1 或 /dev/sdX1（FAT32 / EFI System Partition，约 300MB）
→ 一般 Ubuntu 会自动选对（默认第一个 EFI 分区），请确认它属于 WD 盘
```

**✅ 确认无误后点「现在安装」**

### 4.4 后续

```
1. 时区：Shanghai
2. 用户名：robotac（小写），密码自设（记住！）
3. 不勾选「加密主目录」
4. 等待安装完成（15-30 分钟）→ 重启 → 拔 U 盘
5. 重启后出现 GRUB 菜单：
   → 第一项 Ubuntu（默认）
   → 第二项 Windows Boot Manager（选这个进 Windows）
```

---

## 第 5 步：首次开机配置（Ubuntu 里，10 分钟）

```bash
# 1. 更新系统
sudo apt update && sudo apt upgrade -y

# 2. 安装常用工具
sudo apt install -y vim git curl wget htop tree build-essential \
    cmake net-tools openssh-server gnome-tweaks \
    software-properties-common apt-transport-https \
    ca-certificates gnupg lsb-release

# 3. 时区
sudo timedatectl set-timezone Asia/Shanghai
```

---

## 第 6 步：NVIDIA 驱动（关键！决定仿真能否运行）

```bash
# 1. 检查显卡
lspci | grep -i vga
# 应看到：NVIDIA ... RTX 4050 Laptop GPU

# 2. 安装推荐驱动（550+ 支持 Vulkan SM6）
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt update
sudo ubuntu-drivers autoinstall
# 或手动指定：sudo apt install -y nvidia-driver-550

# 3. 重启
sudo reboot

# 4. 验证（重启后）
nvidia-smi                # 显示驱动版本 ≥ 550
vulkaninfo --summary | grep -E "apiVersion|GPU id"
# 必须看到 apiVersion = 1.3.x 以上！低于 1.3 说明驱动不行
```

> 仿真要求：驱动 ≥ 535，Vulkan ≥ 1.3（SM6）。RTX 4050 装 550+ 驱动后完全满足。

---

## 第 7 步：ROS2 Humble 安装

```bash
# 1. locale
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# 2. 添加源
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
ros2 --version   # 应显示 humble
```

---

## 第 8 步：项目代码同步 + 仿真验证

### 8.1 访问 Windows 的 E 盘

```bash
# Ubuntu 下访问 E 盘（Windows 数据盘）—— 只读挂载最安全
sudo apt install -y ntfs-3g
lsblk  # 找到 Acer 2TB 盘的 NTFS 分区（E 盘）
# 自动挂载：文件管理器左侧边栏通常直接能看到
```

> ⚠️ **不要**在 Ubuntu 里写 E 盘 NTFS 分区（有损坏风险），只读读取。项目代码建议**复制**到 Ubuntu 的 ext4 分区再使用。

### 8.2 复制项目到 Ubuntu

```bash
mkdir -p ~/matrix_robotac_worksapce
# 把 E:\ROBOTAC\matrix_robotac_worksapce 里的内容复制过来
# （推荐用 U 盘/网盘/或从 NTFS 分区复制）
```

### 8.3 安装 MATRiX 依赖并启动

```bash
cd ~/matrix_robotac_worksapce/matrix_robotac_first

# 安装依赖（先看注意事项.md 2.1 的坑）
# 核心依赖：
sudo apt install -y python3-pip libeigen3-dev libpcl-dev libopencv-dev \
    libsuitesparse-dev libyaml-cpp-dev \
    ros-humble-pcl-conversions ros-humble-cv-bridge ros-humble-sensor-msgs

# 启动仿真（有窗口模式！）
./run_sim.sh xgb 1 0 0 0
# 预期：UE5 窗口打开，看到机器狗和场景！
```

### 8.4 验证传感器

```bash
# 另开终端
source /opt/ros/humble/setup.bash
ros2 topic list
# 应看到 /front_lidar /front_camera/image/compressed 等
ros2 topic hz /front_lidar   # 应约 10Hz
```

---

## 第 9 步：双系统日常维护

### 9.1 时间不一致修复（双系统必做）

```bash
# Ubuntu 里执行（让 Ubuntu 用本地时间）
sudo timedatectl set-local-rtc 1 --adjust-system-clock
```

### 9.2 修改 GRUB 默认启动系统

```bash
sudo nano /etc/default/grub
# GRUB_DEFAULT=0  → Ubuntu 默认
# GRUB_DEFAULT=2  → Windows 默认（或写 "Windows Boot Manager"）
sudo update-grub
```

### 9.3 卸载 Ubuntu（救急，如果不需要了）

```
1. 进 Windows → 磁盘管理 → 删除 Acer 盘的 150GB 未分配区（Ubuntu 分区）
2. 删除后 E 盘恢复原容量（或保留为未分配）
3. 修复引导（如果 GRUB 残留）：
   Windows PE / 安装盘 → diskpart → 或用 bcdedit 重建
```

---

## 常见问题（FAQ）

| 问题 | 原因 | 解决 |
|------|------|------|
| 安装时黑屏 | 显卡兼容 | GRUB 菜单按 E 加 `nomodeset` |
| 重启没有 GRUB 菜单 | Fast Boot / 引导顺序 | BIOS 关 Fast Boot；或 F12 手动选系统 |
| 进 Ubuntu 黑屏/循环登录 | 驱动问题 | Ctrl+Alt+F3 进 TTY，重装驱动 |
| vulkaninfo 版本 < 1.3 | 驱动旧 | 升级 nvidia-driver-550+ |
| UE 弹 Vulkan 错误 | 驱动没生效 | 重启 + `nvidia-smi` 验证 |
| 时间错乱 | 双系统 UTC 冲突 | 第 9.1 节命令 |
| 仿真卡/慢 | E 盘是数据盘？ | 本项目装在 Acer NVMe 上，速度没问题；确认驱动装好 |

---

## 检查清单（装完逐条打钩）

```bash
echo "=== 系统 ==="; uname -a && grep PRETTY_NAME /etc/os-release
echo "=== GPU ==="; nvidia-smi | head -5
echo "=== Vulkan ==="; vulkaninfo --summary 2>&1 | grep -E "apiVersion|GPU id"
echo "=== ROS2 ==="; ros2 --version
echo "=== 项目 ==="; ls ~/matrix_robotac_worksapce/matrix_robotac_first/ | head
```

全部通过 = 仿真环境就绪 ✅

---

> **注意**：安装过程中遇到任何报错，截图发我，我帮你逐条排查。核心就三步：缩容 E 盘 → 装 Ubuntu → 装 NVIDIA 驱动。
