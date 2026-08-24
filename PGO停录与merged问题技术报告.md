# PGO 关键帧停录与 merged 问题技术报告（供 AI 分析）

> 撰写: 2026-08-25 02:20（距中期检查提交 ~10 小时）
> 目的: 完整记录建图管线的两个核心问题（PGO 停录、merged 不完整）的现象、根因分析、已尝试方案及证据，供另一个 AI 接手分析。

---

## 一、系统背景

- 平台: MATRiX 仿真（UE5 + mc_ctrl），Ubuntu 22.04 + ROS2 Humble
- SLAM: [SadCream/FASTLIO2_ROS2](https://github.com/SadCream/FASTLIO2_ROS2)（fastlio2 + pgo 两个包）
  - `fastlio2`：LIO 里程计（IESKF + ikd-Tree，21 维状态 `[r_wi, t_wi, r_il, t_il, v, bg, ba]`，无重力估计，g 为静态常量）
  - `pgo`：回环检测（Scan-Context 风格 ICP + GTSAM iSAM2）+ 关键帧录制
- 数据流：`/front_lidar`(10Hz, 整帧冻结快照) → livox_bridge.py → `/livox/lidar`(CustomMsg) → fastlio2 → `/fastlio2/body_cloud` + `/fastlio2/lio_odom`（同回调同时间戳发布）→ pgo（关键帧 patches + poses.txt）与 lio_map_builder.py（实时累积图）
- 源码位置：
  - `~/fastlio2_ws/src/FASTLIO2_ROS2/pgo/src/pgo_node.cpp`（订阅/配对/定时器/save_maps 服务）
  - `~/fastlio2_ws/src/FASTLIO2_ROS2/pgo/src/pgos/simple_pgo.{h,cpp}`（关键帧判定/GTSAM/回环 ICP）
  - `~/fastlio2_ws/src/FASTLIO2_ROS2/fastlio2/src/map_builder/ieskf.cpp`（IESKF 更新）
  - `~/matrix_robotac_workspace/matrix_robotac_first/user_code/merge_pgo_maps.py`（patches+poses → merged.pcd）
- 数据与日志：
  - 每次运行结果：`~/robotac_maps/run_时间戳/`（patches/、poses.txt、map.pcd、merged.pcd）
  - pgo 日志：`~/.ros/log/pgo_node_*.log`
  - 实时图：`~/robotac_maps/lio_map_时间戳.pcd`

---

## 二、问题一：PGO 关键帧停录（未根治，核心阻塞）

### 2.1 现象

关键帧录制在运行中途**静默停止**，之后机器人继续行驶但不再产生新关键帧 → save_maps 导出的 patches 只覆盖前半程 → merged 缺整块区域。

多次运行的实测停止点（各不相同）：

| 运行 | 停止时关键帧数 | 停止位置 (x,y) | 当时 pgo 版本 |
|---|---|---|---|
| 08-24 20:29 | 125 | (15.7, 0.6) | message_filters 同步器 + 乱序拒绝 |
| 08-24 21:45 | 97 | (15.8, 4.0) | 同上（重启后） |
| 08-24 22:02（诊断轮） | 1 | 原点附近 | 手动配对 v1 + 诊断日志 |
| 08-24 22:24 | 57 | (6.7, -16.4) | 手动配对 + 回环开启 |
| 08-25 01:57 | 92 | (16.5, 4.9) | 手动配对 + 回环关闭 + NaN 拒收 |

**关键证据（01:57 运行）**：live 图覆盖 262 个 1m 格（x[-2.8,27.2] y[-16.6,7.6]），而 merged 只覆盖 151 格（y 只到 -7.2）——机器人南段（y -7.2~-16.6）的行驶**没有产生任何关键帧**。

### 2.2 停录时的现场证据

1. **日志静默**：停录后 pgo 日志数分钟无任何新行（诊断日志每 5s 应有 `syncCB alive` / `non-key pose`，全部停止）
2. **话题正常**：停录期间实测 `/fastlio2/body_cloud` 与 `/fastlio2/lio_odom` 持续 10Hz+10Hz 流动，时间戳逐对完全一致（中位差 0.0ms）
3. **进程状态**：一次为 CPU 12.5%（8 核 = 单核满转，疑似忙循环）；另一次为 State=S（睡眠）、平均 CPU 18%、26 线程——非典型死循环
4. **无法获取调用栈**：`gdb -p` 被 YAMA ptrace_scope 拦截（非 root），`perf` 被 perf_event_paranoid=4 拦截——停录瞬间的栈从未抓到
5. save_maps 服务在停录后**仍然响应**（说明执行器线程未整体死锁）

### 2.3 已定位并修复的三个 bug（但停录仍复现）

| # | bug | 证据 | 修复 |
|---|---|---|---|
| ① | `message_filters` ApproximateTime 同步器运行约 35s 后静默饿死 | 诊断日志：sync 存活日志只打了 35s，之后 12 分钟静止，而两话题持续正常流动 | 弃用同步器，改手动配对：cloud/odom 各自 deque 缓冲 + `|dt|<50ms` 配对 |
| ② | `syncCB` 乱序拒绝：`if (t < last_message_time) return`——时钟跳变一次后所有消息被永久丢弃 | 代码审查 | 删除守卫 |
| ③ | `timerCB` 每拍取队首后清空整个队列 | 代码审查 | 改为每拍消费一条 |
| ④ | pgo.yaml 注释粘连数值行（`5.0# 注释`）→ YAML 解析异常 → 节点启动即死（日志 0 行） | 两次启动空日志 | 拆行 |

**修复①~④后，停录仍在 08-25 01:57 运行复现（kf=92）**，此时 pgo 代码路径只剩：手动配对 → `addKeyPose`（isKeyPose 判定 + GTSAM 图添加）→ `smoothAndUpdate`（`m_isam2->update(m_graph, m_initial_values); m_isam2->update();`，每个关键帧都跑）→ 回环路径（已用 `enable_loop=false` 整体关闭）。

### 2.4 已排除的假设

1. ❌ **同步器饿死**（修复①后仍停）
2. ❌ **时钟跳变+乱序拒绝**（修复②后仍停；且新日志从未出现 out-of-order 警告）
3. ❌ **ICP/GTSAM 回环挂起**（修复③：回环整体关闭后仍停）
4. ❌ **NaN 进入 GTSAM 死循环**（加了 isKeyPose NaN 拒绝 + timerCB 位姿有限性检查后仍停）
5. ❌ **时间戳偏移超配对容差**（实测逐对 0.0ms，100% 在 50ms 内）
6. ❌ **话题停止流动**（实测持续流动）
7. ❌ **执行器整体死锁**（save_maps 服务在停录后仍正常响应）

### 2.5 剩余嫌疑（未验证）

1. **GTSAM iSAM2 在某个特定图结构上阻塞/慢到不可接受**：`smoothAndUpdate` 每个关键帧跑一次 `m_isam2->update()`（即使没有回环因子）。停止点各不相同但都发生在机器人**转弯后**——转弯产生大量角向关键帧（10° 一个），图结构在转弯处密集。曾在停录进程上观察到"单核满转"与"睡眠+18% CPU"两种状态
2. **isKeyPose 恒 false**：若最后一个关键帧的 `t_local` 被污染（非 NaN 但异常大/异常小），后续 delta 恒小于阈值 → 永不新增。NaN 已排除，但未排除"巨大有限值"（例如位姿跳变 1e6）——然而跳变本身会先触发 `delta > 阈值` 收为新关键帧，逻辑上自相矛盾，未实测
3. **`pcl::fromROSMsg`/内存问题**：停录时 buffer 为 1 且正常消费，无证据
4. **ISAM2 变量数上限**：每次 `m_graph.resize(0)` + `m_initial_values.clear()` 后 ISAM2 内部历史仍在增长，relinearizeThreshold=0.01 触发频繁重线性化——无证据但未排除

### 2.6 下一步建议（供接手 AI）

- 在 `smoothAndUpdate` / `addKeyPose` / `timerCB` 里加**逐关键帧的耗时与心跳日志**（`std::chrono` 打点），下一次运行即可分辨"卡在哪个函数"
- 或改用 **LevenbergMarquardtOptimizer 每 N 帧批量优化**替代 iSAM2 增量更新，规避 iSAM2 潜在阻塞
- 或**完全绕过 pgo**：提交图已切换为 lio_map_builder 的实时累积图（见问题三的最终方案），pgo 仅作可选增强

---

## 三、问题二：merged 高度分裂（已解决）

### 3.1 现象

同一张 merged 里不同区域高度不同——实测一次运行的关键帧 z 轨迹：前 250 帧 z≈0.2，之后 0.85 → 6.2 → **10.7m**（LIO z 跑飞）。

### 3.2 根因

LIO（无 z 绝对约束）在三面墙退化场景 z 不可观，IMU ba_z 误差持续积分 → z 单调漂移/跑飞；PGO 照单全收这些位姿 → merged 各 patch 高度分裂。

### 3.3 已实施的修复（merge_pgo_maps.py 内，管线内处理）

1. **逐 patch 地面对齐**：每个 patch 变换到世界系后，取 z 最低 5% 分位带中位数作为该 patch 的地面高度，整体平移使地面归 0（PGO 地图系地面在 z≈-0.65，用全场中位数做参考自适应，不假设 0）
2. **发散丢弃**：patch 地面高度偏离全场中位数 >0.5m 视为位姿已发散（实测跑飞后 x/y/航向同样损坏，"全救"会导致大面积墙体重影），直接丢弃
3. 实测效果：z>2.5 悬挂点 43k → 431（-99%）

### 3.4 LIO 侧尝试（均回退）

| 尝试 | 结果 |
|---|---|
| z-prior 软约束（H[5][5]+=1e4，b[5]+=1e4·dz） | 能钉住 z，但实测 rviz x/y 偏移/重影加重——疑似退化场景奇异 H 下 H⁻¹ 耦合把 z 残差（步态 ±5cm × 1e4）泄漏进 x/y 更新。**未单独验证**（与 near_search_num 8→5 同时改动，变量混淆）→ 回退 |
| IESKF delta 爆炸中止（非有限或 >10 时放弃更新） | 保护性改动，随 14:44 整体回退一并移除 |
| 旋转雅可比 `hat(r_il·p + t_wi)` → `hat(r_il·p + t_il)` | 右扰动链式法则推导上 t_il 正确，但实测 t_il 版 x/y 漂移更重，t_wi 版（上游原版）建图最好 → 回退 |

### 3.5 最终方案（当前）

**builder 逐帧地面对齐**（提交图路径）：LIO z 怎么漂，每帧按自身地面拉回启动期参考高度，地图高度全程统一。单测通过（地面漂 0.3m → 对齐归位）。提交图 = `lio_map_时间戳.pcd`。

---

## 四、问题三：x/y 转弯漂移与重影（缓解未根治）

- **现象**：同一条走廊去程回程各扫一遍时，同一面墙出现两条相隔 ~0.3m 的平行线
- **根因**：无回环的 LIO 里程计纯累积漂移（转弯时航向欠转实测 1-11%，转圈位置漂 ≤0.26m）——0.3m/60m 路程 = 0.5%，属正常量级
- **已做**：keep-first 累积（实测比质心平均更抗漂移）、门控拦跳变帧、孤立点剔除
- **未做**：回环闭合（PGO 回环从未成功运行，见问题一）
- **当前对策**：驾驶纪律——每条走廊只走一遍不折返（双线只出现在重访处）

---

## 五、其他已修复问题（供参考）

| 问题 | 根因 | 修复 |
|---|---|---|
| merged 少一半点 | （曾怀疑 8 字段误读，实测 patches 为 4 字段 PointXYZI，读取无误）真因是问题一的关键帧停录 | 关键帧停录修复中 |
| 地面点稀疏（-64%） | min_hits≥2 滤掉单命中地面 | 地面带 hits≥1 |
| 提交图散点 | 位姿跳变幽灵帧 + 离散野点 | 门控 + 26 邻域孤立剔除（实测清 885 离散体素、结构无损） |
| pgo 启动即死 | pgo.yaml 注释粘连（见 2.3④） | 拆行 |
| live 图半路冻结 | 门控检测到 z 漂移按设计冻结地图 | 提交轮用 `--guard 0` + 逐帧对齐替代 |

---

## 六、给接手 AI 的直接入口

```bash
# 复现一次运行（6 终端，详见 SLAM启动手册.md）
# 观察点：终端 3 的 keyframes 计数（每 10 个打一次）何时停止

# 停录后立即取证：
ps -eo pid,pcpu,etime,cmd | grep pgo_node          # CPU 状态（满转=忙循环 / 睡眠=阻塞）
ls -lt ~/.ros/log/pgo_node_*.log                    # 最新日志
tail -20 ~/.ros/log/pgo_node_<pid>_*.log

# 已测过的事实（勿重复）：
# - 停录时两个话题持续 10Hz 流动、时间戳逐对一致
# - save_maps 服务停录后仍响应
# - gdb/perf 被系统权限限制（ptrace_scope、perf_event_paranoid=4）
#   sudo gdb -p <pid> -batch -ex "bt 20" 可绕过（需密码）

# 数据复盘：
python3 - <<'EOF'
# 分析任意 run 目录的 poses.txt 关键帧轨迹
import numpy as np, re, os
run = '~/robotac_maps/run_<时间戳>'
EOF
```

## 七、时间约束

- 中期检查提交 deadline：2026-09-12（本文撰写时约 10 小时后的"提交轮"是最后一次完整测试机会）
- 提交物：PCD（管线原生输出，禁后处理）+ 一镜到底视频（mp4 ≤15min ≤500MB）
- 当前提交路径：`~/robotac_maps/lio_map_时间戳.pcd`（覆盖完整、z 逐帧对齐、keep-first + 孤立剔除 + 地面 hits≥1）
