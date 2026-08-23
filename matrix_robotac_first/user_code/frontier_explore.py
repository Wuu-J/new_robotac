#!/usr/bin/env python3
"""Frontier 前沿探索（Yamauchi 1997）：系统化覆盖未知迷宫，替代左手规则随机游走

数据流:
    /map_2d (OccupancyGrid，建图节点发布) → 空闲/墙/未知三类栅格
    /odom/mujoco_odom (Best Effort) → 当前位姿
    /front_lidar (Best Effort) → 局部安全刹车（复用 auto_explore 的扇区感知）

主循环（20Hz 控制 / 1Hz 重规划）:
    1. 前沿 = 未知栅格中与"可达空闲"相邻的格子（按连通域分组）
    2. 选离机器人最近的前沿簇 → A*（8 邻域，墙膨胀 0.4m 保持距离）规划路径
    3. 纯追踪沿路径行驶；每次重规划时重校验目标（已被探索则自动换下一个，Holz 改进）
    4. 无任何前沿 → 探索完成，打印覆盖率，自动趴下
    5. 卡住检测：6s 位移 <8cm 且期望前进 → 清路径原地转，靠新地图重规划

用法（先启动仿真 + slam_odom_mapper）:
    python3 frontier_explore.py
    python3 frontier_explore.py --speed 0.35 --replan-period 0.8   # 调参
"""
import argparse
import heapq
import os
import platform
import signal
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry, OccupancyGrid
from scipy import ndimage

# ---------------------------------------------------------------- SDK 导入
SDK_LIB = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', 'deps', 'zsibot_sdk', 'lib', 'zsl-1',
    platform.machine().replace('amd64', 'x86_64').replace('arm64', 'aarch64')))
sys.path.insert(0, SDK_LIB)
import mc_sdk_zsl_1_py

from auto_explore import LidarPerception   # 复用扇区感知（lidar 订阅 + front/left/right）


# ---------------------------------------------------------------- 主逻辑
class FrontierExplorer:
    def __init__(self, args):
        self.app = mc_sdk_zsl_1_py.HighLevel()
        self.app.initRobot(args.local_ip, args.port, args.dog_ip)
        self.speed = args.speed
        self.speed_min = args.speed_min
        self.yaw_max = args.yaw_max
        self.kp_head = args.kp_head
        self.lookahead = args.lookahead
        self.goal_tol = args.goal_tol
        self.min_goal_dist = args.min_goal_dist
        self.hard_front = args.hard_front
        self.robot_radius = args.robot_radius
        self.replan_period = args.replan_period
        self.loop_hz = args.loop_hz
        self.running = True
        # 地图与位姿状态
        self.grid = None        # 0未知 1空闲 2墙
        self.grid_res = 0.1
        self.grid_xmin = 0.0
        self.grid_ymin = 0.0
        self.pose = None        # (x, y, yaw)
        self.path = None        # Nx2 world 坐标
        self.goal = None
        self.goal_cell = None   # 目标栅格坐标（卡住时封锁用）
        self.blocked = {}       # 封锁目标栅格 -> 过期时刻（卡住后 60s 内不重选）
        self.last_replan = 0.0
        self.stuck_pos = None
        self.stuck_t = 0.0

    # ------------------------------------------------------------ 订阅
    def on_grid(self, msg):
        d = np.array(msg.data, dtype=np.int8)
        h, w = msg.info.height, msg.info.width
        if d.size != h * w:
            return
        d = d.reshape(h, w)
        g = np.zeros((h, w), dtype=np.uint8)
        g[d == 0] = 1
        g[d == 100] = 2
        self.grid = g
        self.grid_res = msg.info.resolution
        self.grid_xmin = msg.info.origin.position.x
        self.grid_ymin = msg.info.origin.position.y

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    # ------------------------------------------------------------ 规划
    def _nearest_safe(self, free_safe, cr, cc):
        """机器人所在格不可达时，找最近的可达空闲格"""
        H, W = free_safe.shape
        for rng in (30, H):
            rs = slice(max(0, cr - rng), min(H, cr + rng + 1))
            cs = slice(max(0, cc - rng), min(W, cc + rng + 1))
            idx = np.argwhere(free_safe[rs, cs])
            if len(idx):
                d2 = (idx[:, 0] - min(rng, cr)) ** 2 + (idx[:, 1] - min(rng, cc)) ** 2
                i = int(np.argmin(d2))
                return int(idx[i, 0] + rs.start), int(idx[i, 1] + cs.start)
        return None, None

    def _goal_for(self, free_safe, fr, fc, cr, cc):
        """前沿格的 8 邻域中离机器人最近的可达空闲格 → A* 目标"""
        H, W = free_safe.shape
        best, bd = None, np.inf
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                r2, c2 = fr + dr, fc + dc
                if 0 <= r2 < H and 0 <= c2 < W and free_safe[r2, c2]:
                    d = (r2 - cr) ** 2 + (c2 - cc) ** 2
                    if d < bd:
                        bd, best = d, (r2, c2)
        return best

    def astar(self, free_safe, start, goal):
        """8 邻域 A*，返回 [(r,c),...] 或 None"""
        H, W = free_safe.shape
        sr, sc = start
        gr, gc = goal
        g = np.full((H, W), np.inf)
        g[sr, sc] = 0.0
        heap = [(np.hypot(gr - sr, gc - sc), 0, 0.0, sr, sc)]
        seq = 0
        parent = {}
        while heap:
            f, _, gold, r, c = heapq.heappop(heap)
            if gold != g[r, c]:
                continue                      # 过期条目
            if (r, c) == (gr, gc):
                path = [(r, c)]
                while (r, c) != (sr, sc):
                    r, c = parent[(r, c)]
                    path.append((r, c))
                return path[::-1]
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    if dr == 0 and dc == 0:
                        continue
                    r2, c2 = r + dr, c + dc
                    if not (0 <= r2 < H and 0 <= c2 < W) or not free_safe[r2, c2]:
                        continue
                    ng = g[r, c] + (1.0 if dr == 0 or dc == 0 else 1.414)
                    if ng < g[r2, c2]:
                        g[r2, c2] = ng
                        parent[(r2, c2)] = (r, c)
                        seq += 1
                        heapq.heappush(heap, (ng + np.hypot(gr - r2, gc - c2),
                                              seq, ng, r2, c2))
        return None

    def replan(self):
        """找最近可达前沿 → A* 路径。无前沿 → 探索完成"""
        if self.grid is None or self.pose is None:
            return
        rx, ry, ryaw = self.pose
        grid, res = self.grid, self.grid_res
        H, W = grid.shape

        # 墙膨胀 → 不可通行区（路径与墙保持 robot_radius 距离）
        r_cells = int(np.ceil(self.robot_radius / res))
        obstacle = ndimage.binary_dilation(
            grid == 2, np.ones((2 * r_cells + 1, 2 * r_cells + 1)))
        free_safe = (grid == 1) & ~obstacle

        cr = int((ry - self.grid_ymin) / res)
        cc = int((rx - self.grid_xmin) / res)
        if not (0 <= cr < H and 0 <= cc < W):
            print('[WARN] 机器人超出栅格范围，等待新地图')
            return
        if not free_safe[cr, cc]:
            cr, cc = self._nearest_safe(free_safe, cr, cc)
            if cr is None:
                return

        # 前沿 = 未知格中与可达空闲相邻的格子；栅格边界不算（迷宫应在边界内，
        # 边界前沿永远探索不完，会导致机器人在边界反复振荡）
        frontier = ndimage.binary_dilation(free_safe, np.ones((3, 3))) & (grid == 0)
        frontier[0, :] = frontier[-1, :] = frontier[:, 0] = frontier[:, -1] = False
        if not frontier.any():
            self.path, self.goal = None, None
            print(f'[INFO] 无前沿，探索完成！覆盖率 {self.coverage():.1f}%')
            self.running = False              # 完成，自动停
            return

        # 按连通域分组，每簇取其离机器人最近的前沿格
        lab, n = ndimage.label(frontier)
        cand = []
        for k in range(1, n + 1):
            idx = np.argwhere(lab == k)
            d2 = ((idx[:, 0] * res + self.grid_ymin - ry) ** 2
                  + (idx[:, 1] * res + self.grid_xmin - rx) ** 2)
            i = int(np.argmin(d2))
            cand.append((d2[i], int(idx[i, 0]), int(idx[i, 1])))
        cand.sort()

        now = time.time()
        # 清理已过期的封锁
        for k in [k for k, v in self.blocked.items() if v <= now]:
            del self.blocked[k]

        # 排序分数 = 距离² × (2 - cos(方向差))：同距离优先"正前方"的前沿，
        # 避免在开阔区来回折返（航向偏好）
        def _score(d2, fr, fc):
            th = np.arctan2(fr * res + self.grid_ymin - ry,
                            fc * res + self.grid_xmin - rx)
            return d2 * (2.0 - np.cos(th - ryaw))
        cand.sort(key=lambda c: _score(c[0], c[1], c[2]))
        # 优先尝试 ≥min_goal_dist 的较远前沿（大跳跃=全速行驶，探索效率高），
        # 都不可达再退回最近的前沿（map 快探索完时的收尾）
        far = [c for c in cand if c[0] >= self.min_goal_dist ** 2]
        order = far + cand
        seen = set()
        tried = 0
        for d2, fr, fc in order:
            if (fr, fc) in seen:
                continue
            seen.add((fr, fc))
            gr, gc = self._goal_for(free_safe, fr, fc, cr, cc)
            if gr is None:
                continue
            if self.blocked.get((gr, gc), 0) > now:
                continue                      # 该目标刚卡住过，跳过
            path = self.astar(free_safe, (cr, cc), (gr, gc))
            if path is not None:
                self.path = [(c * res + self.grid_xmin, r * res + self.grid_ymin)
                             for r, c in path]
                self.goal = self.path[-1]
                self.goal_cell = (gr, gc)
                print(f'[PLAN] 前沿目标({self.goal[0]:.1f},{self.goal[1]:.1f}) '
                      f'直线 {np.sqrt(d2):.1f}m 路径 {len(path)} 格')
                return
            tried += 1
            if tried >= 16:
                break
        self.path, self.goal = None, None
        print('[PLAN] 前沿均不可达，原地旋转收集地图数据')

    # ------------------------------------------------------------ 控制
    def drive(self, sec, dt):
        """纯追踪 + 安全刹车 + 卡住检测 → (vx, yaw)"""
        d_front = sec[0] if sec is not None else None
        if self.pose is None:
            return 0.0, 0.0
        rx, ry, ryaw = self.pose

        # ---- 安全刹车：前墙太近，清路径原地转 ----
        if d_front is not None and d_front < self.hard_front:
            self.path, self.goal = None, None
            return 0.0, +0.4

        if self.path is None or self.goal is None:
            return 0.0, +0.3                  # 无路径：原地慢转收集数据，等重规划

        # ---- 目标已到达 ----
        if np.hypot(rx - self.goal[0], ry - self.goal[1]) < self.goal_tol:
            self.path, self.goal = None, None
            return 0.0, 0.0

        # ---- 纯追踪 ----
        p = np.asarray(self.path)
        dist2 = (p[:, 0] - rx) ** 2 + (p[:, 1] - ry) ** 2
        i = int(np.argmin(dist2))
        j = i
        while j < len(p) - 1 and np.hypot(p[j, 0] - rx, p[j, 1] - ry) < self.lookahead:
            j += 1
        tx, ty = p[j]
        target_dist = np.hypot(p[-1, 0] - rx, p[-1, 1] - ry)
        heading = np.arctan2(ty - ry, tx - rx)
        err = (heading - ryaw + np.pi) % (2 * np.pi) - np.pi
        yaw = float(np.clip(self.kp_head * err, -self.yaw_max, self.yaw_max))
        if abs(yaw) < 0.02:
            yaw = 0.0
        vx = max(self.speed * min(1.0, target_dist / 1.5), self.speed_min)
        if abs(err) > 1.2:
            vx = 0.0                          # 方向差太大：先原地转
        if d_front is not None and d_front < 0.8:
            vx = min(vx, self.speed_min)      # 前距近：爬行

        # ---- 卡住检测：路径活跃期间 12s 位移 <12cm → 封锁当前目标，重新规划 ----
        if self.stuck_pos is None:
            self.stuck_pos, self.stuck_t = (rx, ry), 0.0
        self.stuck_t += dt
        if np.hypot(rx - self.stuck_pos[0], ry - self.stuck_pos[1]) > 0.12:
            self.stuck_pos, self.stuck_t = (rx, ry), 0.0
        if self.stuck_t > 12.0:
            if self.goal_cell is not None:
                self.blocked[self.goal_cell] = time.time() + 60.0
            print('[PLAN] 卡住：封锁当前目标 60s，立即重规划')
            self.path, self.goal, self.goal_cell = None, None, None
            self.last_replan = 0.0
            return 0.0, +0.4
        return vx, yaw

    def coverage(self):
        """覆盖率自评：已知区域包围盒（外扩 1m）内 已知格 占比"""
        if self.grid is None:
            return 0.0
        known = self.grid > 0
        if not known.any():
            return 0.0
        rows = np.where(known.any(axis=1))[0]
        cols = np.where(known.any(axis=0))[0]
        m = int(1.0 / self.grid_res)
        r0, r1 = max(0, rows[0] - m), min(self.grid.shape[0], rows[-1] + m + 1)
        c0, c1 = max(0, cols[0] - m), min(self.grid.shape[1], cols[-1] + m + 1)
        sub = self.grid[r0:r1, c0:c1]
        return 100.0 * (sub > 0).sum() / sub.size

    def status(self):
        if self.goal is not None:
            d = np.hypot(self.pose[0] - self.goal[0], self.pose[1] - self.goal[1])
            g = f'目标({self.goal[0]:.1f},{self.goal[1]:.1f})距{d:.1f}m'
        else:
            g = '无路径(等待重规划)'
        print(f'[STATUS] {g} | 覆盖率 {self.coverage():.1f}% | '
              f'路径点 {len(self.path) if self.path else 0}')

    # ------------------------------------------------------------ 主循环
    def run(self):
        time.sleep(1.0)
        if not self.app.checkConnect():
            print('[ERROR] SDK 未连接（检查仿真/mc_ctrl），退出')
            return 1
        print('[INFO] 连接成功，站立...')
        self.app.standUp()
        time.sleep(4)

        rclpy.init()
        node = Node('frontier_explore')
        be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(OccupancyGrid, '/map_2d', self.on_grid, 10)
        node.create_subscription(Odometry, '/odom/mujoco_odom', self.on_odom, be)
        perc = LidarPerception(front_angle=12.0)
        print(f'[INFO] 就绪: 速度 {self.speed}m/s | 重规划 {self.replan_period}s | '
              f'Ctrl+C 退出')

        period = 1.0 / self.loop_hz
        last_print = 0.0
        try:
            while self.running and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.02)
                rclpy.spin_once(perc, timeout_sec=0.02)
                now = time.time()
                if now - self.last_replan >= self.replan_period:
                    self.last_replan = now
                    self.replan()
                vx, yaw = self.drive(perc.sectors(), period)
                ret = self.app.move(vx, 0, yaw)
                if ret != 0:
                    print(f'[WARN] move 返回 {ret:#x}')
                if now - last_print >= 2.0:
                    last_print = now
                    self.status()
                time.sleep(period)
        except KeyboardInterrupt:
            print('\n[INFO] Ctrl+C，安全退出')
        finally:
            self.app.move(0, 0, 0)
            time.sleep(0.8)
            self.app.lieDown()
            time.sleep(0.5)
            perc.destroy_node()
            node.destroy_node()
            rclpy.try_shutdown()
            print('[INFO] 已停止并趴下')
        return 0


def main():
    ap = argparse.ArgumentParser(description='Frontier 前沿探索（系统化覆盖迷宫）')
    ap.add_argument('--local-ip', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=43988)
    ap.add_argument('--dog-ip', default='127.0.0.1')
    ap.add_argument('--speed', type=float, default=0.3, help='前进速度 m/s')
    ap.add_argument('--speed-min', type=float, default=0.08, help='近目标/近墙爬行速度 m/s')
    ap.add_argument('--yaw-max', type=float, default=0.4, help='yaw 指令上限 rad/s')
    ap.add_argument('--kp-head', type=float, default=1.2, help='航向误差→yaw 增益')
    ap.add_argument('--lookahead', type=float, default=0.7, help='纯追踪前视距离 m')
    ap.add_argument('--goal-tol', type=float, default=0.5,
                    help='到达目标判定距离 m（空闲圆盘 1m 时 0.5m 处前沿即被覆盖）')
    ap.add_argument('--min-goal-dist', type=float, default=1.5,
                    help='优先选择 ≥此距离 的较远前沿 m（大跳跃全速行驶效率高）')
    ap.add_argument('--hard-front', type=float, default=0.55, help='紧急前距 m')
    ap.add_argument('--robot-radius', type=float, default=0.4,
                    help='墙膨胀半径 m（规划路径与墙保持的距离）')
    ap.add_argument('--replan-period', type=float, default=0.7, help='重规划周期 s')
    ap.add_argument('--loop-hz', type=float, default=20.0, help='控制频率 Hz')
    args = ap.parse_args()

    exp = FrontierExplorer(args)

    def _sigterm(signum, frame):
        print('\n[INFO] 收到 SIGTERM，安全退出')
        exp.running = False
    signal.signal(signal.SIGTERM, _sigterm)

    sys.exit(exp.run())


if __name__ == '__main__':
    main()
