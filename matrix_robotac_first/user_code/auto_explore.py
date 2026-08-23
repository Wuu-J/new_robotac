#!/usr/bin/env python3
"""自主场地遍历（无人工干预）：左手规则沿墙走 + lidar 避障

原理:
    - 订阅 /front_lidar（Best Effort），按方位角分扇区取最近障碍距离
    - 保持左侧墙距离 ≈ --wall-dist（默认 1.2m），PD 阻尼抑制振荡
    - 前墙判定：窄扇区(--front-angle) + 连续 --confirm-frames 帧确认才触发转弯
    - 转弯不停车：弧线转弯（--turn-vx 前进 + yaw），前墙 < --hard-front 才原地转
    - 速度随前方距离自适应（--speed-ramp），近角提前减速、少急刹
    - 左侧墙丢失（> --lost-left）时弧线左转找回墙
    - 右手侧 < --side-clear 时轻微左偏，防止窄通道蹭右墙
    - SDK 命令 20Hz 持续发送（3 秒无数据自动趴下的看门狗约束）
    - Ctrl+C 安全退出（先停稳再趴下）

用法（先启动仿真 + 建图节点）:
    python3 auto_explore.py
    python3 auto_explore.py --wall-dist 1.0 --speed 0.3   # 调参
"""
import argparse
import os
import platform
import signal
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

# ---------------------------------------------------------------- SDK 导入
SDK_LIB = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', 'deps', 'zsibot_sdk', 'lib', 'zsl-1',
    platform.machine().replace('amd64', 'x86_64').replace('arm64', 'aarch64')))
sys.path.insert(0, SDK_LIB)
import mc_sdk_zsl_1_py


def cloud_to_xyz(msg):
    offsets = {f.name: f.offset for f in msg.fields}
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    cols = []
    for name in ('x', 'y', 'z'):
        o = offsets[name]
        cols.append(buf[:, o:o + 4].copy().view(np.float32).reshape(-1))
    return np.stack(cols, axis=1).astype(np.float64)


# ---------------------------------------------------------------- 感知节点
class LidarPerception(Node):
    """订阅 lidar，提供分扇区最近障碍距离"""

    def __init__(self, z_min=-0.6, z_max=1.5, max_range=12.0, front_angle=12.0):
        super().__init__('auto_explore_lidar')
        self.z_min, self.z_max, self.max_range = z_min, z_max, max_range
        self.front_angle = front_angle
        self._lock = threading.Lock()
        self._pts = None
        be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointCloud2, '/front_lidar', self._on_cloud, be)

    def _on_cloud(self, msg):
        pts = cloud_to_xyz(msg)
        r_xy = np.linalg.norm(pts[:, :2], axis=1)
        keep = ((pts[:, 2] > self.z_min) & (pts[:, 2] < self.z_max)
                & (r_xy > 0.2) & (r_xy < self.max_range))
        with self._lock:
            self._pts = pts[keep]

    def sectors(self):
        """返回 (d_front, d_left, d_right)：各扇区最近障碍的水平距离。
        扇区内无点 → None（表示"没有墙"，与"墙太远"区分）。
        角度定义（lidar 系，前 +x 左 +y）:
          front = |angle| < front_angle; left = [50°, 130°]; right = [-130°, -50°]"""
        with self._lock:
            pts = self._pts
        if pts is None or len(pts) < 50:
            return None
        ang = np.arctan2(pts[:, 1], pts[:, 0])
        r = np.linalg.norm(pts[:, :2], axis=1)

        def mind(mask):
            return r[mask].min() if mask.any() else None

        front = mind(np.abs(ang) < np.radians(self.front_angle))
        left = mind((ang > np.radians(50)) & (ang < np.radians(130)))
        right = mind((ang > np.radians(-130)) & (ang < np.radians(-50)))
        return front, left, right


# ---------------------------------------------------------------- 主逻辑
class AutoExplorer:
    def __init__(self, args):
        self.app = mc_sdk_zsl_1_py.HighLevel()
        self.app.initRobot(args.local_ip, args.port, args.dog_ip)
        self.wall_dist = args.wall_dist
        self.hyst = args.hysteresis
        self.safe_front = args.safe_front
        self.lost_left = args.lost_left
        self.side_clear = args.side_clear
        self.speed = args.speed
        self.kp = args.kp
        self.yaw_max = args.yaw_max
        self.turn_yaw = args.turn_yaw
        self.loop_hz = args.loop_hz
        self.front_angle = args.front_angle
        self.confirm_frames = args.confirm_frames
        self.hard_front = args.hard_front
        self.kd = args.kd
        self.turn_vx = args.turn_vx
        self.speed_min = args.speed_min
        self.speed_ramp = args.speed_ramp
        self.spin_max = args.spin_max
        self.spin_straight = args.spin_straight
        self.spin_t = 0.0     # 连续原地旋转计时
        self.seek_t = 0.0     # 强制直行计时
        self.state = 'SEEK_LEFT'
        self.state_t = 0.0
        self.lost_t = 0.0
        self.last_err = None      # PD 阻尼用：上一帧贴墙误差
        self.front_confirm = 0    # 前墙多帧确认计数
        self.running = True

    def run(self):
        time.sleep(1.0)
        if not self.app.checkConnect():
            print('[ERROR] SDK 未连接（检查仿真/mc_ctrl），退出')
            return 1
        print('[INFO] 连接成功，站立...')
        self.app.standUp()
        time.sleep(4)

        rclpy.init()
        perc = LidarPerception(front_angle=self.front_angle)
        print(f'[INFO] 就绪: 贴左墙 {self.wall_dist}m | 前刹 {self.safe_front}m | '
              f'速度 {self.speed}m/s | Ctrl+C 退出')

        period = 1.0 / self.loop_hz
        last_print = 0.0
        try:
            while self.running and rclpy.ok():
                rclpy.spin_once(perc, timeout_sec=0.02)
                sec = perc.sectors()

                if sec is None:
                    # 尚无点云：原地待命（持续发 0 速防止 3 秒趴下）
                    self.app.move(0, 0, 0)
                    time.sleep(period)
                    continue
                d_front, d_left, d_right = sec

                vx, yaw = self._control(d_front, d_left, d_right, period)
                ret = self.app.move(vx, 0, yaw)
                if ret != 0:
                    print(f'[WARN] move 返回 {ret:#x}')

                now = time.time()
                if now - last_print >= 2.0:
                    last_print = now
                    sf = f'{d_front:.2f}' if d_front is not None else '无'
                    sl = f'{d_left:.2f}' if d_left is not None else '无'
                    sr = f'{d_right:.2f}' if d_right is not None else '无'
                    print(f'[STATUS] {self.state} front={sf}m left={sl}m right={sr}m '
                          f'| vx={vx:.2f} yaw={yaw:+.2f}')
                time.sleep(period)
        except KeyboardInterrupt:
            print('\n[INFO] Ctrl+C，安全退出')
        finally:
            self.app.move(0, 0, 0)
            time.sleep(0.8)
            self.app.lieDown()
            time.sleep(0.5)
            perc.destroy_node()
            rclpy.try_shutdown()
            print('[INFO] 已停止并趴下')
        return 0

    def _set_state(self, st):
        if st != self.state:
            self.state, self.state_t = st, 0.0
            self.last_err = None       # 状态切换时清 PD 记忆
            self.front_confirm = 0
            print(f'[STATE] → {st}')

    def _control(self, d_front, d_left, d_right, dt):
        """左手规则状态机 v2 → (vx, yaw)

        SEEK_LEFT:  左侧无近墙 → 弧线左转寻找前方墙（<2.5m）
        APPROACH:   直行逼近前方墙（速度随前方距离自适应）
        TURN_RIGHT: 前方太近 → 弧线右转，把前墙转到左侧（太近时原地转防撞）
        FOLLOW:     左墙 1.2m 附近 → 沿墙走（PD 阻尼 + 右侧净空）
        前墙判定：窄扇区 + 连续 confirm_frames 帧确认，< hard_front 立即触发
        防卡死看门狗：连续原地旋转 > spin_max 秒 → 强制直行脱困 spin_straight 秒"""
        self.state_t += dt

        # ---- 前墙多帧确认（防单帧噪声/侧墙误入前扇区导致直道急停）----
        if d_front is not None and d_front < self.safe_front:
            self.front_confirm += 1
        else:
            self.front_confirm = 0
        front_trigger = (self.front_confirm >= self.confirm_frames
                         or (d_front is not None and d_front < self.hard_front))

        # ---- 状态迁移 ----
        if self.state == 'SEEK_LEFT':
            if d_front is not None and d_front < 2.5:
                self._set_state('APPROACH')
        elif self.state == 'APPROACH':
            if front_trigger:
                self._set_state('TURN_RIGHT')
            elif d_front is None or d_front > 4.0:
                self._set_state('SEEK_LEFT')
        elif self.state == 'TURN_RIGHT':
            if (d_front is None or d_front > 1.2) and d_left is not None and d_left < 2.0:
                self._set_state('FOLLOW')
            elif self.state_t > 6.0:   # 转了 ~180° 还没找到左墙 → 重新寻墙
                self._set_state('SEEK_LEFT')
        elif self.state == 'FOLLOW':
            if front_trigger:
                self._set_state('TURN_RIGHT')
            elif d_left is None or d_left > self.lost_left:
                self.lost_t += dt
                if self.lost_t > 3.0:
                    self._set_state('SEEK_LEFT')
            else:
                self.lost_t = 0.0

        # ---- 动态速度：前方越近越慢（近角提前减速，少急刹）----
        if d_front is None:
            vx = self.speed
        else:
            vx = self.speed * np.clip((d_front - self.safe_front) / self.speed_ramp,
                                      0.0, 1.0)
            vx = max(vx, self.speed_min)

        # ---- 各状态控制输出 ----
        if self.state == 'SEEK_LEFT':
            if d_front is not None and d_front < self.hard_front:
                vx, yaw = 0.0, +self.turn_yaw            # 前墙太近：原地左转
            else:
                vx, yaw = self.turn_vx, +self.turn_yaw   # 弧线左转
        elif self.state == 'APPROACH':
            yaw = 0.0
        elif self.state == 'TURN_RIGHT':
            if d_front is not None and d_front < self.hard_front:
                vx, yaw = 0.0, -self.turn_yaw            # 前墙太近：原地右转
            else:
                vx, yaw = self.turn_vx, -self.turn_yaw   # 弧线右转
        else:  # FOLLOW
            yaw = 0.0
            if d_right is not None and d_right < self.side_clear:
                yaw = +self.kp * (self.side_clear - d_right)
            err = d_left - self.wall_dist
            if err > self.hyst:
                yaw += self.kp * err          # 离墙太远 → 左转靠近
            elif err < -self.hyst:
                yaw += self.kp * err          # 离墙太近 → 右转远离
            # PD 阻尼项：抑制贴墙距离来回振荡（纯 P 会摆）
            if self.last_err is not None:
                yaw += self.kd * (err - self.last_err) / dt
            self.last_err = err
            yaw = float(np.clip(yaw, -self.yaw_max, self.yaw_max))
            if abs(yaw) < 0.02:
                yaw = 0.0

        # ---- 防卡死看门狗 ----
        spinning = (vx == 0.0 and abs(yaw) > 0.3)
        if spinning:
            self.spin_t += dt
            if self.spin_t > self.spin_max:
                self.seek_t += dt
                if self.seek_t < self.spin_straight:
                    print(f'[STATE] 看门狗: 强制直行脱困 {self.seek_t:.0f}s')
                    return self.speed, 0.0
                self.spin_t, self.seek_t = 0.0, 0.0
        else:
            self.spin_t = 0.0
        return vx, yaw


def main():
    ap = argparse.ArgumentParser(description='自主场地遍历（左手规则）')
    ap.add_argument('--local-ip', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=43988)
    ap.add_argument('--dog-ip', default='127.0.0.1')
    ap.add_argument('--wall-dist', type=float, default=1.2, help='目标左墙距离 m')
    ap.add_argument('--hysteresis', type=float, default=0.3, help='距离滞回半带宽 m')
    ap.add_argument('--safe-front', type=float, default=0.8, help='前方刹停距离 m')
    ap.add_argument('--lost-left', type=float, default=4.0, help='左侧判丢墙距离 m')
    ap.add_argument('--side-clear', type=float, default=0.45, help='右侧最小净空 m')
    ap.add_argument('--speed', type=float, default=0.25, help='前进速度 m/s')
    ap.add_argument('--kp', type=float, default=0.6, help='距离误差→yaw 增益')
    ap.add_argument('--yaw-max', type=float, default=0.35, help='yaw 指令上限 rad/s')
    ap.add_argument('--turn-yaw', type=float, default=0.5, help='原地转向速度 rad/s')
    ap.add_argument('--spin-max', type=float, default=10.0, help='连续原地右转超过此秒数触发脱困')
    ap.add_argument('--spin-straight', type=float, default=3.0, help='脱困直行秒数')
    ap.add_argument('--front-angle', type=float, default=12.0,
                    help='前扇区半角（度）。25° 太宽：贴墙修正航向时侧墙会误入前扇区导致直道误刹')
    ap.add_argument('--confirm-frames', type=int, default=3,
                    help='前墙连续确认帧数（防单帧噪声误触发）')
    ap.add_argument('--hard-front', type=float, default=0.55,
                    help='紧急前距 m：低于此值跳过确认立即转向')
    ap.add_argument('--kd', type=float, default=0.5, help='PD 阻尼增益（抑制贴墙振荡）')
    ap.add_argument('--turn-vx', type=float, default=0.15, help='弧线转弯前进速度 m/s（0=原地转）')
    ap.add_argument('--speed-min', type=float, default=0.08, help='逼近前墙时最低速度 m/s')
    ap.add_argument('--speed-ramp', type=float, default=2.0,
                    help='减速斜坡长度 m（前方距离每小 1m 速度按比例降）')
    ap.add_argument('--loop-hz', type=float, default=20.0, help='控制频率 Hz')
    args = ap.parse_args()

    exp = AutoExplorer(args)

    def _sigterm(signum, frame):
        print('\n[INFO] 收到 SIGTERM，安全退出')
        exp.running = False
    signal.signal(signal.SIGTERM, _sigterm)

    sys.exit(exp.run())


if __name__ == '__main__':
    main()
