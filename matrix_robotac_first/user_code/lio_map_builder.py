#!/usr/bin/env python3
"""LIO 累积地图节点：按 FAST-LIO2 位姿把当前帧拼成持久地图

FASTLIO2_ROS2 的 /fastlio2/world_cloud 是"当前帧在世界系"的快照（每帧替换），
rviz 里闪一下留不住。本节点订阅:
    /fastlio2/body_cloud  (PointCloud2, base_link 系, 去畸变后的当前帧)
    /fastlio2/lio_odom    (Odometry, world→base_link 位姿, 与帧同时间戳)
变换到 world 系做 5cm 体素累积（keep-first + 命中计数），
发布 /lio_map（持久地图, 1Hz）供 rviz 显示；Ctrl+C 或 --save-after 保存 PCD。

位姿异常门控（--guard 默认开）：检测 LIO 位姿跳变/旋转突跳/速度尖峰，
锁定期间暂停入图，防漂移帧把地图带飞（止血层，只防污染不治漂移；
渐进漂移的根治在 C++ 层地面约束，后续实施）。

用法:
    source /opt/ros/humble/setup.bash
    python3 lio_map_builder.py --out map.pcd --save-after 1800
"""
import argparse
import bisect
import os
import time
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2 as pc2


def cloud_to_xyz(msg):
    offsets = {f.name: f.offset for f in msg.fields}
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    cols = []
    for name in ('x', 'y', 'z'):
        o = offsets[name]
        cols.append(buf[:, o:o + 4].copy().view(np.float32).reshape(-1))
    return np.stack(cols, axis=1).astype(np.float64)


def quat_to_R(qx, qy, qz, qw):
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


def write_pcd(path, pts):
    pts = np.asarray(pts, dtype=np.float32)
    with open(path, 'w') as f:
        f.write('# .PCD v0.7 - Point Cloud Data file format\n')
        f.write('VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n')
        f.write(f'WIDTH {len(pts)}\nHEIGHT 1\n')
        f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
        f.write(f'POINTS {len(pts)}\nDATA ascii\n')
        np.savetxt(f, pts, fmt='%.4f')


def _quat_inv(q):
    """四元数共轭（单位四元数的逆），q=[x,y,z,w]"""
    return np.array([-q[0], -q[1], -q[2], q[3]])


def _quat_mul(a, b):
    """四元数乘法 a⊗b，a/b=[x,y,z,w]"""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz])


class DegeneracyGuard:
    """LIO 位姿异常门控（止血层）

    检测: 单帧位置跳变 / 旋转突跳 / 速度尖峰（相对近 20 帧均值 N 倍以上），
    外加绝对高度门控（迷宫为平地，LIO z 偏离启动锚定值超 z_band 即异常——
    专治三面墙角落"跳高后停在错误高度"的确定性漂移，z 不回来就一直锁），
    连续 confirm_frames 帧异常后锁定；锁定期间 on_cloud 不入图，
    防止漂移帧把地图带飞。

    定位: 止血层。拦跳变 + 高度漂移（含"跳一次后停在错误高度"模式）；
    x/y 方向的渐进漂移仍需 C++ 层地面约束根治（后续实施）。
    锁定期间位姿基准照常跟进（下一帧相对当前位姿检测），
    连续 release_frames 帧正常后自动解锁继续入图。
    属算法管线内门控（非事后加工），符合 PCD 原生输出红线。

    阈值基准（10Hz 帧率）: pos_jump=0.5m/帧=5m/s 远高于机器人 3m/s 上限；
    rot_jump=15°/帧=150°/s 远高于比赛转弯 0.4rad/s=23°/s；
    z_band=0.3m 远大于步态颠簸 ±0.05m，但小于角落漂移的典型偏移。
    """
    def __init__(self, pos_jump=0.5, rot_jump_deg=15.0, vel_ratio=3.0,
                 confirm_frames=3, release_frames=10, z_band=0.3,
                 log_path=None):
        self.pos_jump = pos_jump
        self.rot_jump_deg = rot_jump_deg
        self.vel_ratio = vel_ratio
        self.confirm_frames = confirm_frames
        self.release_frames = release_frames
        self.z_band = z_band           # 绝对高度门控带 m（0=关闭）
        self.z_anchor = None           # 平地锚定高度（启动期 z 中位数）
        self.z_anchor_frames = 50      # 锚定采样帧数（10Hz 下约 5s）
        self._z_init = []
        self.last_pose = None      # np.array [x,y,z,qx,qy,qz,qw]
        self.last_t = None
        self.vel_hist = []         # 近 20 帧瞬时速度
        self.bad_streak = 0
        self.good_streak = 0
        self.locked = False
        self.skipped = 0
        self.last_reason = ''
        self.log = None
        if log_path:
            self.log = open(log_path, 'w')
            self.log.write('t,skip,pos_jump,rot_jump,vel,mean_vel,locked,bad_streak\n')

    def check(self, pose, t):
        """返回 True = 锁定（调用方跳过本帧入图）。pose=[x,y,z,qx,qy,qz,qw]，t 秒。"""
        if self.last_pose is None:
            self.last_pose = pose
            self.last_t = t
            return False
        dt = t - self.last_t
        if dt <= 1e-6:
            return self.locked        # 时间戳回退/重复：无法判断，维持现状
        dp = pose[:3] - self.last_pose[:3]
        pos_jump = float(np.linalg.norm(dp))
        qr = _quat_mul(_quat_inv(self.last_pose[3:]), pose[3:])
        qw = min(1.0, abs(float(qr[3])))
        rot_jump = 2.0 * np.degrees(np.arccos(qw))
        vel = pos_jump / dt
        self.vel_hist.append(vel)
        if len(self.vel_hist) > 20:
            self.vel_hist.pop(0)
        mean_vel = float(np.mean(self.vel_hist)) if self.vel_hist else 0.0

        abnormal = (pos_jump > self.pos_jump
                    or rot_jump > self.rot_jump_deg
                    or (mean_vel > 0.01 and vel > self.vel_ratio * mean_vel))
        reason = (f'pos_jump={pos_jump:.2f}m rot_jump={rot_jump:.1f}° '
                  f'vel={vel:.2f}m/s(mean {mean_vel:.2f})')

        # 绝对高度门控：迷宫为平地，LIO z 偏离启动锚定值超带 → 异常
        # （跳一次后停在错误高度会持续触发，z 恢复前一直锁定）
        if self.z_band > 0:
            if len(self._z_init) < self.z_anchor_frames:
                self._z_init.append(float(pose[2]))
                if len(self._z_init) == self.z_anchor_frames:
                    self.z_anchor = float(np.median(self._z_init))
            elif self.z_anchor is not None and \
                    abs(float(pose[2]) - self.z_anchor) > self.z_band:
                abnormal = True
                reason += f' z_off={float(pose[2]) - self.z_anchor:+.2f}m'

        if abnormal:
            self.good_streak = 0
            self.bad_streak += 1
            if self.bad_streak >= self.confirm_frames:
                self.locked = True
            self.last_reason = reason
        else:
            if self.locked:
                self.good_streak += 1
                if self.good_streak >= self.release_frames:
                    self.locked = False
                    self.bad_streak = 0
                    self.good_streak = 0
            else:
                self.bad_streak = 0

        # 基准照常跟进（即使锁定）：下一帧相对当前位置检测
        self.last_pose = pose
        self.last_t = t
        if self.locked:
            self.skipped += 1
        if self.log is not None:
            self.log.write(f'{t:.3f},{1 if self.locked else 0},{pos_jump:.4f},'
                           f'{rot_jump:.3f},{vel:.4f},{mean_vel:.4f},'
                           f'{self.locked},{self.bad_streak}\n')
            self.log.flush()
        return self.locked


class LioMapBuilder(Node):
    def __init__(self, args):
        super().__init__('lio_map_builder')
        self.voxel = args.voxel
        self.out_path = args.out
        self.save_after = args.save_after
        self.min_hits = args.min_hits
        self.saved = False
        self.voxel_map = {}            # key -> [x, y, z, hits]
        self._odo_t = []
        self._odo_pose = []
        self.last_stamp = None
        self.frame_count = 0
        self.guard = None
        if args.guard:
            self.guard = DegeneracyGuard(
                args.guard_pos_jump, args.guard_rot_jump, args.guard_vel_ratio,
                args.guard_confirm, args.guard_release, args.guard_z_band,
                args.guard_log or None)

        self.sub_cloud = self.create_subscription(
            PointCloud2, '/fastlio2/body_cloud', self.on_cloud, 10)
        self.sub_odom = self.create_subscription(
            Odometry, '/fastlio2/lio_odom', self.on_odom, 200)
        self.pub_map = self.create_publisher(PointCloud2, '/lio_map', 10)
        self.timer = self.create_timer(1.0 / args.publish_hz, self.publish_map)
        self.start_time = self.get_clock().now()
        self.get_logger().info(
            f'LIO 累积地图就绪: voxel={self.voxel}m out={self.out_path} '
            f'save_after={self.save_after}s')

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        i = bisect.bisect_right(self._odo_t, t_ns)
        self._odo_t.insert(i, t_ns)
        self._odo_pose.insert(i, (p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        if len(self._odo_t) > 2000:
            del self._odo_t[:len(self._odo_t) - 2000]
            del self._odo_pose[:len(self._odo_pose) - 2000]

    def on_cloud(self, msg):
        t_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if not self._odo_t:
            return
        # 按时间戳取 LIO 位姿（LIO 在同一回调里发 odom 和帧，戳一致）
        i = bisect.bisect_left(self._odo_t, t_ns)
        if i == len(self._odo_t):
            i -= 1
        elif i > 0 and abs(self._odo_t[i - 1] - t_ns) < abs(self._odo_t[i] - t_ns):
            i -= 1
        x, y, z, qx, qy, qz, qw = self._odo_pose[i]
        # 退化门控：跳变/速度尖峰时暂停入图（防地图污染，止血）
        if self.guard is not None:
            if self.guard.check(np.array([x, y, z, qx, qy, qz, qw]), t_ns * 1e-9):
                self.get_logger().warn(
                    f'退化门控锁定: 跳过本帧入图（{self.guard.last_reason}，'
                    f'已拦 {self.guard.skipped} 帧）',
                    throttle_duration_sec=2.0)
                return
        R = quat_to_R(qx, qy, qz, qw)
        t = np.array([x, y, z])

        pts = cloud_to_xyz(msg)
        if len(pts) == 0:
            return
        pts_w = (R @ pts.T).T + t

        keys = (pts_w / self.voxel).astype(np.int64)
        for k, p in zip(keys, pts_w):
            kt = (k[0], k[1], k[2])
            v = self.voxel_map.get(kt)
            if v is None:
                self.voxel_map[kt] = [p[0], p[1], p[2], 1]
            else:
                v[3] += 1

        self.frame_count += 1
        self.last_stamp = self.get_clock().now()
        if (self.save_after > 0 and not self.saved
                and (self.get_clock().now() - self.start_time).nanoseconds / 1e9 >= self.save_after):
            self.save_pcd()

    def publish_map(self):
        if not self.voxel_map or self.last_stamp is None:
            return
        header = Header(frame_id='world')
        header.stamp = self.last_stamp.to_msg()
        vals = [v for v in self.voxel_map.values() if v[3] >= self.min_hits]
        if not vals:
            return
        pts = np.array(vals, dtype=np.float32)[:, :3]
        self.pub_map.publish(pc2.create_cloud_xyz32(header, pts))

    def save_pcd(self):
        if self.saved or not self.voxel_map:
            return
        vals = [v for v in self.voxel_map.values() if v[3] >= self.min_hits]
        pts = np.array(vals, dtype=np.float32)[:, :3]
        write_pcd(self.out_path, pts)
        self.saved = True
        skipped = getattr(self.guard, 'skipped', 0)
        self.get_logger().info(
            f'已保存 PCD: {self.out_path}（{len(pts)} 点，{self.frame_count} 帧，'
            f'门控拦截 {skipped} 帧，'
            f'命中 <{self.min_hits} 次的体素已剔除 {len(self.voxel_map) - len(pts)} 点）')


def main():
    ap = argparse.ArgumentParser(description='LIO 累积地图（显示 + PCD 输出）')
    ap.add_argument('--voxel', type=float, default=0.05, help='体素大小 m')
    ap.add_argument('--min-hits', type=int, default=2,
                    help='体素命中次数 < 此值不保存/不显示（时间共识滤波：'
                         '实测 3-7cm 分层里有大量单次命中噪声，≥2 次才保留）')
    ap.add_argument('--out', default='',
                    help='PCD 输出路径（空=自动生成时间戳文件名到 ~/robotac_maps/，'
                         '每次运行互不覆盖）')
    ap.add_argument('--save-after', type=float, default=0,
                    help='运行 N 秒后自动保存（0=仅 Ctrl+C 保存）')
    ap.add_argument('--publish-hz', type=float, default=1.0, help='/lio_map 发布频率')
    ap.add_argument('--guard', type=int, default=1, choices=[0, 1],
                    help='退化门控开关（1=开，默认；检测位姿跳变/速度尖峰，锁定期间暂停入图）')
    ap.add_argument('--guard-pos-jump', type=float, default=0.5,
                    help='单帧位置跳变阈值 m（10Hz 下 = 5m/s，远高于机器人 3m/s 上限）')
    ap.add_argument('--guard-rot-jump', type=float, default=15.0,
                    help='单帧旋转跳变阈值 度（10Hz 下 = 150°/s）')
    ap.add_argument('--guard-vel-ratio', type=float, default=3.0,
                    help='速度尖峰倍数（相对近 20 帧均值；均值 <0.01m/s 时不启用此项）')
    ap.add_argument('--guard-confirm', type=int, default=3,
                    help='连续异常 N 帧后锁定')
    ap.add_argument('--guard-release', type=int, default=10,
                    help='连续正常 N 帧后解锁')
    ap.add_argument('--guard-z-band', type=float, default=0.3,
                    help='绝对高度门控带 m（0=关；迷宫平地，LIO z 偏离启动锚定值超带即锁定，'
                         '专治角落确定性高度漂移）')
    ap.add_argument('--guard-log', default='',
                    help='门控 CSV 日志路径（空=不写；事后分析退化时空分布用）')
    args = ap.parse_args()
    if not args.out:
        out_dir = os.path.expanduser('~/robotac_maps')
        os.makedirs(out_dir, exist_ok=True)
        args.out = os.path.join(
            out_dir, f"lio_map_{datetime.now().strftime('%Y%m%d_%H%M%S')}.pcd")
    rclpy.init()
    node = LioMapBuilder(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        pass
    finally:
        node.save_pcd()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
