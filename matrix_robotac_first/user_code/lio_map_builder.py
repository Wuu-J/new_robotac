#!/usr/bin/env python3
"""LIO 累积地图节点：按 FAST-LIO2 位姿把当前帧拼成持久地图

FASTLIO2_ROS2 的 /fastlio2/world_cloud 是"当前帧在世界系"的快照（每帧替换），
rviz 里闪一下留不住。本节点订阅:
    /fastlio2/body_cloud  (PointCloud2, base_link 系, 去畸变后的当前帧)
    /fastlio2/lio_odom    (Odometry, world→base_link 位姿, 与帧同时间戳)
变换到 world 系做 5cm 体素累积（keep-first + 命中计数），
发布 /lio_map（持久地图, 1Hz）供 rviz 显示；Ctrl+C 或 --save-after 保存 PCD。

用法:
    source /opt/ros/humble/setup.bash
    python3 lio_map_builder.py --out map.pcd --save-after 1800
"""
import argparse
import bisect
import time

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
        self.get_logger().info(
            f'已保存 PCD: {self.out_path}（{len(pts)} 点，{self.frame_count} 帧，'
            f'命中 <{self.min_hits} 次的体素已剔除 {len(self.voxel_map) - len(pts)} 点）')


def main():
    ap = argparse.ArgumentParser(description='LIO 累积地图（显示 + PCD 输出）')
    ap.add_argument('--voxel', type=float, default=0.05, help='体素大小 m')
    ap.add_argument('--min-hits', type=int, default=2,
                    help='体素命中次数 < 此值不保存/不显示（时间共识滤波：'
                         '实测 3-7cm 分层里有大量单次命中噪声，≥2 次才保留）')
    ap.add_argument('--out', default='lio_map.pcd', help='PCD 输出路径')
    ap.add_argument('--save-after', type=float, default=0,
                    help='运行 N 秒后自动保存（0=仅 Ctrl+C 保存）')
    ap.add_argument('--publish-hz', type=float, default=1.0, help='/lio_map 发布频率')
    args = ap.parse_args()
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
