#!/usr/bin/env python3
"""odom 累积建图节点（中期检查，关键帧 scan-to-map 点对平面 ICP 精配准）

数据流:
    /front_lidar (PointCloud2, frame=lidar)  +  /odom/mujoco_odom (frame=world)
        → 每帧按 odom 位姿 ∘ 运行修正量 变换到 world 系
        → 关键帧触发时：与最近子图做点对平面 ICP（trimmed，按需法线）
          修正 odom 的微小累积误差（平面切向滑动是点对点 ICP 的经典失效模式）
        → 体素降采样累积成全局地图
        → 发布 /slam_map 供 rviz2 实时显示；Ctrl+C 或 --save-after 保存 PCD

用法（实时仿真）:
    source /opt/ros/humble/setup.bash
    python3 slam_odom_mapper.py --out map.pcd --save-after 1500

用法（bag 回放）:
    python3 slam_odom_mapper.py --out map.pcd --ros-args -p use_sim_time:=true
"""
import argparse
import bisect
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
import tf2_ros
from scipy.spatial import cKDTree
from sensor_msgs_py import point_cloud2 as pc2


# ---------------------------------------------------------------- 工具函数
def quat_to_R(qx, qy, qz, qw):
    """四元数 → 3x3 旋转矩阵（ROS 顺序 xyzw）"""
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


def cloud_to_xyz(msg):
    """PointCloud2 → Nx3 float64（按 msg.fields 找 x/y/z 偏移，支持 height=1）"""
    offsets = {f.name: f.offset for f in msg.fields}
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    cols = []
    for name in ('x', 'y', 'z'):
        o = offsets[name]
        cols.append(buf[:, o:o + 4].copy().view(np.float32).reshape(-1))
    return np.stack(cols, axis=1).astype(np.float64)


def voxel_downsample(pts, voxel):
    """体素降采样（keep-first）。pts: Nx3"""
    keys = (pts / voxel).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[idx]


def rotate_points(R, t, pts):
    return (R @ pts.T).T + t


def exp_so3(omega):
    """罗德里格斯：旋转向量 → 旋转矩阵（小角度线性化用）"""
    theta = np.linalg.norm(omega)
    if theta < 1e-9:
        return np.eye(3)
    k = omega / theta
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)


def estimate_normals_25d(tree, idxs, k=8):
    """按需估计 2.5D 法线（迷宫=竖直墙+水平地面）：
    对给定目标点索引，取 k 近邻，xy 平面 2D PCA 得水平法线；
    邻域 z 方差显著大于水平方差时判为地面/顶面，法线取竖直方向。"""
    pts = tree.data
    neighbors = tree.query(pts[idxs], k=k + 1)[1][:, 1:]  # 去掉自身
    nb = pts[neighbors]                                    # (M, k, 3)
    mu_xy = nb[:, :, :2].mean(axis=1, keepdims=True)
    d_xy = nb[:, :, :2] - mu_xy
    cov = np.einsum('nki,nkj->nij', d_xy, d_xy) / k        # (M, 2, 2)
    a, b, c = cov[:, 0, 0], cov[:, 0, 1], cov[:, 1, 1]
    # 2x2 最小特征向量（闭式）
    lam = (a + c - np.sqrt((a - c) ** 2 + 4 * b * b)) / 2
    v = np.stack([b, lam - a], axis=1)                     # (M, 2)
    n_xy = v / np.maximum(np.linalg.norm(v, axis=1, keepdims=True), 1e-9)
    normals = np.hstack([n_xy, np.zeros((len(idxs), 1))])
    # 竖直面判据：邻域 z 方差 > 2× 水平协方差迹
    var_z = nb[:, :, 2].var(axis=1)
    var_xy = cov[:, 0, 0] + cov[:, 1, 1]
    vert = var_z > 2 * var_xy
    normals[vert] = [0, 0, 1]
    return normals


def icp_pt2plane(tree, src, max_iter=20, max_dist=0.5, trim=0.9,
                 min_inliers=300, k_normals=8):
    """线性化点对平面 ICP：src 向 tree 配准，返回 (R, t) 使 tree ≈ R·src + t。
    外点按残差裁剪（trim 比例）；法线按需对匹配点估计。"""
    R_total, t_total = np.eye(3), np.zeros(3)
    cur = src
    for _ in range(max_iter):
        d, idx = tree.query(cur, k=1, distance_upper_bound=max_dist)
        mask = np.isfinite(d)
        if mask.sum() < min_inliers:
            return None
        s = cur[mask]
        tgt = tree.data[idx[mask]]
        # 按需估计匹配目标点的法线（按唯一索引算，减少重复开销）
        uniq, inv = np.unique(idx[mask], return_inverse=True)
        n_uniq = estimate_normals_25d(tree, uniq, k=k_normals)
        n = n_uniq[inv]
        # 残差 = n·(s - tgt)
        r = np.einsum('ij,ij->i', s - tgt, n)
        # 裁剪外点：保留 |r| 最小的 trim 比例
        if trim < 1.0:
            keep = int(len(r) * trim)
            sel = np.argsort(np.abs(r))[:keep]
            s, tgt, n, r = s[sel], tgt[sel], n[sel], r[sel]
        # 线性化系统: n·(R s + t - tgt) ≈ n·(ω×s + t + s - tgt) = 0
        #   → [ (s×n)ᵀ  nᵀ ] · [ω; t] = -n·(s - tgt)
        A = np.hstack([np.cross(s, n), n])   # (N, 6)
        b = -r
        x, *_ = np.linalg.lstsq(A, b, rcond=None)
        omega, t = x[:3], x[3:]
        dR = exp_so3(omega)
        cur = rotate_points(dR, t, cur)
        R_total = dR @ R_total
        t_total = dR @ t_total + t
    return R_total, t_total


def write_pcd(path, pts):
    """写出 ASCII PCD（FIELDS x y z）。pts: Nx3"""
    pts = np.asarray(pts, dtype=np.float32)
    with open(path, 'w') as f:
        f.write('# .PCD v0.7 - Point Cloud Data file format\n')
        f.write('VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n')
        f.write(f'WIDTH {len(pts)}\nHEIGHT 1\n')
        f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
        f.write(f'POINTS {len(pts)}\nDATA ascii\n')
        np.savetxt(f, pts, fmt='%.4f')


# ---------------------------------------------------------------- 建图节点
class OdometryMapper(Node):
    def __init__(self, args):
        super().__init__('slam_odom_mapper')
        # 注: use_sim_time 由 rclpy 按命令行 --ros-args -p use_sim_time:=true 自动处理

        self.voxel = args.voxel
        self.max_range = args.max_range
        self.min_range = args.min_range
        self.z_min = args.z_min
        self.z_max = args.z_max
        self.mount = np.array(args.mount, dtype=np.float64)
        self.out_path = args.out
        self.save_after = args.save_after
        self.saved = False

        # 2D 占据栅格（/map_2d，供 frontier 探索）：空闲=机器人经过，墙=≥2 次命中
        self.grid_res = args.grid_res
        # 注: argparse 不对 default 做 type 转换，必须手动转 float
        self.grid_xmin, self.grid_xmax, self.grid_ymin, self.grid_ymax = \
            (float(v) for v in args.grid_bounds)
        self.grid_W = int(round((self.grid_xmax - self.grid_xmin) / self.grid_res))
        self.grid_H = int(round((self.grid_ymax - self.grid_ymin) / self.grid_res))
        self.grid_free = np.zeros((self.grid_H, self.grid_W), dtype=bool)
        self.grid_hits = np.zeros((self.grid_H, self.grid_W), dtype=np.uint16)

        # ICP 参数
        self.use_icp = args.icp
        self.icp_voxel = args.icp_voxel
        self.icp_max_dist = args.icp_max_dist
        self.icp_trim = args.icp_trim
        self.icp_max_angle = args.icp_max_angle
        self.icp_max_trans = args.icp_max_trans
        self.kf_dist = args.kf_dist       # 关键帧触发位移 m
        self.kf_yaw = args.kf_yaw         # 关键帧触发角度 rad
        self.icp_stats = {'n': 0, 'skip': 0, 'corr': []}

        # 运行修正量（odom 先验的累积修正）：T_run = T_c ∘ T_run
        self.R_run = np.eye(3)
        self.t_run = np.zeros(3)

        # 关键帧状态
        self.submap_ring = deque(maxlen=args.icp_ring)  # 最近关键帧的修正后降采样点
        self.submap_tree = None
        self.tree_dirty = True
        self.last_kf_R = None
        self.last_kf_t = None
        self.odom_trail = deque(maxlen=80)   # 最近 odom (t, R, t)，运动门控用

        # 全局体素地图：voxel_key -> (x, y, z)（keep-first）
        self.voxel_map = {}
        self.last_cloud_stamp = None

        # odom 环形缓冲（时间对齐）
        self._odo_t = []
        self._odo_pose = []

        be_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_cloud = self.create_subscription(
            PointCloud2, '/front_lidar', self.on_cloud, be_qos)
        self.sub_odom = self.create_subscription(
            Odometry, '/odom/mujoco_odom', self.on_odom,
            QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.pub_map = self.create_publisher(PointCloud2, '/slam_map', 10)
        self.pub_grid = self.create_publisher(OccupancyGrid, '/map_2d', 10)
        self.timer = self.create_timer(1.0 / args.publish_hz, self.publish_map)

        # TF 发布（仿真无 TF 树，rviz 报 "Frame [lidar] does not exist" 就是缺它）：
        #   静态 base_link→lidar = lidar 安装偏移；动态 odom frame_id→child_frame_id
        self.tf_br = tf2_ros.TransformBroadcaster(self)
        self.tf_static_br = tf2_ros.StaticTransformBroadcaster(self)
        st = TransformStamped()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id = 'base_link'
        st.child_frame_id = 'lidar'
        st.transform.translation.x = float(self.mount[0])
        st.transform.translation.y = float(self.mount[1])
        st.transform.translation.z = float(self.mount[2])
        st.transform.rotation.w = 1.0
        self.tf_static_br.sendTransform(st)

        self.start_time = self.get_clock().now()
        self.frame_count = 0
        self.get_logger().info(
            f'建图节点就绪: voxel={self.voxel}m icp={self.use_icp} '
            f'关键帧触发={self.kf_dist}m/{np.degrees(self.kf_yaw):.1f}° '
            f'out={self.out_path} save_after={self.save_after}s')

    # ------------------------------------------------------------ 订阅
    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        i = bisect.bisect_right(self._odo_t, t_ns)
        self._odo_t.insert(i, t_ns)
        self._odo_pose.insert(i, (p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        if len(self._odo_t) > 4000:
            del self._odo_t[0]
            del self._odo_pose[0]

        # 同步发布 TF（world→base_link），rviz 渲染 lidar 系点云靠这条链
        tfm = TransformStamped()
        tfm.header.stamp = self.get_clock().now().to_msg()
        tfm.header.frame_id = msg.header.frame_id
        tfm.child_frame_id = msg.child_frame_id
        tfm.transform.translation.x = p.x
        tfm.transform.translation.y = p.y
        tfm.transform.translation.z = p.z
        tfm.transform.rotation.x = q.x
        tfm.transform.rotation.y = q.y
        tfm.transform.rotation.z = q.z
        tfm.transform.rotation.w = q.w
        self.tf_br.sendTransform(tfm)

    def _is_moving(self):
        """运动门控：最近 0.5 秒 odom 位移 > 2cm 才算在动（静止时不做 ICP，防噪声注入）"""
        if len(self.odom_trail) < 10:
            return True
        p0 = self.odom_trail[0][2]
        p1 = self.odom_trail[-1][2]
        return np.linalg.norm(p1 - p0) > 0.02

    def on_cloud(self, msg):
        t_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if not self._odo_t:
            self.get_logger().warn('收到点云但没有 odom 数据，跳过', throttle_duration_sec=5)
            return
        i = bisect.bisect_left(self._odo_t, t_ns)
        if i == len(self._odo_t):
            i -= 1
        elif i > 0 and abs(self._odo_t[i - 1] - t_ns) < abs(self._odo_t[i] - t_ns):
            i -= 1
        x, y, z, qx, qy, qz, qw = self._odo_pose[i]
        R_odo = quat_to_R(qx, qy, qz, qw)
        t_odo = np.array([x, y, z])
        self.odom_trail.append((t_ns, R_odo, t_odo))

        pts = cloud_to_xyz(msg)
        r = np.linalg.norm(pts, axis=1)
        pts = pts[(r > self.min_range) & (r < self.max_range) & np.isfinite(r)]
        if len(pts) == 0:
            return

        # odom 先验 + 运行修正量
        pts = rotate_points(self.R_run, self.t_run,
                            rotate_points(R_odo, t_odo, pts + self.mount))
        # 野点过滤：迷宫高度约 3~4m，z 越界点（漏到天空/地下的射线）直接剔除
        pts = pts[(pts[:, 2] > self.z_min) & (pts[:, 2] < self.z_max)]

        # ---- 2D 占据栅格（frontier 探索用）----
        # 墙带命中计数（z 0.5~2.5 排除地面点；≥2 次命中才判为墙，滤单点噪声）
        wall = pts[(pts[:, 2] > 0.5) & (pts[:, 2] < 2.5)]
        if len(wall):
            ix = np.clip(((wall[:, 0] - self.grid_xmin) / self.grid_res).astype(np.int64),
                         0, self.grid_W - 1)
            iy = np.clip(((wall[:, 1] - self.grid_ymin) / self.grid_res).astype(np.int64),
                         0, self.grid_H - 1)
            u, c = np.unique(iy * self.grid_W + ix, return_counts=True)
            self.grid_hits.flat[u] += c.astype(np.uint16)
        # 机器人经过位置标空闲（1.0m 圆盘：要盖住到达目标时 ~0.6m 外的前沿格，
        # 否则前沿格永不转已知、探索器在原地振荡；圆盘大=跳跃大=探索快。
        # 墙优先级更高不会造洞，且 A* 前有 0.4m 膨胀会封住假空闲小孔）
        cx = int((t_odo[0] - self.grid_xmin) / self.grid_res)
        cy = int((t_odo[1] - self.grid_ymin) / self.grid_res)
        r_cells = int(np.ceil(1.0 / self.grid_res))
        if 0 <= cx < self.grid_W and 0 <= cy < self.grid_H:
            self.grid_free[max(0, cy - r_cells):min(self.grid_H, cy + r_cells + 1),
                           max(0, cx - r_cells):min(self.grid_W, cx + r_cells + 1)] = True

        # ---- 关键帧触发 ICP ----
        if self.use_icp:
            trigger = (self.last_kf_R is None)
            if not trigger:
                dR = self.last_kf_R.T @ R_odo
                ang = np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1))
                disp = np.linalg.norm(t_odo - self.last_kf_t)
                trigger = disp > self.kf_dist or ang > self.kf_yaw
            if trigger and self._is_moving() and len(self.submap_ring) > 1:
                src = voxel_downsample(pts, self.icp_voxel)
                if self.tree_dirty:
                    sub = np.vstack(self.submap_ring)
                    if len(sub) > 1000:
                        self.submap_tree = cKDTree(sub)
                        self.tree_dirty = False
                if self.submap_tree is not None:
                    res = icp_pt2plane(self.submap_tree, src,
                                       max_dist=self.icp_max_dist,
                                       trim=self.icp_trim)
                    if res is not None:
                        R_c, t_c = res
                        angle = np.arccos(np.clip((np.trace(R_c) - 1) / 2, -1, 1))
                        if angle < self.icp_max_angle and np.linalg.norm(t_c) < self.icp_max_trans:
                            # 更新运行修正量并重新变换当前帧
                            self.R_run = R_c @ self.R_run
                            self.t_run = R_c @ self.t_run + t_c
                            pts = rotate_points(R_c, t_c, pts)
                            self.icp_stats['n'] += 1
                            self.icp_stats['corr'].append((angle, np.linalg.norm(t_c)))
                        else:
                            self.icp_stats['skip'] += 1
                    else:
                        self.icp_stats['skip'] += 1
                # 无论是否修正，都记录关键帧（修正后的点）
                self.submap_ring.append(voxel_downsample(pts, self.icp_voxel))
                self.tree_dirty = True
                self.last_kf_R = R_odo
                self.last_kf_t = t_odo
            elif trigger:
                # 第一个关键帧/子图不足：只记录不配准
                self.submap_ring.append(voxel_downsample(pts, self.icp_voxel))
                self.tree_dirty = True
                self.last_kf_R = R_odo
                self.last_kf_t = t_odo

        # ---- 累积到全局体素地图 ----
        keys = (pts / self.voxel).astype(np.int64)
        for k, p in zip(keys, pts):
            kt = (k[0], k[1], k[2])
            if kt not in self.voxel_map:
                self.voxel_map[kt] = p

        self.frame_count += 1
        self.last_cloud_stamp = self.get_clock().now()
        if self.frame_count % 200 == 0 and self.icp_stats['corr']:
            c = np.array(self.icp_stats['corr'][-200:])
            self.get_logger().info(
                f'[ICP] 修正{self.icp_stats["n"]}次/放弃{self.icp_stats["skip"]}次 | '
                f'最近{len(c)}次: 角度 均值={np.degrees(c[:,0].mean()):.3f}° '
                f'最大={np.degrees(c[:,0].max()):.3f}° | 位移 均值={c[:,1].mean()*100:.1f}cm '
                f'最大={c[:,1].max()*100:.1f}cm | 地图={len(self.voxel_map)}点')

        if (self.save_after > 0 and not self.saved
                and (self.get_clock().now() - self.start_time).nanoseconds / 1e9 >= self.save_after):
            self.save_pcd()

    # ------------------------------------------------------------ 输出
    def publish_map(self):
        header = Header(frame_id='world')
        if self.last_cloud_stamp is not None:
            header.stamp = self.last_cloud_stamp.to_msg()

        # 2D 占据栅格（frontier 探索用）：空闲=机器人经过，墙=≥2 次命中，其余未知。
        # 注意：墙优先级高于空闲（圆盘不能把已确认的墙抹成空闲，否则 A* 会穿墙）
        g = np.full((self.grid_H, self.grid_W), -1, dtype=np.int8)
        g[self.grid_free & ~(self.grid_hits >= 2)] = 0
        g[self.grid_hits >= 2] = 100
        gm = OccupancyGrid()
        gm.header = header
        gm.info.resolution = self.grid_res
        gm.info.width = self.grid_W
        gm.info.height = self.grid_H
        gm.info.origin.position.x = self.grid_xmin
        gm.info.origin.position.y = self.grid_ymin
        gm.info.origin.orientation.w = 1.0
        gm.data = g.ravel().tolist()
        self.pub_grid.publish(gm)

        if not self.voxel_map:
            return
        pts = np.array(list(self.voxel_map.values()), dtype=np.float32)
        msg = pc2.create_cloud_xyz32(header, pts)
        self.pub_map.publish(msg)

    def save_pcd(self):
        if self.saved:
            return
        n = len(self.voxel_map)
        if n == 0:
            self.get_logger().warn('地图为空，不保存 PCD')
            return
        pts = np.array(list(self.voxel_map.values()))
        write_pcd(self.out_path, pts)
        self.saved = True
        icp_info = (f', ICP 修正 {self.icp_stats["n"]} 次/放弃 {self.icp_stats["skip"]} 次'
                    if self.use_icp else '')
        self.get_logger().info(
            f'已保存 PCD: {self.out_path}（{n} 点，{self.frame_count} 帧{icp_info}）')


def main():
    ap = argparse.ArgumentParser(description='odom 累积建图（关键帧点对平面 ICP）')
    ap.add_argument('--voxel', type=float, default=0.05, help='地图体素大小 m（默认 0.05）')
    ap.add_argument('--max-range', type=float, default=15.0,
                    help='最大测距 m（实测：远点角度误差放大、墙线变糊且计入噪声点扣分，'
                         '15m 内点精度明显更好；探索全覆盖后不影响覆盖率）')
    ap.add_argument('--min-range', type=float, default=0.2, help='最小测距 m')
    ap.add_argument('--z-min', type=float, default=-0.5, help='世界系 z 下界 m（滤野点）')
    ap.add_argument('--z-max', type=float, default=4.0, help='世界系 z 上界 m（滤野点）')
    ap.add_argument('--mount', type=float, nargs=3, default=[0, 0, 0.3],
                    help='lidar 在 base 系安装偏移 (x y z)，默认 config.json 的 0 0 0.3')
    ap.add_argument('--out', default='map.pcd', help='PCD 输出路径')
    ap.add_argument('--save-after', type=float, default=0,
                    help='运行 N 秒后自动保存并继续（0=仅 Ctrl+C 保存）')
    ap.add_argument('--publish-hz', type=float, default=1.0, help='/slam_map 发布频率')
    ap.add_argument('--grid-res', type=float, default=0.1,
                    help='2D 占据栅格分辨率 m（/map_2d，供 frontier 探索）')
    ap.add_argument('--grid-bounds', type=float, nargs=4, default=[-5, 35, -10, 30],
                    help='2D 栅格范围 (xmin xmax ymin ymax)，需覆盖迷宫+出生区')
    ap.add_argument('--icp', type=int, default=0, choices=[0, 1],
                    help='是否启用 ICP（默认 0 关闭：实测 odom 无时间滞后、配准已足够准，'
                         'ICP 反而注入漂移；如需试验可开 1）')
    ap.add_argument('--icp-voxel', type=float, default=0.15, help='ICP 降采样体素 m（默认 0.15）')
    ap.add_argument('--icp-max-dist', type=float, default=0.5, help='ICP 对应点最大距离 m')
    ap.add_argument('--icp-trim', type=float, default=0.9, help='ICP 残差裁剪保留比例（默认 0.9）')
    ap.add_argument('--icp-max-angle', type=float, default=0.08, help='单次修正角度上限 rad')
    ap.add_argument('--icp-max-trans', type=float, default=0.25, help='单次修正位移上限 m')
    ap.add_argument('--icp-ring', type=int, default=40, help='ICP 子图关键帧数')
    ap.add_argument('--kf-dist', type=float, default=0.25, help='关键帧触发位移 m')
    ap.add_argument('--kf-yaw', type=float, default=0.07, help='关键帧触发角度 rad（约 4°）')
    args = ap.parse_args(remove_ros_args()[1:])

    rclpy.init()
    node = OdometryMapper(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        # Ctrl+C 恰好落在消息反序列化内部时，rclpy(Humble) 会抛出
        # RuntimeError: Unable to convert call argument to Python object（pybind11 中断伪影）。
        # 实测 PCD 仍正常保存，此处静默退出即可。
        pass
    finally:
        node.save_pcd()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
