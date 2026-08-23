#!/usr/bin/env python3
"""仿真传感器 → FAST-LIO2 桥接节点

FASTLIO2_ROS2（SadCream fork）只订阅 livox_ros_driver2/CustomMsg（Livox 私有消息），
且 IMU 回调里把加速度 ×10（Livox 驱动按 g 单位输出，×10 还原 m/s²）。
本仿真平台：/front_lidar 是标准 PointCloud2、/imu 加速度是 m/s²。
本节点做两件事：

1. /front_lidar (PointCloud2, BEST_EFFORT) → /livox/lidar (CustomMsg)
   - offset_time 全 0（仿真逐点时间戳为 0，无法做逐点去畸变；
     fork 按 last 点 curvature 算帧时长，0 = 帧时长 0，预积分安全）
   - line=0、tag=0 满足 fork 的点过滤条件 (line<4 且 tag&0x30∈{0x00,0x10})
   - lidar_filter_num=6 由 fork 端降采样，桥接全量转发
2. /imu (BEST_EFFORT) → /livox/imu：加速度 /10 转 g 单位（fork 会 ×10 还原）

用法（需先编译 livox_ros_driver2 并 source ~/livox_ws/install/setup.bash）:
    python3 livox_bridge.py
    python3 livox_bridge.py --lidar-in /front_lidar --imu-in /imu
"""
import argparse

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, Imu
from livox_ros_driver2.msg import CustomMsg, CustomPoint


class LivoxBridge(Node):
    def __init__(self, args):
        super().__init__('livox_bridge')
        self.acc_g = args.acc_g
        self.sweep_ms = args.sweep_ms   # 帧扫掠时长 ms（实测：仿真帧是真实旋转扫描）

        be_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_lidar = self.create_subscription(
            PointCloud2, args.lidar_in, self.on_lidar, be_qos)
        self.sub_imu = self.create_subscription(
            Imu, args.imu_in, self.on_imu, be_qos)
        # 下游 FAST-LIO2 用默认 QoS（RELIABLE）订阅，发布侧也用默认
        self.pub_lidar = self.create_publisher(CustomMsg, args.lidar_out, 10)
        self.pub_imu = self.create_publisher(Imu, args.imu_out, 10)
        self.get_logger().info(
            f'桥接就绪: {args.lidar_in}→{args.lidar_out} (CustomMsg), '
            f'{args.imu_in}→{args.imu_out} (acc/10→g)')

    def on_lidar(self, msg):
        if self.pub_lidar.get_subscription_count() == 0:
            return
        # PointCloud2 → xyz float32 数组（按 fields 找偏移）
        offsets = {f.name: f.offset for f in msg.fields}
        buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
        cols = [buf[:, offsets[n]:offsets[n] + 4].copy().view(np.float32)
                for n in ('x', 'y', 'z')]

        out = CustomMsg()
        out.header = msg.header
        out.point_num = len(cols[0])
        out.lidar_id = 0
        # 逐点时间：默认全 0（实测仿真帧为"整帧冻结"快照，无帧内畸变——
        # 相邻帧残差呈均匀平移型而非剪切型；给假扫掠时间反而引入假畸变）。
        # 若后续确认帧为真实旋转扫描，可用 --sweep-ms 100 开启方位角线性映射
        if self.sweep_ms > 0:
            az = np.arctan2(cols[1].astype(np.float64), cols[0].astype(np.float64))
            off_ns = ((az + np.pi) / (2.0 * np.pi) * self.sweep_ms * 1e6).astype(np.uint32)
        else:
            off_ns = np.zeros(len(cols[0]), dtype=np.uint32)
        for i in range(out.point_num):
            p = CustomPoint()
            p.offset_time = int(off_ns[i])   # 单位：纳秒
            p.x = float(cols[0][i])
            p.y = float(cols[1][i])
            p.z = float(cols[2][i])
            p.reflectivity = 0
            p.tag = 0                  # (0 & 0x30) == 0x00 通过 fork 过滤
            p.line = 0                 # < 4 通过 fork 过滤
            out.points.append(p)
        self.pub_lidar.publish(out)

    def on_imu(self, msg):
        if self.pub_imu.get_subscription_count() == 0:
            return
        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.angular_velocity = msg.angular_velocity
        out.linear_acceleration.x = msg.linear_acceleration.x / 10.0
        out.linear_acceleration.y = msg.linear_acceleration.y / 10.0
        out.linear_acceleration.z = msg.linear_acceleration.z / 10.0
        # 协方差照抄（fork 只用加速度值，不读协方差）
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self.pub_imu.publish(out)


def main():
    ap = argparse.ArgumentParser(description='仿真→FAST-LIO2 桥接')
    ap.add_argument('--lidar-in', default='/front_lidar')
    ap.add_argument('--lidar-out', default='/livox/lidar')
    ap.add_argument('--imu-in', default='/imu')
    ap.add_argument('--imu-out', default='/livox/imu')
    ap.add_argument('--acc-g', type=int, default=1, choices=[0, 1],
                    help='加速度 m/s²→g 转换（fork 内部 ×10 还原；默认 1）')
    ap.add_argument('--sweep-ms', type=float, default=0.0,
                    help='帧扫掠时长 ms（默认 0=整帧冻结快照；实测相邻帧残差为均匀平移型，'
                         '无剪切证据。若确认帧为真实旋转扫描再设 100）')
    args = ap.parse_args()
    rclpy.init()
    node = LivoxBridge(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
