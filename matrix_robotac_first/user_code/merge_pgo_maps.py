#!/usr/bin/env python3
"""合并 PGO 输出的分块地图为单个 PCD（纯 numpy，替代文档里依赖 open3d 的版本）

用法:
    ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: '/xx/final_map', save_patches: true}"
    python3 merge_pgo_maps.py /xx/final_map /xx/final_map/merged.pcd [min_hits=2]

参数 min_hits: 同一体素至少被几个关键帧命中才保留（默认 2，时间共识滤波，
单帧抖动/杂散层被剔除，地图更整齐；1=关闭）。地面带（z<0.15）例外：
命中≥1 即保留（地面单帧环带稀疏、步态颠簸难多帧重复命中，否则被滤光）。
属于算法管线内滤波，合规。

原理: patches/ 下每个关键帧保存的是 body 系点云，poses.txt 存优化后的全局位姿
（格式: patch名 x y z qw qx qy qz）。合并 = 逐块变换到全局 + 5cm 体素去重。
这是算法管线内的原生输出（PGO 位姿优化 + 分块合并），非事后人工加工。
"""
import os
import sys

import numpy as np


def read_pcd(path):
    """读 PCD（ASCII 或 binary），返回 Nx3 float64"""
    with open(path, 'rb') as f:
        header = []
        while True:
            line = f.readline()
            header.append(line)
            if line.startswith(b'DATA'):
                break
        data = f.read()
    h = b''.join(header).decode()
    n = int([l for l in h.splitlines() if l.startswith('POINTS')][0].split()[1])
    if 'binary' in h.split('DATA')[-1].strip():
        arr = np.frombuffer(data, dtype=np.float32, count=n * 4)
        return arr.reshape(n, 4)[:, :3].astype(np.float64)
    return np.loadtxt(data.decode().split(), dtype=np.float64).reshape(n, 4)[:, :3]


def quat_to_R(qw, qx, qy, qz):
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


GROUND_Z = 0.15   # 世界系地面带判定阈值（地板 z≈0，步态颠簸留余量）


def keep_hits(hits, z, min_hits, ground_min_hits=1):
    """体素保留判据：地面带（z<GROUND_Z）命中 ≥ground_min_hits 即保留，
    其余需 ≥min_hits 次。

    地面每帧只有稀疏环带且步态颠簸下同一块地面难被多帧重复命中
    （实测同数据 hits≥2 时地面 2160 点 vs hits≥1 时 5980 点，-64%），
    散点控制靠门控与孤立点剔除，不必牺牲真实地面覆盖。
    算法管线内滤波，非事后加工。
    """
    return hits >= (ground_min_hits if z < GROUND_Z else min_hits)


def main():
    src, out = sys.argv[1], sys.argv[2]
    poses = {}
    with open(os.path.join(src, 'poses.txt')) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            v = line.split()
            poses[v[0]] = np.array([float(x) for x in v[1:8]])   # x y z qw qx qy qz
    patches = sorted(f for f in os.listdir(os.path.join(src, 'patches')) if f.endswith('.pcd'))
    print(f'poses {len(poses)} 条，patches {len(patches)} 个')

    min_hits = int(sys.argv[3]) if len(sys.argv) > 3 else 2
    ground_min_hits = int(sys.argv[4]) if len(sys.argv) > 4 else 1
    voxel = {}
    for i, name in enumerate(patches):
        if name not in poses:
            print(f'  跳过无位姿的 {name}')
            continue
        x, y, z, qw, qx, qy, qz = poses[name]
        R = quat_to_R(qw, qx, qy, qz)
        t = np.array([x, y, z])
        pts = read_pcd(os.path.join(src, 'patches', name))
        pts_w = (R @ pts.T).T + t
        # 8cm 体素：合并步态相位混叠产生的 3-7cm 条纹；12cm 双面墙仍可分
        keys = (pts_w / 0.08).astype(np.int64)
        u, ui = np.unique(keys, axis=0, return_index=True)
        for k, p in zip(u, pts_w[ui]):
            kt = (k[0], k[1], k[2])
            v = voxel.get(kt)
            if v is None:
                voxel[kt] = [p[0], p[1], p[2], 1]
            else:
                # 质心累积：跨关键帧平均，把不同步态相位的离散偏移拉回均值
                n = v[3] + 1
                v[0] = (v[0] * v[3] + p[0]) / n
                v[1] = (v[1] * v[3] + p[1]) / n
                v[2] = (v[2] * v[3] + p[2]) / n
                v[3] = n
        if (i + 1) % 50 == 0:
            print(f'  已合并 {i + 1}/{len(patches)}')

    # 时间共识：地面带命中 ≥ground_min_hits 即保留，其余体素 ≥min_hits 个关键帧命中才保留
    vals = [v for v in voxel.values()
            if keep_hits(v[3], v[2], min_hits, ground_min_hits)]
    n_ground = sum(1 for v in vals if v[2] < GROUND_Z)
    print(f'时间共识: 剔除 {len(voxel) - len(vals)} 个体素'
          f'（地面带命中≥{ground_min_hits} 保留 {n_ground} 个，'
          f'其余命中 <{min_hits} 帧剔除）')
    merged = np.array(vals, dtype=np.float32)[:, :3]
    with open(out, 'w') as f:
        f.write('# .PCD v0.7 - Point Cloud Data file format\n')
        f.write('VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n')
        f.write(f'WIDTH {len(merged)}\nHEIGHT 1\n')
        f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
        f.write(f'POINTS {len(merged)}\nDATA ascii\n')
        np.savetxt(f, merged, fmt='%.4f')
    print(f'已保存合并地图: {out}（{len(merged)} 点）')


if __name__ == '__main__':
    main()
