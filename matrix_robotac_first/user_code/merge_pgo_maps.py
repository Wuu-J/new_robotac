#!/usr/bin/env python3
"""合并 PGO 输出的分块地图为单个 PCD（纯 numpy，替代文档里依赖 open3d 的版本）

用法:
    ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: '/xx/final_map', save_patches: true}"
    python3 merge_pgo_maps.py /xx/final_map /xx/final_map/merged.pcd

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
        keys = (pts_w / 0.05).astype(np.int64)
        for k, p in zip(keys, pts_w):
            voxel.setdefault((k[0], k[1], k[2]), p)
        if (i + 1) % 50 == 0:
            print(f'  已合并 {i + 1}/{len(patches)}')

    merged = np.array(list(voxel.values()), dtype=np.float32)
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
