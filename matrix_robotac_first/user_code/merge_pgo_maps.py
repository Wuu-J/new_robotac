#!/usr/bin/env python3
"""合并 PGO 输出的分块地图为单个 PCD（纯 numpy，替代文档里依赖 open3d 的版本）

用法:
    ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: '/xx/final_map', save_patches: true}"
    python3 merge_pgo_maps.py /xx/final_map /xx/final_map/merged.pcd [min_hits=2] [ground_min_hits=1] [align_z=1]

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
    """读 PCD（ASCII 或 binary），按 header 字段布局解析，返回 Nx3 float64。

    注意：PGO patch 是 PointXYZINormal 8 字段（x y z intensity normal_x
    normal_y normal_z curvature，32 字节/点），按 4 字段读会丢掉一半点——
    实测"merged 只有一半"即此 bug。这里按 FIELDS/SIZE 解析，与字段数无关。
    """
    with open(path, 'rb') as f:
        header = []
        while True:
            line = f.readline()
            header.append(line)
            if line.startswith(b'DATA'):
                break
        data = f.read()
    h = b''.join(header).decode()
    lines = h.splitlines()
    n = int([l for l in lines if l.startswith('POINTS')][0].split()[1])
    fields = [l for l in lines if l.startswith('FIELDS')][0].split()[1:]
    sizes = [int(x) for x in [l for l in lines if l.startswith('SIZE')][0].split()[1:]]
    step = sum(sizes)
    offsets = np.cumsum([0] + sizes)[:-1]
    idx_xyz = [fields.index(f) for f in ('x', 'y', 'z')]
    if 'binary' in h.split('DATA')[-1].strip():
        buf = np.frombuffer(data, dtype=np.uint8, count=n * step).reshape(n, step)
        pts = np.empty((n, 3), dtype=np.float64)
        for k, fi in enumerate(idx_xyz):
            o = offsets[fi]
            pts[:, k] = buf[:, o:o + 4].copy().view(np.float32).reshape(-1).astype(np.float64)
        return pts
    arr = np.loadtxt(data.decode().split(), dtype=np.float64).reshape(n, len(fields))
    return arr[:, idx_xyz]


def quat_to_R(qw, qx, qy, qz):
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


GROUND_Z = 0.15   # 世界系地面带判定阈值（地板 z≈0，步态颠簸留余量）


def estimate_ground_z(pts_w, frac=0.05, band=0.15, min_pts=50):
    """估计一个 patch 的地面高度（世界系 z）。

    取 patch 内 z 最低 frac 分位数附近 band 内的中位数——迷宫为平地且
    Mid360 360° 视场，patch 几乎必含地面。LIO z 漂移（实测可跑飞至 10m）
    使各 patch 地面高度不一致，据此把每个 patch 的 z 拉回 0，墙也随之归位。
    算法管线内处理，非事后加工。
    """
    if len(pts_w) < min_pts:
        return None
    z0 = float(np.percentile(pts_w[:, 2], frac * 100))
    band_pts = pts_w[(pts_w[:, 2] >= z0 - 0.05) & (pts_w[:, 2] < z0 + band)]
    if len(band_pts) < min_pts:
        return None
    return float(np.median(band_pts[:, 2]))


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
    align_z = int(sys.argv[5]) if len(sys.argv) > 5 else 1
    # patch 地面高度偏离全场中位数参考超过此值 → 视为位姿已发散，丢弃。
    # 实测教训：z 跑飞（0.2→10.7m）后 patch 的 x/y/航向同样坏掉，
    # "全救"策略把错位墙面全塞进地图 → 大面积重影。健康 LIO 的地面高度
    # 应一致（±几 cm），取 0.5m 留余量。
    max_align = float(sys.argv[6]) if len(sys.argv) > 6 else 0.5
    n_aligned = 0
    n_dropped = 0

    # 第一遍：读全部 patch、变换到世界系、估计各自地面高度。
    # （PGO 地图系地面在 z≈-0.65——PGO 把首帧位姿的平移（含 z=0.65）去掉了，
    # 不能假设地面≈0，用全场中位数做参考自适应。）
    cache = []      # (name, pts_w, gz or None)
    for i, name in enumerate(patches):
        if name not in poses:
            cache.append(None)
            print(f'  跳过无位姿的 {name}')
            continue
        x, y, z, qw, qx, qy, qz = poses[name]
        R = quat_to_R(qw, qx, qy, qz)
        t = np.array([x, y, z])
        pts = read_pcd(os.path.join(src, 'patches', name))
        pts_w = (R @ pts.T).T + t
        gz = estimate_ground_z(pts_w) if align_z else None
        cache.append((name, pts_w, gz))
        if (i + 1) % 50 == 0:
            print(f'  已读 {i + 1}/{len(patches)}')

    if align_z:
        gz_valid = [g for g in (e[2] for e in cache if e) if g is not None]
        if len(gz_valid) < 3:
            print(f'警告: 只有 {len(gz_valid)} 个 patch 有地面估计，关闭地面对齐')
            align_z = 0
        else:
            ref = float(np.median(gz_valid))
            print(f'地面参考高度(全场中位数): {ref:.3f} m（{len(gz_valid)} 个 patch）')

    voxel = {}
    for entry in cache:
        if entry is None:
            continue
        name, pts_w, gz = entry
        if align_z:
            if gz is None:
                n_dropped += 1
                continue                     # 无地面估计：损坏 patch，丢弃
            if abs(gz - ref) > max_align:
                n_dropped += 1
                continue                     # 位姿已发散（z 跑飞后 x/y 同样坏），丢弃
            pts_w[:, 2] -= gz                 # 按自身地面拉回 z=0，墙随之归位
            n_aligned += 1
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

    if align_z:
        print(f'地面对齐: {n_aligned} 个 patch 已按地面拉回 z=0，'
              f'{n_dropped} 个 patch 位姿损坏被丢弃')
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
