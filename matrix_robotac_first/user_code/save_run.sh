#!/usr/bin/env bash
# 一键保存本次建图结果：PGO 分块地图 + 优化轨迹 + 合并 PCD + xyz（CloudCompare 用）
# 每次调用生成独立时间戳目录，互不覆盖。
# 用法（绕场建图完成后执行）:
#   bash ~/matrix_robotac_workspace/matrix_robotac_first/user_code/save_run.sh
set -e

TS=$(date +%Y%m%d_%H%M%S)
DIR="$HOME/robotac_maps/run_$TS"
mkdir -p "$DIR"

source /opt/ros/humble/setup.bash
source "$HOME/fastlio2_ws/install/setup.bash"

echo "==> 保存 PGO 地图到 $DIR"
ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '$DIR', save_patches: true}" | tail -2

echo "==> 合并分块地图"
python3 "$HOME/matrix_robotac_workspace/matrix_robotac_first/user_code/merge_pgo_maps.py" \
  "$DIR" "$DIR/merged.pcd"

echo "==> 转 xyz（CloudCompare 用）"
tail -n +12 "$DIR/merged.pcd" > "$DIR/merged.xyz"

echo "✅ 已保存到 $DIR"
ls -la "$DIR"
echo "CloudCompare 查看: CloudCompare $DIR/merged.xyz"
