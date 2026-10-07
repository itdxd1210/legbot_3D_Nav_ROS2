# Runtime data

This directory contains runtime output and is not source code. PCT tomograms,
maps and experiment output are generated here. The SCAN keypoint recorder
creates `routes/scan_keypoints.yaml` here when the first route is saved.

导航 JSON 的 `start_mode` 支持 `current` 和 `fixed`（未填写时为 `fixed`）。
`current` 需要 `initial_localization:=true`，等待初始重定位通过审核后，将当前
`base` 位置按 `map_from_pct` 三维转换为 PCT 起点，忽略 JSON 的 `start`；不使用
出生点或 Gazebo 真值代替定位。`fixed` 使用 `start` 的 PCT 地图坐标，例如建筑入口。
两种模式均使用 `goal`，不会修改地面上方 0.50 m 的路径高度约定。
改 JSON 后重启原来的 launch 即生效，不需要编译；`spawn` 仍是 Gazebo 世界坐标。
当前位置无有效地图或 PCT 搜索失败时保持不导航，不回退固定起点。
