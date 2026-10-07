# 初始重定位与地图修正

需要已有、自洽的地图数据包；Git 只上传源码，不上传个人 PCD、Tomogram 和实验日志。
导航 JSON 及同目录的 `alignment.json`、`map_odom.pcd`、初始搜索资产必须来自同一地图。
`tools/prepare_initial_localization.py --help` 给出搜索资产的生成参数。

导航 JSON 中：`spawn` 是仿真的出生位置；`goal` 是 PCT 地图坐标。
`start_mode=current` 等待重定位通过审核，以当前 base 位置转换后的坐标作为 PCT 起点，
忽略固定 `start`；`start_mode=fixed` 使用指定的 PCT 起点，例如建筑入口。
改 JSON 后重新启动生效，不需要重新编译。无有效起点或规划失败不会偷偷回退固定起点。

```bash
ros2 launch legbot_bringup pct_cross_floor_demo.launch.py \
  map_config:=/absolute/path/to/navigation.json \
  navigation_source:=fastlio initial_localization:=true \
  align_with_ground_truth:=false localization_correction:=true \
  localization_candidate_review:=true localization_scan_max_range:=20.0 \
  localization_diagnostics:=true diagnose_fastlio:=true \
  lidar_pitch:=0.7853981633974483 gui:=true rviz:=true
```

此模式下初始搜索不使用 Gazebo 真值；`diagnose_fastlio` 打开真值旁路误差监测。
原始 FAST-LIO 输出保持不变，地图匹配使用里程计预测；只有通过审核的匹配更新
`/navigation/odometry_base` 及配套点云所在坐标关系，SCAN 使用这组一致的输出。
拒绝的匹配保留此前变换，继续使用里程计增量预测，不代表漂移会自动消失。

方向约束目前仍是观察模式。重复楼层、覆盖不足及长距离漂移仍需实测评估，
不能承诺任意位置均可重定位。完整运行诊断、终点接近和站姿过渡另行整理，
本批提交不包含那些近期改动。
