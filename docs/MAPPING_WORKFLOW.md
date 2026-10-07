# 激光惯性建图

需要配置好 ROS 2 Jazzy、Gazebo、项目依赖并完成构建；本指南不安装依赖。
在已构建环境中，先加载对应的 workspace setup（本机使用 `source tools/env_host.sh`）。

```bash
ros2 launch legbot_bringup manual_mapping.launch.py \
  lidar_pitch:=0.7853981633974483 gui:=true rviz:=true
```

另一终端加载同一环境后运行：

```bash
ros2 run legbot_bringup go2_mapping_keyboard
```

按终端显示的键位启动、站稳及行走。默认低速，`f` 为 0.4 m/s 档，`h` 为
0.6 m/s 档；楼梯和转角应使用低速。结束先空格停步，再在另一终端保存：

```bash
ros2 service call /mapping/flush std_srvs/srv/Trigger '{}'
```

确认返回 `success=True` 后退出。flush 保存已采集点云，但不停止后续记录。
输出为 `lio_maps` 下独立的分块 PCD 和 `manifest.json`，不在 Git 中。
重力对齐是初始化时选择水平坐标系，不把原点移到地面，也不强制运动中机身水平。

建图中 RViz 显示限点数、降采样的累积预览，不是全部原始点云。建好后可查看：

```bash
ros2 launch legbot_bringup view_mapping.launch.py \
  map_directory:=/absolute/path/to/session voxel_size:=0.05 max_points:=1000000
```

`tools/prepare_mapping_pcd.py` 可合并分块并降采样；不会改动原始分块。
随后用 PCT Tomography 生成 Tomogram，通行性需要离线验证，不因机器人走过就必然连通。
