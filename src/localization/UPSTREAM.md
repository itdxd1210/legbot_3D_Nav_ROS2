# 定位源码来源

这两个目录是主仓库管理的普通源码，不再需要 `git submodule update`。
保留各自原有的 `LICENSE`、README 和测试。原始版本如下：

| 目录 | 上游 | 基准提交 |
| --- | --- | --- |
| `lidar_localization_ros2` | https://github.com/rsasaki0109/lidar_localization_ros2 | `537ac5b02155d78e2ce519b8241fe39c6574a936` |
| `ndt_omp_ros2` | https://github.com/rsasaki0109/ndt_omp_ros2，`humble` 分支 | `8b77fa5a6cdcad45bf35918361c892b6d94a287e` |

两者均保留 BSD 许可证，以各目录内的 LICENSE 为准。
`ndt_omp_ros2` 当前没有本地算法源码修改；新增 `.gitattributes` 让其小型上游样例
保持普通 Git 文件，不继承主项目的个人地图 LFS 规则。`lidar_localization_ros2` 包含本项目
已经使用的里程计预测基准、候选连续审核、恢复审核、完整旋转保护及相关测试；
方向约束仍为 shadow 观察，不宣称已参与导航修正。

初始搜索、定位结果准入、导航坐标桥接在 `legbot_bringup` 中实现。
FAST-LIO 始终继续运行；通过审核的地图匹配更新坐标变换，而不是关闭 FAST-LIO。
转换 Git 管理方式本身不改变这些运行行为。原子仓库的 Git 元数据已在本地保留，
不作为运行依赖，也不上传到主仓库。
