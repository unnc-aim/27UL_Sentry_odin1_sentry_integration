# Odin navigation

本仓库统一管理三个 ROS 2 包，保留原包名与节点接口：

- `odin_ros_driver`：Odin SDK 驱动、图像、点云、里程计和原生地图服务。
- `odin_sentry_bringup`：安装关系、点云过滤、raw 去畸变、里程计与 TF 适配。
- `odin_3d_test`：原生建图、全局定位、导航地形和 Nav2 接入。

运行操作见 [USAGE](odin_3d_test/USAGE.md)。比赛树继续由 `sentry_ws/src/behavior` 提供。配置、话题和算法参数沿用迁移前版本。

正式地图与航点位于 `sentry_ws/maps/field_v2_20261024`。源码历史与迁移备份见 [MIGRATION](MIGRATION.md)。

```bash
source /opt/ros/humble/setup.bash
source "$HOME/sentry_ws/install/setup.bash"
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 OPENBLAS_NUM_THREADS=1
ODIN_MAP_DIR="$HOME/sentry_ws/maps/field_v2_20261024"
env -u PYTHONNOUSERSITE ros2 launch odin_3d_test odin_3d.launch.py \
  mode:=navigation native_map:="$ODIN_MAP_DIR/native.bin" \
  map:="$ODIN_MAP_DIR/navigation_ground_continuous/navigation.yaml"
```
