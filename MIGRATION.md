# 2026-10-27 工作空间迁移

三个包从现有工作目录复制到本仓库。原 `~/odin_ws` 源码、Git 历史、安装与日志完整保留；迁移前 `sentry_ws/src/odin_3d_test` 的副本和文档位于 `sentry_ws/log/odin_migration_20261027`。

旧 `sentry_ws/src/odin_3d_test` 增加 `COLCON_IGNORE`，其内容继续供原安装使用。统一构建选择本仓库内的同名包。新配置把旧 `log/behavior_speed_test/nav2_3p0_no_spin.yaml` 复制为 `odin_3d_test/config/nav2_competition.yaml`，参数数值保持原样。

地图复制至 `sentry_ws/maps/field_v2_20261024`，原地图目录保留。当前配套文件为 `native.bin`、`navigation_ground_continuous/navigation.yaml` 和 `waypoints.yaml`。地图配置中的图片路径保持相对形式。

驱动来源：本机 `odin_ws/src/odin_ros_driver`，上游 https://github.com/manifoldsdk/odin_ros_driver ，本车分支远端 https://github.com/SHENJLYL/odin1- 。驱动原许可证和来源文件随目录保留。

恢复原运行方式：使用保留的原工作空间和迁移前文档，按原顺序加载环境。原进程在本次复制过程中继续运行；新入口在手动重启后加载。

合并仓库沿用 `https://github.com/unnc-aim/27UL_Sentry_odin1_slam_test.git`，工作空间主仓库为 `https://github.com/unnc-aim/27UL_Sentry_ws.git`。迁移使用独立分支 `migration/sentry-workspace`。原 Odin 工作空间的主线历史保留在本仓库中。

`loam_interface` 的当前并行回调版本已经单独构建并更新到 `sentry_ws/install`，原二进制保存在迁移备份中。其源码继续位于原导航子仓库，后续常规构建会生成同样版本。
