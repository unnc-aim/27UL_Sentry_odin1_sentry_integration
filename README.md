# ODIN1 ROS 2 工作区

面向 Ubuntu 22.04 和 ROS 2 Humble，使用
[官方 odin_ros_driver](https://github.com/manifoldsdk/odin_ros_driver)
子模块。当前使用 v0.14.4，具体版本由父仓库的子模块引用记录。

## 目录

| 路径 | 用途 | Git 管理方式 |
| --- | --- | --- |
| `src/odin_ros_driver/` | 官方驱动与 SDK | 子模块 |
| `src/odin_sentry_bringup/` | 哨兵导航配置、数据适配与启动入口 | 跟踪 |
| `src/odin_sentry_bringup/test/` | 当前功能的自动检查，由 CMake 注册运行 | 跟踪 |
| `.vscode/*.json` | 使用工作区变量的编辑器配置 | 跟踪 |
| `colcon_defaults.yaml` | ROS 2 构建与编译命令导出设置 | 跟踪 |
| `build/`、`install/`、`log/` | 构建输出、诊断记录与临时测试脚本 | 忽略 |
| `.vscode/browse.vc*.db*` | 编辑器索引数据库及其锁文件、临时文件 | 忽略 |
| `maps/odin_<日期时间>/` | 本机导出的地图与续建数据 | 忽略 |
| `.claude/settings.local.json` | 本机助手设置 | 忽略 |

## 获取源码和构建

在工作区根目录执行：

```bash
git submodule update --init --recursive
```

以下命令在子 shell 中重新加载 ROS 2 环境，构建官方驱动。
哨兵导航适配包的完整构建步骤见下方接入说明。

```bash
(
  unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
  unset PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH
  export PATH=/usr/local/bin:/usr/bin:/bin
  source /opt/ros/humble/setup.bash
  MAKEFLAGS=-j2 colcon build --packages-select odin_ros_driver
)
```

`colcon_defaults.yaml` 启用符号链接安装，并生成
`build/odin_ros_driver/compile_commands.json`，供 C++ 编辑器读取真实编译参数。
已有构建目录切换开发环境时，可在 `colcon build` 后添加
`--cmake-clean-cache`，重新生成 CMake 配置。

`CMakeCache.txt`、编译命令和安装脚本中的绝对路径由构建工具生成，随本机目录变化重新构建即可。

## 运行与配置

构建完成后，在工作区根目录执行：

```bash
source install/setup.bash
ros2 launch odin_ros_driver odin1_ros2.launch.py
```

设备参数位于 `src/odin_ros_driver/config/control_command.yaml`。
驱动默认读取 `$HOME/.ros/odin_ros_driver/calib.yaml`；设置 `ODIN_CALIB_DIR`
可指定标定文件所在目录。源码目录下的 `config/calib.yaml` 是设备标定副本。

驱动子模块自行忽略 `config/calib.yaml`、`recorddata/`、`log/` 和 `map/`。
`lib/` 中的 SDK 静态库随官方子模块保留。设备固件和升级工具保存在工作区外。

## 哨兵导航接入

接入程序和实车检查步骤见 [odin_sentry_bringup](src/odin_sentry_bringup/README.md)。
`cloud_slam` 用于建图和定位；`cloud_raw` 经运动补偿与独立地形处理，供局部和全局实时障碍层使用。
该包提供 `driver`、`perception`、`localization`、`mapping`、`navigation` 五种运行模式。
导航侧需要构建 `sentry_ws` 中更新的 `loam_interface`、`sensor_scan_generation` 与 `pb2025_nav_bringup`；运行环境显式加载两个工作区。
水平安装位置已用实车转动数据复核，默认配置为 `config/mount.yaml`；高度和姿态继续检查。
