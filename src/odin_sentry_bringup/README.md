# Odin 哨兵导航接入

使用 Odin 的局部里程计接入哨兵导航：`cloud_slam` 用于建图和 AMCL，`cloud_raw` 用于实时障碍层。
官方 `odin_ros_driver` 保持独立子模块；本包保存接入配置、数据转换和启动入口。
驱动补充读取 ROS 2 的 `config_file` 参数，使独立配置实际生效；默认配置路径沿用官方值。
驱动的 `ptp_smooth_window` ROS 参数控制校时样本平均窗口，范围为 1–300；官方独立入口默认 300。
本包默认传入 `ptp_smooth_window:=1`，采用当前 PTP 校时值。设备校时约为 10 Hz，
原 300 样本窗口约覆盖 30 秒，实测会让 IMU、里程计和点云共同出现超过 0.13 秒的时间偏差。

## 双路点云

默认启动入口已将两个用途分开：

| 用途 | 输入 | 输出与使用方 |
| --- | --- | --- |
| 建图、AMCL | `/odin1/cloud_slam` + 标准里程计 | `/registered_scan`、原地形节点、`/obstacle_scan` |
| 近距离实时障碍 | `/odin1/cloud_raw` + 高频里程计 + `/odin1/wiwc` | `/odin_obstacles/registered_scan`、独立地形节点、`/odin_obstacles/terrain_map` |
| 局部和全局代价地图 | `/odin_obstacles/terrain_map` | 两张地图的 `intensity_voxel_layer` |

官方 [FAQ Q5.1、Q4.5–Q4.7](https://manifoldtechltd.github.io/wiki/odin_series/odin1/15.%20FAQ.html)
说明了近距离数据选择和 32 组顺序曝光。`odin_raw_obstacles` 按每点 `offset_time`，
用前后高频位姿的平移插值与四元数 SLERP 补偿运动。每组使用对应时刻最近一份设备 T_IL，
再采用标准里程计和 `/lidar_odometry` 的同刻配对结果，统一转换到 `odom`。

设备 T_IL 会在线更新。本机 10 秒静止记录中，平移变化范围最大约 `7.9 mm`，
相对首帧的旋转变化最大约 `0.019 rad`。适配器保留带时间的外参，按曝光时间选取，
同时发布动态 `odin_imu` 到 `odin_lidar` 的关系。`mount.yaml` 继续描述 IMU 与云台的安装关系。
输出点云和该路传感器里程计使用最后有效曝光组的时间，保持物理采样时间。

`config/raw_obstacles.yaml` 提供这一路的参数。默认范围为 `3 m`，置信度下界为 `35`，
车体球形裁剪半径为 `0.15 m`。原 SLAM 路的实测方形裁剪和近点密度检查独立保留；
raw 路在 IMU 周围 `1 m` 内，要求 `4 cm` 邻域至少有另外 `5` 个点支持，
在地形累积前移除孤立回波。该检查会影响回波稀疏的小物体，实物复测需包括细杆和低矮障碍。
实时障碍范围可在 `0.15–4 m` 内调整，配合现有局部地形覆盖范围。
SDK SPAD 过滤默认由本入口以 `spad_filter:=1` 启用，作用于四通道 raw DTOF。
官方独立驱动新增参数默认 `-1`，沿用 SDK 当前设置；`0` 关闭，`1` 开启。

raw 的反射强度与障碍高度含义不同。转换点云先补充 FLOAT32 `intensity=0`，
再交给命名空间 `/odin_obstacles` 内独立的 `terrain_analysis` 计算离地高度。
该地形实例的 `decayTime=0.2 s`、`voxelTimeUpdateThre=0.1 s`；原建图与定位地形参数保持原值。
两张代价地图的实时障碍范围均为上述 `3 m`，范围外由已有静态地图提供场地结构。

输出要求有效外参、完整曝光时段的位姿和新鲜的 SLAM 配对参考。
默认样本年龄上限 `0.3 s`、位姿插值间隔上限 `0.02 s`。
小幅 PTP 回调引起的乱序高频样本直接跳过；明显时间倒退、位姿或导航坐标关系突变时暂停输出。
标准和高频位姿的同刻差超过 `0.05 m` 或 `0.05 rad` 时等待恢复。
整帧缺少有效回波时跳过，保留缺测状态；有效回波全部位于范围外时可输出空的近场点云。
两张代价地图将 `expected_update_rate=0.5 s`，raw 地形持续缺测后报告数据过期。

所有非 `driver` 模式都启动双路处理。已有驱动单独运行时，可启动：

```bash
ros2 launch odin_sentry_bringup odin_sentry.launch.py mode:=perception start_driver:=False
```

RViz 检查实时障碍选 `/odin_obstacles/terrain_map`，颜色使用 `intensity`；
建图与定位扫描继续查看 `/obstacle_scan`。调参通过
`raw_obstacles_file:=/path/to/raw_obstacles.yaml` 加载完整副本。
实车复测包含静止地面、云台往返、低速平移和近距离实物；同时检查散点减少与真实障碍保留。

## 安装位置

`config/mount.yaml` 使用已提供的初始尺寸：测量点为雷达与相机向外观测的平面，
云台正对车头时，位置相对底盘中心地面为 `[0.130, 0.070, 0.430] m`。
当前机器人模型的 `gimbal_yaw` 原点离地 `0.103 m`，因此安装位姿为
`[0.130, 0.070, 0.327] m`、RPY 为零。

原始测量点保存在 `mount_xyz`，水平修正保存在 `mount_to_imu_xyz`，单位为米。
高度与安装姿态继续使用初始假设，`mount_to_imu_rpy` 供后续按弧度调整姿态。
传感器仅随云台 Yaw 转动，实车 `/joint_states` 提供该关节反馈。

`config/mount_yaw_fit.yaml` 保存 2026-10-19 双向转动试验得到的水平校正候选值。
两个转向分别拟合得到的有效水平位置为 `[0.1229, 0.0491] m` 和 `[0.1235, 0.0465] m`；
共同拟合并按毫米量级取值后，相对 `gimbal_yaw` 的有效位置为 `[0.1232, 0.0478, 0.3270] m`。
原测量值保留在 `mount_xyz`，`mount_to_imu_xyz` 保存 `[-0.0068, -0.0222, 0.0] m` 的修正。
这项修正同时包含安装测量误差和观测平面到 IMU 原点的偏移。
独立复测得到的水平位置为 `[0.1232, 0.0504] m`，与候选值的最大差异约为 `2.6 mm`。
默认 `mount.yaml` 已采用上述水平修正，`mount_yaw_fit.yaml` 保留首次拟合结果。

## 构建

在已加载 ROS 2 Humble 的终端，先构建 `sentry_ws` 中更新的启动包：

```bash
cd ~/sentry_ws
source /opt/ros/humble/setup.bash
MAKEFLAGS=-j2 colcon build --packages-select loam_interface sensor_scan_generation pb2025_nav_bringup --symlink-install
source install/setup.bash
cd ~/odin_ws
MAKEFLAGS=-j2 colcon build --packages-select odin_ros_driver odin_sentry_bringup --symlink-install
source install/local_setup.bash
```

运行时依次加载 Humble、`sentry_ws/install/setup.bash` 和 `odin_ws/install/local_setup.bash`。
独立构建官方驱动仍可采用工作区根目录 README 中的 Humble 环境构建方式。
实车数据处理与现有控制服务统一使用 `ROS_DOMAIN_ID=0`、`ROS_LOCALHOST_ONLY=0`。
跨域观察时，观察程序与对应域内节点使用相同的 `ROS_LOCALHOST_ONLY` 设置。

## 运行模式

每次选择一个模式，切换模式前结束上一条 launch。
`start_driver:=False` 可用于读取已经按本包配置启动的 Odin 驱动或回放数据。

实车云台外参检查先进入正常手动状态，确认 Yaw 电机反馈有效，再启动感知流程。
当前这台车在急停时会出现 `enabled=0`、`encoder=0`，机器人描述仍会据此生成关节角，
因此启动时的外参检查要结合电机状态。正常采样期间保持手动模式，异常时随时急停。

1. 单独查看原始传感器数据：

   ```bash
   ros2 launch odin_sentry_bringup odin_sentry.launch.py mode:=driver
   ```

2. 查看安装 TF、转换后点云、地形和扫描：

   ```bash
   ros2 launch odin_sentry_bringup odin_sentry.launch.py mode:=perception
   ```

   此模式启动传感器与数据处理。车辆控制服务沿用当前哨兵启动方式。
   `/joint_states`、机器人描述、Odin 时间同步共同决定转换结果。

3. 使用现有地图检查静态定位：

   ```bash
   ros2 launch odin_sentry_bringup odin_sentry.launch.py \
     mode:=localization \
     map:="$(ros2 pkg prefix --share pb2025_nav_bringup)/map/reality/field_20260925.yaml"
   ```

   此模式运行感知、map_server、AMCL 和原生全局初始化检查。
   结合现场位置检查地图与扫描是否吻合，再进入导航模式。

4. 使用当前场地地图定位与导航：

   ```bash
   ros2 launch odin_sentry_bringup odin_sentry.launch.py \
     mode:=navigation map:=/absolute/path/to/current_map.yaml
   ```

5. 使用转换后的扫描建图：

   ```bash
   ros2 launch odin_sentry_bringup odin_sentry.launch.py mode:=mapping
   ```

建图模式由 `slam_toolbox` 发布 `map` 与 `odom` 的关系，保存地图模式由 AMCL 发布。
切换模式前先结束已有的 Odin 启动，确认主机只运行一套该入口，再启动新模式。
Odin 配置将 `minimum_travel_distance` 和 `minimum_travel_heading` 设为 `0.0`，
保持 `minimum_time_interval=0.2 s`。底盘静止、云台转动时，新视角扫描也参与建图。
原参数要求底盘平移约 `0.1 m`；云台运动补偿后，固定底盘的位置保持不变，因此新扫描被跳过。
地图每 `0.5 s` 发布一次。完全静止时仍会按时间接收扫描，长时间停留会增加保存的扫描数量。
首次实车检查从 `driver`、`perception` 依次进行；固定车体检查云台转动时的里程计，
随后在受控低速条件下检查平移、方向、地形覆盖、定位和目标取消。
Odin 的实际视场、自身遮挡和扫描缺测区域需现场检查后再用于建图及自主行驶。

## 数据处理

原始输入为 `/odin1/odometry` 和 `/odin1/cloud_slam`。驱动使用里程计模式、主机对齐时间，
原生 TF 放在 `/odin1/tf` 与 `/odin1/tf_static`。

`odin_nav_adapter` 配对两路数据，保留 Odin 局部世界中的位姿与点云坐标，
并将点云转为 `x/y/z/intensity`，初始 intensity 为零。
Odin 启动入口同时加载 `config/self_filter.yaml`，在地形处理和扫描生成之前移除紧贴传感器的点。
`self_filter_radius=0.15` 表示移除距离 Odin 里程计 IMU 原点小于 `15 cm` 的点，使用三维球形范围。
适配器直接计算同一世界坐标下的点与 IMU 位置之差，过滤范围随传感器移动和云台转动。
保留下来的点继续使用原世界坐标；整帧只含近点时输出空点云和有效里程计。
`0.0` 关闭该过滤。半径内的真实外部物体也会被移除，调整时结合实物检查附近障碍物覆盖。
2026-10-19 的后续云台转动记录发现：原始点云在 IMU 左前下方有一簇随传感器运动的点，
代表帧距离约 `0.42–0.63 m`，超出上述球形范围。约 `70°` 的转动过程中，这簇点在 IMU
坐标中的中心仍约为 `[0.329, 0.241, -0.316] m`，在世界坐标中随云台移动。
因此增加 `self_filter_box_min=[0.23, 0.10, -0.36]`、
`self_filter_box_max=[0.45, 0.41, -0.23]`，专门裁剪这个实测区域。
这两个 XYZ 范围使用 IMU 坐标，前、左、上为正，单位为米；移除这两项参数可关闭方形区域过滤。
区域外的地面和障碍物保留。进入区域内的真实外部物体同样会被裁剪，现场检查继续确认该区域的物理遮挡。
平台离地 `43 cm`，用户说明 Odin 位于平台边缘后约 `3 cm`；平台到 IMU 的高度差仍需补充。
该近距离过滤直接使用传感器位姿，沿用现有安装外参。
通过 `self_filter_file:=/path/to/self_filter.yaml` 加载独立配置。
原始 `/odin1/cloud_slam` 保留设备输出，查看 `/registered_scan` 或 `/terrain_map_ext` 检查过滤结果。

近距离地面散点另用邻近点数量检查：在 IMU 周围 `1.0 m` 内，每个点的 `0.04 m`
邻域至少需要另外 `5` 个点支持，符合条件的点再进入地形处理。范围外的点保持原样。
参数为 `near_outlier_range`、`near_outlier_radius`、`near_outlier_min_neighbors`，
将 `near_outlier_range` 设为 `0.0` 可关闭该检查。实现使用已安装的 PCL KD-tree，保留点的坐标与顺序一致。
该检查会同时移除缺少邻近回波的细小物体，现场验证需包含实际需要避让的小障碍物。
输入新鲜度会在点云处理完成后再次检查，时间异常的处理沿用现有配置。

SLAM 路的点云订阅使用与官方驱动相同的 Reliable QoS，支持分片重传。
标准里程计订阅使用 SensorDataQoS，优先接收新样本，减少旧消息重传带来的等待。
raw 路的高频里程计使用 Reliable QoS，并保留两秒有序样本用于曝光时刻插值。
两路标准输出约为 10 Hz；配对器使用 10 ms 的消息间隔下界，
让时间同步校正带来的微小时间戳差异直接完成配对，减少等待下一帧的情况。
启动时车辆放置在水平地面，后续地形程序继续计算离地高度。

适配输出 `/odin_navigation/odometry`、`/odin_navigation/cloud` 交给现有 `loam_interface`。
该节点启用 `align_initial_pose`，在收到有效安装 TF 的首帧同时使用安装变换与传感器位姿，
计算局部世界到导航坐标的固定变换，再对位姿和点云统一转换。
输出为 `/lidar_odometry`、`/registered_scan`。
`sensor_scan_generation` 按点云时间查询动态云台 TF，继续输出车体里程计、`sensor_scan`
及 `odom` 到 `base_footprint` 的关系。`/odometry` 的 child 保持 `gimbal_yaw`。

时间未对齐时等待有效数据，单次过期消息和空点云直接跳过。
初始化后，持续输入超时、有效数据时间倒退、位置或姿态突变会暂停适配输出。
恢复时结束当前接入 launch，再整体启动传感器、转换和定位，重新建立导航原点。
启动参数 `max_stamp_age`、`max_future_stamp`、`max_pair_dt`、`input_timeout`、
`max_translation_step`、`max_rotation_step` 用于按实际频率和运动速度调整检查范围。

## 参数与 Sentry 侧改动

`params_file` 指定现有 Nav2 参数文件。默认读取 `pb2025_nav_bringup/config/reality/nav2_params.yaml`。
运行时生成临时副本，更新传感器输入和 frame，并应用明确的速度上限。
Odin 定位扫描的高度下界为 `0.2 m`，用于移除地形处理中变成正 intensity 的近地点。
同刻快照复算与原 181 条扫描一致，调整后保留 174 条，其中 63 条选择了更远的有效点。
AMCL 的 `max_beams` 设为 720，保留有限视场内的有效射线；双路接入后，代价地图使用独立的 raw 地形话题。
这些输入修正已单独检查，地图定位的现场一致性仍在验证。
默认线速度上限 `0.3 m/s`、角速度上限 `0.5 rad/s`；原参数文件内容保留。
此前 `nav2_3p0_no_spin.yaml` 的控制器上限为 `2.5 m/s`、平滑器上限为 `3.0 m/s`。
使用该文件时，初期仍应用上述调试上限；速度调整通过 `linear_speed_limit`、`angular_speed_limit` 指定。

Sentry 侧共更新 `pb2025_nav_bringup/launch/` 下四个文件：

- `rm_navigation_reality_launch.py`：外部里程计模式控制 Livox 启动；独立选择手柄节点。
- `bringup_launch.py`：将 `use_external_odometry` 传递给建图与定位入口。
- `localization_launch.py`：外部里程计模式由外部提供里程计与点云。
- `slam_launch.py`：外部里程计模式采用 slam_toolbox 的地图 TF。

新参数默认 `False`，原入口继续使用 Livox 与 Point-LIO。
本包传入 `use_external_odometry:=True`、`use_joy_teleop:=False`。
`loam_interface` 的头文件与源文件增加 `align_initial_pose`，默认 `False`，Odin 接入时启用。
`sensor_scan_generation` 的源文件将点云订阅设为 Reliable，匹配 `loam_interface` 的发布方式。
新增 `sync_min_interval` 参数默认 `0.0`，Odin 接入设为 `0.01 s`，及时配对时间戳接近的两路数据。
机器人模型、Hub、底盘、云台控制器和行为树的源码沿用现有实现。

## 验证

新增自动检查覆盖 32 组运动补偿、非零设备外参和世界对齐、置信度与范围裁剪、
位姿缺段和过期参考，以及导航对齐变化。
原生代价地图检查使用实际 `IntensityVoxelLayer`，验证障碍高度、地面点排除、数据过期与恢复。


```bash
source ~/sentry_ws/install/setup.bash
source ~/odin_ws/install/local_setup.bash
ctest --test-dir ~/odin_ws/build/odin_sentry_bringup --output-on-failure
```

检查程序在 ROS Domain 87 中发送合成数据，验证驱动配置参数、时间过滤、非零初始姿态、XYZI 字段、
云台转动时车体位置保持稳定、单次过期数据后的恢复，以及持续断流和里程计跳变后暂停输出。
另用相差 35 微秒和正负 700 微秒的消息，检查配对完成时间，覆盖实机观察到的时间同步校正情况。
ROS Domain 89 中另行检查感知模式和地图定位模式的启动。
近距离过滤检查使用非零世界原点和旋转云台，确认半径内点被移除、六个方向半径外 `1 cm` 的点保留，
并覆盖安装 TF 尚未到齐以及整帧近点的情况。
同一检查覆盖方形区域内部及其六个边界外 `1 cm` 的点，确认过滤随传感器运动、输出世界坐标保持一致。
近场散点检查另用非零世界原点和两个云台角度，确认孤立近点被移除、紧密点组和范围外的独立点保留；
也覆盖整帧孤立点以及下一帧恢复有效点组的情况。
ROS Domain 90 运行两个原生 `slam_toolbox`，保持底盘 TF 完全相同，仅将扫描视场转到另一侧。
原参数在新方向新增的占用格数为零；Odin 参数新增 `64` 格，验证静止底盘的新视角参与建图。
此次另采集 `30` 帧原始点云与里程计，记录在 `log/odin_platform_20261019/`。
该时段 IMU 原点 `15 cm` 内点数为零，平台回波是否对应用户观察到的障碍仍需在再次出现时核对。
后续 `800` 帧转动记录、坐标分析和图表保存在 `log/odin_cloud_issue_20261019/`。
检查时还发现旧感知入口与新建图入口同时存在；旧入口已结束，并保留单套建图。
此前地图及其扫描图保存于该目录的 `before_restart/`，用于回看。
加载方形区域过滤后，同刻实机快照的原始点数为 `35201`，过滤后为 `34718`，移除 `483` 个区域内点。
逐点比较，保留点的坐标差为零；车体水平距离 `0.75 m` 内达到障碍 intensity 的地形点，
前后快照分别为 `8` 和 `0`。结果写入 `filter_result.json`，动态现场显示继续由人工核对。

19:09 的完整手动记录约 `108 s`，保存在 `log/odin_motion_20261019_190957/`。
最大的一批新增障碍出现在有人经过的时段，彩色原始点云可看到对应人体回波。
剩余少量近点主要投影到地面方向，部分点缺少空间邻近支持，并在地形点中保留约 `0.2–0.48 s`。
该记录的主机时钟差变化小于 `1 ms`。分析与原始点云投影图在同一目录中，供后续复测对照。

2026-10-18 在本机完成构建，两项自动检查通过。
使用真实 Odin 数据，在 ROS Domain 88 中读取并转发 Domain 0 的关节反馈，完成 42 秒导航启动检查。
最后 10 秒内，适配点云为 `9.8 Hz`，`/odometry`、`/terrain_map` 和 `/obstacle_scan` 均为 `10.0 Hz`。
`map_server`、`amcl`、`controller_server`、`planner_server`、`bt_navigator`、`velocity_smoother` 均为 `active`。
本次发送目标数为零，观察到的 `/cmd_vel` 消息数为零；检查结束后已停止本次启动的节点。
结果保存在工作区 `log/odin_navigation_verified_20261018_122807/summary.json`。

2026-10-19 的人工云台往返记录覆盖约 90 秒。编码器与 Odin 的 Yaw 变化范围均约为 `132.44°`，
补偿后的车体航向变化范围为 `1.42°`。原外参下，车体位置的单轴变化范围最大为 `45.3 mm`。
正反两个转向给出的水平安装位置接近，据此生成 `mount_yaw_fit.yaml` 候选配置。
按底盘固定假设重算同一记录，水平误差 RMS 从 `16.3 mm` 降至 `5.6 mm`，独立实车复测继续确认该结果。
记录和曲线保存在 `log/odin_gimbal_manual_20261019_113756/`。

随后调整里程计与点云的订阅方式，适配器连续检查 180 秒，过期提示与暂停次数均为零。
`sensor_scan_generation` 同步调整后，完整感知流程再检查 60 秒，末段输出约为 `9.8–9.9 Hz`，
TF 查询失败与适配暂停次数均为零。车体里程计的消息年龄 95 分位为 `0.183 s`，
最长消息间隔约为 `0.414 s`。两项自动检查通过。
最新记录为 `log/odin_pipeline_verified_20261019_115239/summary.json`。

主机随后重启，12:09 的追加复测又出现一次 `0.703 s` 的过期输入及输出暂停。
该轮前后切换了遥控拨杆或急停，Yaw 原始读数在前后为零；完整车体 TF 未覆盖转动区间。
原始数据仍支持水平修正候选值，完整动态验证与持续输出验证继续进行。
此轮记录保存在 `log/odin_mount_validation_20261019_120917/`。

12:22 在正常手动模式下重新采样，录制话题缩减为位姿、TF、电机反馈和遥控状态。
110 秒内模式保持一致，电机反馈有效，云台与 Odin 的转角范围分别为 `75.88°`、`75.84°`。
车体 TF 覆盖完整记录；位置各轴变化范围为 `22.1 / 15.9 / 3.1 mm`，航向变化范围为 `1.12°`。
记录期间底盘平移和旋转输入均为零，适配暂停、过期提示和 TF 查询失败均为零。
同段独立数据的水平误差 RMS 从原参数的 `7.5 mm` 变为修正后的 `6.6 mm`。
水平安装位置的独立拟合支持当前修正值，默认配置已更新；底盘平移方向与实际定位继续检查。
结果保存在 `log/odin_light_validation_20261019_122232/motion_validation.json`。

12:33 的手动平移记录覆盖前进、后退及左右横移。沿云台方向的估计位移分别约为
前进 `1.02 m`、后退 `0.97 m`、左移累计 `0.71 m`、右移 `0.72 m`。
用户确认现场实际距离接近 `1 m` 和 `0.7 m`，方向与底盘指令一致。
末端估计位置距记录起点约 `4.9 cm`；该数值属于手动回位后的估计差值。
记录为 `log/odin_translation_20261019_123340/translation_analysis.json`。

随后使用 `field_20260925.yaml` 启动地图定位模式，map_server 与 AMCL 均进入 active。
首轮静态候选位置约为 `(-9.55, 4.80) m`，用户确认车辆实际位于中部大厅，
该轮定位结果与现场不符。14:00:43 适配器又因配对输入超时暂停输出。
记录、地图叠加图和现场确认保存在 `log/odin_localization_20261019_135838/`。

第二轮记录覆盖约 `145.3°` 的人工云台转动，底盘运动输入保持为零，电机反馈有效。
最终候选位置约为 `(-5.87, 4.10) m`，协方差虽已减小，扫描与地图仍存在明显偏差。
地图定位与实际位置的一致性继续检查，自主行驶留待定位确认后进行。
记录为 `log/odin_localization_timing_20261019_140431/`。

同轮输入时间记录发现，配对时间戳差达到约 `0.67 ms` 时，原 `1 ms` 间隔下界
仍会让 ApproximateTime 等待下一帧，额外等待最高约 `88 ms`。
使用相差 `0.7 ms` 的合成输入复现该问题后，将适配器与 sensor_scan_generation
的 Odin 间隔下界统一设为 `10 ms`；正负 `0.7 ms` 的配对检查与两项完整自动检查通过。
消息年龄上限继续为 `0.3 s`，持续输入超时继续为 `0.5 s`。

设备仍报告 RGB 参数写入超时；上述里程计与点云输出正常。
现有两个地形节点在 launch 退出时报告 `RCLError`，运行期间持续输出数据。
后续现场工作包括安装高度与姿态检查、地图与扫描对齐，以及遥控接管和低速路线测试。
当前结果覆盖软件连接、云台水平修正和手动平移方向，实车导航效果继续通过现场记录确认。

### 双路接入实测：2026-10-19 21:31

完整记录保存在 `log/odin_hybrid_20261019_210949/dual_motion/`，分析为同目录
`dual_validation.json` 与 `dual_motion_check.png`。记录约 90 秒，包含人工云台往返与底盘平移。
raw 转换点云为 `10.14 Hz`，独立 raw 地形为 `10.14 Hz`；地形消息年龄 95 分位为 `0.106 s`，
最大为 `0.215 s`，最长采样间隔为 `0.493 s`。两路适配均持续工作。
按原始 raw、逐组时间、设备在线外参和高频位姿独立复算 14 帧，输出点数全部一致，
逐点坐标差最大约 `0.45 mm`。这项结果验证转换实现与记录一致。
云台视角变化范围约 `92°`，底盘 TF 航向变化范围约 `2.33°`，平移两轴范围约 `0.44 / 0.41 m`。
`/odometry` 的 child 为 `gimbal_yaw`，底盘航向复算使用 `odom/base_footprint` 的 TF。

近点主要在镜头右前方约 `0.8 m` 和左前方约 `0.6 m`，用户确认对应位置有实物或人。
这两簇回波继续保留。raw 与 SLAM 的点密度和地形保留时间不同，单凭总点数评估效果需结合实物。
当前实测覆盖手动动作与数据处理；自动行驶的避让距离、停车和遥控接管由后续现场试验确认。

### 近距离孤立回波处理：2026-10-19 22:10

`log/odin_noise_20261019_220959/` 保存 RViz、原始数据和代价地图对照。
本轮车旁彩色区域来自两格实时障碍，静态地图对应位置为空闲。
129 帧 raw 中，这组假回波出现在镜头左前方约 25 cm、离地约 34–36 cm；
用户确认该位置只有下方地面。共 253 个候选点，每帧 1–2 个，水平偏角约 62°，
4 cm 邻域含自身共 1–2 点。SDK SPAD 过滤当时已启用。

raw 路此前将近点密度检查设为 `0.0`。现在默认设为 `1.0 m`，复用现有的
`near_outlier_radius=0.04 m`、`near_outlier_min_neighbors=5`。
这组记录中的 253 个候选点全部满足移除条件。点云先完成逐组运动补偿，再检查空间邻近支持。
运行中的节点在启动时读取这组参数，修改后通过重启 Odin 导航入口加载。

重启后再次记录约 20 秒：192 帧原始 raw 中仍有 382 个目标假点，
165 帧转换点云与 174 帧地形中的目标区域点数均为零。
两张代价地图最后 5 秒在 IMU 周围 1 m 内的障碍格数均为零；
记录初段曾出现其他近场障碍格，完整逐帧结果保存在 `after/all_frames.json`。
raw 地形频率约 `9.39 Hz`，消息年龄 95 分位约 `0.153 s`。
此前人工确认有实物的区域，30 帧抽样复算保留约 `98.13%` 的点。
运动补偿、孤立点过滤和启动配置两项自动检查通过，细杆与低矮障碍继续现场复测。
