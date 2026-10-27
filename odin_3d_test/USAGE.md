# 独立包运行说明

本包新增代码、配置和测试均位于 `src/odin_3d_test`。现有驱动、控制器、比赛行为树及其他子仓库继续作为依赖使用。
实现使用已安装的 PCL/Eigen、NumPy/SciPy；开源组件说明见 THIRD_PARTY.md。

RViz 彩色点云预览上限为 10 Hz，实际更新取决于上游有效点云频率。

## 当前功能

- 三种启动模式：`mapping`、`localization`、`navigation`；均开启 RGB 和设备彩色 SLAM 点云，`rgb:=false` 可关闭独立图像输出。
- 原生 map/odom 变换合成，三次新时间戳确认，过期数据检查及位姿突变停车。
- 复用 raw 去畸变和点云过滤。平地模式由现有 PCL 地板与净空判断统一提供实时障碍；terrain_analysis 输出保留用于对照。坡面、0.50 m 车高净空、未知地面和地面连通性沿用本包几何计算。
- 局部和全局代价地图接入新地形插件。每帧地形结果替换上一帧实时障碍；当前帧未观察到地面的区域沿用静态导航图。在实时观测盲区内，静态地图中的未知区域和障碍继续保留。
- 导航指令经原 fake_vel_transform 后，再检查定位、raw、地形和车体 TF，最后交给原 Hub。空闲时以 20 Hz 向转换节点发送零平移输入，保留遥控小陀螺。定位或感知异常后平移归零；定位突变或输入持续中断后请求取消已有目标；小陀螺继续使用 0.2 秒内的有效遥控旋转指令。行为树发布旋转指令时优先使用其新鲜值，包括显式零值。遥控关闭旋转、急停或失联时停转。

车高 0.50 m、离地间隙 0.05 m、坡度上限 30°写在 `config/implementation_profile.yaml`。
台阶上限临时取 0.03 m，属于保守的软件配置，后续根据实车结果调整。它与离地间隙分别保存。

设备坐标跳变等定位异常进入 `FAULT_RESTART_REQUIRED`。导航模式已就绪后遇到输入过期，进入 `WAITING_FRESH_INPUTS` 并立即保持零平移；重新收到三组连续、新鲜的里程计和 raw 点云，且地形与 TF 均有效时恢复。输入连续失效超过 2 秒时，取消当前目标并继续保持零平移。三组有效数据恢复后允许重新下发目标。定位原点变化或非法速度输入进入要求重启的状态。中间恢复有效的数据会重新开始计算连续失效时间，多个短暂中断分别处理。当前视野全部为未知地面时，地形图仍输出未知格子；只要处理结果及时间有效，便交由代价地图结合已扫描地面决定通行。小陀螺仍按有效遥控和行为树旋转指令运行。设备坐标突变继续立即要求重启。只读定位模式中，地形等待显示为 `LOCALIZED_WAITING_PERCEPTION`，有效的原生地图坐标继续更新。
彩色点云、raw 避障和车体里程计共用 `loam_interface` 的世界坐标变换。`odin_nav_adapter` 使用同一测量时间的原始位姿与转换后位姿生成 `odom / odin_local` 世界坐标关系，避免启动时云台角度不同造成两路偏转。

设备原生 TF 当前要求连续新消息，允许间隔 2 s；若实际固件仅输出一次成功结果，需要按该固件正式状态接口调整。首轮实车确认此项。

## 构建与环境

在新终端执行：

```bash
source /opt/ros/humble/setup.bash
cd "$HOME/sentry_ws"
source install/local_setup.bash
colcon build --packages-select odin_ros_driver odin_sentry_bringup odin_3d_test \
  --symlink-install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 OPENBLAS_NUM_THREADS=1
```

运行终端加载 `/opt/ros/humble/setup.bash` 和 `~/sentry_ws/install/setup.bash`。本包 Python 运行节点使用系统 NumPy/SciPy；启动命令沿用 `env -u PYTHONNOUSERSITE`，机器人描述使用本机已有的 xmacro。

原工作空间与迁移前的安装保留，详情见仓库根目录 `MIGRATION.md`。同一设备使用一套驱动。切换导航前结束上一套节点，保留底层控制服务。

## 1. 一轮建图，同时生成定位图与导航图

手动扫描场地一轮，保存设备原生 BIN，再从同一份 BIN 导出几何点云和 5 cm 导航图。本流程合并了原来的原生建图与导航图制作，第二轮 raw 录制作为第 4 节的补充选项。

**本轮统一保存到 `~/sentry_ws/maps/field_v2_20261024/`。** 每次重新建图选择新目录，并在两个终端使用同一个目录名。已有 `field_v1` 文件保留。

**终端 A：启动建图。** 先在旧导航终端按 Ctrl+C，等其结束。保留底层控制服务，车辆停在平整地面，随后执行：

```bash
source /opt/ros/humble/setup.bash
source "$HOME/sentry_ws/install/setup.bash"
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 OPENBLAS_NUM_THREADS=1
ODIN_MAP_DIR="$HOME/sentry_ws/maps/field_v2_20261024"

env -u PYTHONNOUSERSITE ros2 launch odin_3d_test odin_3d.launch.py \
  mode:=mapping output:="$ODIN_MAP_DIR/native.bin"
```

等待点云和车体坐标稳定，先观察起点附近地面，再手动覆盖场地、转动云台补充墙面与洞口。重点覆盖拱门两侧立柱、拱下地面以及通道前后；结束前回到起点附近观察已扫区域，让设备有机会完成回环。采集期间尽量保持场地布置固定。

RViz 固定坐标为 `odin_local`。`Native Color Preview` 实时显示约 10 Hz，`Accumulated Color Map` 每秒更新累计图。导航用的黑白图在保存后离线生成。

**终端 B：扫描完成、车辆停稳后，整段执行保存与导出。** 终端 A 保持运行：

```bash
source /opt/ros/humble/setup.bash
source "$HOME/sentry_ws/install/setup.bash"
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
export PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1
ODIN_MAP_DIR="$HOME/sentry_ws/maps/field_v2_20261024"

ros2 run odin_3d_test save_native_map.py \
  --output "$ODIN_MAP_DIR/native.bin" &&
ros2 run odin_3d_test map_to_ply_amd64 \
  "$ODIN_MAP_DIR/native.bin" "$ODIN_MAP_DIR/native.ply" &&
ros2 run odin_3d_test export_surface \
  "$ODIN_MAP_DIR/native.ply" "$ODIN_MAP_DIR/navigation_ground_continuous" \
  0 0 -0.43 0.05 8 flat 1 0.20
```

保存工具从 `/rosout` 或本机驱动日志确认本次文件的 `Map transfer completed` 事件，随后执行两个离线导出工具；前一项成功后才执行后一项。本机日志读取从本次保存请求开始，匹配本次输出路径。看到 `Saved geometry map and original input` 后，本轮文件生成完成，再结束终端 A。

最后一行参数依次是地面参考点 `(0, 0, -0.43)`、分辨率 `0.05 m`、每格障碍支持点数 `8`、平地模式 `flat`、每格地面支持点数 `1`、平地小空洞最大宽高 `0.20 m`。本车当前安装高度取 `0.43 m`，新建原生地图原点对应启动时设备里程计原点，因此使用起点下方的近似地面点。该高度用于选择地板平面附近的点，实际地板由 PCL 拟合；它属于安装估计值，随安装高度或起始地面变化一同调整。旧地图里的 `(3.325, 0.171, -0.504)` 属于旧地图坐标，本轮使用上述新起点参考。

生成文件：

- `native.bin`：设备原生三维地图，用于重定位。
- `native.ply`：从该 BIN 提取的 XYZ 几何点云，供导航图制作。本机转换工具本轮输出 XYZ；实时彩色预览继续使用设备 RGB 点云。
- `navigation_ground_continuous/navigation.yaml`、`navigation.pgm`：5 cm 导航栅格。
- `navigation_ground_continuous/terrain.csv`、`source_map.ply`：地面分类结果及导图输入副本。

BIN 内的点来自设备建图；主机 raw 的逐帧近点过滤继续用于实时避障。BIN 导图应用地板、车高净空和障碍支持点分类，仍需在新图上核对墙体、洞口及残留杂点。新地图与重新采集的航点配套使用。

## 2. 启动导航（包含原生全局定位）

建图文件保存完成、mapping 退出后，直接启动 navigation。此入口同时加载原生 BIN、执行原生全局重定位、运行感知与坐标转换，并启动 Nav2。正常使用由这一条启动命令完成定位和导航。

```bash
source /opt/ros/humble/setup.bash
source "$HOME/sentry_ws/install/setup.bash"
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 OPENBLAS_NUM_THREADS=1
ODIN_MAP_DIR="$HOME/sentry_ws/maps/field_v2_20261024"

env -u PYTHONNOUSERSITE ros2 launch odin_3d_test odin_3d.launch.py \
  mode:=navigation \
  native_map:="$ODIN_MAP_DIR/native.bin" \
  map:="$ODIN_MAP_DIR/navigation_ground_continuous/navigation.yaml"
```

2026-10-27 更新：此入口默认启用独立里程计发布，设备位姿及时送往车体 TF 和 raw 去畸变参考；SLAM 点云配对、过滤在单独回调中处理，待处理数据保留最新一组。测量时间、原过滤参数和比赛速度保持原值，过期感知仍发零平移。重启本节命令后加载新版本。

启动时保持车辆静止，等待原生定位建立。状态可查看 `/odin_3d/state` 和 `/odin_3d/ready`。位置、朝向与现场吻合，且状态为 `READY` 后，通过 RViz 或比赛行为树下发目标。本轮新地图的巡航点按第 3 节重新选择。

本轮导航图直接从新 BIN 点云按 0.05 m 分类，与 sentry_ws 原导航分辨率一致。地图原点随新建图重新建立，巡航点在新图上重新选取。

实时 Ground Support 使用 5 cm，18×18 m 窗口包含 360×360 格，更新设置为 10 Hz。raw 保留 10 m 内回波，密度过滤仍覆盖近 4 m；局部地形使用车旁窗口处理。局部、全局代价地图使用 5 cm；全局地图的实际分辨率由加载的地图 YAML 决定。当前局部膨胀半径为 0.25 m，全局为 0.30 m。全局规划单独设置 `footprint_padding: 0.10`，给局部网格投影及实际跟随误差留出空间。两张地图的 `robot_radius` 仍为 0.20 m，局部车体轮廓及速度参数继续沿用。

当前采用按保存地图绕开历史障碍的方式。黑色历史障碍继续参与规划，灰色未知区域也保持未知。两张代价地图都启用 `track_unknown_space` 和 `inflate_around_unknown`；ThetaStar 的 `allow_unknown` 设为 `false`，路径选择已扫描的通路。这样规划端与跟随端使用一致的已知空间要求，并给未知边缘留出车体空间。地面可见本身只说明有地面，45 cm 低拱等上方结构仍需保留其高度信息。

当前 3D 导航的进展检查使用 3 秒等待、10 cm 位移要求；单点和多点导航的 FollowPath 恢复重试为 2 次。正常重规划频率保持 3 Hz，控制器错误容忍时间保持 0.3 秒，外层恢复重试仍为 10 次、Wait 为 1 秒。参数通过本包 launch 写入运行配置及 XML 副本，原比赛配置文件和行为树继续作为输入使用。

平地模式的新增障碍由 SurfaceLayer 更新到局部、全局代价地图，两张地图共用同一套地板与车高净空判断。原 Nav2 行为树每秒 3 次重新计算路径；存在可行绕路时更新路径，整个通路被占据时停止，原地等待有效观测并重新规划。实时障碍移开后，新的有效观测更新代价地图。静态地图上的历史障碍继续按历史障碍绕行，保存地图的内容由单独的地图维护操作更新。

RViz 显示蓝色 `Global Path`（`/plan`）、绿色 `Local Path`（`/local_plan`），以及两张彩色代价地图。路径随成功的规划结果更新；导航失败时结合 `Navigation 2` 状态与代价地图判断原因。当前路径显示与 3D 点云共用 `map` 固定坐标。

RViz 的 `Nav2 Goal` 工具与 `Navigation 2` 面板配套使用，面板负责发送导航目标并显示执行状态。当前窗口缺少面板时，选择 `Panels → Add New Panel → nav2_rviz_plugins → Navigation 2`。新启动的窗口会自动加载该面板。

默认比赛速度：启动参数 `linear_speed_limit:=4.0 angular_speed_limit:=3.0`，控制器和平滑器平移上限为 4.0 m/s，平移加速度和减速度均为 2.5 m/s²，弯道横向加速度上限为 2.5 m/s²。路径剩余距离、前方曲率和障碍距离继续参与控速；提前减速距离约 4.5 m，最大前视点距离 1 m，局部代价地图至少 10×10 m。 目标接近速度按剩余路径计算可停车速度上限；短目标仍可正常加速，4.5 m 用作全速停车预算。速度仍由原控制器根据路径、障碍距离和目标距离调节。低速检查时可显式追加 `linear_speed_limit:=0.3 angular_speed_limit:=0.5`。
行进跟踪采用 0.6 s 前视时间、1 m 最大前视距离，平移比例增益为 4.0、微分增益为 0.0。前视点因重规划或避障发生变化时，继续由距离制动和速度平滑器控制减速。中间航点进入 0.35 m 范围后才从待经过列表移除。运行状态读取 raw 处理节点与过滤点云配对发布的里程计时间，并继续检查地形、车体 TF 和定位更新。

基础配置默认读取 包内 `config/nav2_competition.yaml`，也可通过 `params_file:=` 传入等效文件。原生 3D 入口再用 `config/implementation_profile.yaml` 的 navigation 项覆盖速度、平移加减速度、弯道横向加速度和 raw 范围，较低平移上限通过 `linear_speed_limit:=` 指定。

### 可选调试：单独查看原生定位

需要单独检查定位、彩色点云或录制 raw 时，可结束当前 Odin 启动进程，再在已加载上述环境的终端选择 localization。它是独立调试入口，正常导航直接使用上面的 navigation 命令。

```bash
ODIN_MAP_DIR="$HOME/sentry_ws/maps/field_v2_20261024"
env -u PYTHONNOUSERSITE ros2 launch odin_3d_test odin_3d.launch.py \
  mode:=localization native_map:="$ODIN_MAP_DIR/native.bin"
```

## 3. 地图选点与比赛树

### 3.1 在新地图上选点

新终端加载四个环境，切换到选点域 85。地图显示和点位记录在该域运行，车辆导航使用域 0。

```bash
source /opt/ros/humble/setup.bash
source "$HOME/sentry_ws/install/setup.bash"
export ROS_DOMAIN_ID=85 ROS_LOCALHOST_ONLY=1 PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1
ODIN_MAP_DIR="$HOME/sentry_ws/maps/field_v2_20261024"
ODIN_ROUTE_FILE="$ODIN_MAP_DIR/waypoints_$(date +%Y%m%d_%H%M%S).yaml"

ros2 launch odin_sentry_bringup route_picker.launch.py \
  map:="$ODIN_MAP_DIR/navigation_ground_continuous/navigation.yaml" \
  output_file:="$ODIN_ROUTE_FILE"
```

在选点窗口选择 **2D Goal Pose**，按住左键拖出朝向，按前进顺序逐个选点。沿用六点路线时，第 1 点是撤退终点，第 6 点是前进终点；点位放在通道中部，为车体留出空间。每个有效点立即保存，文件包含 `map`、`frame_id` 及按顺序排列的 `waypoints`。保存目录为 当前地图目录，文件名使用启动时的时间。

撤销末点时，在已加载环境的另一终端执行：

```bash
ROS_DOMAIN_ID=85 ROS_LOCALHOST_ONLY=1 ros2 service call /odin_route/undo std_srvs/srv/Trigger '{}'
```

### 3.2 更新点列并启动原比赛树

2026-10-27 选取的六点已写入原比赛树：前进按第 1 至第 6 点，撤退按第 6 至第 1 点，各点朝向沿用选点记录。点列来源为 `~/sentry_ws/maps/field_v2_20261024/waypoints.yaml`，配套地图为 `field_v2_20261024/navigation_ground_continuous/navigation.yaml`。

`competition_phase1.xml` 和 `competition_retreat.xml` 的修改仅涉及 `SendNav2ThroughPoses.goals`。每点格式为 `x;y;yaw`，点之间使用 `|`。以后重新选点时同步更新这两处，并重新启动行为树加载。

保留第 2 节的 navigation 运行，在新终端执行：

```bash
source /opt/ros/humble/setup.bash
source "$HOME/sentry_ws/install/setup.bash"
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 TZ=Asia/Shanghai

ros2 launch behavior pb2025_sentry_behavior_launch_new.py \
  params_file:="$HOME/sentry_ws/src/behavior/params/sentry_behavior_phase1.yaml"
```

看到 `Behavior Tree startup request accepted: competition_phase1` 表示执行请求已接收。原树继续检查 `/map`、`map` 到 `base_footprint` 的 TF 和 `navigate_through_poses`，等待期间沿用云台扫描及底盘零速；就绪后等待裁判开赛。前进、战斗、撤退、恢复及结束逻辑继续使用原比赛树。本包继续提供 `NavigateToPose`、`NavigateThroughPoses` 接口。前进、撤退阶段的旋转指令为 4 rad/s，战斗及恢复阶段为 7 rad/s；初始化与开赛前的零速保持原样。底盘仍有 1200 rpm 轮速限制，平移与旋转同时请求时按轮速和功率条件确定实际运动。

### 3.3 模拟裁判流程

确认新点列已加载、定位吻合，现场具备遥控接管条件后，在另一个终端执行。此命令发布模拟开赛状态，会触发车辆执行比赛路线；本轮模拟裁判话题使用这一组发布者。

```bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
bash "$HOME/sentry_ws/src/behavior/scripts/run_competition_trial_recorded.sh"
```

脚本自动加载 ROS 和 sentry_ws 环境，记录保存在 `~/sentry_ws/log/behavior_spin_detail/`。真实比赛由原裁判通信节点提供状态。

## 4. 可选补充：用过滤后的 raw 重建导航图

第 1 节已经生成一套定位图与导航图。需要进一步利用 raw 去畸变、confidence、15 cm 裁剪及近 4 m 内密度过滤时，再执行本节。此录制工具使用已存在的 BIN 和定位后的 `map` 坐标。

使用第 2 节的 navigation，或该节末的可选 localization 调试入口。确认位置、朝向吻合后，在另一终端加载相同四个环境，执行：

```bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
export PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1
ODIN_MAP_DIR="$HOME/sentry_ws/maps/field_v2_20261024"
ros2 run odin_3d_test record_navigation_cloud.py \
  --native-map "$ODIN_MAP_DIR/native.bin" \
  --output "$ODIN_MAP_DIR/filtered.ply" --seconds 180
```

手动覆盖需要导航的区域，转动云台补充地面与墙面。raw 范围为 10 m，按有效观测覆盖场地。Ctrl+C 可提前结束并保存。工具记录与该 BIN 配套的地面参考点，未观察区域保持未知。采样时尽量保持场地物体静止。

录制完成后导出，再将导航命令的 `map:=` 换成 `navigation_filtered/navigation.yaml`；`native_map:=` 继续使用同一个 BIN：

```bash
ros2 run odin_3d_test build_navigation_map.py \
  --recording "$ODIN_MAP_DIR/filtered.ply" \
  --output "$ODIN_MAP_DIR/navigation_filtered" --resolution 0.05
```

本节输出另存目录。5 cm 格子的地面支持取决于 raw 录制密度，灰色空缺可通过补充观测处理。

## 2026-10-24：静态地面支持与里程计更新

新地图保留 5 cm 分辨率。原生地图已经合并多次观测，同一格可能只保留一个地面点；离线导图将每格地面支持点数从 3 改为 1。只有落在已拟合地板附近的真实点参与支持，零地面点格继续保持未知。墙体和低顶仍按每格 8 个障碍点判断，实时地形仍使用原来的 3 点要求。

本轮修正图另存到 `field_v2_20261024/navigation_observed_floor/`。对照原图，空地格由 34285 增至 74459，5112 个障碍格的位置全部保留。按 20.6 cm 车体内切半径检查，当前车位在地图上的连通面积约 55.7 m²；此项为离线几何结果，实车通过性另行核对。两张代价地图的 25 cm 膨胀半径及未知边缘处理继续沿用。

同一批消息测得 `loam_interface` 的里程计转发额外等待最长约 0.31 s。点云转换与里程计回调分别使用独立回调组，独立进程采用多线程执行器，让位姿更新与点云处理并行。初始坐标变换设置完成后再供点云读取；已有 0.5 s 数据有效时间、急停和遥控处理继续沿用。隔离安装环境包含修正后的 `loam_interface`，上面的第四条 source 会加载它。

## 已扫描平地中的灰色空洞与启动通信

针对本轮已经确认的平地，静态导图补齐被实测地面完整包围、宽和高均在 10 cm 内的小空洞。算法按八邻接检查：接触地图边界、连接大片未知区域或紧邻障碍的空白继续保持未知；墙体和低顶障碍位置继续保留。该处理仅用于离线平地图，实时 raw、坡面分类和地形更新继续沿用。新版本另存于 `navigation_ground_continuous/`；`navigation_observed_floor/` 保留对照。

本轮发现导航启动收到请求后，控制器或规划器已完成配置，但 Fast DDS 在回复客户端时等待发现结束超过默认的 0.1 s，日志显示 `client will not receive response`。导航管理器停在该次请求，后续节点仍处于 inactive 或 unconfigured，路径和代价地图随之缺少输出。

`config/fastdds_services.xml` 将本轮导航进程的服务回复发现等待设为 2 s。launch 通过 `FASTRTPS_DEFAULT_PROFILES_FILE` 加载，仅使用名为 `service` 的发布配置。点云话题、0.5 s 数据有效时间和实际速度控制继续沿用。对应 ROS 服务实际配置与三次新客户端请求由 `service_reply_discovery` 检查。

参考：[Fast DDS 服务发现处理](https://github.com/ros2/rmw_fastrtps/blob/humble/rmw_fastrtps_shared_cpp/src/rmw_response.cpp)、[ROS 2 服务配置](https://fast-dds.docs.eprosima.com/en/2.x/fastdds/ros2/ros2_configure.html)。

## 2026-10-26：车体轮廓内的实时标记

本轮实机启动后，map_server、controller_server、planner_server 和 bt_navigator 均进入 active，单独路径计算成功。现场确认车辆周围为空地，最新 raw 点云与地形图在近车区域均未给出障碍，但代价地图仍保留旧标记。

SurfaceLayer 复用 Nav2 的车体轮廓栅格化处理，跳过自身格的实时标记。静态地图内容继续参与代价计算，轮廓外的实时障碍由最新有效帧替换。车体尺寸、25 cm 膨胀半径、速度及数据有效时间继续沿用。

复现检查覆盖车体内实时点、车体位置改变后的自身格、轮廓外障碍、静态地图障碍，以及原有绕行和观测恢复。已运行的导航在下次重启时加载新插件，随后核对车旁标记与实际移动。

## 2026-10-26：拱门高度与实时障碍来源

拱门静止记录显示，原 terrain_analysis 的局部分位数地面在拱顶附近升高，部分约 60 cm 高的回波被计算成约 36 cm 的障碍。此前的保留机制还会持续累积拱沿偶发的低回波。当前已取消实时障碍的跨帧保留。

当前平地模式的两张代价地图使用 `static_layer`、`surface_layer`、`inflation_layer`。SurfaceLayer 继续接收经过既有 raw 去畸变、confidence 和密度过滤的数据，统一使用 PCL 地板平面与车体净空。`/odin_obstacles/terrain_map` 保留用于对照；平地导航的实际地形来源为 `/odin_3d/surface`。`ground_mode: local` 继续保留原强度层组合，坡面实测另行完成。

实时地形目标更新率为 10 Hz。每个 5 cm 格内，离地超过 10 cm 且处于 50 cm 车高内的回波达到 3 点即标记障碍；离地超过 10 cm 且处于 55 cm 总净空内的回波达到 6 点，也标记障碍。这样保留车体高度内的障碍，同时抑制顶部 5 cm 余量中的稀疏边缘回波。参数位于 `implementation_profile.yaml` 的 `obstacle_support`、`clearance_support` 和 `initial_rate_hz`。离线导图继续使用原有点数配置。

检查覆盖 15 cm 矮障碍、45 cm 低拱、53 cm 结构、60 cm 拱沿回波、静态地图保留、实时障碍更新及数据过期。20 秒实车记录中的 161 帧 raw 经最终地形节点与实际 Nav2 在隔离 ROS 域回放，规划成功，生成 43 个路径点、约 2.46 m 路径。记录位于 `~/odin_ws/log/odin_3d_checks/arch_height_20261026_135400/raw_planner_replay/result.json`。实车通过拱门在重启加载后核对。

## 检查与实际限制

```bash
source "$HOME/sentry_ws/install/setup.bash"
OPENBLAS_NUM_THREADS=1 ctest \
  --test-dir "$HOME/sentry_ws/build/odin_3d_test" --output-on-failure
```

检查涵盖平地、20°坡面、陡坡、低顶、抬高平台、坑洞未知区域、物体移除、TF 合成、过期指令、失效保持以及真实 Nav2 插件加载。
ROS 合成测试使用域 113–117；检查过程与实车控制分开。速度测试加载现有 fake_vel_transform，覆盖空闲旋转、导航平移、行为树零旋转优先、短暂中断后的三组新数据恢复、持续中断后的零平移保持、感知过期后继续旋转、遥控关闭或失联后的停转。

当前是单层高度场。桥上桥下等同 XY 多层道路需要后续扩展。
当前帧的地面盲区沿用已扫描地图中的地面信息，避免把车下和视野外的空白全写成障碍。静态地图未知区域和已记录的坑洞继续限制通行。运行期间新出现的坑洞仍需有效观测或专用负障碍检测确认；本版本的单帧空白仅表示缺少观测。
运动目标通过实时点云标记与地形更新处理，目标身份、速度预测属于额外功能。
30°坡面实际通行、低顶洞口、小陀螺中的定位稳定性和整机 CPU 占用，均待现场检查。
软件通过检查与实车比赛完成分别记录。

2026-10-23 自动检查：6/6 通过，完整输出位于 `~/odin_ws/log/odin_3d_checks/tests.log`。该结果对应软件与合成数据，实车效果后续单独记录。

## 建图累计显示

建图模式的 `Accumulated Color Map` 显示 `/odin_3d/color_map`，保留所有已接收方向，转向后旧区域继续显示。显示按 10 cm 体素去重，最多 50 万点，整图每秒更新一次；`Native Color Preview` 保持实时约 10 Hz。达到显示容量时保留旧区域，并提示调整显示参数。

累计显示只处理本次启动后收到的观测。设备原生地图独立保存至 BIN；回环优化后的最终几何以 BIN 导出为准。二维 `Navigation Map` 仅在导航模式启用。

2026-10-23 定位实测修复：原生定位与运动输入状态分离；地形节点缓存最多 5 帧，从有效期内选择已有对应测量时间 TF 的最新帧。帧间沿用仍在有效期内的地形结果，过期后停止使用。两项针对性 ROS 检查通过。

2026-10-23 已生成本车 field_v1 导航图：`~/odin_3d_maps/field_v1_navigation/navigation.yaml`。原生重定位位置与朝向已获现场确认，导航运动测试后续进行。离线导图默认每格至少 8 个离地点支持障碍，抑制多帧地图中的稀疏历史散点；命令末尾可追加支持点数量。实时 raw 与地形输入沿用现有参数。初次导出结果保存在 `field_v1_navigation_initial` 供对照。

## 彩色分支的车体与近点过滤

`odin_nav_adapter` 按既有 `self_filter.yaml` 执行 SLAM 点云过滤：裁去 15 cm 球内点及随云台转动车体区域，再应用 1 m 内、4 cm 邻域至少 5 个其他点的过滤。`color_filter` 配对同一时间戳的原始彩色点云与 `/odin_navigation/cloud`，按已保留的 XYZ 点顺序复制原始 RGB 记录，省去第二次邻点搜索。RGB 字段随点记录完整保留。

实时彩色预览、累计彩色显示和 PLY 预览导出统一读取 `/odin_3d/color_filtered`。raw 实时避障使用本测试配置的 10 m 范围，并保留按距离缩放的邻点过滤。 邻点统计继续采用现有 FLANN KD 树，找到所需数量后结束搜索；过滤半径、邻点数量和距离缩放保持原值。近墙密集输入的对照检查覆盖逐点保留结果及处理时间。设备内部 BIN 地图保存过程仍使用设备原生数据；主机显示过滤与设备地图分别记录。已有累计显示包含旧点时，重新开始累计显示可清除历史显示缓存。

若两个工作空间采用不同目录布局，构建时通过 `-DODIN_CLOUD_FILTER_INCLUDE=<odin_sentry_bringup/include>` 指定现有头文件目录。过滤实现保留在原包，新增节点复用同一份源码。

## 平地与拱下净空修正

当前场地已确认全为平地，`implementation_profile.yaml` 的 `added_terrain.ground_mode` 设为 `flat`。使用 PCL 带重力方向约束的平面分割，在车体地面估计附近选地板；天花板和拱顶保留为上方结构。每格仍需真实地面回波，只有天花板的格子保持未知。

当前车高 50 cm，顶部余量 5 cm。合成数据检查中，60 cm 上方结构保留地面通行候选，45 cm 结构标记障碍。通道宽度与曲面拱的侧缘仍由实际整车轮廓检查，现场两个拱的分类需要对照其位置确认。

新对照图：`~/odin_3d_maps/field_v1_filtered_20261023_151641_plane/navigation.yaml`。原始录制和此前地图全部保留。后续起伏场地可选择 `ground_mode: local`，离线导出对应使用 `--ground-mode local`。

此前 field_v1 平地导航图的道路与拱洞分类已由现场确认。本轮新图使用第 2 节导航命令；新图分类、实车通过拱洞和整条路线分别检查。

## 当前帧障碍更新与 10 m raw 范围

本轮 3D 导航的 raw 距离上限为 10 m，密度过滤覆盖近 4 m；既有二维导航配置分别保留。实时地形窗口为 18×18 m，5 cm 栅格、10 Hz。

SurfaceLayer 使用最新有效地形帧更新实时障碍，已移除 odom 网格中的跨帧障碍存储及连续三帧空地清除计数。新帧中变为空地、未知或位于观测窗口之外的旧实时标记随下一次代价地图更新移除。平地模式使用自身地形分类；其他模式的强度障碍层按原方式独立更新，SurfaceLayer 仅叠加当前地形帧。静态地图中的黑格和未知区域继续参与规划。

因此，新障碍离开视野后，其实时标记也会消失；对应区域沿用静态地图。当前可见障碍、车体轮廓排除、10 m raw 范围和原有高度筛选继续使用。

## 原生地图中的障碍坐标与时间

当前原生地图地面约为 map Z=-0.45 m。两张代价地图的障碍层和观测源最低世界 Z 均设为 -2 m，避免把 map Z=0 误作地面；真实离地高度筛选继续使用 terrain_analysis 的 intensity。该世界高度范围适用于当前平地地图，换用高度范围不同的地图时应重新核对。

raw 节点额外发布 odom 到 odin_obstacle_sensor，直接使用点云曝光结束时刻的对齐位姿与设备外参。两张代价地图以该坐标作为观测原点，原安装 TF 继续保留。复用的 terrain_analysis 保留输入点云的原始秒、纳秒字段，避免时间转为浮点数后产生舍入误差。

SurfaceLayer 在帧间使用仍处于 0.5 秒有效期内的最新观测；过期后停止写入该帧障碍，并报告观测过期。原生运行节点继续执行输入过期后的零平移处理，收到有效数据后按原恢复流程继续。

## 中途绕障跟随

实车录制中，规划折线绕开了障碍，但原 0.4 m 前视点对应的直线运动段切入了障碍边缘，触发持续零速。复用的 PB 控制器现在检查这两条几何路线：直线段比规划路径更早遇障时，沿原路径逐次缩短前视距离，选择整段通过车体检查的近点；最短为两个栅格。缩短后的距离同时约束制动速度。路径本身有障碍或车体范围发生重叠时继续停车。

本项修改位于 pb_omni_pid_pursuit_controller，重启导航后加载。先用 `linear_speed_limit:=0.2` 做同一弯道的低速检查，原比赛速度配置保留；实车通过情况单独确认。

## 恢复过程中的实时地形更新

Nav2 整图清理调用会清空 SurfaceLayer 当前帧，等待下一帧有效地形重新标记。跨帧障碍保留已移除；保存地图仍由 map_server 加载。输入恢复所需的三组新里程计与 raw 点云检查继续执行。

Odin 测试入口复用现有两个 Nav2 导航 XML，在运行副本中把自动 BackUp 替换成 1 秒 Wait，并将 FollowPath 恢复重试设为 2 次，随后继续原有清理与每秒 3 次重规划。比赛行为树文件和原 Nav2 XML 均保留。这样，暂时没有通路时保持原地等待，路径恢复后继续跟随。

驱动 PTP 平滑窗口设为 5 个采样。此前单次异常校时造成约 20 ms 的时间变化，触发高频位姿时间检查；同段日志复算中，五点均值的最大变化为约 4.5 ms。时间异常及位姿跳变后的停车检查继续执行。

## 2026-10-26：当前地图的小块灰区修正

当前 `field_v2_20261024/navigation_ground_continuous/` 使用同一原生点云重新导出，平地空洞最大宽高从 10 cm 调整为 20 cm。仍按八邻接检查，只补齐由已确认空地完整包围的小块灰区，墙边、连接地图外部及较大灰区继续保留。地图中的 5112 个障碍格、5 cm 分辨率及原点保持一致。

本次补齐 323 块、1295 格，共约 3.24 m²。修正直接写入当前目录的 `navigation.pgm` 和 `terrain.csv`；原文件保存在该目录下的 `backup_before_gap20_20261026/`，检查统计保存在 `map_repair_20261026.json`。继续使用原 `navigation.yaml` 启动命令，重启导航后加载地图与新的等待参数。原生定位 BIN 继续使用当前文件。

## 2026-10-26：贴墙路径与局部车体检查

15:27 的停车记录中，同一车位在全局代价地图中的值为 244，在局部地图中为 253；最近静态墙格中心距车辆分别约为 20.5 cm 和 18.2 cm。当前 ThetaStar 允许代价低于 252 的格子，控制器还检查完整车体外缘及障碍格面积，所以全局地图允许的贴墙起点在局部图中会触发停车。

同图规划回放中，原路线最小墙格中心距离约 28.3 cm。全局 `footprint_padding` 调整为 10 cm、膨胀半径调整为 30 cm 后，记录路线及返回路线的最小距离分别约为 31.8 cm 和 30.4 cm；60 cm 高拱门对应的路径仍规划成功。已进入车体重叠范围的起点会返回规划失败，车辆停稳后由现场人员手动移回空地。SurfaceLayer 的车体自身排除使用原始物理轮廓；规划增加的余量区域内，真实实时障碍继续保留。局部碰撞检查和原速度上限继续使用。

本项修改集中在 `odin_3d_test` 的启动配置，原控制器与比赛树作为依赖继续使用。规划回放使用 ROS 域 126，结果位于 `~/odin_ws/log/odin_3d_checks/planner_clearance_20261026/`。该结果验证规划几何，实际跟随和连续过拱效果在下一轮现场记录中核对。
