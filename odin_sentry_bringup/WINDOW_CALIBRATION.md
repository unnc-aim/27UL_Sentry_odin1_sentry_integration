# 亚克力窗口的几何复核、离线校正与预览

这套工具用于保留亚克力板时，检查主机测距校正的实际效果。流程包括几何复核、受约束模型比较、实物与地面对照、独立点云预览，以及后续新样本采集。当前候选用于预览，模型文件记录 `preview_only: true` 和 `field_verified: false`。

新增工具位于本包 `scripts/`，使用已有 NumPy、SciPy、Pillow、Matplotlib 和 ROS 2 依赖。配置及原始记录分别保存，运行结果写入新目录。

## 1. 环境与输入

```bash
source /opt/ros/humble/setup.bash
source "$HOME/sentry_ws/install/setup.bash"
source "$HOME/odin_ws/install/local_setup.bash"
export PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1

ODIN_WINDOW_DATA="$HOME/odin_ws/log/odin_3d_checks/acrylic_20261026/calibration"
ODIN_WINDOW_SHARE="$(ros2 pkg prefix --share odin_sentry_bringup)"
```

`config/window_calibration.yaml` 使用相对记录目录。每组记录包含 `capture.npz`、`measurement.json`、`calib.yaml` 和相机图像。三个 `train` 记录用于拟合；`review` 记录用于回放比较；`object` 记录用于真实物体保留检查。各组的图像矩形明确对应墙身、地面或实物。

本轮尺量暂按沿雷达前向轴从观测面到墙面解释。`observation_face_offset_m` 是观测面相对雷达光学原点沿前向轴的有符号偏移，当前取 0；`reference_offset_uncertainty_m` 取 0.03 m 用于展示参考误差范围。后续机械图或实测结果可填写到配置中。

墙顶法向估计采用“墙顶水平”的几何假设和已记录的设备重力方向。程序在图像提示区域内寻找墙顶，排除偏离拟合轮廓的点，再计算墙面法向。不同扫描方向使用各自与该平面的交点作为参考距离。墙身以外的背景和地面由各自图像区域区分。

## 2. 复核几何并比较模型

先单独生成图像复核结果，每次使用新输出目录：

```bash
ros2 run odin_sentry_bringup window_calibration.py \
  --manifest "$ODIN_WINDOW_SHARE/config/window_calibration.yaml" \
  --data-root "$ODIN_WINDOW_DATA" \
  --output "$HOME/odin_ws/log/window_geometry_review" --review-only
```

打开输出的 `geometry_review.png`。绿色点应贴合墙顶，黄色矩形应位于墙身，青色矩形应位于空地。`geometry.json` 保存墙面法向、角度、轮廓拟合误差和参考距离变化。修改图像提示或取样范围后重新生成结果。当前配置中的四组墙面图像已完成此项检查。

完成图像复核后，将配置中的 `image_reviewed` 设为 `true`，执行完整回放：

```bash
ros2 run odin_sentry_bringup window_calibration.py \
  --manifest "$ODIN_WINDOW_SHARE/config/window_calibration.yaml" \
  --data-root "$ODIN_WINDOW_DATA" \
  --output "$HOME/odin_ws/log/window_replay"
```

比较三种距离模型：固定偏移、单系数比例和单调分段线性校正。confidence 仅用于有效数据选择。模型限定在实际记录的扫描方向与原始距离范围内，增益限制由配置给出。范围外的点保持原值。

输出包含 `geometry_review.png`、`geometry.json`、三份 `*_preview.npz` 模型、`evaluation.json` 和 `report.md`。评估同时报告：

- 已校正点与完整输出的距离误差、校正覆盖率。
- 沿用当前 15 cm 裁剪、4 cm 邻域及按距离调整的 20 邻点过滤后的实物保留情况。
- 图像地面区域内的假障碍点数量。该项采用配置中的 IMU 离地高度和设备姿态，属于离线估计。
- 单帧模型应用耗时。

配置中的初步目标是：校正点距离误差 P95 在 5 cm 内，覆盖至少 80% 已标注方向，真实物体在至少 95% 帧中保留至少 3 个点，已有目标点保留率至少 95%，地面假点减少。完整输出与已校正子集分别列出，以便看到覆盖不足造成的剩余误差。

`review` 数据已用于候选比较，新的现场检查使用另取的记录。离线地面统计和点云保留率与 Nav2 代价地图、整车实际运动分别验证。

## 3. 10 Hz 独立点云预览

先用录制回放查看结果。回放使用独立 ROS 域 132，并直接发布预览话题。两个终端分别执行完整代码块，各自加载环境和变量。

终端 A：启动持续回放。

```bash
source /opt/ros/humble/setup.bash
source "$HOME/sentry_ws/install/setup.bash"
source "$HOME/odin_ws/install/local_setup.bash"
export ROS_DOMAIN_ID=132 ROS_LOCALHOST_ONLY=1 PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1
ODIN_WINDOW_DATA="$HOME/odin_ws/log/odin_3d_checks/acrylic_20261026/calibration"
ODIN_WINDOW_RESULT="$HOME/odin_ws/log/odin_3d_checks/acrylic_20261026/workflow_replay_01"

ros2 run odin_sentry_bringup preview_window_calibration.py \
  --model "$ODIN_WINDOW_RESULT/piecewise_preview.npz" \
  --capture "$ODIN_WINDOW_DATA/100cm_20261026_204138" --rate 10
```

终端 B：打开显示窗口。

```bash
source /opt/ros/humble/setup.bash
source "$HOME/sentry_ws/install/setup.bash"
source "$HOME/odin_ws/install/local_setup.bash"
export ROS_DOMAIN_ID=132 ROS_LOCALHOST_ONLY=1 PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1
ODIN_WINDOW_SHARE="$(ros2 pkg prefix --share odin_sentry_bringup)"
rviz2 -d "$ODIN_WINDOW_SHARE/config/window_preview.rviz"
```

RViz 固定坐标为 `lidar`。灰色为原始点，青色为完整预览，绿色显示项仅展示经过模型校正的点。实时输入保留原消息时间、坐标名、confidence、intensity 和逐点时间；录制回放使用当前显示时间。话题如下：

- `/odin_window/preview/original`
- `/odin_window/preview/cloud`
- `/odin_window/preview/corrected_only`
- `/odin_window/preview/status`

实时预览在已有只读 Odin 驱动运行后使用域 0，并省略 `--capture`。预览节点只订阅点云并发布以上显示话题。

## 4. 新实物验证与后续接入

当 `evaluation.json` 中某个候选的 `eligible_for_new_field_check` 为 `true` 时，再安排新的实物检查。新的距离和表面保持独立，用于检查已确定的模型。板材、安装、相机标定和雷达布局与模型记录配套使用。

新记录可用下面的只读采集命令保存。运行前保持已有只读驱动、底盘和云台静止，并填写实际尺量距离：

```bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
ros2 run odin_sentry_bringup capture_window_calibration.py \
  --output "$HOME/odin_ws/log/window_new_object" \
  --distance-cm 100 --surface opaque_object --seconds 10
```

将新记录及其相机取样区域加入一份配置副本，填写 `review` 或 `object` 角色。使用已有模型进行固定参数复算：

```bash
ros2 run odin_sentry_bringup window_calibration.py \
  --manifest "$HOME/odin_ws/log/window_check.yaml" \
  --data-root "$ODIN_WINDOW_DATA" \
  --model "$ODIN_WINDOW_RESULT/piecewise_preview.npz" \
  --output "$HOME/odin_ws/log/window_fixed_model_check"
```

`--model` 路径中的系数保持原样，输出报告标记 `model_refitted: false`。通过独立预览核对点云与实物的位置、形状和保留情况。新的模型比较继续保留原拟合记录。

新实物验证通过后，校正输出的接入位置位于 `odin_raw_obstacles` 的原始点云输入处，之后继续复用原去畸变、车体过滤、地形和代价地图。接入时分别核对静止、云台转动和车辆运动。设备内部建图定位继续单独核对。

## 5. 本轮结果

2026-10-26 的本轮结果位于 `~/odin_ws/log/odin_3d_checks/acrylic_20261026/workflow_replay_01/`。墙面法向相对雷达前向的偏航角约为 1.48°、1.57°、2.70°、4.71°；图像轮廓已核对。

单调分段候选覆盖 553 个方向，在 100 cm 墙面记录中已校正点的误差 P95 约 21.0 cm，覆盖约 14.2% 的已标注方向。该候选对已有小方块取样区域的校正覆盖为 0，地面假点统计保持原值。当前三种候选均为 `eligible_for_new_field_check: false`。本轮结果支持继续离线研究，新的现场采集与导航接入尚待候选效果改善。

## 6. 固定安装下的本机处理

亚克力板及其安装位置保持原样，后续分析和实现均在本机完成。已有资料包保留为本地复算材料。

2026-10-26 已完成 10 Hz、14.5 Hz、10 Hz 三组深度采样对比，共保存 145 帧。100 cm 墙面在 1209 个共同方向上的前向距离中位数分别为 31.1、31.1、31.0 cm。提高采样频率后，明显的测距偏差仍然存在。深度频率已恢复 10 Hz，临时驱动已退出，导航配置保持原值。

本轮墙面和地面选区已分别核对。报告、图像范围、逐项统计与原始数据位于 `~/odin_ws/log/odin_3d_checks/acrylic_20261026/sampling_modes_231733/`。参阅其中 `report.md`、`comparison.json` 和 `geometry_review.png`。相机频率设置返回超时，本轮结论限定为实测确认的深度频率变化。

已有软件校正继续用于独立预览。下一项本机分析使用现有记录核对真实目标与异常回波的可区分程度，分别统计真实障碍保留、剩余杂点及距离误差，再决定局部过滤或距离修正的适用范围。新现场摆放采集暂告一段落。

## 7. 最新移板对照结果

2026-10-26 23:35，用户移开亚克力板，并确认车辆与云台位置、朝向保持原样。原 10 Hz 配置采集 51 帧。与装板的最后一组记录比较，1264 个共同墙面方向的距离中位数从 31.0 cm 恢复至 97.5 cm；尺量参考为 100 cm。

地面选区中估计离地 10–55 cm 的假障碍点中位数从 107 点/帧降至 0，同时地面保留总点数增加到约 2068 点/帧。墙身选区过滤后保留约 6935 点/帧。原 SPAD、confidence 和主机过滤参数保持原样。

这次对照支持亚克力板当前安装是本轮严重测距异常的主要来源。完整视野及运动场景仍分别核对。现有墙面几何参考误差中位数约 7.5 cm、P95 约 12.6 cm，包含测距、尺量参考与几何估计的影响。

详细结果见 `~/odin_ws/log/odin_3d_checks/acrylic_20261026/plate_off_20261026_233516/report.md`。移板后继续使用原有处理流程；上述亚克力补偿模型用于装板历史数据的独立研究。
