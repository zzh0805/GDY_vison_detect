# GDY 多视角彩色图采集工具

本工具用于让JAKA带动知象SurfacePro50相机，从目标左、右、上、下、四个斜向以及近、远位置逐步回到人工设置的参考拍照位，并保存多视角彩色图。真实模式在开始时自动采集一次对齐点云，用ROI外围安装面锁定物理三维中心；之后每个运动点都重新计算相机朝向、手眼逆变换和JAKA Rx/Ry/Rz。

## 与原视觉服务的关系

- 原有 `/snapshot`、`/get_tcp_pose` 的请求字段、响应字段、YOLO、点云配准和TCP解算逻辑不变。
- 原服务增加 `/capture_color` 和仅供本工具使用的 `/datas_get/reference`。
- `/capture_color` 不生成深度和点云，也不读取、替换或释放 `/snapshot` 的三维缓存。
- `/datas_get/reference` 只在一次采集开始时生成三维参考，也不读取、替换或释放 `/snapshot` 缓存。
- 相机仍然只由 `run_service.py` 持有；本工具不会再次连接相机。
- 所有相机任务由原服务的单工作线程串行执行，因此不会并发访问OpenNI2。采集期间同时调用生产接口不会损坏数据，但会排队并增加响应时间，建议安排独立采集时段。

## 一次运行流程

1. 启动原视觉服务。
2. 手动把机械臂和相机调整到标准拍照位，让工件位于画面ROI内。
3. 修改本目录的 `config.yaml`。
4. 运行一次 `run_collection.py`。
5. 程序读取当前JAKA TCP，用参考图的对齐点云拟合ROI外围安装面，把ROI中心射线和平面的交点固定到JAKA基座系。
6. 根据最大位置步长和最大视角步长自动加密轨迹，并完成全部路线的ROI、入射角、位移和旋转预检查。
7. 所有路线都通过后，机器人依次走到各方向起点，再逐点回到参考位并拍照；每次拍照前读取实际TCP并校验光轴误差。
8. 每条路线结束于对准后的参考位置，下一条路线不会从一个侧向起点直接跨到另一个侧向起点。
9. 全部完成后再次返回启动时读取的原始TCP。

## 第一次必须先模拟

保持：

```yaml
runtime:
  mode: simulation
robot:
  enable_motion: false
```

Windows：

```powershell
cd D:\porject\hitqz\datas_get_integration\GDY_vison_detect_v3\datas_get
python run_collection.py
```

Ubuntu：

```bash
cd /你的路径/GDY_vison_detect_v3/datas_get
python run_collection.py
```

模拟模式不会连接真实相机和机械臂，会在 `output` 中生成模拟彩色图、轨迹和清单。重点检查本次目录下的 `plan.json`。

## 真实采集

先按原方式启动视觉服务。由于新增了HTTP路由，更新代码后的第一次需要重启服务；以后采集不需要反复重启。

修改：

```yaml
runtime:
  mode: real
  require_confirmation: true
robot:
  enable_motion: true
```

确认以下内容：

- `reference.roi_xyxy_px` 是初始1920×1080彩色图中的 `[左,上,右,下]`；
- `reference.target_source` 保持 `point_cloud_plane`，真实模式自动获取目标三维位置；
- ROI外围必须能看到足够面积的安装平面，不能全部被工件或遮挡物占满；
- 相机标定文件和手眼文件路径正确；
- `trajectory.views` 中的角度和距离符合现场空间；
- JAKA已经上电、使能并位于人工设置的参考拍照位。

然后仍然只运行：

```bash
python run_collection.py
```

输入 `START` 后开始。需要无人值守自动开始时，可把 `runtime.require_confirmation` 改成 `false`。

## YAML中的主要参数

- `dataset.output_directory`：最终数据集保存根目录；支持相对或绝对路径。
- `service.discard_stale_frames_s`：机器人到位后，拍照前消费积压原始帧的时间；不生成点云。
- `dataset.name`：本次工件数据集名称。
- `reference.roi_xyxy_px`：初始图像工件ROI。
- `reference.target_source`：`point_cloud_plane`使用真实点云，`configured_depth`仅用于无点云调试。
- `reference.target_depth_mm`：模拟模式或配置深度模式的备用光轴深度；真实点云模式不依赖它定位目标。
- `reference.geometry`：外围平面范围、RANSAC阈值、最少点数、内点率和RMS质量门槛。
- `trajectory.samples_per_path`：每条路线的最少拍照数量。
- `trajectory.max_position_step_mm`：相邻点最大平移，超出时自动增加点数。
- `trajectory.max_look_angle_step_deg`：相邻点最大观察角变化，超出时自动增加点数。
- `trajectory.max_samples_per_path`：自动加密允许的单路线最大点数。
- `trajectory.max_view_incidence_deg`：相机相对安装面法向允许的最大侧视角。
- `trajectory.verification`：机器人实际到位的位置、旋转、光轴误差和重试次数。
- `trajectory.image_margin_px`：ROI投影到图像后的边缘余量。
- `trajectory.max_tcp_translation_mm`：任一采样TCP相对参考位允许的最大平移。
- `trajectory.max_tcp_rotation_deg`：任一采样TCP相对参考位允许的最大旋转。
- `trajectory.views`：采集方向、角度和距离列表。

建议第一次真实测试使用5°以内侧向角和20mm以内距离偏移，确认实际运动方向后再逐步使用默认范围。

## 保存结果

```text
output/<dataset.name>/<时间戳>/
├── images/
│   ├── reference.png
│   ├── left_000.png
│   └── ...
├── plan.json
├── manifest.jsonl
└── session.json
```

只保存彩色图，不保存深度图和PLY。参考点云只在内存中拟合一次。`manifest.jsonl` 同步记录计划TCP、拍照前后实际TCP、观察角度、相机距离、动态ROI、位置误差、姿态误差和光轴误差。

## ROI保证方式

真实模式使用已经对齐到彩色图的点云，在ROI外围拟合安装平面，再用去畸变后的ROI中心/四角射线与平面求交，得到固定的三维中心和区域。规划每个相机位置时：

1. 相机Z轴始终指向固定三维中心；
2. 相机X轴从前一个姿态平行传输，防止滚转和欧拉角突然跳变；
3. 通过 `T_base_tcp = T_base_camera × inverse(T_tcp_camera)` 得到机器人TCP；
4. 转换JAKA Rx/Ry/Rz时选择与前一点最近的±360°等价角；
5. ROI四角投影必须全部在图内并满足边缘余量；
6. 到位后使用实际TCP重新计算光轴误差，合格后才拍照。

路径会按 `max_position_step_mm` 和 `max_look_angle_step_deg` 自动加密，因此机器人在相邻离散点之间的朝向偏差会尽量减小。当前JAKA适配器使用阻塞式直线运动，不是高频伺服控制，所以不能声称运动中的每个毫秒都严格零误差；但所有规划点和拍照点都会严格执行look-at并做实际TCP校验。

## 参考平面质量失败

以下情况会在机械臂运动前终止，不会退回可能错误的人工深度：

- ROI外围有效点云少于 `min_plane_points`；
- 平面内点率低于 `min_inlier_ratio`；
- 平面RMS大于 `max_plane_rms_mm`；
- 点云没有逐像素对齐彩色图或坐标系与手眼矩阵不兼容；
- 路线越出图像、超过TCP范围、超过安装面入射角或需要过多路径点。

若ROI紧贴面板边界，可减小 `plane_expand_ratio`；若ROI周围仍被工件占据，应调整初始拍照位或扩大可见安装面，而不是放宽质量阈值强行运动。

## 运行严格测试

在项目根目录运行：

```bash
python -m unittest discover -s datas_get/tests -v
python -m unittest tests.test_http_protocol -v
```

这些测试完全使用模拟机器人和模拟图像，不连接真实硬件。
