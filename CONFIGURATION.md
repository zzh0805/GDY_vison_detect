# 配置说明

固定运行配置为 [config/workflow.yaml](config/workflow.yaml)，可热更新的工具标定为 [config/tool_offsets.yaml](config/tool_offsets.yaml)。相对路径以 `config` 目录为基准，例如 `../models/best.pt` 指向项目根目录下的模型。

## 部署前必须确认

| 配置 | 当前值 | 说明 |
|---|---:|---|
| `http.listen_port` | `48051` | HTTP监听端口，需与防火墙和客户端一致 |
| `http.snapshot_delivery` | `minio` | 快照上传MinIO并返回对象URL；可改为`http`备用模式 |
| `http.snapshot_minio.endpoint` | `http://192.168.1.189:9000` | MinIO S3兼容地址 |
| `http.snapshot_minio.bucket` | `test` | 快照对象桶 |
| `http.snapshot_delay_s` | `0` | 上位流程已负责稳定等待，不再固定额外等待 |
| `http.live_capture_delay_s` | `0` | 实时解算不再固定额外等待 |
| `camera.ip` | `192.168.16.122` | SurfacePro50地址 |
| `camera.calibration_file` | `../calibration/chishine_192_168_16_122_calibration.yml` | 相机内外参文件 |
| `calibration.handeye_result_file` | `../calibration/handeye_result.json` | 手眼标定结果 |
| `yolo.model_file` | `../models/best.pt` | YOLO模型 |
| `pose.standoff_mm` | `50.0` | 光心参考位到目标平面的距离 |
| `pose.pipeline_version` | `2` | 强制使用v2统一坐标链，防止误载v1配置 |
| `tool_offsets.file` | `./tool_offsets.yaml` | 每次目标TCP解算重新读取的工具标定文件 |
| `target_selection.use_yolo` | `false` | `true`使用YOLO；`false`使用code选择工件且不运行YOLO |
| `target_selection.circle_refinement_enabled` | `true` | 无YOLO时：`true`使用圆拟合中心；`false`直接使用请求框中心；支持热加载 |
| `tools.<工件>.code` | 例如 `9-8-1` | 无YOLO模式由请求code动态选择工件；每个值必须唯一 |
| `tools.<工件>.target_color` | `red/green/black/auto` | 无YOLO模式期望颜色；缺省为自动识别 |
| `target_matching.source_image_width/height` | `1920/1080` | 客户端框选坐标对应的原图尺寸 |
| `target_matching.circle_refinement.enabled` | `true` | 仅作为旧工具配置缺少热加载开关时的兼容默认值 |

如果更换相机、相机与法兰的安装关系、活动TCP或工具安装位置，必须重新验证相机标定、手眼标定及 `standard_to_tool`，不能只改IP。

## `system`

- `initialization_timeout_s`：加载模型并连接相机的启动等待上限；
- `task_timeout_s`：内部视觉任务默认等待时间；
- `output_directory`：运行时结果目录；
- `save_report`：是否保存完整 `result.json`；
- `save_debug_image`：是否保存检测叠加图；
- `save_point_cloud`：是否保存点云；
- `log`：日志等级、文件和保留天数。

本交付目录没有历史输出。服务运行后会按上述开关自行创建 `output`、`shared_images` 和日志文件。若现场不需要检测图或报告，可关闭对应保存开关，不影响HTTP成功响应。

## `http`

- `listen_host`：通常保持 `0.0.0.0`；
- `listen_port`：当前为 `48051`；
- `snapshot_directory`：`/snapshot` 保存彩色图的位置；
- `snapshot_delivery`：`minio`直传对象存储，`http`由视觉服务自身提供下载；
- `snapshot_public_base_url`：只用于`http`模式；留空时从请求Host自动生成；
- `snapshot_minio.endpoint/bucket/object_prefix`：MinIO地址、桶和对象前缀；
- `snapshot_minio.url_mode`：`public`返回稳定公共URL，`presigned`返回有期限签名URL；
- `snapshot_minio.access_key_env/secret_key_env`：保存凭据的环境变量名，YAML中禁止写真实密钥；
- `snapshot_minio.delete_local_after_upload`：上传成功后是否删除视觉机临时JPG；
- `snapshot_delay_s`：两步流程额外稳定等待时间，当前为 `0` 秒；
- `live_capture_delay_s`：实时检测额外稳定等待时间，当前为 `0` 秒；
- `save_snapshot`：是否将快照保存到磁盘；
- `request_timeout_s`：可选，未填写时为 15 秒；
- `snapshot_cache_ttl_s`：可选，未填写时为 300 秒。

相机后台线程持续消费OpenNI2队列并只保存最新帧，因此默认不再增加固定等待。机械臂到位稳定仍由上位流程负责。MinIO模式要求 `save_snapshot: true`，视觉主机能访问MinIO的9000端口，并已安装requirements中的 `minio` SDK。实时 `live=true` 流程不依赖快照文件；缓存解算不会在 `/get_tcp_pose` 阶段重复采集。

## `camera`

- `type`：保持 `surfacepro50`；
- `ip`：相机地址；
- `calibration_file`：厂家相机标定文件；
- `python_path/openni_redist`：可在 YAML 中填写，也可以留空后由 `linux_sdk_paths.env` 提供；
- `show_driver_frame_output`：是否显示厂家底层取帧输出；
- `unload_openni_on_disconnect`：是否在断开时卸载 OpenNI，现场默认保持 `false`。

相机连接由服务长期持有。不要同时运行厂家 Viewer、相机测试脚本和正式服务。

## 坐标与角度单位

HTTP输入和输出均为 JAKA 基座系 `mm + RPY rad`。算法内部及 YAML 中的位姿修正使用 `mm + RPY deg`，角度转换只在 `vision_solver/http_protocol.py` 的接口边界进行。

请求里的拍照 TCP 必须对应实际采集帧。机械臂基座移动后，必须使用移动后重新读取的当前 TCP 和重新采集的图像，不能复用移动前的快照或 TCP。

现场运动的预备距离不属于正式视觉解算配置，位于 `field_test/test_case.yaml`：

```yaml
case:
  live: true
  base_label: 5

jaka_test:
  approach_distance_mm: 200.0
  return_to_capture_pose: true
```

`case.live` 决定现场测试请求是实时采集还是使用最近快照缓存；`case.base_label` 指定LabelMe中的安装面板标签，填写 `null` 或 `0` 可关闭。二者配置后，直接执行 `bash run_field_test.sh` 即可；命令行参数只用于临时覆盖。

方向不在YAML中指定。`run_field_test` 通过 `/motion/get_tcp_pose` 取得当次平面法向在JAKA基座系下的单位向量，自动适应车辆正对、侧对或斜对柜体。工具标定和 `/get_tcp_pose` 正式响应不受该配置影响。

`return_to_capture_pose` 控制到达工作位后的行为：`true` 沿相反法向退回预备位并继续返回拍照TCP，`false` 则保持在最终工作TCP，不执行退出动作。该字段必须填写YAML布尔值 `true/false`，不要加引号。命令行 `--return-to-capture` 或 `--stay-at-work` 可以临时覆盖它。关闭返回后工具可能持续接触工件，必须由现场流程负责后续安全撤退。

## v3光心参考、标准TCP与工具工作位

当前 `tool` 模式的唯一实现顺序为：

1. 使用实时目标中心和面板法向建立光心参考位姿，距离为 `standoff_mm`；
2. 计算 `T_base_camera_reference @ inverse(T_tcp_camera)`，得到逆手眼活动TCP；
3. 将 `pose.tcp_correction` 作为基座系 `mm + RPY°` 分量修正施加一次，形成标准活动TCP；
4. 重新读取 `tool_offsets.file`：YOLO模式按识别类别选工具，无YOLO圆心模式按本次请求 `code` 选工具；
5. 计算 `T_base_tcp_standard @ T_standard_tool`，得到最终活动TCP。

`pose.tcp_correction`不得在最终工具TCP上再次叠加。工具偏移的数值以 `config/tool_offsets.yaml` 当前内容为准。

`TBaseCameraReference`是视觉几何得到的理想50mm光心位姿；`TBaseTcpStandard`是逆手眼后再加入现场修正的实际执行基准。由于全局修正在补偿真实安装/标定残差，用标称手眼矩阵回算的 `TBaseCameraTarget` 不要求与理想参考矩阵逐元素相等。

v3拒绝加载旧字段 `camera_to_tool`，以免把旧综合参数误当成新工具偏移。已有工具使用 `migrate_legacy_tool_offset.py` 迁移；新工具使用 `calibrate_tool_offset.py` 和v3 `result.json` 中的 `cameraReference.TBaseTcpStandard` 标定。

## 工具偏移热加载

`workflow.yaml` 中不再允许内联 `tools`；它只通过 `tool_offsets.file` 指向独立文件。服务启动时先校验一次，之后每个 `/get_tcp_pose` 请求在目标选择和检测前重新读取一次，不使用文件内容缓存。

运行中只允许热更新 `tool_offsets.yaml`。修改 `workflow.yaml` 中的相机、YOLO、手眼、全局修正、端口或其他参数后仍需重启服务。

### 一次运行的交互式标定工具

现场推荐执行 `bash tool/run_interactive_tool_calibration.sh`。它从 `tool/interactive_tool_calibration.yaml` 读取JAKA地址和文件路径，随后全部操作通过终端提示完成：按code选择工件、选择模式、确认读取当前TCP、等待人工示教、计算并自动写回工具偏移。每次启动先覆盖生成唯一备份 `tool/tool_offsets.backup.yaml`，最近一次成功结果写到 `tool/interactive_tool_calibration_result.yaml`。

模式3默认使用标准/相机参考坐标系。XYZ增量在统一解算的标准TCP坐标系中表达，因此相机朝向JAKA基座 `+X`、`+Y`、`-Y` 或斜向时，微调仍跟随柜体/相机参考方向；只有在提示中显式选择JAKA基座坐标系时，XYZ才表示固定的基座轴方向。两种模式都会用刚体矩阵重新计算 `standard_to_tool`，而不是直接猜测YAML中的偏移分量。

该程序只读取JAKA当前活动TCP，不控制机械臂运动。正式配置采用原子替换并由视觉服务加载器校验；校验失败会立即用本次启动备份恢复。

### 旧版YAML分步骤标定工具

`tool/tool_calibration.yaml` 是独立的现场标定操作面板，不参与视觉服务启动。主要字段：

- `operation.mode`：`1`从50mm标准位零标定，`2`从旧工作位纠正到新工作位，`3`按JAKA基座系增量微调；
- `operation.step`：模式1/2依次使用1、2、3；模式3使用1、2；
- `workpiece.class_name`：必须与 `tool_offsets.yaml` 中工件键一致；
- `robot.ip/sdk_path`：只用于读取当前JAKA活动TCP；
- `adjustment.xyz_mm/rpy_deg`：模式3的基座系位置和JAKA RPY分量增量；
- `files.record_file`：自动记录每一步与最后结果；
- `output.apply_to_tool_offsets`：默认 `false`；设为 `true` 时备份并更新正式工具偏移。

该入口继续保留用于离线或分阶段操作。详细说明见 [tool/README.md](tool/README.md)。

每个工具必须包含：

```yaml
version: 1
target_selection:
  use_yolo: true
  circle_refinement_enabled: true
tools:
  class_name:
    code: "9-8-1"
    target_color: green
    tool_id: tool_name
    enabled: true
    standard_to_tool:
      xyz_mm: [X, Y, Z]
      rpy_deg: [RX, RY, RZ]
```

`xyz_mm` 和 `rpy_deg` 必须分别是3个有限数字；可选 `standoff_mm` 必须为非负数。`target_color` 可填写 `red`、`green`、`black` 或 `auto`，不填写等同于 `auto`。文件无效时本次请求返回 `TOOL_CONFIG_RELOAD_FAILED`，不会使用内存中的旧参数。修复并保存后下一次请求自动恢复。

### 目标选择模式

- `use_yolo: true`：保持原有流程。YOLO先检测并分类，再用请求 `target` 框中心选择最近的检测目标；请求 `code` 可以省略。
- `use_yolo: false`：本次请求不执行YOLO推理，并通过请求 `code` 选择工件及工具偏移。
- `circle_refinement_enabled: true`：仅在无YOLO模式生效。算法将请求 `target` 粗框附近的圆候选按圆心聚类，使用该工件的 `target_color` 参与筛选，再采用最近工件组的最大有效外圆圆心。
- `circle_refinement_enabled: false`：完全跳过圆拟合和颜色筛选，目标中心严格等于请求 `target` 的几何中心。
- 工件 `code` 必须是非空字符串并全局唯一；对应工件必须 `enabled: true`。缺少code返回400，未知或禁用的code返回422。
- 模式、圆拟合开关、code映射和工具偏移一起热加载，修改保存后下一次解算生效，无需重启。初始即为无YOLO模式时不会预加载YOLO；以后热切换为YOLO时会在第一次YOLO请求中按需加载。
- v3.3配置中的 `target_selection.selected_tool` 可以暂时保留以方便文件升级，但v3.4无YOLO流程会忽略它。

无YOLO圆心模式可以使用YOLO模型中不存在的新工件，只需先在 `tools:` 下新增、标定并分配唯一 `code`。若 `pose.alignment_mode: camera_center`，仍只输出标准光心TCP；若要叠加指定工件的 `standard_to_tool`，应设置为 `pose.alignment_mode: tool`。

### 无YOLO圆心拟合参数

运行时开关应修改 `config/tool_offsets.yaml` 的 `target_selection.circle_refinement_enabled`。下面这些参数仍位于 `workflow.yaml`，只控制开关开启后的拟合细节，修改后需要重启服务。旧工具文件没有新开关时，`enabled` 继续作为兼容默认值。

- `enabled`：是否在无YOLO分支启用圆心修正；关闭时保留v3.4的原框中心行为；
- `expand_ratio`：相对请求框宽高向外扩大的搜索比例；
- `min_score`：圆候选最低综合评分；
- `min_radius_ratio/max_radius_ratio`：圆半径相对原始请求框短边的范围；
- `min_edge_support`：单个圆候选的最低边缘支持度；
- `radius_band_count`：分多少个半径段检测，使同心内外圆都能进入候选；
- `center_cluster_tolerance_ratio`：同一工件的圆心聚类容差，相对原始请求框短边；
- `outer_circle_min_support_ratio`：外圆边缘支持度至少达到组内最佳值的比例，用于排除虚假大圆；
- `color_fusion.enabled`：是否使用红、绿、黑颜色证据筛选候选工件；
- `color_fusion.center_mode`：`edge` 使用最大有效外圆圆心（正式运行默认值）；`weighted_centroid` 才会把颜色质心按权重混入圆心，仅供实验；
- `color_fusion.min_coverage`：候选圆内部颜色连通区域的最低覆盖率，当前为较宽松的 `0.06`；
- `color_fusion.score_weight/selection_weight`：颜色对圆候选评分和工件组选择的权重；
- `color_fusion.center_blend`：仅在 `center_mode: weighted_centroid` 时生效，表示颜色质心对最终圆心的融合比例；
- `color_fusion.max_center_shift_ratio`：仅在 `weighted_centroid` 时生效，限制颜色最多修正多少个圆半径；
- `hough_param2`：霍夫圆阈值，越高越严格；
- `max_center_distance_px`：拟合圆心距离请求框中心的最大距离，单位为原始输入图像像素；
- `fallback_to_box_center`：找不到圆时是否回退框中心。正式机械臂运行建议保持 `false`，使本次请求直接失败。

这些参数位于固定的 `workflow.yaml`，修改后需要重启服务。颜色采用较宽松的HSV阈值；颜色不足时自动回退纯边缘结果。检测图中黄色是原框，绿色是最终采用的最大有效外圆及轴心，蓝色箭头是相对请求框的中心修正量。默认 `edge` 模式没有颜色质心位移；实验性的 `weighted_centroid` 模式才用洋红叉标记融合前的纯边缘圆心。

## Ubuntu SDK环境

复制并编辑环境模板：

```bash
cp linux_sdk_paths.env.example linux_sdk_paths.env
```

主要变量：

- `SURFACEPRO50_OPENNI2_REDIST`：直接包含 `libOpenNI2.so` 的 ARM64目录；
- `SURFACEPRO50_PYTHON_PATH`：包含 OpenNI Python包的目录；
- `SURFACEPRO50_LIBRARY_PATH`：厂家其他共享库目录；
- `JAKA_SDK_PATH/JAKA_LIBRARY_PATH`：现场联调读取或运动机械臂时使用；
- `VISION_CONDA_ENV`：运行服务的 Conda环境名。

`linux_sdk_paths.env` 是每台 Ubuntu 设备的本地配置，已被 `.gitignore` 排除。
