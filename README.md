# GDY_vison_detect_v3 高低压视觉检测

本目录是用于 Ubuntu ARM64 部署的洁净版本，组合如下：

- 解算沿用v2统一坐标链；目标匹配和相机流程保持原项目实现；
- 工具标定独立到 `config/tool_offsets.yaml`，每次解算自动重新读取，无需重启服务；
- SurfacePro50 使用后台最新帧缓存，并按彩色主帧选择后续深度帧；
- 不包含历史快照、检测图、点云、结果报告、日志或 `__pycache__`；
- 保留运行必需的 YOLO 模型、相机标定、手眼标定、配置和启动脚本。

正式服务只计算目标 TCP，不控制机械臂运动。

## 当前版本

`v3.8.3`（2026-09-18）更新内容：

- SurfacePro50 改为后台持续消费相机队列并只保存彩色、深度各自的最新原始帧，减少请求时读取到旧机位缓存的概率；
- 收到请求后以彩色帧为主：最多等待3秒获取请求后的新彩色帧，再选择彩色之后到达的深度帧；若3秒没有新彩色帧，则按现场“机器人已到位并保持静止”的前提使用现有彩色缓存，不再直接报超时；
- 移除快照和实时检测固定额外等待4秒，机械臂到位稳定时间继续由上位流程负责；
- 兼容厂家返回0、缺失或非有限帧号/时间戳的情况，不再用无效元数据误判旧帧；
- 新增彩深序号、到达时间差、驱动帧号/时间戳、慢取帧和原始数据量日志，并增加独立流速诊断脚本；
- 客户端提前超时断开时单独处理 Broken Pipe，不影响后续请求；
- 当前现场彩色1920x1080原始帧约6.22MB，百兆交换机实测约2.064fps；推荐使用全千兆链路，巨帧不能替代千兆带宽。

完整说明见 [RELEASE_NOTES_v3.8.3.md](RELEASE_NOTES_v3.8.3.md)。HTTP请求、TCP解算、工具偏移和机械臂控制边界保持兼容。

`v3.8.2`（2026-09-17）更新内容：

- `field_test/test_case.yaml` 新增 `case.live` 和 `case.base_label`；
- 日常现场测试只需执行 `bash run_field_test.sh`，脚本自动读取实时拍照模式、安装面板标签和工件code；
- `--live`、`--no-live`、`--base-label` 和 `--code` 继续保留为单次命令行覆盖参数；
- 正式视觉服务、TCP解算和后台HTTP接口不受影响。

`v3.8.1`（2026-09-17）更新内容：

- `run_field_test` 新增 `jaka_test.return_to_capture_pose` 开关，默认 `true` 保持原有返回拍照位行为；
- 设为 `false` 时，机械臂到达工作TCP后停止并保持在工作位，不执行退出预备位或返回拍照位；
- 新增单次命令行覆盖参数 `--return-to-capture` 和 `--stay-at-work`，不需要反复修改YAML；
- 正式视觉服务、TCP解算和后台HTTP接口不受影响。

`v3.8.0`（2026-09-15）更新内容：

- 新增一次启动即可完成整个工件标定的终端交互程序，不再反复修改YAML中的mode/step并多次运行；
- 可按 `code` 或工件名称选择目标，支持从零标定、已有工件纠正、XYZ/RPY微调三种模式；
- 每次启动先覆盖保存唯一的 `tool/tool_offsets.backup.yaml`，成功解算后自动校验并原子更新正式 `config/tool_offsets.yaml`；
- 微调默认使用标准/相机参考坐标系，跟随柜体朝向，不受车辆正对、侧对或斜对柜体造成的JAKA基座方向变化影响；同时可显式选择JAKA基座坐标系；
- 原有YAML分步骤标定入口继续保留，正式视觉服务和后台HTTP接口没有变化。

`v3.7.1`（2026-09-15）更新内容：

- 在热加载的 `config/tool_offsets.yaml` 新增 `target_selection.circle_refinement_enabled`；
- `true` 时保留无YOLO颜色筛选与最大有效外圆拟合，`false` 时完全跳过拟合并直接使用请求框几何中心；
- 开关在每次解算时重新读取，修改后下一次请求生效，不需要重启服务；
- 旧工具配置缺少该字段时继续沿用 `workflow.yaml` 中原有的 `circle_refinement.enabled`，避免升级后行为突变；
- `/snapshot`、`/get_tcp_pose` 和 `/motion/get_tcp_pose` 的请求与响应接口保持不变。

`v3.7.0`（2026-09-14）更新内容：

- 现场运动不再固定沿JAKA基座X进入，而是读取当次安装平面法向在基座系中的 `approachDirectionBase`；
- 车正对、侧对或斜对柜体时，统一按法向反退可调距离生成预备TCP，再沿同一直线进入和退出；
- 预备TCP保持最终工作位的全部RPY，工具标定、最终TCP坐标链以及正式 `/get_tcp_pose` 的输入输出完全不变；
- 新增隔离的 `/motion/get_tcp_pose` 供现场运动脚本读取方向，方向无效时拒绝运动；旧 `--approach-x-mm` 仅作为距离兼容入口。

`v3.6.0`（2026-09-12）更新内容：

- 新增 `tool/tool_calibration.yaml`，通过 `mode + step` 分步骤记录JAKA当前TCP，不再手输长位姿参数；
- 支持从50mm标准位从零标定、旧工作位到新工作位纠正、JAKA基座系XYZ/RPY微调三种模式；
- 每一步和最终结果自动保存到独立记录YAML，可选择只输出，或备份后自动更新正式工具偏移；
- 标定脚本只读取机械臂位姿，不发送运动指令，并提供Ubuntu Bash和Windows PowerShell入口。

`v3.5.3`（2026-09-11）更新内容：

- 颜色继续用于从相邻目标中筛选正确的红、绿、黑工件；
- 最终机械轴心固定采用最大有效外圆圆心，不再默认混入颜色区域质心；
- 避免黑色旋钮手柄、红色按钮高光/阴影把最终中心拉偏；
- 保留可选的 `weighted_centroid` 实验模式，正式运行默认使用 `center_mode: edge`。

`v3.5.2`（2026-09-11）更新内容：

- 无YOLO模式把红、绿、黑HSV颜色证据与最大有效外圆融合；
- 每个工件可用热加载字段 `target_color` 指定期望颜色，旧配置缺少该字段时自动识别；
- 颜色证据不足时自动回退纯边缘外圆，不会因反光或曝光变化直接停止；
- HTTP输入输出保持不变，运行报告新增颜色、覆盖率、颜色质心和融合状态。

`v3.5.1`（2026-09-11）更新内容：

- 圆形工件改为“先选离请求框中心最近的同心圆组，再选该组最大有效外圆”；
- 分半径段搜索内外圆，并过滤边缘支持不足的虚假大圆，避免选中偏心内圈；
- HTTP输入输出和 `targetSelection.mode` 保持兼容，报告新增同心圆组候选数量与选择规则。

`v3.5.0`（2026-09-11）更新内容：

- `use_yolo=false` 时不再直接信任粗框中心，而是在粗框附近用灰度边缘拟合圆心；
- 如果搜索区域内有多个工件，只选择离请求框中心最近的工件，一次请求仍只解算一个目标；
- 圆拟合不依赖颜色和YOLO类别，适用于模型中没有的圆形按钮、旋钮；
- 找不到可靠圆时默认终止本次解算，避免机械臂使用可能偏移的框中心；
- `/snapshot`、`/get_tcp_pose` 请求和成功响应格式保持不变。

`v3.4.0`（2026-09-10）更新内容：

- 无YOLO模式由请求新增的 `code` 动态选择工件，不再锁定 `selected_tool`；
- 每个工件可在 `config/tool_offsets.yaml` 的自身配置下增加唯一 `code`；
- `code`、模式和工具偏移都在每次解算时热加载，切换工件无需重启服务；
- `run_field_test.sh`/`.ps1` 支持 `--code 9-8-1`，也可在 `field_test/test_case.yaml` 设置 `case.code`；
- 现场运动支持可调的基座X预备距离：预备位摆好姿态，仅沿基座X进入，完成后按相同预备位返回。

`v3.3.0`（2026-09-08）更新内容：

- 在 `config/tool_offsets.yaml` 新增热加载的 `target_selection.use_yolo` 与 `target_selection.selected_tool`；
- `use_yolo: false` 时完全跳过YOLO推理，以请求中 `target` 矩形的几何中心作为唯一目标中心；
- 框中心模式每次只使用 `selected_tool` 指定的工件及其工具偏移，即使该工件不在YOLO模型类别中也能解算；
- `/snapshot`、`/get_tcp_pose` 的请求及成功响应格式保持不变；模式和指定工件修改后无需重启服务。

`v3.2.0`（2026-09-07）更新内容：

- 用持续消费原始彩色/深度帧替代服务端纯 `sleep`，降低导航空闲期间OpenNI2积压帧进入解算的风险；
- 丢帧阶段不解码图像、不执行软件配准或点云重建，避免额外的大块内存开销；
- `/snapshot` 和 `live=true` 都使用相同策略，缓存解算不会重复丢帧；
- 运行日志和完整 `result.json` 记录最终采集前丢弃的帧对数量；
- 新增SurfacePro50原始流丢帧及三条HTTP路径的自动测试。

`v3.1.0`（2026-09-03）更新内容：

- `/get_tcp_pose` 使用 `live=true` 时，服务收到请求后等待 `live_capture_delay_s`（当前4秒），再拍照、检测和解算；
- `/snapshot` 继续单独使用 `snapshot_delay_s`，缓存解算不会重复等待；
- 新增 `run_tool_calibration.sh`，在Ubuntu上修改脚本顶部参数后即可通过Bash完成新工具标定、旧工具重标定、验证和微调；
- 增加实时延时自动测试，并同步更新配置、接口、现场和部署文档。

## 项目功能

- 长期独占连接一台SurfacePro50，只在收到请求时采集彩色图和配准点云；
- 支持YOLO识别并匹配目标，也支持完全跳过YOLO、用红绿黑颜色筛选目标并采用最大有效外圆圆心；
- 使用目标外围安装平面的点云拟合中心和法向，避免依赖目标自身深度；
- 根据拍照TCP、手眼标定、50 mm光心参考位和全局修正计算标准活动TCP；
- 按目标类别叠加工具固定偏移，返回JAKA基座系最终活动TCP；
- 现场运动按当次柜体法向自动生成预备位，适应移动车底座正对、侧对及斜对柜体；
- 支持工具偏移热更新、运行报告、检测叠加图、现场LabelMe联调和无硬件模拟测试。

## 目录与文件

| 路径 | 功能 |
|---|---|
| `run_service.py` | 正式HTTP服务入口，处理退出信号并保证相机关闭 |
| `vision_solver/` | HTTP协议、串行任务、相机管理、目标匹配、位姿解算和结果保存 |
| `handeye_calib/` | SurfacePro50直接取流、JAKA适配、手眼数学和YOLO三维几何 |
| `config/workflow.yaml` | 启动时读取的固定配置：相机、模型、手眼文件、端口、全局修正等 |
| `config/tool_offsets.yaml` | 每次解算重新读取的工具标定，可在服务运行时修改 |
| `calibration/` | 相机标定与手眼标定文件 |
| `models/xuncao.pt` | YOLO模型 |
| `field_test/` | 快照、LabelMe标注、坐标解算、JAKA只读/运动联调脚本 |
| `tool/` | YAML驱动的三模式工具标定、记录、备份及跨平台运行入口 |
| `tool/interactive_tool_calibration.py` | 一次运行完成工件选择、位姿读取、计算、备份和自动写回的交互标定 |
| `datas_get/` | 独立多视角采集：点云锁定ROI三维中心、自动look-at姿态和到位校验 |
| `calibrate_tool_offset.py` | 根据标准TCP与工具示教TCP反算 `standard_to_tool` |
| `run_tool_calibration.sh` | Ubuntu可编辑参数式工具标定、验证和微调入口 |
| `migrate_legacy_tool_offset.py` | 将旧版工具标定迁移到统一坐标链 |
| `start_vision_service.*` / `stop_vision_service.*` | Ubuntu和Windows后台启停脚本 |
| `tests/` / `run_simulated_test.py` | 不连接真实相机和机器人的自动测试 |
| `systemd/` | Ubuntu开机自启动服务模板 |

## 当前相机取流方式

服务启动时创建一个 `SurfacePro50SyncAdapter` 并连接一次相机。所有视觉任务由一个串行的 `vision-task-worker` 执行，请求到来时直接调用旧项目的 `adapter.capture()` 或 `capture_color()`；空闲时不循环拍照，也没有额外的相机采集线程或取帧命令队列。服务退出时才断开相机。

`POST /snapshot` 当前在4秒稳定期内持续丢弃原始帧，再采集彩色图和配准点云。`POST /get_tcp_pose` 使用 `live=true` 时也会在收到请求后主动丢帧4秒，然后采集新帧、检测并解算；不传 `live` 时使用最近一次 `/snapshot` 的内存缓存，不再重复丢帧。

## v3解算方式

算法根据拍照TCP、正向手眼矩阵、目标中心和平面法向构造光心距目标平面 `pose.standoff_mm` 的参考位姿，然后统一执行：

```text
拍照TCP × T_tcp_camera → 拍照相机位姿
光心参考位姿 × inverse(T_tcp_camera) → 逆手眼活动TCP
逆手眼活动TCP + pose.tcp_correction → 标准活动TCP
每次读取tool_offsets.yaml中的目标模式；无YOLO时按请求code映射工件
无YOLO时读取工件target_color筛选目标，采用最大有效外圆圆心
读取tools.<选定工件>.standard_to_tool
标准活动TCP × standard_to_tool → 最终工作TCP
```

全局修正只施加一次。工具偏移为零时，输出就是现场验证过的“逆手眼 + 全局修正”光心对齐结果。绿色/红色按钮参数已从v1现场示教结果完成数学迁移，详见 [migration/TOOL_OFFSET_MIGRATION.md](migration/TOOL_OFFSET_MIGRATION.md)。

## 工具标定热更新

服务启动后可直接修改 [config/tool_offsets.yaml](config/tool_offsets.yaml)。下一次 `/get_tcp_pose` 会在目标选择和位姿计算前重新读取并校验整个文件。相机连接、手眼矩阵和 `pose.tcp_correction` 不会重新初始化；若从无YOLO模式热切换回YOLO，模型会在首次YOLO请求时按需准备。

无YOLO动态工件选择示例：

```yaml
target_selection:
  use_yolo: false
  circle_refinement_enabled: true

tools:
  greenbtn:
    code: "9-8-1"
    target_color: green
    tool_id: tool_green_button
    enabled: true
    standard_to_tool:
      xyz_mm: [109.525640, 52.685308, -16.277166]
      rpy_deg: [-3.005278, -4.276208, -0.141311]
```

`use_yolo: true` 保持原有“YOLO中心与请求框匹配”流程，请求 `code` 可以省略。`use_yolo: false` 时不调用YOLO，并由 `circle_refinement_enabled` 决定中心策略：设为 `true` 时，先把请求 `target` 粗框附近的内外圆按圆心聚类，选择离粗框中心最近且符合 `target_color` 的工件组，再采用该组最大且边缘支持可靠的外圆圆心；设为 `false` 时，完全跳过圆检测和颜色筛选，直接采用请求 `target` 的几何中心。请求中的 `code` 在两种情况下都负责选择具有相同 `tools.<工件>.code` 的已启用工件。每个 `code` 必须是非空字符串且在整个文件中唯一；要叠加对应工具偏移，还应保持 `pose.alignment_mode: tool`。

`target_color` 可填写 `red`、`green`、`black` 或 `auto`。不填写等同于 `auto`；颜色证据不可靠时自动使用纯边缘结果。

圆心拟合参数示例：

```yaml
target_matching:
  circle_refinement:
    enabled: true
    expand_ratio: 0.45
    min_score: 0.35
    min_radius_ratio: 0.08
    max_radius_ratio: 0.75
    min_edge_support: 0.55
    radius_band_count: 7
    center_cluster_tolerance_ratio: 0.18
    outer_circle_min_support_ratio: 0.65
    color_fusion:
      enabled: true
      center_mode: edge
      min_coverage: 0.06
      score_weight: 0.20
      center_blend: 0.35
      max_center_shift_ratio: 0.20
      selection_weight: 0.20
    hough_param2: 28
    max_center_distance_px: 250
    fallback_to_box_center: false
```

黄色框/点代表请求粗框及其中心，绿色圆/点代表最终采用的最大有效外圆及机械轴心，蓝色箭头代表中心修正方向。默认 `center_mode: edge` 时不会显示洋红叉；只有启用实验性的 `weighted_centroid` 时，洋红叉才表示融合前的纯边缘圆心。`workflow.yaml` 内的拟合阈值和算法参数属于启动配置，修改后需要重启服务；`tool_offsets.yaml` 内的 `use_yolo`、`circle_refinement_enabled`、`code`、`target_color` 和工具偏移支持热加载。

请求示例：

```json
{
  "pos": [379.5, -432.0, 509.3, 1.539, -0.836, 1.536],
  "base": {"x1": 100, "y1": 100, "x2": 900, "y2": 900},
  "target": {"x1": 852, "y1": 548, "x2": 982, "y2": 679},
  "live": true,
  "code": "9-8-1"
}
```

如果文件存在YAML语法错误、缺少类别、数组不是3个有限数字或仍使用v1字段 `camera_to_tool`，本次请求会失败并且不会沿用旧偏移。修正文件后直接重试即可，不需要重启服务。`service.log` 和保存的 `result.json` 会记录本次实际使用的数值及文件SHA-256。

现场推荐直接启动一次交互式标定：

```bash
bash tool/run_interactive_tool_calibration.sh
```

程序会在同一个终端会话中列出工件并提示输入 `code`、模式和每个机械臂示教步骤。按Enter或输入yes读取当前活动TCP；程序保持运行，等待手动调整完成后再次确认。最后自动计算并写回 `config/tool_offsets.yaml`，视觉服务下一次解算热加载生效。交互程序不会发送机械臂运动指令。旧的 `tool/run_tool_calibration.sh` YAML分步骤入口和根目录手动参数入口继续保留。完整流程和坐标系说明见 [tool/README.md](tool/README.md)。

## Ubuntu 快速启动

假设部署目录为 `/home/nvidia/software/GDY_vison_detect_v3`：

```bash
cd /home/nvidia/software/GDY_vison_detect_v3
cp linux_sdk_paths.env.example linux_sdk_paths.env
```

编辑 `linux_sdk_paths.env`、`config/workflow.yaml` 和 `config/tool_offsets.yaml`，至少确认相机 IP、SDK路径、Python环境、HTTP端口、模型路径、两个标定文件及工具类别。然后先前台启动：

```bash
bash run_service.sh
```

确认相机、目标选择模式和端口均初始化成功后，可改为后台运行：

```bash
bash start_vision_service.sh
tail -f service.log
```

停止服务：

```bash
bash stop_vision_service.sh
```

默认 HTTP 端口为 `48051`。完整部署步骤见 [UBUNTU_DEPLOY.md](UBUNTU_DEPLOY.md)。

## Windows快速启动

安装64位Python依赖和厂家Windows版OpenNI/SurfacePro50 SDK，设置对应环境变量并确认 `config/workflow.yaml` 后，可以前台运行：

```powershell
python -m pip install -r requirements.txt
python run_service.py
```

或指定Python解释器后台启动：

```powershell
powershell -ExecutionPolicy Bypass -File .\start_vision_service.ps1 `
  -Python ".\.venv\Scripts\python.exe" -Port 48051
```

完整环境变量、现场脚本和排障步骤见 [WINDOWS_DEPLOY.md](WINDOWS_DEPLOY.md)。

## 文档

- [ARCHITECTURE.md](ARCHITECTURE.md)：代码结构、相机所有权和数据流；
- [CONFIGURATION.md](CONFIGURATION.md)：配置项和坐标含义；
- [INTERFACE.md](INTERFACE.md)：HTTP请求与响应；
- [FIELD_TEST.md](FIELD_TEST.md)：现场联调和相机稳定性测试；
- [UBUNTU_DEPLOY.md](UBUNTU_DEPLOY.md)：Ubuntu部署与 systemd。
- [WINDOWS_DEPLOY.md](WINDOWS_DEPLOY.md)：Windows安装、SDK配置、启停和联调；
- [migration/TOOL_OFFSET_MIGRATION.md](migration/TOOL_OFFSET_MIGRATION.md)：v1工具参数迁移依据与复算方法。

## 运行安全

- 同一时间只能有一个进程或软件占用相机；启动服务前关闭厂家 Viewer 和旧视觉服务。
- 拍照、读取当前 TCP 和发送 `/get_tcp_pose` 期间，机械臂、相机、目标均应保持静止。
- 首次验证必须关闭机械臂运动开关，只检查返回坐标；确认坐标、单位和方向后再启用运动。
- `workflow.yaml`、相机标定和手眼标定修改后必须重启；只有 `tool_offsets.yaml` 支持热更新。
- 如果厂家原生库出现 `std::bad_alloc`、`terminate()` 或永久阻塞，Python线程无法恢复原生进程，应停止服务、确认旧进程退出，必要时给相机断电重启。
