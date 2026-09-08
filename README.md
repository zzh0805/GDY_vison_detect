# GDY_vison_detect_v3 高低压视觉检测

本目录是用于 Ubuntu ARM64 部署的洁净版本，组合如下：

- 解算沿用v2统一坐标链；目标匹配和相机流程保持原项目实现；
- 工具标定独立到 `config/tool_offsets.yaml`，每次解算自动重新读取，无需重启服务；
- SurfacePro50 连接与拍照使用旧项目已验证的直接取流流程；
- 不包含历史快照、检测图、点云、结果报告、日志或 `__pycache__`；
- 保留运行必需的 YOLO 模型、相机标定、手眼标定、配置和启动脚本。

正式服务只计算目标 TCP，不控制机械臂运动。

## 当前版本

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
- 支持YOLO识别并匹配目标，也支持完全跳过YOLO、直接使用客户端框中心；
- 使用目标外围安装平面的点云拟合中心和法向，避免依赖目标自身深度；
- 根据拍照TCP、手眼标定、50 mm光心参考位和全局修正计算标准活动TCP；
- 按目标类别叠加工具固定偏移，返回JAKA基座系最终活动TCP；
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
每次读取tool_offsets.yaml中的目标模式及tools.<选定工件>.standard_to_tool
标准活动TCP × standard_to_tool → 最终工作TCP
```

全局修正只施加一次。工具偏移为零时，输出就是现场验证过的“逆手眼 + 全局修正”光心对齐结果。绿色/红色按钮参数已从v1现场示教结果完成数学迁移，详见 [migration/TOOL_OFFSET_MIGRATION.md](migration/TOOL_OFFSET_MIGRATION.md)。

## 工具标定热更新

服务启动后可直接修改 [config/tool_offsets.yaml](config/tool_offsets.yaml)。下一次 `/get_tcp_pose` 会在目标选择和位姿计算前重新读取并校验整个文件。相机连接、手眼矩阵和 `pose.tcp_correction` 不会重新初始化；若从框中心模式热切换回YOLO，模型会在首次YOLO请求时按需准备。

目标选择的两个开关如下：

```yaml
target_selection:
  use_yolo: false
  selected_tool: redkonb
```

`use_yolo: true` 保持原有“YOLO中心与请求框匹配”流程，此时 `selected_tool` 不参与选择。`use_yolo: false` 时不调用YOLO，直接使用请求 `target` 框的中心像素，并把 `selected_tool` 作为本次唯一工件。该名称必须是同一文件 `tools:` 下已启用的键；要使用对应工具偏移，还应保持 `workflow.yaml` 的 `pose.alignment_mode: tool`。

如果文件存在YAML语法错误、缺少类别、数组不是3个有限数字或仍使用v1字段 `camera_to_tool`，本次请求会失败并且不会沿用旧偏移。修正文件后直接重试即可，不需要重启服务。`service.log` 和保存的 `result.json` 会记录本次实际使用的数值及文件SHA-256。

Ubuntu工具标定不需要手写长命令。编辑 [run_tool_calibration.sh](run_tool_calibration.sh) 顶部“用户参数区”，然后运行：

```bash
bash run_tool_calibration.sh
```

脚本只计算并输出可粘贴的YAML，不连接相机、不控制机械臂，也不会自动覆盖配置文件。

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
