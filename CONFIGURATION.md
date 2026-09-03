# 配置说明

固定运行配置为 [config/workflow.yaml](config/workflow.yaml)，可热更新的工具标定为 [config/tool_offsets.yaml](config/tool_offsets.yaml)。相对路径以 `config` 目录为基准，例如 `../models/xuncao.pt` 指向项目根目录下的模型。

## 部署前必须确认

| 配置 | 当前值 | 说明 |
|---|---:|---|
| `http.listen_port` | `48051` | HTTP监听端口，需与防火墙和客户端一致 |
| `camera.ip` | `192.168.16.122` | SurfacePro50地址 |
| `camera.calibration_file` | `../calibration/chishine_192_168_16_122_calibration.yml` | 相机内外参文件 |
| `calibration.handeye_result_file` | `../calibration/handeye_result.json` | 手眼标定结果 |
| `yolo.model_file` | `../models/xuncao.pt` | YOLO模型 |
| `pose.standoff_mm` | `50.0` | 光心参考位到目标平面的距离 |
| `pose.pipeline_version` | `2` | 强制使用v2统一坐标链，防止误载v1配置 |
| `tool_offsets.file` | `./tool_offsets.yaml` | 每次目标TCP解算重新读取的工具标定文件 |
| `target_matching.source_image_width/height` | `1920/1080` | 客户端框选坐标对应的原图尺寸 |

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
- `snapshot_delay_s`：快照请求到达后等待稳定帧的时间，当前为旧流程使用的 `5.0` 秒；
- `save_snapshot`：是否将快照保存到磁盘；
- `request_timeout_s`：可选，未填写时为 15 秒；
- `snapshot_cache_ttl_s`：可选，未填写时为 300 秒。

两步流程依赖快照文件供人工或后台框选，因此 `save_snapshot` 应保持开启。实时 `live=true` 流程不依赖快照文件。

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

## v3光心参考、标准TCP与工具工作位

当前 `tool` 模式的唯一实现顺序为：

1. 使用实时目标中心和面板法向建立光心参考位姿，距离为 `standoff_mm`；
2. 计算 `T_base_camera_reference @ inverse(T_tcp_camera)`，得到逆手眼活动TCP；
3. 将 `pose.tcp_correction` 作为基座系 `mm + RPY°` 分量修正施加一次，形成标准活动TCP；
4. 重新读取 `tool_offsets.file`，根据YOLO类别取得 `tools.<类别>.standard_to_tool`；
5. 计算 `T_base_tcp_standard @ T_standard_tool`，得到最终活动TCP。

`pose.tcp_correction`不得在最终工具TCP上再次叠加。工具偏移的数值以 `config/tool_offsets.yaml` 当前内容为准。

`TBaseCameraReference`是视觉几何得到的理想50mm光心位姿；`TBaseTcpStandard`是逆手眼后再加入现场修正的实际执行基准。由于全局修正在补偿真实安装/标定残差，用标称手眼矩阵回算的 `TBaseCameraTarget` 不要求与理想参考矩阵逐元素相等。

v3拒绝加载旧字段 `camera_to_tool`，以免把旧综合参数误当成新工具偏移。已有工具使用 `migrate_legacy_tool_offset.py` 迁移；新工具使用 `calibrate_tool_offset.py` 和v3 `result.json` 中的 `cameraReference.TBaseTcpStandard` 标定。

## 工具偏移热加载

`workflow.yaml` 中不再允许内联 `tools`；它只通过 `tool_offsets.file` 指向独立文件。服务启动时先校验一次，之后每个 `/get_tcp_pose` 请求在检测到目标类别后重新读取一次，不使用文件内容缓存。

运行中只允许热更新 `tool_offsets.yaml`。修改 `workflow.yaml` 中的相机、YOLO、手眼、全局修正、端口或其他参数后仍需重启服务。

每个工具必须包含：

```yaml
version: 1
tools:
  class_name:
    tool_id: tool_name
    enabled: true
    standard_to_tool:
      xyz_mm: [X, Y, Z]
      rpy_deg: [RX, RY, RZ]
```

`xyz_mm` 和 `rpy_deg` 必须分别是3个有限数字；可选 `standoff_mm` 必须为非负数。文件无效时本次请求返回 `TOOL_CONFIG_RELOAD_FAILED`，不会使用内存中的旧参数。修复并保存后下一次请求自动恢复。

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
