# v5.0.0 发布说明与使用策略

日期：2026-09-22。基于 v3.8（源 VERSION 为 3.8.4），仅替换正式服务相机采集层。

这里的“仅替换”是相对本地 v3.8.4 基线。GitHub 上较早版本到本次发布的差异还包含继承的
v3.8.4 MinIO 快照交付、相关测试与配置，详见 RELEASE_NOTES_v3.8.4.md；不是 v5 新改的 TCP 算法。

## 更新内容

- 使用厂家原生 3DCamera SDK 软触发，替代正式同步接口中的 OpenNI 持续取流。
- 常驻独立相机子进程；每次请求触发一次，读取 RGB8 与 Z16，复制后释放 SDK 帧。
- 固定共享缓冲区传递原始图像；空闲不触发、不读取，不在后台不断积累 Python 帧列表。
- 外部调用超时保护及相机子进程 RSS+Swap 2 GiB 保护。异常不使用旧图顶替结果。
- SDK 关闭卡死时由进程隔离机制终止子进程；这是故障隔离，不是修复厂家 SDK 析构问题。
- HTTP 请求/响应、端口、业务 YAML、YOLO、工具偏移热加载、手眼标定、TCP 解算、进入方向和机械臂脚本保持原实现。

## 文件导航

| 路径 | 作用 |
| --- | --- |
| `run_service.py`、`vision_solver/` | 原业务服务、HTTP、检测和 TCP 解算 |
| `handeye_calib/native_surfacepro50.py` | 相机子进程管理、数据转换及原配准调用 |
| `handeye_calib/native_camera/worker.py` | 原生相机连接、启动排空、软触发和取帧 |
| `handeye_calib/native_camera/bridge.cpp` | SDK 与 Python 之间的桥接 |
| `build_native_camera.sh` | Ubuntu 本机编译桥接库，不控制机器人 |
| `config/workflow.yaml` | 原服务、检测、相机和通信配置 |
| `config/tool_offsets.yaml` | 原工件映射、标定及偏移配置 |
| `calibration/` | 相机与手眼标定，部署时保留现场版本 |
| `tool/soft_trigger_test/` | 独立取流、空闲恢复与内存验证，不可与服务同时运行 |
| `field_test/`、`tool/`、`datas_get/` | 继承的现场测试、工具标定和数据采集工具 |

## Ubuntu 部署

1. 备份旧版本与现场配置、模型和标定。不要用仓库样例覆盖现场最后一次工具微调。
2. 停止旧服务及其自动重启，关闭独立测试器和厂家软件，确保相机只有一个使用者。
3. 进入 v5 项目根目录（GitHub 检出目录名称不影响运行），激活原环境，配置完整厂家 SDK：

```bash
conda activate ur_odcam
export SDK_ROOT="/home/nvidia/workspace/vision/GDY_vision_detect_service/3DCameraSDK-v3.2.229.20251110"
export SDK_LIB_DIR="$SDK_ROOT/lib/3dcamera/linux/aarch64"
bash build_native_camera.sh
bash start_vision_service.sh
tail -n 80 service.log
```

依赖 g++（build-essential）和匹配 CPU 架构的 SDK 头文件、lib3DCamera.so；只有 libOpenNI2.so 不够。
x86_64 主机使用 SDK 的 linux/x64 目录。生成的 .so 与 sdk_location.json 不随仓库发布，必须在部署机生成。
原 Python 依赖与 JAKA 环境继续沿用；配置方法见主 README 与 V5_NATIVE_CAMERA.md。
systemd 的工作目录、启动路径须指向 v5，不要让旧版和新版同时启动。

初次验证先在 field_test/test_case.yaml 中设置 `jaka_test.start_jaka: false`、`jaka_test.work_jaka: false`，
避免测试脚本移动机械臂；机械臂和车辆保持静止，由现场确认拍照姿态。
随后可运行 `bash run_field_capture.sh` 取快照；按原流程准备标注并配置 case.live/base_label/code，再运行 `bash run_field_test.sh`。
这些现场脚本具备运动能力，开启运动前必须检查其 YAML；正式视觉服务本身只解算、不下发运动。

停止服务：`bash stop_vision_service.sh`。SDK 关闭异常时可能需要相机断电重启。

## 取帧时间与丢帧策略

本次发布保持代码默认 `getPairedFrame(10000)`，不是曝光 10 秒，也不是新增 HTTP 参数。
实测该 SDK 经常到等待边界才返回成功；已有缓存时可能提前返回，不能以返回快判断图像新鲜。

若现场选用 4 秒：修改 `handeye_calib/native_camera/worker.py` 中
`lib.v5_read_pair(10000, ...)` 的首参数为 `4000`，保存并重启服务；无需重编译 C++。
不要修改外部进程看门狗为 4 秒。独立测试器的 `--timeout-ms` 不会修改正式服务。

- 启动连接时有限排空旧帧，直到连续三次无帧；不是固定丢一两组。
- 每次拍照不额外丢弃一两组：一次触发，一组数据用于解算。
- `discard_frames(duration)` 在 v5 中仅执行不触发的稳定等待。
- 若将来增加每次丢弃一两组，需额外触发并读取，4 秒配置下会额外增加约 4/8 秒；本版没有加入。
- 仅彩色快照也读取彩深组，但跳过点云；复用已有快照的解算不一定重新拍照。
- 请求超时/相机错误会终止相机子进程，检查后需重启服务，不自动重试、不返回旧图。

## 现场证据与限制

以下为用户提供的独立测试日志，不是完整视觉服务的性能保证：

| 配置 | 已观察结果 |
| --- | --- |
| 2 秒 | 连续 20 组成功，约 2032 ms；另一轮空闲 300 秒后首次取帧超时，因此不建议直接固定为生产值 |
| 4 秒 | 空闲测试后续完成 119 组，约 4032 ms，118 次 RGB 内容变化；所给后半段 RSS 39.5 MiB 持平并进入采集后空闲，未提供该轮完整关闭结果 |
| 10 秒 | 之前测试完成空闲 300 秒、约 609 秒采集 55 组、再空闲 300 秒；这几阶段 RSS 44484 KiB 持平，但最终释放对象卡死 |

测试脚本每组额外约 1 秒不触发检查，所以 4 秒配置观察间隔约 5.07 秒，正式服务没有这段额外检查。
用户已反馈 v5 桥接编译成功并启动服务。完整服务包含模型、点云、缓存和上传，不能套用独立测试内存数字。
RGB 内容哈希变化也可能来自噪声；时间戳异常/为零时，不能证明绝对新鲜度和严格同步。
深度尺度从 SDK 读取，配准继续使用原标定文件和算法；曝光、增益、白平衡未强制锁定，需核对实际画质和测距。

## 独立测试和验收

先停服务和其他相机程序，再运行（脚本位于项目根目录下）：

```bash
MODE=soft STREAMS=rgbd DURATION=60 bash tool/soft_trigger_test/run_test.sh --timeout-ms 4000
IDLE_BEFORE=300 DURATION=600 IDLE_AFTER=300 bash tool/soft_trigger_test/run_idle_test.sh --timeout-ms 4000
```

验收须包含多轮启动、空闲后首次拍照、持续请求、停止重启；检查保存的实际 RGB/深度对齐和 TCP，
分别观察服务主进程及 native_camera/worker.py 子进程的 RSS。2 GiB 保护仅针对子进程，不限制主服务。
原生释放对象卡死仍是已知问题；整体测试 False 需查看原始异常，不能一律忽略。

## Windows 与离线验证

本版实时相机桥接仅支持 Linux。Windows 可编辑配置、使用模拟/离线测试，不能把旧 OpenNI 的 Windows 说明当成 v5 实时支持。

```powershell
python -m unittest discover -s tests -q
python -m unittest discover -s tool/soft_trigger_test -p test_offline.py -q
```

不上传日志、拍照结果、缓存、本机 SDK 路径文件、密钥或编译产物。MinIO 凭据继续通过环境变量提供。
回退时停止 v5，恢复备份的旧版启动路径及现场配置；不改变工具坐标定义，也不自动要求重新标定。

发布前离线复验：2026-09-22，业务与相机层 102 项通过，独立测试器 19 项通过，共 121 项。
