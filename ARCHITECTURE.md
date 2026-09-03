# 架构与数据流

## 服务结构

```text
HTTP客户端
    ↓
ThreadingHTTPServer
    ↓
VisionHttpProtocol（字段检查、rad/deg转换）
    ↓
vision-task-worker（单线程串行执行所有视觉任务）
    ├─ CameraManager → SurfacePro50SyncAdapter
    ├─ YoloTargetDetector
    ├─ 平面拟合与目标匹配
    ├─ ToolOffsetsLoader → 每次读取config/tool_offsets.yaml
    └─ TargetPoseSolver
```

HTTP层可以接收多个连接，但相机、检测和解算统一进入一个视觉任务线程，因此不会并发调用相机。服务进程只创建一个相机适配器和一个 YOLO 实例。

## 相机生命周期

本版本使用旧项目已验证的直接取流方式：

1. 服务初始化时创建适配器并执行一次 `connect(camera.ip)`；
2. `/snapshot` 或实时解算请求到达后，在视觉任务线程内直接调用 `capture()`；
3. 仅采集标注彩色图时优先调用 `capture_color()`；
4. `CameraManager` 的可重入锁避免同一进程内并发访问；
5. 服务关闭时执行一次 `disconnect()`。

空闲时不会持续读取相机，也没有 `ContinuousCaptureThread`、帧缓存生产循环或相机命令队列。`/snapshot` 的完整 `CameraFrame` 会暂存在内存中，供一次后续解算复用；这与后台持续取流不是一回事。

## 两种业务流程

```text
两步流程：
POST /snapshot
    → 等待 snapshot_delay_s
    → 直接采集彩色图和配准点云
    → 保存彩色 JPG，并在内存缓存完整帧
POST /get_tcp_pose（live省略或false）
    → 使用缓存帧检测和解算
    → 成功后释放缓存

实时流程：
POST /get_tcp_pose（live=true）
    → 直接采集一份新帧
    → 检测、平面拟合和解算
    → 返回目标TCP
```

两步流程中，从快照拍摄到提交解算前不得移动机械臂、相机或目标。实时流程中，请求里的 `pos` 必须是本次采集时的实际 JAKA TCP。

## 坐标解算链

```text
当前TCP与手眼矩阵
    → 拍照时相机位姿
目标中心与安装平面法向
    → 基座系目标中心和法向
standoff_mm
    → 光心参考位姿
inverse(T_tcp_camera)
    → 逆手眼活动TCP
tcp_correction
    → 所有工具共用的标准活动TCP
standard_to_tool
    → 最终工具工作TCP
```

`tcp_correction`只对逆手眼活动TCP施加一次，工具偏移位于其后。零 `standard_to_tool` 表示输出标准活动TCP。算法实现位于 `vision_solver/pose_solver.py`。

## 工具配置热加载边界

服务初始化时只连接一次相机、创建一个YOLO实例并加载一次手眼矩阵。每次目标检测完成后、调用 `TargetPoseSolver` 前，`vision_solver/api.py` 会完整读取并校验 `config/tool_offsets.yaml`。读取成功后，该份不可混用的快照贯穿本次位姿计算；下一次请求再读新文件。

热加载失败只终止当前请求，不关闭相机或服务，也不会退回上一次旧参数。这样可以避免操作员已经保存新值、机械臂却仍按旧值运动。工具文件修复后可直接再次请求。

## 主要文件

| 文件 | 职责 |
|---|---|
| `run_service.py` | 服务入口与退出信号处理 |
| `vision_solver/http_server.py` | HTTP监听和JSON收发 |
| `vision_solver/http_protocol.py` | 接口字段、错误码和角度单位转换 |
| `vision_solver/api.py` | 串行任务、快照缓存和业务编排 |
| `vision_solver/config.py` | 主配置加载、独立工具文件热读取与严格校验 |
| `vision_solver/camera.py` | 旧项目直接取流方式的长期连接包装 |
| `handeye_calib/surfacepro50_adapter.py` | SurfacePro50/OpenNI底层适配器 |
| `vision_solver/target_matcher.py` | 标注矩形与YOLO目标匹配 |
| `vision_solver/pose_solver.py` | 光心参考、逆手眼、标准TCP、工具偏移和最终TCP |
| `config/tool_offsets.yaml` | 可在服务运行中修改的工具工作位偏移 |
| `calibrate_tool_offset.py` | 基于标准TCP标定工具偏移 |
| `migrate_legacy_tool_offset.py` | 将v1工具参数/示教结果迁移到v2 |
| `vision_solver/image_writer.py` | 运行时快照、报告和调试图写入 |

## 故障边界

Python任务超时只能结束HTTP等待，不能强制打断已经卡在厂家原生 `read_frame()` 内的线程。如果日志长时间停在取帧阶段，应停止整个服务进程；旧进程退出后仍无法连接时，再给相机断电重启。
