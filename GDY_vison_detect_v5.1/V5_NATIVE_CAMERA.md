# v5.0.0 原生SDK软触发相机

> v5.1.1 起相机的运行形态已改为“待机静止（双流挂起）+ 按需推流（只 paused/resume）”，
> 拍照改为单流 getFrame×2，不再使用 getPairedFrame。本文档描述的接入与标定部分仍然有效，
> 但与配对取帧/流重建相关的过程已被取代，以 RELEASE_NOTES_v5.1.1.md 为准。

## 范围

基于本地GDY_vison_detect_v3.8（VERSION=3.8.4）的工作目录建立独立v5。
未修改v3.8源目录。复制保留配置、模型、手眼/相机标定、测试样例；不复制.git、
Python缓存、日志、output等运行输出。发布说明见RELEASE_NOTES_v5.0.0.md。

保持不变：config/workflow.yaml、config/tool_offsets.yaml、HTTP请求/响应协议、端口、
超时、MinIO上传配置、YOLO参数、工具标定与热加载、目标匹配、进入方向、
手眼/全局修正/TCP坐标链、机械臂控制脚本、软件深度到彩色投影算法。
无需因版本升级重新数学迁移工具偏移，但现场必须验证新采集深度与原标定的一致性。

修改入口仅为SurfacePro50SyncAdapter的后端创建、CameraManager断开时无条件清理。
新增native_surfacepro50.py、native_camera桥接/子进程、build_native_camera.sh和离线测试。
旧SurfacePro50Backend、异步Qt相机和OpenNI专用诊断仍保留原样供历史对照；
不要用这些旧诊断判断v5服务是否在使用原生SDK，也不要与服务同时占用相机。

## 数据路径

服务启动 -> 独立原生SDK子进程连接相机一次 -> 开启双流、暂停双流、设置mode=2
-> 清两路队列并验证静止（连续超时）-> 等待服务请求。

每次实时拍照 -> 清除SDK深度队列 -> 清除SDK彩色队列 -> 排空到超时 -> softTrigger(1)
-> getFrame(深度) + getFrame(彩色)  （v5.1.1 起不再使用 getPairedFrame(timeout=10000ms)）
-> SDK帧释放前复制RGB8和Z16到固定共享缓冲区 -> 主进程复制本次数据
-> RGB转换为原接口要求的BGR -> 原软件配准、RGB光学系毫米点云 -> 原解算流程。
仅彩色快照仍触发彩深配对，但跳过点云计算，不改变彩色接口返回结构。
缓存快照复用语义沿用原业务层；并非所有基于既有快照的解算都会重新拍照。

空闲不触发、不消费、不执行点云转换。没有无限队列，仅一组固定32MiB共享容量
（每路最大16MiB）及当前请求数组。保留既有上层缓存，不能用测试器内存数字
推断含YOLO/点云完整服务的RSS。

## 标定与单位

- 选用测试中profile=0的Z16和RGB8规格，启动日志输出实际尺寸，运行中规格变化拒绝。
- 深度尺度必须成功读取SDK PROPERTY_EXT_DEPTH_SCALE，不猜测1mm/0.1mm。
- 使用原SURFACEPRO50_MIN_DEPTH_MM/MAX_DEPTH_MM及原有效深度检查；无有效对齐点拒绝解算。
- 使用原相机标定YAML，沿用chishine_registration.py的分辨率缩放、外参方向、投影和z-buffer。
- RGB畸变与K继续按原规则用于camera_model；手眼和工具矩阵未改。
- 彩色时间戳不递增不拒绝有效图像，时间戳仅作为元数据。SDK配对不证明同时曝光，
  metadata明确synchronization_proven=false。需要机位静止并实测画面/深度对应。

## 原参数如何处理

所有业务YAML数值逐字保留。ip、calibration_file、稳定延迟仍使用原参数。
discard_frames(duration)在新相机层变成不触发的稳定等待，不引入额外拍照。
稳定等待结束后，实际capture在同一个串行相机子进程内清除两路SDK队列再触发。
任一路PROPERTY_EXT_CLEAR_FRAME_BUFFER失败均拒绝触发和取帧；不降级为使用缓存。
这不保证清除设备端/网络在途数据，也不证明严格同步或绝对新鲜度。
本次清队列更新必须停服务后重编译桥接库；旧库缺少接口时会提示重新编译。
旧OpenNI运行库选项及fresh_frame_timeout_s仍保留，但不用于新SDK的取帧超时
（v5.1.1 为 capture_timeout_ms=3000 + rgb_read_timeout_ms=500）；
这是采集实现更换带来的语义区别，不暗改原YAML。初始化可能比旧方式不同。
不自动应用原OpenNI重复取深度的多次重试，原生取帧失败明确返回错误，避免超时叠加及误用旧帧。

取帧超时是配置值（capture_timeout_ms / rgb_read_timeout_ms），不是固定 10 秒；
每次拍照的实际分阶段耗时以 CAPTURE 日志为准。现有任务/HTTP超时不变。
多请求仍走原串行执行逻辑；并发请求排队仍可能导致上游超时，本版本不改变请求调度。

## Ubuntu部署

先停止旧服务/自启动和独立测试器，关闭厂家相机软件；旧版本不要与v5同时占用相机/48051端口。
将整个GDY_vison_detect_v5复制到Ubuntu，进入该目录。保留现场的业务配置和标定文件，
本地副本不代表服务器上最后一次微调配置；部署前对比并把现场值迁入，勿覆盖丢失。

```bash
conda activate ur_odcam
export SDK_ROOT="/home/nvidia/workspace/vision/GDY_vision_detect_service/3DCameraSDK-v3.2.229.20251110"
export SDK_LIB_DIR="$SDK_ROOT/lib/3dcamera/linux/aarch64"
bash build_native_camera.sh
bash start_vision_service.sh
```

需匹配CPU的厂家完整SDK和g++（Ubuntu build-essential）。只需初次部署/修改桥接源码后编译；
生成.so和sdk_location.json是目标机专用，不要把Windows或其他机器的构建产物拿来用。
worker使用记录的SDK绝对路径；SDK移动后重新构建。现有linux_sdk_paths.env的JAKA/模型环境
设置照旧，不要求删除OpenNI库；SDK子进程去掉torch LD_PRELOAD并将原生库路径置前。
systemd使用v5时需由部署者把工作目录/启动路径指向v5，不能同时保留旧服务运行；本次不修改系统单元。

服务日志应出现“v5相机原生软触发连接成功”和“v5按请求软触发完成”。
相机子进程入口为 `handeye_calib/native_camera/worker.cpp` 编译出的 `gdy_camera_worker`
（v5.1.0 起；`worker.py` 只是旧测试夹具，正式服务不再启动它），不是另一个run_service.py。
Windows目前仅支持v5离线/模拟测试，原生实时桥接本版仅为Linux构建，不能直接用旧OpenNI启动脚本
在Windows连接v5相机。原Windows业务文档保留作历史参考。

## 故障隔离，不等于已修复SDK

实测SDK在release camera object处可能挂起。正常退出先请求SDK清理，8秒仍未完成则强制
终止SDK子进程并记录警告，主服务不无限等待。操作系统回收该进程内存；设备端是否已恢复
仍需现场确认，必要时断电重启。绝不宣称SDK析构死锁已经修复。

启动/连接最多45秒，单次触发+取帧最多25秒（外部进程看门狗）；SDK worker RSS+Swap超过
2GiB（包括空闲阶段）会被终止。正常请求一旦SDK超时/错误，不返回旧图、不自动重发机器人动作，
相机进入不可用状态，需要检查设备后重启服务。父服务异常退出时Linux父进程死亡信号杀死worker。
共享缓冲及数据复制会使整体内存高于独立测试，主服务内存不受这2GiB子进程限额保护。

## 离线验证与现场验收

```bash
python -m unittest discover -s tests -v
python -m unittest discover -s tool/soft_trigger_test -p test_offline.py -v
```

离线覆盖真实假数据子进程IPC/共享内存所有权、阻塞终止、错误隔离、颜色通道、深度尺度、
配准一致性和原业务HTTP/TCP回归。不会连接相机或机械臂，不会调用真实MinIO。
开发机仅完成离线验证；用户已反馈Ubuntu原生编译成功、服务启动及独立取流测试结果。
这些结果不等于完整服务长测通过，不能保证现场可直接无人值守。

现场先仅请求快照/只输出TCP，关闭测试脚本work_jaka，核对彩色尺寸、深度量程、点云边缘对齐、
同一机位旧/新TCP差异，确认后再授权运动。之后做实际服务连续请求、至少两段5分钟空闲、
观察父/子进程RSS及重复停止/启动；启动成功不等于已经验证首帧可用。
若使用旧快照ID，注意原业务缓存语义，避免将缓存复用误判为新相机未触发。
