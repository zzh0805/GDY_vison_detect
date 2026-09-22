# v5.1：连续RGB预览与触发拍照

## 部署和录制

本目录是v5的独立副本；原v5未修改。保留模型、标定、工具偏移和原接口。
实时SDK仍只在Linux运行。先停止旧v5服务和所有独立相机诊断程序，不能同时占用相机。
不要直接切换机器人生产流程；先在静止机位完成文末验收。

在目标设备进入本目录，使用原来的ur_odcam环境：

```bash
conda activate ur_odcam
bash build_native_camera.sh
bash run_service.sh
```

构建默认使用同级 `3DCameraSDK-v3.2.229.20251110`，其他位置通过 `SDK_ROOT` / `SDK_LIB_DIR` 指定。
构建产物是 `handeye_calib/native_camera/gdy_camera_worker`，不是旧 `.so`。
Python依赖沿用原项目；MP4需要OpenCV带可用的视频编码器（默认mp4v）。

另开终端录制（服务保持运行）：

```bash
python field_test/14_record_rgb_preview.py
python field_test/14_record_rgb_preview.py --seconds 60
python field_test/14_record_rgb_preview.py --base-url http://192.168.16.100:48051 --seconds 30
```

默认录制30秒到 `preview_recordings/rgb_日期时间.mp4`，并生成同名JSON统计。
也可用 `--output /path/to/new_file.mp4`，不允许覆盖已有视频。
脚本不读取机器人位姿、不移动机器人、不另开SDK连接、不调用触发拍照。
它调用 `/preview/start` 后订阅连续RGB，自己开启的预览在结束/Ctrl+C时自动停止。
如果预览之前已开启，脚本只录制，不替别人关闭；可调用stop或等待会话超时。

控制台区分：

- SDK采集fps：C++连续读取RGB帧计数/当前连续模式运行时间（拍照恢复后重新统计）。
- 接收fps：客户端收到的新JPEG数量/墙钟时间；编码、发布上限、网络会影响此值。
- MP4时间轴fps：配置值，不是相机能力。低采集率时重复上一张图补足视频时长，高采集率时采样，JSON记录重复帧数。

录像从第一张收到的图开始，不补画面出现之前的启动等待。超时、无图、断流会报错；
已写入的视频会尽量正常封口。长时间无帧时最多等待客户端10秒网络读取超时。

## 接口（原有接口不变）

使用原HTTP端口 `http.listen_port`，不额外打开第二个相机或另一个网络端口：

| 新接口 | 方法 | 用途 |
|---|---|---|
| /preview/start | POST | 开始/复用连续预览，返回session、started_new、stream_path |
| /preview/status | GET | active、采集/发布fps、帧龄、错误 |
| /preview/rgb.mjpg | GET | multipart JPEG连续图像流；必须先start |
| /preview/stop | POST | JSON `{}`，或 `{"session":"start返回的值"}` 防止误停其他会话 |

示例：

```bash
curl -X POST http://127.0.0.1:48051/preview/start
curl http://127.0.0.1:48051/preview/status
curl -X POST -H 'Content-Type: application/json' -d '{}' http://127.0.0.1:48051/preview/stop
```

后台可直接订阅 `http://视觉设备IP:48051/preview/rgb.mjpg`。这是MJPEG，不是RTSP或直接MP4下载。
MP4由测试客户端保存。沿用旧HTTP服务的网络信任模型，无新增身份认证，应限于受控内网。

`/snapshot`、`/capture_color`、`/get_tcp_pose`、`/motion/get_tcp_pose`的请求/响应不变。
`live=true`仍触发彩深拍照；默认识别仍使用快照缓存。连续RGB不会覆盖快照缓存，
也不会把预览帧与另一时刻的深度拼成解算帧。后台若只看预览后识别，仍需调用snapshot或live=true。
预览中的框选坐标必须对应当前分辨率；目标/机位变化后不能沿用旧框。

## 全部新增参数在config/workflow.yaml

“workfile”在本项目对应 `config/workflow.yaml`。修改后重启服务；未实现运行时热改相机参数。

- `camera.native`：取帧超时、SDK进程内存上限、关闭超时、固定曝光/增益、彩深匹配、时间戳策略；
  v5.1.1 新增 `rgb_read_timeout_ms` / `drain_timeout_ms` / `drain_max_frames` / `periodic_clear_s` /
  `memory_report_s`（队列与内存验收相关）。
- `preview`：发布上限、JPEG质量、并发客户端、会话时长、旧帧阈值、录制时长/编码器/时间轴帧率/输出目录。
- 监听地址和端口继续使用 `http.listen_host` / `http.listen_port`。

曝光/增益默认为null，保持设备原设置，避免默认值改变已调好的现场亮度。
要固定RGB，例如把 `rgb_exposure_us` 改为10000、`rgb_gain`改为1。
会关闭RGB自动曝光，查询范围并设置/读回，任何不支持/越界/读回不一致都会报错，不静默忽略。
只填gain也会关闭对应自动曝光，但曝光时间保持设备当前值；建议固定时成对配置。
深度使用 `depth_exposure` / `depth_frame_time` / `depth_gain`，单位遵从深度标准属性和设备范围，曝光必须小于帧时间。
不会写入SDK的永久存储；正常关闭时尽力恢复修改前的参数。SDK卡死被kill时无法保证设备参数已恢复。

`match_enabled: false`时不对配匹参数做写入。v5.1.1 的拍照已改用单流 `getFrame`，配对接口不再参与，
因此该开关只用于实机对比（true 时仍会显式设置并读回 `PROPERTY_EXT_DEPTH_RGB_MATCH_PARAM`）。
`timestamp_policy: warn`是兼容默认：时间戳为0、倒退或原始时差过大时告警，但不追加重试、不改用旧缓存。
`strict`才拒绝不满足时间戳条件的拍照。原始时间差检查不应用SDK offset，避免把补偿后的值误认成物理同步。
时间戳符合阈值也不是同一时刻曝光证明；输出元数据 `synchronization_proven`仍为false。
开启匹配可能增加超时，必须先实测本型号和固件，不能在未验证时把阈值调得极小。

`preview_with_depth: false`时推流只 resume RGB，深度保持挂起（帧率/内存/SDK负担最小）。
若本型号需要同时启动深度才出RGB，设 true 测试：同时开连续深度并在 pump 中循环排空，不重建点云。
不要用重新启动第二个进程来解决无图。

## C++状态与内存所有权

v5.1.1：open 时只启流一次（两路分开 startStream）；**待机 = 双流挂起 + 软触发模式**，相机不产帧，
SDK队列不可能增长；拍照 = clear两路 → 排空到超时 → softTrigger(1) → getFrame(深度)+getFrame(彩色)，
不切模式、不重建流；推流 = resumeStream(RGB)（必要时加深度）并持续消费，结束推流 = pauseStream + clear。
模式切换只做 pause/resume 与 TRIGGER_MODE，**不再 stopStream/startStream**，也不再使用 getPairedFrame。
所有SDK调用在C++单线程内串行执行。Python只发控制命令、复制共享内存、编码JPEG、运行业务算法。

- SDK帧使用局部 `IFramePtr`，正常/异常路径均RAII释放；不向Python传递SDK指针。
- 共享内存固定32MiB，每路上限16MiB；格式/长宽/字节数校验后才能复制。
- C++仅保留一张最新RGB。共享内存只在RPC响应前写，后台采集不写共享区。
- **待机不产帧**：双流挂起后 `require_idle` 用连续超时证明静止，启动不通过直接报错。
- **触发前队列必空**：`clear` + 排空到超时；达到 `drain_max_frames` 说明流没停，报错而不是给旧帧。
- **零重建保证**：日志与响应里的 `stream_restarts` 恒为0；离线测试断言 100 次拍照后 `startStream` 仍是2次。
- **原始参数跨进程**：首次读到的曝光/增益/触发模式写入 `native_property_backup.txt`，worker 被kill
  后重建时用它作为恢复目标，不会把改过的值当成原始值。
- **崩溃自愈**：worker 退出后按 2/5/15s 退避重建子进程，新进程重走完整 queryCameras/connect 流程；
  同进程内不尝试复活 SDK（文档没有复位接口，且 camera.reset() 已知会挂）。
- Python在同一RPC锁下复制数据，避免预览与capture互相覆盖。
- JPEG只缓存最新一张；限客户端数和套接字超时，慢客户端不产生无限队列；无订阅者时不做1080p编码。
- 相机/系统对象、映射和文件描述符有退出清理；保留父进程死亡保护、RSS+Swap上限、RPC超时kill+wait。
- v5.1同用户进程使用排他锁防止双开；旧v5/厂商GUI不认识此锁，必须手动停掉。

这些措施约束本项目分配及异常隔离，不能证明厂商SDK内部绝无泄漏。长期RSS实测仍是验收必需项。

## 验收顺序

1. 运行 `python -m unittest discover -s tests -v`，确认离线接口/数学/录制测试。
2. 在目标Ubuntu用实际SDK编译；当前CHM内部版本3.2.206，部署SDK可能为3.2.229，不能混用头文件/动态库。
3. 停其他相机程序、机器人保持静止，先运行原test，确认触发结果与v5一致。
4. 录制30秒，检查实际采集fps、接收fps、MP4、Ctrl+C停止、再次开启。
5. 预览时调用原snapshot/live=true，验证暂停→拍照→恢复，以及彩深尺寸/时间差日志。
6. 配置固定曝光/增益，检查读回日志、亮度和有效深度；随后再开启匹配参数测试。
7. 增大max_session_s，连续预览/反复切换至少30分钟，监测 `gdy_camera_worker` RSS/Swap趋势和文件描述符数量。
   排除启动缓存后仍持续增长、SDK异常或时间戳不可信时，不进入机器人生产运行。

可另外编译运行不连接相机的C++状态/帧释放测试（不可把假SDK头文件用于正式构建）：

```bash
g++ -std=c++14 -Itests/native_sdk_fake tests/native_state_test.cpp -o /tmp/gdy_state_test
/tmp/gdy_state_test
```

本次离线验证为116项Python测试通过；C++假SDK完成100次模式切换/拍照及1000张预览帧，
局部帧对象操作后均释放。生产代码通过CHM头文件下ARM64/x86_64目标文件编译；
真实SDK链接、帧率、内存长期稳定性仍待现场验收，详见 `RELEASE_NOTES_v5.1.0.md`。

## 文件变更

新增：C++ `worker.cpp`、独立生命周期 `sdk_lifecycle.hpp`、参数校验 `native_settings.py`、
`vision_solver/preview.py`、`field_test/14_record_rgb_preview.py`、预览测试和本文档。
修改：构建脚本、`native_surfacepro50.py`、同步适配器参数透传、`vision_solver/camera.py`、
`http_server.py`、`config.py`、`workflow.yaml`、VERSION/README/gitignore及IPC测试夹具。
`native_camera/worker.py`只保留旧测试辅助，正式v5.1不再启动它。
配准/点云算法、YOLO、TCP数学、原test机械臂动作、模型和标定文件未改。
