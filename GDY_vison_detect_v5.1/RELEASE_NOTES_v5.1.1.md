# v5.1.1 更新说明

日期：2026-09-22。基于本地 `GDY_vison_detect_v5.1`（5.1.0）修改；v5 目录未改动。

## 目标

现场形态是"大部分时间软触发拍照 + 小部分时间推 RGB 流"，硬要求是**内存有界、不重启服务**。
5.1.0 的实现每次进出推流都会 `stopStream()` + `startStream()` 重建两路流，且待机时 RGB 仍在
自由出帧。本版把相机改成一个**长期不变的状态机**：

```
open（进程生命周期内只做一次）
   setSdkEnableNetworking/setEnableNetworking -> getSystemPtr -> queryCameras(5000)
   -> getCameraPtr -> connect(info) -> getStreamInfos
   -> startStream(DEPTH) + startStream(RGB)        ← 官方 SampleDepthRGBFrameMatch 的写法，一路一次
   -> save_settings() + settings()                 ← 流正在跑，符合指南1.7的写入时机
   -> pauseStream(DEPTH) + pauseStream(RGB)        ← 双流挂起
   -> setPropertyExtension(TRIGGER_MODE, SOFTWAER) ← SampleSoftTrigger 的顺序
   -> clear 两路队列 + 连续超时校验（证明真的静止）

【待机·大部分时间】双流挂起 + 软触发模式：相机不产帧、不占带宽、队列不可能增长
   拍照 = clear(depth) + clear(rgb) + 排空到超时 + softTrigger(1)
        + getFrame(depth) + getFrame(rgb)
   ※ 不做任何模式切换、不 stopStream/startStream、不重连

【推流·小部分时间】resumeStream(RGB) [+ TRIGGER_MODE=OFF]，pump 持续消费 RGB
   结束推流 = pauseStream(RGB) + TRIGGER_MODE=SOFTWAER + clear + 排空校验
```

关键点：**待机静止 ≠ 不能拍照**。`pauseStream` 只是停止"自由出帧"，触发通道照常工作
（`SampleSoftTrigger.cpp` 演示的就是"暂停流 + 软触发 + getFrame"）。因此每张照片依然来自
`softTrigger` 当场曝光的一帧，而不是队列里的历史帧。

## 相比 5.1.0 修掉的问题

| # | 5.1.0 的问题 | 本版处理 |
|---|---|---|
| 1 | 每次进出推流都重建两路流（`stopStream`+`startStream`），秒级、失败面大 | 改为 `pauseStream`/`resumeStream`，毫秒级；`stream_starts` 全程恒为 2 |
| 2 | 待机时只暂停深度，**RGB 自由跑且无人消费** → SDK 队列可能持续增长 | 待机双流都暂停；推流结束额外 `clear`+排空校验 |
| 3 | `mode(false)` 在"两条流都已 stop"后调用，官方示例中该属性都在流运行中设置 | 全程有流处于已启流状态；`pauseStream → TRIGGER_MODE` 与示例一致 |
| 4 | 帧时间/曝光/增益在 `pauseStream` 之后写入 | 只在 `open()` 里、`startStream` 之后、`pauseStream` 之前写入一次 |
| 5 | 拍照用 `getPairedFrame`，无匹配时只能靠超时兜底返回 | 改为单流 `getFrame`×2（指南1.6"主动取帧方式"），不再经过配对匹配器 |
| 6 | 配对启流用 `startStream(dep, rgb, nullptr, nullptr)`（回调重载传空指针，文档未覆盖） | 改为两路分别 `startStream(STREAM_TYPE, ...)` |
| 7 | `std::string(info.uniqueId)` 对定长 `char[32]` 无界读取（可能越界） | 改为按数组长度截断的 `bounded_field()`；已用突变测试证明原写法会崩溃 |
| 8 | 清帧后直接触发，"clear 是否立即生效"只是假设 | 增加 `clear → 排空到超时 → 触发`，把"队列为空"变成结构保证 |
| 9 | worker 被 kill 后重启会把自己改过的参数当成"原始值" | 原始参数持久化到 `native_property_backup.txt`，跨进程恢复 |
| 10 | SDK 进程崩溃后必须重启整个服务 | 检测到 worker 退出后按 2/5/15s 退避重建子进程并重新走完整连接流程 |
| 11 | 预览没有客户端看时仍在做 1080p JPEG 编码 | 无订阅者时跳过编码（取帧保留，故障仍能在 `/preview/status` 看到） |
| 12 | 拍照耗时无法归因 | 每次拍照打印并回传 clear/drain/trigger/depth/rgb 分阶段耗时 |

`match_enabled` 与 `PROPERTY_EXT_DEPTH_RGB_MATCH_PARAM` 保留为实机对比开关，但拍照路径已不再
使用配对接口，因此它不再是必需品。

## 新增配置（`config/workflow.yaml` 的 `camera.native`）

| 键 | 默认 | 说明 |
|---|---|---|
| `rgb_read_timeout_ms` | 500 | 拍照时彩色单流取帧超时（深度用 `capture_timeout_ms`） |
| `drain_timeout_ms` | 100 | 排空单次读超时；流已暂停时只需付一次 |
| `drain_max_frames` | 10 | 排空帧数上限；达到上限说明流仍在出帧，直接报错 |
| `periodic_clear_s` | 60 | 推流期间的周期清队列兜底（0=关闭） |
| `memory_report_s` | 300 | worker 定期打印 RSS/Swap，便于长时间待机观察内存趋势（0=关闭） |

`Settings::parse()`（C++）与 `wire_settings()`（Python）的字段顺序由两侧各一份黄金字符串锁定：
`3000 100 3 60 -1 -1 -1 -1 -1 0 60 0 0 0 0 500 100 10 60 300`。

## 改动文件

| 文件 | 说明 |
|---|---|
| `handeye_calib/native_camera/worker.cpp` | 新状态机；拍照/推流/待机；分阶段耗时；内存日志；参数备份 |
| `handeye_calib/native_camera/bridge.cpp` | 新增 `v5_read_frame()`（单流）与 `v5_drain_stream()`；保留旧接口与 `v5_read_pair()` 兼容 |
| `handeye_calib/native_camera/sdk_lifecycle.hpp` | 拆出 `st_connect()` / `start_streams()`；修复定长数组越界；错误信息带 SDK 错误串 |
| `handeye_calib/native_surfacepro50.py` | 自动重建 worker（退避）、参数备份路径、拍照分阶段元数据 |
| `handeye_calib/native_settings.py` | 5 个新字段与边界校验 |
| `vision_solver/preview.py` | 无订阅者时不编码 JPEG |
| `vision_solver/http_server.py` | MJPEG 订阅登记/注销 |
| `tests/native_sdk_fake/3DCamera.hpp` | 假 SDK 改为**真 union** ABI、真实枚举值、FIFO 队列、暂停/触发语义、多种故障注入 |
| `tests/native_state_test.cpp` | 重写：待机静止、100 次零重建拍照、推流暂停/恢复、脏队列、故障、RAII、参数备份、协议黄金串 |

## 离线验证结果

```bash
# C++（无相机、无SDK、无网络；Windows 上用 zig/clang 也可跑）
g++ -std=c++14 -Itests/native_sdk_fake tests/native_state_test.cpp -o /tmp/gdy_state_test && /tmp/gdy_state_test
# Python
python -m unittest discover -s tests
```

- C++ 状态测试：`PASS`，`-Wall -Wextra` 下本项目代码零告警。覆盖：
  待机双流挂起且不产帧；100 次拍照零流重建、零触发前脏队列、帧对象峰值 ≤2 且全部释放；
  推流进入/退出只用 pause/resume 且幂等；推流中拍照可安全挂起/恢复；
  注入积压后仍从空队列触发；clear 失败/格式错误/超时/时间戳不可用/挂起失败全部 fail-closed；
  32 字节无终止符的 `uniqueId` 能正确匹配；`close()` 后相机/系统对象释放且 `stopStream` 恰好 1 次。
- 越界读修复用**突变测试**验证：把 `bounded_field()` 换回 `std::string(uniqueId)` 后测试进程
  直接以 `0xC0000409`（缓冲区溢出）终止，说明该缺陷真实存在且测试能捕获。
- Python：**120 项通过**（原 116 + 新增 4：协议黄金串、worker 命令行含备份路径、崩溃后重建、
  退避窗口内不重复重试）。

## 必须实机确认的假设（每一条都有退路，都不推翻架构）

| # | 假设 | 判定方法 | 若不成立的退路 |
|---|---|---|---|
| A1 | 挂起的流经 `resumeStream` 能正常出帧 | 推流态读 100 帧看帧率与内容变化 | `preview_with_depth: true`；或退回到"按需 `clear`+排空"的旧形态 |
| A2 | `softTrigger(1)` 让**两路挂起的流各出一帧** | 连续拍照 20 次，深度与彩色都应返回 | 若彩色不响应：拍照前 `resumeStream(RGB)` 取一帧再挂起（仍是零重建） |
| A3 | 切换 `TRIGGER_MODE` 不会把挂起的流唤醒 | 待机 5 分钟不取帧，再 `getFrame` 应超时 | 每次切换后补一次 `pauseStream`；`drain_max_frames` 会先报错而不是给旧帧 |
| A4 | 反复 `pauseStream`/`resumeStream` 长期稳定 | 30 分钟反复切换，看 RSS/fd 曲线 | 用 `PROPERTY_EXT_PAUSE_DEPTH_STREAM/RESUME_DEPTH_STREAM`；或改为整段只推流/只触发 |

## 真实 SDK 3.2.229 核对（2026-09-22）

把 `3DCameraSDK-v3.2.229.20251110` 放回工程后，用**真实头文件**编译生产代码：

```bash
# 两种目标架构都做编译检查（本地用 zig/clang 交叉编译验证；目标机用 build_native_camera.sh）
zig c++ --target=aarch64-linux-gnu -std=c++14 -Wall -Wextra -c handeye_calib/native_camera/worker.cpp \
  -I 3DCameraSDK-v3.2.229.20251110/inc/3dcamera -o worker.o
zig c++ --target=x86_64-linux-gnu  ... 同上
```

结果：**两种架构均编译通过，本项目代码零错误零告警**（21 个告警全部来自厂家头文件的
`Frame.hpp`/`Processing.hpp`，与我们的代码无关）。

逐项核对真实头文件，与本版代码的假设一致：

| 检查点 | 真实 3.2.229 |
|---|---|
| `PropertyExtension` | `typedef union PropertyExtension`（确认是联合体 → 离线 fake 已同步改成 union） |
| `TRIGGER_MODE_OFF / SOFTWAER` | `0 / 2` |
| `STREAM_FORMAT_RGB8 / Z16` | `0x01 / 0x02` |
| `PROPERTY_GAIN/EXPOSURE/FRAMETIME/ENABLE_AUTO_EXPOSURE` | `0x00 / 0x01 / 0x02 / 0x05` |
| `PROPERTY_EXT_CLEAR_FRAME_BUFFER` | `0x13`，字段确实是 `streamType` |
| `AUTO_EXPOSURE_MODE_CLOSE` | `0` |
| `getCameraErrorString(ERROR_CODE)` | 存在（本版新调用，符号已确认） |
| `pauseStream/resumeStream/stopStream/softTrigger/getFrame/startStream` | 签名与本版调用完全一致 |

两个 `.so`（aarch64 / x64）都导出了我们调用的**全部**入口（含新增的 `getCameraErrorString`），
因此目标机链接不会出现未定义符号。

### 顺手解答了“RGB 曝光单位”这个悬案

真实 `hpp/Types.hpp` 里 `uiExposureTime` 的**同一行**：中文写“曝光时间,单位毫秒”，英文写
`exposure time,unit us` —— 厂家文档自相矛盾实锤。因此本版**不猜单位**：只把配置值与设备
上报的 `objVRange_` 范围比对，越界/非整数直接报错，错误信息里明确提示单位需实测确认。
首次配置固定曝光时，先看日志中打印的范围读回再决定数值。

真实头文件另外给出的可用范围（供现场配置参考）：

- 深度 `PROPERTY_EXPOSURE`：**3000~60000 微秒**，必须小于帧时间；
- `PROPERTY_GAIN`：**1~16**，高精度场合建议 ≤3；
- 深度单帧耗时≈帧时间×18（FAQ 2.4），因此帧时间直接决定拍照延迟与帧率。

### SDK 目录位置

`build_native_camera.sh` 默认在**项目的上一级目录**找同名 SDK（与你 Ubuntu 上的部署布局一致）。
本版已兼容“SDK 放在项目内部”的情况；若两者都没有，按提示 `export SDK_ROOT=...` 与
`SDK_LIB_DIR=.../lib/3dcamera/linux/aarch64`。

## 现场验收

1. 停掉旧服务、旧诊断与厂家查看器（不能有两个 SDK 占用者）。
2. `bash build_native_camera.sh && bash run_service.sh`，确认日志出现
   `Native camera ready: depth=... rgb=... standby(suspended)=true`。
3. **待机静置 10 分钟**，每 30s 记录一次 `gdy_camera_worker` 的 VmRSS+VmSwap 与 fd 数量，
   确认无单调上升（对照第 5 节第 2 条那个"待机 RGB 空转"的历史隐患）。
4. 连续打 20 次 `/snapshot`，看每次日志的
   `CAPTURE depth=..ms rgb=..ms trigger=..ms clear=..ms drain=..ms total=..ms` 与
   `native_stream_restarts=0`；这是回答"拍照上限"的直接数据。
5. `POST /preview/start` → 录制 30s → `POST /preview/stop`，确认结束时回到待机；
   再打一次 `/snapshot` 应仍然成功（验证一次性的模式切换）。
6. 反复"开始推流→结束推流→拍照"至少 30 分钟，观察 RSS/Swap/fd 与
   `/preview/status` 的 `error` 字段。
7. 配置固定曝光/增益 + `match_enabled: true` 各跑一轮 20 次拍照，确认读回日志与画面亮度。

## 回退

改动前的文件全部保留在 `tools_tmp\rollback_v5.1.0\`：

- `native_camera\{worker.cpp, bridge.cpp, sdk_lifecycle.hpp, 3DCamera.hpp}` —— 用这 4 个覆盖
  `handeye_calib/native_camera/` 下的同名文件（`3DCamera.hpp` 放回 `tests/native_sdk_fake/`），
  然后 `bash build_native_camera.sh` 重编译即可回到 5.1.0 的 C++ 行为；
- `tests\{native_state_test.cpp, native_sdk_fake\3DCamera.hpp}` —— 对应的旧离线测试。

Python 侧的回退：`native_settings.py` / `native_surfacepro50.py` / `preview.py` / `http_server.py`
只需把本版新增的字段与 `ensure_connected()` 去掉；`config/workflow.yaml` 中 5 个新增键删掉后，
`native_settings` 会再次以旧字段集校验（未知键会报错，所以两边要一起改）。
