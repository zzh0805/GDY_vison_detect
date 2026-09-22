# Python原生SDK彩色＋深度20分钟测试

独立诊断工具：不启动视觉服务，不连接机械臂，不修改workflow、标定或生产相机实现。
默认 `STREAMS=rgbd MODE=soft DURATION=1200`。旧深度单流仍可用 `STREAMS=depth` 运行。

## 空闲与采集交替测试（新入口）

更新：run_idle_test.sh现在默认先成功预采集5组，失败立即停止，不进入空闲。
预采集逐组记录在warmup.csv，memory.csv阶段名为warmup；预采集不计入后续DURATION。
验证“先正常出帧，再空闲30秒，再恢复”的命令：

```bash
WARMUP_FRAMES=5 IDLE_BEFORE=30 DURATION=60 IDLE_AFTER=0 bash tool/soft_trigger_test/run_idle_test.sh
```

正常顺序为WARMUP 5/5 -> PHASE idle_before -> PHASE capture -> RESUME SUCCESS。
首次恢复超时会明确报“预采集已成功，但空闲后首次触发/取帧超时”，不自动重试掩盖失败。
WARMUP_FRAMES=3可改为3组；设0可复现旧版直接空闲流程。

沿用SDK_ROOT、SDK_LIB_DIR设置，停止其他占用相机的程序后，在项目根目录执行：

```bash
bash tool/soft_trigger_test/run_idle_test.sh
```

同一次连接，先初始化、有限旧帧排空并成功预采集5组，然后：
**空闲300秒 -> 持续软触发采集600秒 -> 空闲300秒 -> 关闭**。
这里“持续采集”指不断发出softTrigger，仍然mode=2，不是切换为连续曝光mode=0。
初始化和关闭另计，因此总时间略大于20分钟。
空闲阶段没有触发、没有getFrame/getPairedFrame，也没有后台丢帧线程；只有父进程采样内存。
相机连接/流保持打开；彩色是否仍在SDK内部生成或缓存正是需要验证的内容。
空闲不是关闭相机，也不承诺网络完全无数据。

可修改run_idle_test.sh中的参数或临时覆盖：

```bash
# 快速冒烟：空闲30秒 -> 采集60秒 -> 空闲30秒
IDLE_BEFORE=30 DURATION=60 IDLE_AFTER=30 bash tool/soft_trigger_test/run_idle_test.sh

# 更长空闲：空闲10分钟 -> 采集10分钟 -> 空闲10分钟
IDLE_BEFORE=600 DURATION=600 IDLE_AFTER=600 bash tool/soft_trigger_test/run_idle_test.sh
```

memory.csv新增phase列：idle_before / capture / idle_after / closing等；
summary.json中的phase_memory给出各段RSS首末值、峰值和变化量（KiB）。
阶段首末采样与边界可能相差约1秒，需要结合完整曲线，不能仅凭首末差认定泄漏。
空闲有独立的预定等待预算，不会被15秒取帧看门狗误杀；内存上限始终有效。
恢复采集前不额外清缓存，以暴露空闲积压；如果恢复后多出帧等校验失败，会保留失败并停止。
末尾空闲后直接关闭，不再额外采集。采集后的rgb内容变化间隔不包含第一次空闲。
关闭对象阻塞的问题尚未修复，最终可能仍失败，已落盘的空闲/采集内存数据仍可分析。

## 彩色内容变化测试更新

彩色的非零时间戳重复或回退现在仅WARNING并计数，不中止；深度原有校验不变。
桥接库在帧释放前遍历完整RGB8数据，计算64位FNV-1a内容指纹，不保留历史图像。
因此必须同时更新bridge.cpp和Python，启动时自动重新编译。
frames.csv新增rgb_hash、rgb_content_changed、rgb_change_interval_s和rgb_timestamp_warning。
每次接受配对帧后更新rgb_changes.json，记录变化次数、平均/最大变化间隔、
最长观察到的不变跨度及末尾不变跨度；正常结束也写入capture_summary.json。

时间来自主机收到配对帧后的单调时钟，不使用不可靠的SDK时间戳。
第一帧是基线，不计为变化；第一个变化间隔从基线开始。
间隔包含配对等待和人为设置的INTERVAL，不能代表相机真实帧率。
最长不变跨度按离散观察计算（包括上次变化至下次观察到变化的间隔），
不是精确的传感器停更时长。指纹极小概率碰撞；噪声和自动曝光同样算内容变化，
本功能不是语义变化检测，更不能证明两路同时曝光。
若要验证现场等待时间，请保持相机/机械臂静止，手动放入或移走明显卡片并结合画面验证。
当前脚本只记录内容指纹，不保存可查看的图片，不能自行测得卡片动作到画面更新的精确延迟。

取帧超时、无效数据、额外配对帧、内存上限和关闭看门狗仍然保留。
即使采集完成，关闭对象阻塞仍会判整体失败；但内容统计文件已经保留可供分析。

## 阻塞定位版：先验证可行性，再测20分钟

异常保留补丁：采集校验异常现在会在关闭前立即打印堆栈，写入
`worker_error.json`，包含尝试序号、已接受帧数、前次时间戳、当次深度和彩色元数据。
即使关闭失败，也不会覆盖这份原始异常。关闭另有异常则记录`cleanup_error.json`。
`cleanup.log`逐项记录恢复触发模式、停止双流、断开设备、释放camera/system对象的
BEGIN/END/ERROR。只有BEGIN没有END的步骤就是待排查阻塞点。
关闭仍使用整体15秒预算，不调整关闭顺序；彩色时间戳按上面的新策略仅告警。
更新必须同时覆盖bridge.cpp和Python脚本（建议整个目录），启动脚本会重新编译。

本版增加独立父进程单次调用看门狗。每次SDK读取、触发、打开和关闭都有
`CALL 编号 BEGIN/END/ERROR`日志，附耗时；read的rc=0为有效返回，rc=1为SDK超时。
`calls.jsonl`保存调用历史，`call_state.json`原子更新最新状态。
例如只有`BEGIN softTrigger(1)`没有END，卡点就是触发调用；如果触发END后只有
`BEGIN getPairedFrame(...)`，卡点就是配对读取。

默认触发/读取/关闭超过15秒，由父进程判失败并发出中断，10秒仍未退出则强制杀死
独立测试子进程。因此通常约25～26秒内停止阻塞测试，不再空等20分钟。
SDK加载/打开允许90秒。超时是中止整个测试，不是取消C++调用后继续复用相机。
若强制终止，请确认相机重新连接、必要时断电重启，不能自动紧接下一项测试。

按顺序逐项执行，每项成功退出并确认相机已关闭后再进行下一项：

```bash
# A: 连续彩深配对，先验证配对接口基础能力
MODE=continuous STREAMS=rgbd INTERVAL=0 DURATION=60 bash tool/soft_trigger_test/run_test.sh

# B: 同一规格的软触发彩深配对，定位触发/配对组合是否可行
MODE=soft STREAMS=rgbd INTERVAL=1 DURATION=60 bash tool/soft_trigger_test/run_test.sh

# C: B通过后才运行20分钟
MODE=soft STREAMS=rgbd INTERVAL=1 DURATION=1200 bash tool/soft_trigger_test/run_test.sh
```

- A通过、B失败：优先排查软触发与配对调用组合，不代表硬件完全不支持。
- A/B均失败：先排查双流规格、配对接口、SDK/设备状态。
- A/B通过：再看C的帧数、时间戳与内存曲线，不能只看RSS。

`summary.json`中的`acquisition_test_passed`只有进程正常退出、有有效帧、完成指定
采集时长且没有监控中止时才为true。它不自动判定内存泄漏已解决或曝光严格同步。
失败时`last_call_at_stop`保留监控停止时的调用现场，`forced_kill`标记是否强制终止。
将这两个JSON、calls.jsonl、frames.csv和memory.csv一起保留便于分析。

## Ubuntu运行

将本目录整体覆盖到Ubuntu项目的 `tool/soft_trigger_test/`，不是只更新Python文件。
先停止视觉服务（包括systemd自动拉起）、关闭厂家查看器，避免抢占相机。

在项目根目录运行，下面路径对应此次实测部署：

```bash
conda activate ur_odcam
export SDK_ROOT="/home/nvidia/workspace/vision/GDY_vision_detect_service/3DCameraSDK-v3.2.229.20251110"
export SDK_LIB_DIR="$SDK_ROOT/lib/3dcamera/linux/aarch64"
bash tool/soft_trigger_test/run_test.sh
```

脚本编译C++桥接库后，由Python通过ctypes调用。需要g++（build-essential）及同一版本
的完整厂家SDK头文件和lib3DCamera.so，不能用libOpenNI2.so替代。
运行1200秒采集，初始化、排空启动帧和关闭相机时间另计。
默认RSS+Swap超过2048MiB中止；不要在系统已接近内存耗尽时开始测试。

## 联合采集方式与限制

1. 列出Z16深度规格、RGB8彩色规格，分别按PROFILE、RGB_PROFILE选择，默认均为0。
2. 用SDK双流startStream启动，暂停深度并设置/读回软件触发模式。
3. 使用getPairedFrame取得SDK返回的深度和彩色帧，不用两次独立getFrame拼接。
4. 有限排空启动帧后，检查不触发时无配对帧；softTrigger(1)后要求一组有效彩深帧。
5. 校验两路格式、宽高及数据字节数。每次读取完成释放两路SDK帧，不累积图片。
6. 每次采集后默认观察1秒，收到额外配对帧则判失败。
7. 记录两路时间戳。深度非零时间戳不递增判失败，彩色仅告警；为0则无法验证同步。

**不触发时无配对帧，不等于彩色传感器完全停流。** 厂家软件触发定义针对深度；
彩色可能仍连续输出。该测试会观察双流同时开启时的内存，不能宣称两路均受软触发控制。
若配对接口不支持此模式、超时或RGB8不可用，会明确失败，不会静默降级为单深度或拼接旧彩色帧。

getPairedFrame是SDK的配对接口，但不等于独立验证同时曝光。
若深度时间戳为0，就不能计算可靠的彩深时间差；即便都有时间戳，
还需要核实时间基准并用运动场景测试同步。
本测试不做点云对齐、YOLO，也不保存或复制完整像素数组到Python；它验证双流采集/释放路径，
并非完整生产工作流验收。

## 参数与对照

```bash
# 默认20分钟：深度软触发＋SDK配对彩色
bash tool/soft_trigger_test/run_test.sh

# 可选：先60秒确认配对接口支持
DURATION=60 bash tool/soft_trigger_test/run_test.sh

# 如果启动输出显示RGB8 profile 1才是希望的分辨率，按实际输出选择
RGB_PROFILE=1 bash tool/soft_trigger_test/run_test.sh

# 连续双流，持续消费，对照20分钟
MODE=continuous INTERVAL=0 DURATION=1200 bash tool/soft_trigger_test/run_test.sh

# 回到之前的深度单流测试
STREAMS=depth DURATION=1200 bash tool/soft_trigger_test/run_test.sh
```

参数均可在run_test.sh中直接修改。PROFILE是Z16列表索引，RGB_PROFILE是RGB8列表索引，
不是所有格式的总索引。不要假定索引0是1920×1080，必须看启动打印。
对照测试保持同一SDK、规格和环境，逐个运行，不能同时占用相机。
新脚本默认双流，因此复现旧深度实验时务必加STREAMS=depth。

## 输出

每次在output/日期时间/生成：

- settings.json：运行参数。
- frames.csv：每组深度和彩色的尺寸、字节数、时间戳、配对读取耗时及时间戳差。
- memory.csv：每秒当前VmRSS、RssAnon、RssFile、VmSwap、峰值VmHWM和线程数，内存单位KiB。
- capture_summary.json：正常完成采集后生成，含帧组数和时间戳缺失组数。
- summary.json：进程退出码、保护中止原因及内存摘要。

退出码0、完成采集阶段、两路有效帧、预热后内存无持续增长，才能说明本次双流测试表现稳定。
VmHWM只是历史峰值，不能用它判断是否释放。
20分钟成功仍不能保证长期无泄漏，也不能证明彩深同步或生产算法正确。

正常退出恢复原触发模式、停止双流并断开。中断或超限会先请求退出；
SDK阻塞超过10秒会强制终止，此时可能无法恢复设备模式，需要重新连接检查，必要时相机断电重启。

## 离线测试

```bash
python -m unittest discover -s tool/soft_trigger_test -p test_offline.py -v
```

离线测试不调用相机，只覆盖Python控制/记录/失败处理和命名空间回归检查。
不能替代Ubuntu原生编译及硬件配对取流测试。
