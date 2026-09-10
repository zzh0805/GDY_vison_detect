# Windows部署与使用

本项目可在Windows上运行视觉服务和现场测试，但SurfacePro50/OpenNI与JAKA部分必须使用厂家提供的Windows 64位SDK，并与Python位数匹配。正式视觉服务只返回TCP，不会主动控制机械臂；`field_test`脚本是否运动机械臂由 `field_test/test_case.yaml` 决定。

## 1. 准备Python环境

先获取代码，再准备Python 3.10或3.11的64位环境：

```powershell
git clone git@github.com:zzh0805/GDY_vison_detect.git GDY_vison_detect_v3
Set-Location .\GDY_vison_detect_v3
py -3.10 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

如果PowerShell禁止激活脚本，可以只对当前用户调整执行策略，或直接使用 `.venv\Scripts\python.exe` 执行后续命令。GPU版PyTorch应按现场CUDA版本安装；普通功能验证可先使用依赖默认安装的版本。

## 2. 配置厂家SDK

`pip`依赖不包含厂家OpenNI、SurfacePro50或JAKA SDK。根据实际安装目录，在启动服务的同一个PowerShell窗口设置：

```powershell
$env:SURFACEPRO50_PYTHON_PATH = "D:\SDK\SurfacePro50\python"
$env:SURFACEPRO50_OPENNI2_REDIST = "D:\SDK\SurfacePro50\OpenNI2\Redist"
$env:PATH = "D:\SDK\SurfacePro50\OpenNI2\Redist;$env:PATH"
```

`SURFACEPRO50_PYTHON_PATH`指向包含 `openni` Python包的目录；`SURFACEPRO50_OPENNI2_REDIST`指向直接包含 `OpenNI2.dll` 的目录。也可在 `config/workflow.yaml` 的 `camera.python_path` 和 `camera.openni_redist` 中填写绝对路径。

只有现场脚本需要读取或控制JAKA时，才需要设置：

```powershell
$env:JAKA_SDK_PATH = "D:\SDK\JAKA\Windows\python3\x64"
$env:JAKA_LIBRARY_PATH = "D:\SDK\JAKA\Windows\lib\x64"
$env:PATH = "$env:JAKA_LIBRARY_PATH;$env:PATH"
```

也可以将JAKA Python SDK路径填写到 `field_test/test_case.yaml` 的 `jaka_test.sdk_path`。

## 3. 检查现场配置

启动前检查：

1. `config/workflow.yaml` 中的相机IP、HTTP端口、相机标定、手眼标定和模型路径；
2. `config/tool_offsets.yaml` 中的目标选择模式、工件 `code` 映射和工具偏移；
3. `target_matching.source_image_width/height` 是否与后台矩形坐标使用的原图尺寸一致；
4. 相机、机器人和Windows电脑位于可互通的网段；
5. Windows防火墙允许TCP端口 `48051`。

路径建议使用正斜杠或带引号的Windows路径，例如：

```yaml
camera:
  python_path: "D:/SDK/SurfacePro50/python"
  openni_redist: "D:/SDK/SurfacePro50/OpenNI2/Redist"
```

## 4. 启动与停止服务

首次建议前台启动，以便直接观察相机和模型初始化日志：

```powershell
.\.venv\Scripts\python.exe run_service.py
```

按 `Ctrl+C` 会执行服务的正常关闭流程并断开相机。确认正常后可以后台启动：

```powershell
powershell -ExecutionPolicy Bypass -File .\start_vision_service.ps1 `
  -Python ".\.venv\Scripts\python.exe" -Port 48051
```

查看日志：

```powershell
Get-Content .\service.log -Wait
Get-Content .\service.log.err -Wait
```

停止后台服务：

```powershell
powershell -ExecutionPolicy Bypass -File .\stop_vision_service.ps1 -Port 48051
```

Windows后台停止脚本使用强制结束进程，厂家驱动可能来不及释放相机。标定和排障阶段优先使用前台运行及 `Ctrl+C`；如果下次连接失败，应确认没有残留Python/Viewer进程，必要时给相机断电约10秒后重新上电。

## 5. 调用接口

拍照：

```powershell
Invoke-RestMethod -Method Post -Uri "http://127.0.0.1:48051/snapshot"
```

TCP解算的字段、坐标单位和 `live` 模式见 [INTERFACE.md](INTERFACE.md)。HTTP边界输入输出为 `mm + JAKA RPY弧度`，配置和内部标定使用 `mm + JAKA RPY度`。

## 6. 工具偏移热更新

服务运行时直接编辑并保存 `config/tool_offsets.yaml`。下一次 `/get_tcp_pose` 会读取新的工具偏移、`target_selection.use_yolo` 和每个工件的 `code`，无需重启服务。无YOLO模式由请求 `code` 动态选择工件。可以在 `service.log` 中核对请求code、对应工件、工具偏移和配置SHA-256。Windows可直接运行 `calibrate_tool_offset.py`；`run_tool_calibration.sh` 是Ubuntu Bash包装脚本。

只有工具文件支持热更新。修改 `workflow.yaml`、模型、相机标定或手眼标定后必须重启服务。工具文件语法或数值无效时，本次请求返回500且不会沿用旧参数；修复后直接重试。

实时 `live=true` 请求在服务端于 `http.live_capture_delay_s`（当前4秒）内持续读取并丢弃原始帧，随后再采集。该时段内不要移动机械臂、相机或目标。

## 7. 现场联调

先确认 `field_test/test_case.yaml`：

```yaml
jaka_test:
  start_jaka: false
  work_jaka: false
```

采集快照并只读记录JAKA当前TCP：

```powershell
powershell -ExecutionPolicy Bypass -File .\run_field_capture.ps1 `
  -Python ".\.venv\Scripts\python.exe"
```

完成LabelMe标注并填写 `field_test/test_case.yaml` 后，只计算不运动：

```powershell
powershell -ExecutionPolicy Bypass -File .\run_field_test.ps1 `
  -Python ".\.venv\Scripts\python.exe"
```

只有确认返回位姿、安全方向、速度和机械臂工作空间后，才可启用 `work_jaka: true`。完整流程见 [FIELD_TEST.md](FIELD_TEST.md)。

启用运动后，脚本会先到最终TCP沿JAKA基座X偏移的预备位，在该处摆好最终姿态，再仅沿基座X进入；完成后沿相同预备位返回拍照位。默认偏移在 `field_test/test_case.yaml` 中为 `approach_offset_base_x_mm: -200.0`，也可用 `--approach-x-mm` 临时覆盖。

## 8. 无硬件测试与排障

不连接相机和机器人的测试：

```powershell
.\.venv\Scripts\python.exe run_simulated_test.py
```

常见问题：

- `No module named openni`：检查 `SURFACEPRO50_PYTHON_PATH`；
- 找不到 `OpenNI2.dll`：检查Redist目录、`PATH`及Python/SDK位数；
- 相机没有图像流：关闭厂家Viewer和其他视觉服务，确认IP、网卡和供电；
- 端口未监听：查看 `service.log.err` 中最早的异常；
- TCP明显异常：确认请求中的拍照TCP属于本次图像、角度单位是弧度，并核对相机标定、手眼标定、活动TCP及工具类别；
- `std::bad_alloc`、`terminate()`或原生取帧永久阻塞：停止整个服务，清理占用进程并重启相机，Python线程无法安全恢复已经崩溃的厂家原生库。
