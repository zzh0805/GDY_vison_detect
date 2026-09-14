# Ubuntu ARM64 部署

以下命令假设项目部署到 `/home/nvidia/software/GDY_vison_detect_v3`。如果使用其他目录，需要同步修改systemd单元中的 `WorkingDirectory` 和 `ExecStart`。

## 1. 复制并检查目录

```bash
cd /home/nvidia/software
git clone git@github.com:zzh0805/GDY_vison_detect.git GDY_vison_detect_v3
cd /home/nvidia/software/GDY_vison_detect_v3
find . -type d -name '__pycache__' -o -type f -name '*.pyc'
```

如果现场设备没有配置GitHub SSH密钥，也可以在其他电脑下载后完整复制目录，或在仓库权限允许时使用HTTPS地址。

本交付目录不应带有历史 `output`、`shared_images` 或 `service.log`。这些目录和文件会在运行时按配置创建。

## 2. 配置SDK与Python环境

```bash
cp linux_sdk_paths.env.example linux_sdk_paths.env
nano linux_sdk_paths.env
```

必须确认所有路径指向 Ubuntu ARM64版本的 OpenNI、SurfacePro50 和 JAKA SDK。然后在目标Python环境安装依赖：

```bash
python -m pip install -r requirements.txt
```

如果使用 Conda，请确保 `VISION_CONDA_ENV` 与实际环境名一致，并在该环境内安装依赖。

## 3. 修改现场配置

编辑 `config/workflow.yaml` 和 `config/tool_offsets.yaml`，重点检查：

1. `camera.ip` 与当前相机一致；
2. 相机标定、手眼标定和 YOLO模型文件存在；
3. `http.listen_port` 与客户端、启动脚本一致，当前统一为 `48051`；
4. `source_image_width/height` 与实际彩色图一致；
5. `snapshot_delay_s=4`、`live_capture_delay_s=4`；这两个时段会主动丢弃旧帧，并确认请求超时大于丢帧、最终拍照和检测总耗时；
6. `pipeline_version=2`，且 `standoff_mm`、`tcp_correction` 使用本套机械结构对应的标定值；
7. `tool_offsets.file` 指向有效工具文件，工具类别及 `standard_to_tool` 正确；
8. `target_selection.use_yolo` 符合现场模式；关闭YOLO时，每个可选工件必须配置唯一 `code`，上位机请求也必须携带该值；
9. 输出目录有写权限和足够空间。

检查网络和文件：

```bash
ping -c 3 192.168.16.122
test -f calibration/chishine_192_168_16_122_calibration.yml
test -f calibration/handeye_result.json
test -f models/xuncao.pt
test -f config/tool_offsets.yaml
```

## 4. 首次前台启动

关闭厂家 Viewer 和其他视觉进程，保证只有本服务占用相机：

```bash
pgrep -af 'run_service.py'
bash run_service.sh
```

YOLO模式应看到模型准备日志；无YOLO模式应看到“跳过YOLO启动预加载”和圆心拟合开关。随后应看到相机连接成功及端口监听日志。另开终端验证：

```bash
ss -tlnp | grep ':48051 '
curl -X POST http://127.0.0.1:48051/snapshot
```

首次只验证拍照和返回坐标，不执行机械臂运动。先用零偏移类别确认返回的是“逆手眼+全局修正”的50mm标准TCP，再低速验证迁移后的按钮工具位。

服务运行期间修改 `config/tool_offsets.yaml` 后无需重启。工具偏移、`use_yolo` 和 `tools` 下的 `code` 映射都在下一次 `/get_tcp_pose` 自动生效；若文件格式错误，该请求返回500并拒绝输出旧偏移结果，修正后直接重试。

在Ubuntu上标定或微调工具时，优先编辑 `tool/tool_calibration.yaml`，按当前模式和步骤运行：

```bash
bash tool/run_tool_calibration.sh
```

脚本只登录JAKA并读取当前活动TCP，不发送运动命令。模式1/2依次执行step 1、2、3；模式3记录step 1后填写基座系微调量，再执行step 2。每一步及最终结果保存在 `tool/tool_calibration_record.yaml`；完整说明见 [tool/README.md](tool/README.md)。根目录旧脚本仍保留供手工输入参数时兼容使用。

## 5. 后台启动

前台验证成功后按 `Ctrl+C` 退出，再运行：

```bash
bash start_vision_service.sh
tail -f service.log
```

停止：

```bash
bash stop_vision_service.sh
```

## 6. systemd自启动

先检查 [systemd/gdy_vison_detect.service](systemd/gdy_vison_detect.service) 中的用户和绝对路径，再安装：

```bash
sudo cp systemd/gdy_vison_detect.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now gdy_vison_detect.service
sudo systemctl status gdy_vison_detect.service
sudo journalctl -u gdy_vison_detect.service -f
```

不要同时使用 `start_vision_service.sh` 和 systemd 启动两个实例。

## 7. 常见问题

- `设备没有可用的 Color/IR/Depth 图像流`：检查相机是否被其他进程占用、OpenNI驱动路径和固件/SDK兼容性；
- 一直输出 `Didn't get frame`：确认网络、曝光、数据流配置和相机端供电，停止所有占用进程后重试；
- `std::bad_alloc` 或 `terminate called`：厂家原生库已经中止，检查内存与帧释放，重启整个服务；仍失败时给相机断电约10秒再上电；
- 端口没有监听：查看 `service.log` 或 `journalctl` 中最早出现的异常；
- 返回坐标异常：首先确认请求 `pos` 对应本次采集帧，RPY使用弧度，并检查标定文件和工具类别映射。
