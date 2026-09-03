# Ubuntu现场联调

本目录只保留生产联调所需脚本，不包含历史图片和结果。

## 1. 先以安全配置启动服务

确认 [field_test/test_case.yaml](field_test/test_case.yaml) 中：

```yaml
jaka_test:
  start_jaka: false
  work_jaka: false
```

然后启动正式服务：

```bash
bash run_service.sh
```

日志应显示相机连接成功并监听 `0.0.0.0:48051`。

## 2. 采集快照并记录TCP

另开终端：

```bash
cd /home/nvidia/software/GDY_vison_detect_v3
bash run_field_capture.sh
```

脚本通过正式 `/snapshot` 拍照，并以只读方式获取当前 JAKA TCP。返回图像路径和拍照 TCP 会写入 `field_test/test_case.yaml`。

## 3. 标注与解算

用 LabelMe 打开快照，标注目标矩形，把生成的 JSON 路径填写到：

```yaml
case:
  labelme_file: /实际路径/标注.json
  labelme_target_label: request_target
```

保持现场不动，然后执行：

```bash
bash run_field_test.sh
```

此时 `work_jaka=false`，脚本只显示和保存坐标，不发送运动命令。结果写入运行时生成的 `field_test/output/test_result.json`。

需要验证实时采集时：

```bash
bash run_field_test.sh --live
```

只有在坐标、角度单位、方向和安全距离全部人工确认后，才可以将 `work_jaka` 改为 `true`。

对于零偏移类别，`result.json` 中应满足 `targetTcpMmRpyDeg == cameraReference.standardTcpMmRpyDeg`。按钮类别则应在标准TCP之后叠加迁移后的 `standard_to_tool`。

需要标定新工具时，先保存v3解算的 `result.json`，再将工具示教到最终工作位并执行：

```bash
python calibrate_tool_offset.py \
  --result-json output/solve-xxx/result.json \
  --taught-tcp=X,Y,Z,RX,RY,RZ \
  --class-name new_class \
  --tool-id tool_new_class
```

这里的RPY输入单位是度。脚本直接使用 `TBaseTcpStandard`，不会重复增加逆手眼或全局修正。

将脚本输出的类别段更新到 `config/tool_offsets.yaml` 并保存。无需重启服务；下一次运行 `run_field_test.sh` 或调用 `/get_tcp_pose` 就会使用新值。日志中的 `配置sha256`、`工具偏移xyz/rpy` 和保存报告的 `toolOffsetsHotReload` 可用于确认实际生效值。

如果正在使用两步快照流程，一次成功解算后仍需重新调用 `/snapshot`；这是图像缓存的既有规则，与工具配置热加载无关。工具YAML写错时解算会失败但快照不会被清除，修正文件后可以直接重试。

## 4. 相机连续取帧测试

该测试会直接占用相机，因此必须先停止正式服务和厂家 Viewer：

```bash
bash stop_vision_service.sh
source ./load_sdk_env.sh
"$PYTHON_BIN" field_test/12_camera_fps_test.py --frames 50 --timeout 30
```

测试结束后再启动正式服务。不要让测试脚本和服务同时连接相机。

## 5. 模拟测试

模拟测试不连接相机或机器人：

```bash
bash run_simulated_test.sh
```

它用于检查HTTP协议、缓存流程和算法调用链，不能代替真实相机取帧与机械臂到位验证。
