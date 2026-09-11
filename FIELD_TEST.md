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

服务会在收到这个实时解算请求后，于 `http.live_capture_delay_s`（当前4秒）内持续读取并丢弃旧的原始彩色/深度帧，再拍照、检测和解算。该时段替代原来的纯等待，期间机械臂、相机和目标必须保持静止。

测试YOLO模型中不存在的工件时，可先在 `config/tool_offsets.yaml` 中设置：

```yaml
target_selection:
  use_yolo: false

tools:
  your_tool_key:
    code: "9-8-1"
    tool_id: tool_your_tool
    enabled: true
    standard_to_tool:
      xyz_mm: [0.0, 0.0, 0.0]
      rpy_deg: [0.0, 0.0, 0.0]
```

并在 `field_test/test_case.yaml` 设置本次工件：

```yaml
case:
  code: "9-8-1"
```

也可以用命令行临时覆盖：

```bash
bash run_field_test.sh --live --base-label 5 --code 9-8-1
```

保存后直接再次运行现场测试，不需要重启服务。此时不会再调用YOLO；程序根据请求 `code` 读取工件的 `target_color`，在传入矩形附近用红绿黑颜色筛选目标，并使用最近工件组的最大有效外圆圆心。若要应用该工件偏移，确认 `workflow.yaml` 使用 `pose.alignment_mode: tool`。

现场第一次启用前检查 `workflow.yaml`：

```yaml
target_matching:
  circle_refinement:
    enabled: true
    expand_ratio: 0.45
    fallback_to_box_center: false
```

修改这组拟合参数后需要重启服务。检测叠加图中黄色为原始框，绿色为实际采用的圆和圆心，蓝色箭头表示修正方向。若没有找到可靠圆，默认返回失败且不输出工作TCP；不要为了让机械臂继续运动而随意开启框中心回退。

只有在坐标、角度单位、方向和安全距离全部人工确认后，才可以将 `work_jaka` 改为 `true`。

启用 `work_jaka` 后，运动顺序固定为：

```text
拍照位
  → 基座X预备位（已经采用最终工作姿态）
  → 只沿基座X直线进入最终工作TCP
  → 只沿基座X原路退回预备位
  → 返回拍照位
```

接近距离和方向在 `field_test/test_case.yaml` 中设置：

```yaml
jaka_test:
  approach_offset_base_x_mm: -200.0
```

预备位计算为 `预备X = 最终X + approach_offset_base_x_mm`，Y、Z和全部RPY与最终工作TCP完全相同。`-200` 表示从最终位基座X负方向200 mm处准备，再沿基座 `+X` 进入；如果现场需要沿 `-X` 进入则改为 `+200`。绝对值就是接近距离。

也可以只覆盖本次运行，不修改YAML：

```bash
bash run_field_test.sh --live --base-label 5 --code 9-8-1 --approach-x-mm -200
```

脚本从工作位返回时严格反向经过同一个预备位。每段均使用JAKA直线运动；服务本身仍然只负责解算TCP，运动仅由现场测试脚本执行。

对于零偏移类别，`result.json` 中应满足 `targetTcpMmRpyDeg == cameraReference.standardTcpMmRpyDeg`。按钮类别则应在标准TCP之后叠加迁移后的 `standard_to_tool`。

需要标定新工具或重新标定旧工具时，先保存v3解算的完整 `result.json`，再将工具示教到最终工作位。推荐编辑 `run_tool_calibration.sh` 顶部的用户参数区，然后执行：

```bash
bash run_tool_calibration.sh
```

脚本的 `MODE` 支持：

- `calibrate`：新工具标定或旧工具完整重标定；
- `verify`：验证已有偏移对应的最终TCP；
- `adjust`：按JAKA基座系位置和小幅RPY分量修正旧偏移。

也可以直接调用底层Python命令：

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
