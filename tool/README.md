# YAML分步骤工具标定

编辑 `tool_calibration.yaml`，每完成一个机械臂姿态就运行一次：

```bash
bash tool/run_tool_calibration.sh
```

脚本只连接JAKA并读取当前活动TCP，不会发出任何运动命令。

## 模式1：从零标定

1. 将相机移动到“光心正对目标平面、距离50mm”的标准位；设置 `mode: 1`、`step: 1` 并运行，记录标准TCP。
2. 将工具示教到最终工作位；改成 `step: 2` 并运行，记录工作TCP。
3. 保持 `mode: 1`，改成 `step: 3` 再运行，计算 `inverse(标准TCP) × 工作TCP`。

最稳妥的标准位是先让视觉服务输出零工具偏移的50mm标准TCP并执行到位，然后再记录；这样与正式视觉解算使用相同的逆手眼和全局修正基准。

## 模式2：纠正已有工件

1. 用当前工具偏移运行到旧工作位；设置 `mode: 2`、`step: 1` 并运行。此时同时冻结当前工具偏移。
2. 手动调整到新的正确工作位；改成 `step: 2` 并运行。
3. 改成 `step: 3` 并运行。脚本从 `config/tool_offsets.yaml` 读取当前偏移，并按旧TCP到新TCP的刚体变化计算新偏移。

模式2的两次记录必须属于同一个目标和同一次校正过程。不要在中间移动机器人基座、目标面板或更换活动TCP。

## 模式3：基座坐标系微调

1. 运行到当前工作位；设置 `mode: 3`、`step: 1` 并运行。此时同时冻结当前工具偏移。
2. 在 `adjustment.xyz_mm` 填写基座系位移，例如X+2mm为 `[2, 0, 0]`，Y-2mm为 `[0, -2, 0]`。把 `step` 改成 `2` 并运行即可计算。

`adjustment.rpy_deg` 是JAKA RPY分量增量，只适合小角度微调。

每次重新执行某个模式的 `step: 1`，脚本会自动清除该模式上一次的后续工作位和结果，防止不同批次记录被错误组合。

## 记录与应用

每一步和最后结果自动写到 `tool_calibration_record.yaml`。默认 `output.apply_to_tool_offsets: false`，不会修改正式配置；确认结果后可以复制输出的工件段。

如果把它改成 `true`，计算步骤会自动更新 `config/tool_offsets.yaml`，并先在 `tool/backups/` 保存旧文件。自动写入会规范化YAML排版和注释，但不改变其他工件的数据；需要原排版时可直接使用备份恢复。视觉服务下一次解算会热加载新值，不需要重启。

Windows运行方式：

```powershell
powershell -ExecutionPolicy Bypass -File .\tool\run_tool_calibration.ps1 `
  -Python ".\.venv\Scripts\python.exe"
```
