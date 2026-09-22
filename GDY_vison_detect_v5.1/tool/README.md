# 工具工作位标定

## 推荐：一次运行的交互式标定

Ubuntu现场只需在项目根目录执行一次：

```bash
bash tool/run_interactive_tool_calibration.sh
```

程序不会控制机械臂运动，只会读取当前活动TCP。运行后按终端提示操作：

1. 输入工件 `code`，例如 `9-8-1`；也可以直接输入 `tools:` 下的工件名称；
2. 输入模式：`1`从零标定、`2`纠正已有工件、`3`微调；
3. 把机械臂手动移动到提示位置，按Enter或输入 `yes` 读取TCP，读取后可确认或重新读取；
4. 模式1/2会保持程序运行并等待第二次手动调整；模式3会提示输入XYZ和可选RPY增量；
5. 程序完成刚体解算、正向校验，并自动更新 `config/tool_offsets.yaml`。视觉服务下一次检测会热加载新偏移，不需要重启。

任何步骤输入 `q` 或按 `Ctrl+C` 都可取消。取消时正式工具配置不会改变。

### 三种模式

- 模式1：先把相机放到光心正对目标安装平面、距离50mm的标准位并读取；再把工具示教到最终工作位并读取。程序计算 `inverse(标准TCP) × 工作TCP`。
- 模式2：先用当前偏移到达旧工作位并读取；再手动调整到新的正确工作位并读取。程序把旧工作位到新工作位的刚体变化迁移到现有工具偏移。
- 模式3：先到达当前工作位并读取，再输入类似 `0,-1,0` 的XYZ微调量；RPY不调整时直接按Enter。

### 微调坐标系与车辆朝向

模式3会提示选择坐标系，直接按Enter默认选择：

```text
1 = 标准/相机参考坐标系（推荐，跟随柜体朝向）
2 = JAKA基座坐标系（固定按基座X/Y/Z）
```

选择1时，`0,-1,0` 表示标准/相机参考系Y−1mm。该参考系跟随当次相机正对的柜体，因此车辆正对、侧对、相机朝基座 `+Y/-Y` 或斜向时，不会因为JAKA基座方向变化而把微调方向翻转。选择2时，`0,-1,0` 才严格表示JAKA基座Y−1mm。

模式1和模式2使用两次完整TCP刚体变换反算偏移，只要两次记录期间车辆底座、目标柜体、活动TCP和相机/工具安装关系不变，相机朝向基座哪个方向都不会破坏标定。不能在两次读取之间移动车辆底座或柜体。

### 唯一备份与恢复

程序每次启动、在询问工件之前就把正式配置备份到：

```text
tool/tool_offsets.backup.yaml
```

备份固定只保留这一份，下次运行会覆盖它。最近一次成功标定记录在 `tool/interactive_tool_calibration_result.yaml`。如果需要人工恢复：

```bash
cp tool/tool_offsets.backup.yaml config/tool_offsets.yaml
```

Windows运行方式：

```powershell
powershell -ExecutionPolicy Bypass -File .\tool\run_interactive_tool_calibration.ps1 `
  -Python ".\.venv\Scripts\python.exe"
```

JAKA地址和SDK设置位于 `tool/interactive_tool_calibration.yaml`，通常只在首次部署时修改。

## 旧版：YAML分步骤工具标定

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
