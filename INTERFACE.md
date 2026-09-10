# 高低压视觉检测 HTTP 接口

默认服务地址：`http://{Ubuntu工控机IP}:48051`。

- 编码：UTF-8 JSON；
- 请求头：`Content-Type: application/json;charset=UTF-8`；
- `code=200` 表示业务成功；
- 服务只返回坐标，不执行机械臂运动。

## `POST /snapshot`

无需请求体。服务在 `http.snapshot_delay_s` 内持续读取并丢弃原始彩色/深度帧，再通过旧项目直接取流流程采集最终彩色图和配准点云。彩色 JPG 保存到 `http.snapshot_directory`，完整帧暂存在内存中。

```bash
curl -X POST http://127.0.0.1:48051/snapshot
```

成功响应：

```json
{
  "code": 200,
  "path": "/home/nvidia/software/GDY_vison_detect_v3/shared_images/20260901120000.jpg"
}
```

## `POST /get_tcp_pose`

推荐请求：

```json
{
  "pos": [379.5, -432.0, 509.3, 1.539, -0.836, 1.536],
  "base": {"x1": 100, "y1": 100, "x2": 900, "y2": 900},
  "target": {"x1": 852, "y1": 548, "x2": 982, "y2": 679},
  "live": true,
  "code": "9-8-1"
}
```

- `pos`：采集该帧时的 JAKA 当前活动 TCP，单位为 `mm + RPY rad`；
- `base`：可选的安装面板矩形，提供时目标中心深度以该平面为准；
- `target`：目标矩形，坐标对应 `target_matching` 中配置的原图尺寸；
- `live=true`：收到请求后在 `http.live_capture_delay_s`（当前4秒）内持续丢弃原始帧，再采集新帧并完成检测、解算；
- `live=false` 或省略：使用最近一次 `/snapshot` 的缓存帧。
- `code`：`use_yolo=false` 时必填的工件业务编码；必须与 `tools.<工件>.code` 一致。YOLO模式下可省略。

注意，请求中的 `code` 是类似 `"9-8-1"` 的字符串工件编码；响应中的 `code` 是 `200/400/422` 等数字状态码，两者用途不同。

兼容旧矩形字段：

```json
{
  "pos": [379.5, -432.0, 509.3, 1.539, -0.836, 1.536],
  "x1": 852,
  "y1": 548,
  "x2": 982,
  "y2": 679
}
```

成功响应严格为：

```json
{
  "code": 200,
  "pos": [689.25, -506.85, 535.67, 1.5722, -0.8269, 1.5523]
}
```

返回 `pos` 为 JAKA 基座系当前活动 TCP，单位仍为 `mm + RPY rad`。

`config/tool_offsets.yaml` 中 `target_selection.use_yolo: true` 时，`target` 用于从YOLO结果中匹配目标，`code` 不参与类别选择；设为 `false` 时，YOLO完全不参与本次推理，算法直接使用 `target` 矩形的几何中心，并根据请求 `code` 查找对应的 `tools` 工件。可选 `base` 仍只用于拟合安装平面。

如果 `system.save_report=true`，磁盘 `result.json` 还会保存坐标链审计字段、`targetSelection` 以及 `toolOffsetsHotReload`。`targetSelection` 记录本次是 `yolo` 还是 `box_center`；后者包含请求code、实际映射工件、读取的文件路径、SHA-256、修改时间和使用的 `xyzMm/rpyDeg`。HTTP成功响应仍保持只有数字 `code` 与 `pos`。

`result.json` 的 `discardedFramePairsBeforeCapture` 记录本次最终采集前主动丢弃的原始彩色/深度帧对数量；服务日志也会输出相同计数。缓存解算该值为0。

每次调用此接口都会重新读取 `config/tool_offsets.yaml`。修改并保存工具偏移后，下一次调用直接生效，不需要重启服务或重新连接相机。

## 两步流程约束

调用 `/snapshot` 后再进行框选和 `/get_tcp_pose` 时，必须保持机械臂、相机和目标不动，并使用拍摄该快照时读取到的 TCP。成功解算后缓存会被释放；下一轮重新调用 `/snapshot`。

## 实时流程示例

```bash
curl -X POST http://127.0.0.1:48051/get_tcp_pose \
  -H 'Content-Type: application/json;charset=UTF-8' \
  -d '{"pos":[379.5,-432.0,509.3,1.539,-0.836,1.536],"base":{"x1":100,"y1":100,"x2":900,"y2":900},"target":{"x1":852,"y1":548,"x2":982,"y2":679},"live":true,"code":"9-8-1"}'
```

发送请求前先读取当前 TCP，随后保持机械臂静止，直到服务完成本次采集和解算。

实时等待发生在服务端，并从请求进入内部解算任务后开始。两步流程的等待发生在 `/snapshot`，后续缓存 `/get_tcp_pose` 不会再次等待。

## 失败响应

```json
{
  "code": 409,
  "status": "请先调用/snapshot获取用于框选的图片"
}
```

| code | 含义 |
|---:|---|
| 400 | JSON、TCP、矩形、单位错误，或无YOLO模式缺少 `code` |
| 404 | 没有匹配到目标 |
| 409 | 缺少快照或缓存已过期 |
| 422 | 未知/禁用的工件 `code`，或点云、平面、目标位姿解算失败 |
| 500 | 相机、内部异常或工具热加载配置无效 |
| 504 | 任务等待超时 |

单次普通异常只结束当前HTTP请求。如果厂家原生取帧永久阻塞或直接中止进程，需要重启整个视觉服务。
