# 高低压视觉检测 HTTP 接口

默认服务地址：`http://{Ubuntu工控机IP}:48051`。

- 编码：UTF-8 JSON；
- 请求头：`Content-Type: application/json;charset=UTF-8`；
- `code=200` 表示业务成功；
- 服务只返回坐标，不执行机械臂运动。

## `POST /snapshot`

无需请求体。服务从后台最新帧缓存取得彩色图和配套深度，先把彩色 JPG 保存到临时目录，再按 `http.snapshot_delivery` 发布。当前配置为 `minio`：上传成功后返回MinIO对象URL并删除本地临时图片；完整彩深帧仍暂存在视觉服务内存中供后续解算。

```bash
curl -X POST http://192.168.1.20:48051/snapshot
```

成功响应：

```json
{
  "code": 200,
  "path": "http://192.168.1.189:9000/test/vision/snapshots/2026/09/18/20260918120000.jpg"
}
```

`path` 字段名为兼容现有后台保持不变，但字段值现在是URL。后台不应再把它当作本机文件路径，而应使用HTTP GET下载或直接交给前端显示。

MinIO配置示例：

```yaml
http:
  snapshot_delivery: minio
  snapshot_minio:
    endpoint: "http://192.168.1.189:9000"
    bucket: test
    object_prefix: vision/snapshots
    url_mode: public
    public_base_url: "http://192.168.1.189:9000"
    delete_local_after_upload: true
    access_key_env: GDY_MINIO_ACCESS_KEY
    secret_key_env: GDY_MINIO_SECRET_KEY
```

真实访问密钥必须设置在视觉主机被Git忽略的 `linux_sdk_paths.env` 中。`url_mode: public` 要求对应桶或前缀允许后台/前端匿名读取；私有桶可改为 `presigned`，此时URL会按 `presigned_expiry_s` 到期。

## `GET /snapshots/{文件名}`（HTTP备用发布模式）

当 `snapshot_delivery: http` 时，视觉服务自身提供快照下载：

```bash
curl -o snapshot.jpg \
  http://192.168.1.20:48051/snapshots/20260901120000.jpg
```

该接口只允许读取 `http.snapshot_directory` 目录中的 `.jpg/.jpeg` 文件，不允许子目录或 `..` 路径。

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
- `live=true`：收到请求后优先等待请求后的新彩色帧，再配套选择深度并完成检测、解算；不增加固定4秒等待；
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

`config/tool_offsets.yaml` 中 `target_selection.use_yolo: true` 时，`target` 用于从YOLO结果中匹配目标，`code` 不参与类别选择；设为 `false` 时，YOLO完全不参与本次推理，算法根据请求 `code` 取得对应工件。此时 `circle_refinement_enabled: true` 会根据工件 `target_color` 在粗框附近筛选工件并采用最近工件组的最大有效外圆圆心；设为 `false` 则完全跳过圆拟合和颜色筛选，直接使用 `target` 框的几何中心。可选 `base` 仍只用于拟合安装平面。

如果 `system.save_report=true`，磁盘 `result.json` 还会保存坐标链审计字段、`targetSelection` 以及 `toolOffsetsHotReload`。为兼容现有后台，`targetSelection.mode` 仍记录 `yolo`、`nearest_circle_center` 或关闭圆拟合时的 `box_center`；圆拟合评分、边缘圆心、最终圆心、识别颜色、颜色覆盖率、颜色质心、是否启用实验性质心融合及候选数量记录在几何质量字段中。HTTP成功响应仍保持只有数字 `code` 与 `pos`。

`result.json` 的 `discardedFramePairsBeforeCapture` 记录本次最终采集前主动丢弃的原始彩色/深度帧对数量；服务日志也会输出相同计数。缓存解算该值为0。

每次调用此接口都会重新读取 `config/tool_offsets.yaml`。修改并保存 `use_yolo`、`circle_refinement_enabled` 或工具偏移后，下一次调用直接生效，不需要重启服务或重新连接相机。

## `POST /motion/get_tcp_pose`（现场运动脚本专用）

请求字段、单位、实时/缓存行为与 `/get_tcp_pose` 完全相同，但成功响应额外提供当次安装平面法向转换到JAKA基座系后的进入方向：

```json
{
  "code": 200,
  "pos": [689.25, -506.85, 535.67, 1.5722, -0.8269, 1.5523],
  "approachDirectionBase": [0.012, 0.999, -0.036],
  "approachDirectionSource": "fitted_panel_normal"
}
```

`approachDirectionBase` 是无单位的归一化向量，从外部预备位指向柜体工作位。现场脚本用 `预备位置 = 最终位置 - 距离 × 方向` 生成预备TCP，RPY保持最终工作位不变。该接口只为本项目运动脚本增加信息；正式后台继续使用 `/get_tcp_pose`，其成功响应仍严格只有 `code` 和 `pos`。

方向缺失、包含NaN/Inf或长度为0时返回422，不会回退为固定基座X方向。

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
