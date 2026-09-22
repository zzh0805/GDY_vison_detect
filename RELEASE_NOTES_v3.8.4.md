# GDY Vision Detect v3.8.4

发布日期：2026-09-18

## 版本目标

视觉服务和后台服务部署在不同设备时，后台无法读取视觉主机返回的本地绝对路径。本版本在拍照后把JPG上传到MinIO，并让 `/snapshot` 返回后台可访问的对象URL。

## 接口变化

`POST /snapshot` 的字段名保持兼容：

```json
{
  "code": 200,
  "path": "http://192.168.1.189:9000/test/vision/snapshots/2026/09/18/20260918120000.jpg"
}
```

原来的 `path` 是视觉主机本地绝对路径；现在是HTTP URL。后台必须通过网络读取，不能再使用本机文件API打开。

`/get_tcp_pose`、`/motion/get_tcp_pose`、快照内存缓存和TCP解算逻辑不变。

## MinIO上传

当前配置：

```yaml
http:
  save_snapshot: true
  snapshot_delivery: minio
  snapshot_minio:
    endpoint: "http://192.168.1.189:9000"
    bucket: test
    object_prefix: vision/snapshots
    url_mode: public
    public_base_url: "http://192.168.1.189:9000"
    presigned_expiry_s: 86400
    delete_local_after_upload: true
    access_key_env: GDY_MINIO_ACCESS_KEY
    secret_key_env: GDY_MINIO_SECRET_KEY
```

对象名称按日期组织：

```text
vision/snapshots/YYYY/MM/DD/yyyyMMddHHmmss.jpg
```

上传成功后才返回200。上传失败、凭据缺失、桶不存在或网络不可达时，本次 `/snapshot` 返回500，不会返回无效URL。

## 凭据配置

访问密钥不写入YAML和Git。视觉主机执行：

```bash
cp linux_sdk_paths.env.example linux_sdk_paths.env
nano linux_sdk_paths.env
```

填写：

```bash
export GDY_MINIO_ACCESS_KEY="实际Access Key"
export GDY_MINIO_SECRET_KEY="实际Secret Key"
```

`linux_sdk_paths.env` 已被 `.gitignore` 排除。修改后重启视觉服务。

## URL模式

### public

返回稳定对象地址，适合后台保存和前端长期显示。MinIO的 `test` 桶或 `vision/snapshots` 前缀必须允许读取，否则URL会返回403。

### presigned

私有桶无需开放匿名读取，返回包含签名的临时URL：

```yaml
url_mode: presigned
presigned_expiry_s: 86400
```

预签名URL最长支持604800秒（7天），不适合需要永久保存链接的业务。

## HTTP备用模式

如果视觉主机暂时无法连接MinIO，可切换：

```yaml
http:
  snapshot_delivery: http
  snapshot_public_base_url: "http://视觉主机IP:48051"
```

此时 `/snapshot` 返回视觉服务自身的 `/snapshots/{文件名}` URL。下载接口只允许读取快照目录中的JPG文件，并阻止目录穿越。

## 部署要求

1. 更新依赖：`pip install -r requirements.txt`；
2. 在视觉主机设置MinIO密钥环境变量；
3. 确认视觉主机能够连接 `192.168.1.189:9000`；
4. 确认 `test` 桶存在；
5. `url_mode: public` 时确认后台或前端能够匿名GET返回的对象URL；
6. 重启视觉服务后调用 `/snapshot`，再从后台设备访问响应中的 `path`。

## 安全说明

MinIO访问密钥属于敏感凭据，不应出现在截图、聊天、日志或代码仓库中。已经对外展示过的密钥建议在MinIO中轮换，并把新密钥只配置到视觉主机的私有环境文件。
