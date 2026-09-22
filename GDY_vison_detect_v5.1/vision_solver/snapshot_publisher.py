# -*- coding: utf-8 -*-
"""把HTTP快照发布为跨设备可访问的URL。"""
from __future__ import annotations

import os
from datetime import datetime, timedelta
from pathlib import Path
from typing import Any, Mapping
from urllib.parse import quote, urlsplit

from .logging_utils import get_logger


log = get_logger()


def _settings(value: Any, name: str) -> dict:
    if value is None:
        return {}
    if not isinstance(value, Mapping):
        raise ValueError(f"{name}必须是字典")
    return dict(value)


def _join_object_url(base_url: str, bucket: str, object_name: str) -> str:
    return (
        f"{base_url.rstrip('/')}/{quote(bucket, safe='')}/"
        f"{quote(object_name, safe='/')}")


class SnapshotPublisher:
    """支持视觉服务HTTP下载或MinIO对象存储两种发布方式。"""

    def __init__(self, config: Any):
        self.config = config
        self.http = dict(config.http)
        self.delivery = str(
            self.http.get("snapshot_delivery", "http") or "http"
        ).strip().lower()
        self._minio_client = None

    def publish(
            self, image_path: Any,
            request_base_url: str | None = None) -> str:
        if not image_path:
            return ""
        if self.delivery == "http":
            return self._http_url(image_path, request_base_url)
        if self.delivery == "minio":
            return self._upload_minio(Path(str(image_path)))
        raise RuntimeError(
            f"不支持的快照发布方式http.snapshot_delivery={self.delivery!r}")

    def _http_url(
            self, image_path: Any,
            request_base_url: str | None) -> str:
        configured_base = str(
            self.http.get("snapshot_public_base_url", "") or "").strip()
        base_url = configured_base or str(request_base_url or "").strip()
        if not base_url:
            raise RuntimeError(
                "无法生成快照URL：请配置http.snapshot_public_base_url，"
                "或通过HTTP Host访问/snapshot")
        file_name = Path(str(image_path)).name
        if not file_name:
            raise RuntimeError("快照文件名为空，无法生成下载URL")
        return (
            f"{base_url.rstrip('/')}/snapshots/"
            f"{quote(file_name, safe='')}")

    def _minio_config(self) -> dict:
        return _settings(
            self.http.get("snapshot_minio"), "http.snapshot_minio")

    def _client(self):
        if self._minio_client is not None:
            return self._minio_client
        cfg = self._minio_config()
        endpoint = str(cfg.get("endpoint") or "").strip()
        parsed = urlsplit(endpoint)
        access_env = str(
            cfg.get("access_key_env") or "GDY_MINIO_ACCESS_KEY").strip()
        secret_env = str(
            cfg.get("secret_key_env") or "GDY_MINIO_SECRET_KEY").strip()
        access_key = str(os.environ.get(access_env) or "").strip()
        secret_key = str(os.environ.get(secret_env) or "").strip()
        if not access_key or not secret_key:
            raise RuntimeError(
                f"MinIO凭据未配置，请设置环境变量{access_env}和{secret_env}")
        try:
            from minio import Minio
        except ImportError as exc:
            raise RuntimeError(
                "缺少MinIO Python SDK，请执行pip install -r requirements.txt"
            ) from exc
        self._minio_client = Minio(
            parsed.netloc,
            access_key=access_key,
            secret_key=secret_key,
            secure=parsed.scheme == "https")
        return self._minio_client

    def _upload_minio(self, image_path: Path) -> str:
        if not image_path.is_file():
            raise RuntimeError(f"待上传快照不存在: {image_path}")
        cfg = self._minio_config()
        bucket = str(cfg.get("bucket") or "").strip()
        prefix = str(
            cfg.get("object_prefix") or "vision/snapshots").strip("/")
        date_path = datetime.now().strftime("%Y/%m/%d")
        object_name = "/".join(
            part for part in (prefix, date_path, image_path.name) if part)
        client = self._client()
        try:
            client.fput_object(
                bucket, object_name, str(image_path),
                content_type="image/jpeg")
        except Exception as exc:
            raise RuntimeError(
                f"快照上传MinIO失败: bucket={bucket} object={object_name}: {exc}"
            ) from exc

        url_mode = str(cfg.get("url_mode") or "public").strip().lower()
        if url_mode == "public":
            public_base_url = str(
                cfg.get("public_base_url") or cfg.get("endpoint") or ""
            ).strip()
            image_url = _join_object_url(
                public_base_url, bucket, object_name)
        elif url_mode == "presigned":
            expiry_s = int(cfg.get("presigned_expiry_s", 86400))
            try:
                image_url = client.presigned_get_object(
                    bucket, object_name,
                    expires=timedelta(seconds=expiry_s))
            except Exception as exc:
                raise RuntimeError(
                    f"生成MinIO预签名下载URL失败: {exc}") from exc
        else:
            raise RuntimeError(
                f"不支持的MinIO URL模式{url_mode!r}")

        deleted = False
        if bool(cfg.get("delete_local_after_upload", True)):
            try:
                image_path.unlink()
                deleted = True
            except OSError as exc:
                log.warning("MinIO上传成功但删除本地快照失败 %s: %s",
                            image_path, exc)
        log.info(
            "快照已上传MinIO: bucket=%s object=%s url_mode=%s "
            "本地删除=%s",
            bucket, object_name, url_mode, deleted)
        return image_url
