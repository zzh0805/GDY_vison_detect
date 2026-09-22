# -*- coding: utf-8 -*-
from __future__ import annotations

import json
import tempfile
import threading
import unittest
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from vision_solver.http_protocol import VisionHttpProtocol
from vision_solver.http_server import VisionHttpServer
from vision_solver.snapshot_publisher import SnapshotPublisher


class _Config:
    matching = {"source_image_width": 1920, "source_image_height": 1080}

    def __init__(self, root: Path, http: dict):
        self.root = root
        self.http = http

    def resolve_path(self, value):
        path = Path(value)
        if not path.is_absolute():
            path = self.root / path
        return path.resolve()


class _SnapshotSolver:
    def __init__(self, config: _Config, image_path: Path):
        self.config = config
        self.image_path = image_path

    def handle_task(self, payload, timeout_s=None):
        return {"ok": True, "imagePath": str(self.image_path)}


class _FakeMinioClient:
    def __init__(self):
        self.uploads = []

    def fput_object(
            self, bucket, object_name, file_path, content_type=None):
        self.uploads.append({
            "bucket": bucket,
            "object": object_name,
            "file": file_path,
            "content_type": content_type,
        })


class SnapshotDeliveryTests(unittest.TestCase):
    def test_minio_public_url_and_local_cleanup(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            image_path = root / "20260918120000.jpg"
            image_path.write_bytes(b"jpeg-data")
            config = _Config(root, {
                "snapshot_delivery": "minio",
                "snapshot_minio": {
                    "endpoint": "http://minio.internal:9000",
                    "bucket": "test",
                    "object_prefix": "vision/snapshots",
                    "url_mode": "public",
                    "public_base_url": "http://minio.public:9000",
                    "delete_local_after_upload": True,
                },
            })
            publisher = SnapshotPublisher(config)
            client = _FakeMinioClient()
            publisher._minio_client = client

            result = publisher.publish(image_path)

            self.assertRegex(
                result,
                r"^http://minio\.public:9000/test/vision/snapshots/"
                r"\d{4}/\d{2}/\d{2}/20260918120000\.jpg$")
            self.assertFalse(image_path.exists())
            self.assertEqual(len(client.uploads), 1)
            self.assertEqual(client.uploads[0]["bucket"], "test")
            self.assertEqual(client.uploads[0]["content_type"], "image/jpeg")

    def test_http_snapshot_url_can_be_downloaded_from_another_client(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            snapshot_root = root / "shared_images"
            snapshot_root.mkdir()
            image_path = snapshot_root / "20260918120000.jpg"
            expected = b"test-jpeg-bytes"
            image_path.write_bytes(expected)
            config = _Config(root, {
                "request_timeout_s": 5,
                "snapshot_delivery": "http",
                "snapshot_public_base_url": "",
                "snapshot_directory": str(snapshot_root),
            })
            protocol = VisionHttpProtocol(
                _SnapshotSolver(config, image_path))
            server = VisionHttpServer(protocol, "127.0.0.1", 0)
            thread = threading.Thread(
                target=server.serve_forever, daemon=True)
            thread.start()
            host, port = server.server_address
            try:
                request = Request(
                    f"http://{host}:{port}/snapshot",
                    data=b"", method="POST")
                with urlopen(request, timeout=5) as response:
                    result = json.loads(response.read().decode("utf-8"))
                self.assertEqual(result["code"], 200)
                self.assertEqual(
                    result["path"],
                    f"http://{host}:{port}/snapshots/{image_path.name}")
                with urlopen(result["path"], timeout=5) as response:
                    self.assertEqual(
                        response.headers.get("Content-Type"), "image/jpeg")
                    self.assertEqual(response.read(), expected)

                with self.assertRaises(HTTPError) as context:
                    urlopen(
                        f"http://{host}:{port}/snapshots/%2e%2e/secret.jpg",
                        timeout=5)
                self.assertEqual(context.exception.code, 404)
            finally:
                server.stop()
                thread.join(timeout=2.0)


if __name__ == "__main__":
    unittest.main()
