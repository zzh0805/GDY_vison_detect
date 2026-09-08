# -*- coding: utf-8 -*-
"""固定主配置与可热加载工具偏移的读取、校验和路径解析。"""
from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Mapping, Sequence

import yaml


def _mapping(value: Any, name: str) -> Dict[str, Any]:
    if value is None:
        return {}
    if not isinstance(value, Mapping):
        raise ValueError(f"配置{name}必须是字典")
    return dict(value)


def _finite_triplet(value: Any, name: str) -> list[float]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise ValueError(f"{name}必须是3个数字")
    if len(value) != 3:
        raise ValueError(f"{name}必须是3个数字")
    try:
        result = [float(item) for item in value]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name}必须是3个数字") from exc
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{name}不能包含NaN/Inf")
    return result


def _validate_tools(value: Any) -> Dict[str, Any]:
    tools = _mapping(value, "tools")
    for raw_class_name, raw_tool in tools.items():
        if not isinstance(raw_class_name, str):
            raise ValueError("tools中的类别名称必须是字符串")
        class_name = str(raw_class_name).strip()
        if not class_name:
            raise ValueError("tools中的类别名称不能为空")
        tool = _mapping(raw_tool, f"tools.{class_name}")
        if "camera_to_tool" in tool:
            raise ValueError(
                f"tools.{class_name}.camera_to_tool是v1字段，"
                "请迁移为standard_to_tool")
        if "enabled" in tool and not isinstance(tool["enabled"], bool):
            raise ValueError(f"tools.{class_name}.enabled必须是true或false")
        if "standard_to_tool" not in tool:
            raise ValueError(f"tools.{class_name}缺少standard_to_tool")
        transform = _mapping(
            tool.get("standard_to_tool"),
            f"tools.{class_name}.standard_to_tool")
        _finite_triplet(
            transform.get("xyz_mm"),
            f"tools.{class_name}.standard_to_tool.xyz_mm")
        _finite_triplet(
            transform.get("rpy_deg"),
            f"tools.{class_name}.standard_to_tool.rpy_deg")
        if "standoff_mm" in tool:
            try:
                standoff_mm = float(tool["standoff_mm"])
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    f"tools.{class_name}.standoff_mm必须是数字") from exc
            if not math.isfinite(standoff_mm) or standoff_mm < 0.0:
                raise ValueError(
                    f"tools.{class_name}.standoff_mm必须是非负有限数字")
    return tools


@dataclass(frozen=True)
class ToolOffsetsSnapshot:
    """一次完整读取并校验后的工具偏移快照。"""

    source_path: Path
    tools: Dict[str, Any]
    use_yolo: bool
    selected_tool: str
    sha256: str
    modified_at_unix_s: float


def load_tool_offsets(path: Any) -> ToolOffsetsSnapshot:
    """读取独立工具配置；每次调用都重新访问磁盘，不使用内存缓存。"""
    source = Path(path).expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(f"工具偏移配置不存在: {source}")

    # 编辑器保存文件时可能先写临时内容。若读取期间文件发生变化，重读一次；
    # 仍不稳定则让本次解算失败，禁止悄悄使用旧偏移。
    payload = b""
    after = None
    for _attempt in range(2):
        before = source.stat()
        payload = source.read_bytes()
        after = source.stat()
        if (before.st_mtime_ns, before.st_size) == (
                after.st_mtime_ns, after.st_size):
            break
    else:
        raise ValueError(f"工具偏移配置正在被修改，请重新执行本次解算: {source}")

    try:
        data = yaml.safe_load(payload.decode("utf-8-sig")) or {}
    except (UnicodeDecodeError, yaml.YAMLError) as exc:
        raise ValueError(f"工具偏移配置不是有效UTF-8 YAML: {source}: {exc}") from exc
    if not isinstance(data, Mapping):
        raise ValueError(f"工具偏移配置顶层必须是字典: {source}")
    version = data.get("version", 1)
    if int(version) != 1:
        raise ValueError(f"工具偏移配置version只支持1，实际为{version!r}")
    tools = _validate_tools(data.get("tools"))
    target_selection = _mapping(
        data.get("target_selection"), "target_selection")
    use_yolo = target_selection.get("use_yolo", True)
    if not isinstance(use_yolo, bool):
        raise ValueError("target_selection.use_yolo必须是true或false")
    selected_tool = str(
        target_selection.get("selected_tool") or "").strip()
    if not use_yolo:
        if not selected_tool:
            raise ValueError(
                "关闭YOLO时必须设置target_selection.selected_tool")
        if selected_tool not in tools:
            raise ValueError(
                "target_selection.selected_tool未在tools中定义: "
                f"{selected_tool!r}")
        if not bool(tools[selected_tool].get("enabled", True)):
            raise ValueError(
                "target_selection.selected_tool对应工具未启用: "
                f"{selected_tool!r}")
    return ToolOffsetsSnapshot(
        source_path=source,
        tools=tools,
        use_yolo=use_yolo,
        selected_tool=selected_tool,
        sha256=hashlib.sha256(payload).hexdigest(),
        modified_at_unix_s=float(after.st_mtime),
    )


@dataclass(frozen=True)
class AppConfig:
    source_path: Path
    data: Dict[str, Any]

    @property
    def root(self) -> Path:
        return self.source_path.parent

    def section(self, name: str) -> Dict[str, Any]:
        return _mapping(self.data.get(name), name)

    def resolve_path(self, value: Any, *, must_exist: bool = False) -> Path:
        text = str(value or "").strip()
        if not text:
            raise ValueError("配置路径不能为空")
        path = Path(text).expanduser()
        if not path.is_absolute():
            path = self.root / path
        path = path.resolve()
        if must_exist and not path.exists():
            raise FileNotFoundError(f"配置文件不存在: {path}")
        return path

    @property
    def camera(self) -> Dict[str, Any]:
        return self.section("camera")

    @property
    def calibration(self) -> Dict[str, Any]:
        return self.section("calibration")

    @property
    def yolo(self) -> Dict[str, Any]:
        return self.section("yolo")

    @property
    def matching(self) -> Dict[str, Any]:
        return self.section("target_matching")

    @property
    def plane(self) -> Dict[str, Any]:
        return self.section("plane_fitting")

    @property
    def pose(self) -> Dict[str, Any]:
        return self.section("pose")

    @property
    def annotation(self) -> Dict[str, Any]:
        return self.section("annotation_capture")

    @property
    def system(self) -> Dict[str, Any]:
        return self.section("system")

    @property
    def network(self) -> Dict[str, Any]:
        return self.section("network")

    @property
    def http(self) -> Dict[str, Any]:
        return self.section("http")

    @property
    def tool_offsets_path(self) -> Path:
        settings = self.section("tool_offsets")
        return self.resolve_path(
            settings.get("file", "tool_offsets.yaml"), must_exist=True)

    @property
    def tools(self) -> Dict[str, Any]:
        """兼容只读入口；每次访问都会从独立文件重新加载。"""
        return load_tool_offsets(self.tool_offsets_path).tools

    def validate(self) -> None:
        self.resolve_path(
            self.camera.get("calibration_file"), must_exist=True)
        self.resolve_path(
            self.calibration.get("handeye_result_file"), must_exist=True)
        self.resolve_path(self.yolo.get("model_file"), must_exist=True)
        mode = str(self.pose.get("alignment_mode", "camera_center"))
        if mode not in ("camera_center", "tool"):
            raise ValueError("pose.alignment_mode只支持camera_center或tool")
        if int(self.pose.get("pipeline_version", 0)) != 2:
            raise ValueError("v2要求pose.pipeline_version=2，禁止误用旧坐标链配置")
        if float(self.pose.get("standoff_mm", 50.0)) < 0.0:
            raise ValueError("pose.standoff_mm不能小于0")
        if "tools" in self.data:
            raise ValueError(
                "v3不再从workflow.yaml读取tools；"
                "请移动到tool_offsets.file指定的独立文件")
        load_tool_offsets(self.tool_offsets_path)
        if int(self.matching.get("source_image_width", 1920)) <= 0 or \
                int(self.matching.get("source_image_height", 1080)) <= 0:
            raise ValueError("原图像尺寸必须大于0")
        http = self.http
        port = int(http.get("listen_port", 48051))
        if port <= 0 or port > 65535:
            raise ValueError("http.listen_port必须在1~65535之间")
        if float(http.get("request_timeout_s", 15.0)) <= 0.0:
            raise ValueError("http.request_timeout_s必须大于0")
        for delay_name, default_value in (
                ("snapshot_delay_s", 5.0),
                ("live_capture_delay_s", 4.0)):
            delay_s = float(http.get(delay_name, default_value))
            if not math.isfinite(delay_s) or delay_s < 0.0:
                raise ValueError(f"http.{delay_name}必须是非负有限数字")
        if float(http.get("snapshot_cache_ttl_s", 300.0)) <= 0.0:
            raise ValueError("http.snapshot_cache_ttl_s必须大于0")
        self.resolve_path(http.get("snapshot_directory", "../shared_images"))


def load_config(path: Any) -> AppConfig:
    source = Path(path).expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(f"主配置不存在: {source}")
    data = yaml.safe_load(source.read_text(encoding="utf-8")) or {}
    if not isinstance(data, Mapping):
        raise ValueError("主配置顶层必须是字典")
    config = AppConfig(source, dict(data))
    config.validate()
    return config
