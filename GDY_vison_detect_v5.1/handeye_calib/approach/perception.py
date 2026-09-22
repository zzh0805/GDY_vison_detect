# -*- coding: utf-8 -*-
"""可替换目标检测器的公共协议与通道检查。"""
from __future__ import annotations

from typing import Protocol, runtime_checkable

from ..hardware_interfaces import CameraFrame
from .models import (CameraModel, FrameChannelRequirements,
                     TargetDetectionBatch)


@runtime_checkable
class TargetPerception(Protocol):
    detector_name: str
    detector_version: str

    def required_channels(self) -> FrameChannelRequirements: ...

    def process(self, frame: CameraFrame, camera_model: CameraModel,
                request_id: str) -> TargetDetectionBatch: ...


def validate_frame_channels(frame: CameraFrame,
                            requirements: FrameChannelRequirements) -> None:
    missing = []
    for field_name, required in (
            ("gray", requirements.require_gray),
            ("color", requirements.require_color),
            ("depth", requirements.require_depth),
            ("point_cloud", requirements.require_point_cloud)):
        if required and getattr(frame, field_name) is None:
            missing.append(field_name)
    if missing:
        raise ValueError(f"当前检测器需要相机通道 {missing}，但 CameraFrame 未提供")
