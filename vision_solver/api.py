# -*- coding: utf-8 -*-
"""长生命周期视觉入口：所有视觉任务共用一个串行任务线程。"""
from __future__ import annotations

import threading
import time
from concurrent.futures import ThreadPoolExecutor, TimeoutError
from pathlib import Path
from typing import Any, Mapping, Optional

import cv2
import numpy as np

from handeye_calib.approach.models import HandEyeCalibration
from handeye_calib.approach.yolo_detector import YoloTargetDetector

from .camera import CameraManager
from .circle_center_refiner import find_nearest_circle_center
from .config import AppConfig, load_config, load_tool_offsets
from .image_writer import ArtifactWriter
from .logging_utils import get_logger
from .models import (AnnotationCaptureRequest, MatchResult, SolveTargetRequest,
                     TaskError, json_ready)
from .pose_solver import TargetPoseSolver
from .target_matcher import match_observation, scale_request_corners

log = get_logger()


class VisionToolTcpSolver:
    """可被 TCP 服务或现场测试直接复用的唯一业务入口。"""

    def __init__(self, config_path: Any, *, camera: Any = None,
                 detector: Any = None):
        self.config: AppConfig = load_config(config_path)
        self.tool_offsets_path = self.config.tool_offsets_path
        self.camera = CameraManager(self.config, camera)
        self.detector = detector
        self.calibration: Optional[HandEyeCalibration] = None
        self.pose_solver: Optional[TargetPoseSolver] = None
        self.artifacts = ArtifactWriter(self.config)
        self._executor = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="vision-task-worker")
        self._closed = False
        self._state_lock = threading.Lock()
        self._http_snapshot_lock = threading.Lock()
        self._http_snapshot_frame = None
        self._http_snapshot_monotonic = 0.0
        self._http_snapshot_timer = None
        timeout = float(self.config.system.get("initialization_timeout_s", 60.0))
        started = time.perf_counter()
        log.info("服务初始化开始: config=%s", self.config.source_path)
        try:
            self._executor.submit(self._initialize).result(timeout=timeout)
        except Exception:
            log.error("服务初始化失败（%.1fs）",
                      time.perf_counter() - started, exc_info=True)
            self._executor.shutdown(wait=True, cancel_futures=True)
            raise
        log.info("服务初始化完成，总耗时 %.1fs",
                 time.perf_counter() - started)

    def _initialize(self) -> None:
        try:
            step_started = time.perf_counter()
            calibration_cfg = self.config.calibration
            self.calibration = HandEyeCalibration.load(
                self.config.resolve_path(
                    calibration_cfg.get("handeye_result_file"), must_exist=True),
                require_accepted=bool(
                    calibration_cfg.get("require_accepted", True)))
            log.info("手眼标定加载: %s 样本=%d accepted=%s (%.1fs)",
                     self.calibration.source_path,
                     len(getattr(self.calibration, "samples", []) or []),
                     bool(getattr(self.calibration, "accepted", False)),
                     time.perf_counter() - step_started)
            if self.detector is None:
                yolo = self.config.yolo
                plane = self.config.plane
                class_names = yolo.get("class_names") or None
                self.detector = YoloTargetDetector(
                    model_path=str(self.config.resolve_path(
                        yolo.get("model_file"), must_exist=True)),
                    confidence=float(yolo.get("confidence", 0.5)),
                    iou=float(yolo.get("iou", 0.7)),
                    device=(str(yolo.get("device") or "").strip() or None),
                    classes=yolo.get("classes") or None,
                    class_names=class_names,
                    max_detections=int(yolo.get("max_detections", 100)),
                    bbox_shrink_ratio=float(
                        plane.get("bbox_shrink_ratio", 0.15)),
                    center_patch_radius_px=int(
                        plane.get("center_patch_radius_px", 5)),
                    min_plane_points=int(plane.get("min_points", 100)),
                    max_plane_sample_points=int(
                        plane.get("max_sample_points", 20000)),
                    plane_distance_threshold_mm=float(
                        plane.get("ransac_threshold_mm", 3.0)),
                    plane_min_inlier_ratio=float(
                        plane.get("min_inlier_ratio", 0.35)),
                    plane_region_mode="surrounding_panel",
                    surrounding_expand_ratio=float(
                        plane.get("surrounding_expand_ratio", 1.0)),
                    surrounding_exclude_margin_ratio=float(
                        plane.get("exclude_box_margin_ratio", 0.15)),
                    require_handeye_compatible_cloud=True,
                )
            tool_offsets = load_tool_offsets(self.tool_offsets_path)
            if (tool_offsets.use_yolo and
                    bool(self.config.yolo.get("load_model_at_start", True))):
                ensure_model = getattr(self.detector, "_ensure_model", None)
                if callable(ensure_model):
                    ensure_model()
            yolo_cfg = self.config.yolo
            if tool_offsets.use_yolo:
                log.info("YOLO模型准备: %s device=%s 类别=%s (%.1fs)",
                         self.config.resolve_path(yolo_cfg.get("model_file")),
                         yolo_cfg.get("device") or "auto",
                         sorted(self.detector.class_names or ()) or "全部",
                         time.perf_counter() - step_started)
            else:
                circle_enabled = bool(dict(
                    self.config.matching.get("circle_refinement") or {}
                ).get("enabled", False))
                log.info(
                    "无YOLO模式已启用: 跳过YOLO启动预加载，"
                    "圆心拟合=%s，由请求code动态选择工件，可用映射=%s",
                    circle_enabled,
                    tool_offsets.code_to_tool)
            step_started = time.perf_counter()
            self.pose_solver = TargetPoseSolver(
                self.calibration, self.config.pose, tool_offsets.tools)
            log.info(
                "工具偏移配置加载: %s 类别=%d use_yolo=%s "
                "code映射数=%d sha256=%s",
                tool_offsets.source_path, len(tool_offsets.tools),
                tool_offsets.use_yolo,
                len(tool_offsets.code_to_tool),
                tool_offsets.sha256[:12])
            cam_cfg = self.config.camera
            log.info("相机连接: type=%s ip=%s",
                     cam_cfg.get("type"), cam_cfg.get("ip", "auto"))
            self.camera.connect()
            log.info("相机连接成功 (%.1fs)",
                     time.perf_counter() - step_started)
        except Exception:
            log.error("相机/求解器初始化异常", exc_info=True)
            self.camera.disconnect()
            raise

    def _handle_annotation(self, payload: Mapping[str, Any]) -> dict:
        request = AnnotationCaptureRequest.from_mapping(payload)
        frame = self.camera.capture_color()
        saved = self.artifacts.save_annotation(frame, request)
        return json_ready({
            "ok": True,
            "taskType": "capture_annotation_image_result",
            "taskId": request.task_id,
            **saved,
        })

    def _handle_http_snapshot(self, payload: Mapping[str, Any]) -> dict:
        """采集完整彩色+点云帧；磁盘只保存彩色图，完整帧留给下一步解算。"""
        # 在原稳定等待时段内持续消费原始彩色/深度帧，避免长时间空闲后
        # 第一次 capture() 取到 OpenNI2 队列中的历史帧。
        started = time.perf_counter()
        delay_s = float(self.config.http.get("snapshot_delay_s", 5.0))
        discarded_frames = 0
        if delay_s > 0.0:
            log.info("快照: 在 %.1fs 稳定期内主动丢弃旧帧", delay_s)
            discarded_frames = self.camera.discard_stale_frames(delay_s)
            log.info("快照: 已丢弃原始彩色/深度帧对 %d 组",
                     discarded_frames)
        frame = self.camera.capture_3d()
        color = np.asarray(frame.color)
        cloud = np.asarray(frame.point_cloud)
        log.info("快照采集: 彩色 %dx%d 点云 %s 耗时 %.1fs",
                 color.shape[1], color.shape[0], cloud.shape,
                 time.perf_counter() - started)
        saved = self.artifacts.save_http_snapshot(frame)
        ttl = float(self.config.http.get("snapshot_cache_ttl_s", 300.0))
        with self._http_snapshot_lock:
            previous_timer = self._http_snapshot_timer
            if previous_timer is not None:
                previous_timer.cancel()
            self._http_snapshot_frame = frame
            captured_at = time.monotonic()
            self._http_snapshot_monotonic = captured_at
            timer = threading.Timer(
                ttl, self._expire_http_snapshot, args=(captured_at,))
            timer.daemon = True
            self._http_snapshot_timer = timer
            timer.start()
        log.info("快照完成: taskId=%s 保存=%s 缓存TTL=%.0fs",
                 payload.get("taskId") or "",
                 bool(saved.get("imagePath")),
                 ttl)
        return json_ready({
            "ok": True,
            "taskType": "http_snapshot_result",
            "taskId": str(payload.get("taskId") or ""),
            "discardedFramePairsBeforeCapture": discarded_frames,
            **saved,
        })

    def _expire_http_snapshot(self, captured_at: float) -> None:
        with self._http_snapshot_lock:
            if self._http_snapshot_monotonic != captured_at:
                return
            self._http_snapshot_frame = None
            self._http_snapshot_monotonic = 0.0
            self._http_snapshot_timer = None
        log.warning("快照缓存已过期（%.0fs）",
                    float(self.config.http.get("snapshot_cache_ttl_s", 300.0)))

    def _clear_http_snapshot(self) -> None:
        with self._http_snapshot_lock:
            timer = self._http_snapshot_timer
            self._http_snapshot_timer = None
            self._http_snapshot_frame = None
            self._http_snapshot_monotonic = 0.0
        if timer is not None:
            timer.cancel()

    def _cached_http_snapshot(self):
        with self._http_snapshot_lock:
            frame = self._http_snapshot_frame
            captured_at = self._http_snapshot_monotonic
        if frame is None:
            raise TaskError(
                "SNAPSHOT_REQUIRED", "请先调用/snapshot获取用于框选的图片")
        ttl = float(self.config.http.get("snapshot_cache_ttl_s", 300.0))
        age = time.monotonic() - captured_at
        if age > ttl:
            self._clear_http_snapshot()
            log.warning("快照已过期: 距今 %.0fs > TTL %.0fs", age, ttl)
            raise TaskError(
                "SNAPSHOT_EXPIRED",
                f"最近快照已超过{ttl:g}秒，请重新调用/snapshot")
        return frame

    def _handle_solve(self, payload: Mapping[str, Any]) -> dict:
        started = time.perf_counter()
        request = SolveTargetRequest.from_mapping(payload)
        base_rect = (
            None if request.base_corners_px is None
            else np.asarray(request.base_corners_px).reshape(-1).tolist())
        log.info("解算请求: taskId=%s code=%s 拍照位=%s target=%s base=%s",
                 request.task_id,
                 request.workpiece_code or "-",
                 np.round(request.capture_tcp_mm_rpy_deg, 3).tolist(),
                 np.asarray(request.target_corners_px).reshape(-1).tolist(),
                 base_rect)
        try:
            # 目标模式、code映射和工具偏移均在每次解算开始时热加载。
            # 先完成code校验，再采集大图/点云，错误请求不会浪费一次拍照。
            tool_offsets = load_tool_offsets(self.tool_offsets_path)
        except (OSError, TypeError, ValueError) as exc:
            log.error("工具偏移配置热加载失败: %s", exc, exc_info=True)
            raise TaskError(
                "TOOL_CONFIG_RELOAD_FAILED",
                f"工具偏移配置读取或校验失败: {exc}") from exc

        selected_tool = None
        if not tool_offsets.use_yolo:
            if request.workpiece_code is None:
                raise TaskError(
                    "INVALID_REQUEST",
                    "关闭YOLO时/get_tcp_pose请求必须提供非空code")
            selected_tool = tool_offsets.code_to_tool.get(
                request.workpiece_code)
            if selected_tool is None:
                available = (
                    ", ".join(sorted(tool_offsets.code_to_tool)) or "无")
                raise TaskError(
                    "TOOL_CODE_UNKNOWN",
                    f"code {request.workpiece_code!r}未配置对应工件；"
                    f"可用code: {available}")
            if not bool(tool_offsets.tools[selected_tool].get(
                    "enabled", True)):
                raise TaskError(
                    "TOOL_DISABLED",
                    f"code {request.workpiece_code!r}对应工件"
                    f"{selected_tool!r}已禁用")
        use_cached_snapshot = bool(
            payload.get("useCachedHttpSnapshot", False))
        live_capture_delay_s = 0.0
        if use_cached_snapshot:
            frame = self._cached_http_snapshot()
        else:
            # live=true路径把原来的纯sleep改为主动消费原始流：等待时间仍
            # 用于机械臂稳定，但同时把OpenNI2积压帧向前推进到最新位置。
            live_capture_delay_s = float(
                self.config.http.get("live_capture_delay_s", 4.0))
            discarded_frames = 0
            if live_capture_delay_s > 0.0:
                log.info(
                    "实时解算: 在 %.1fs 稳定期内主动丢弃旧帧，再拍照检测",
                    live_capture_delay_s)
                discarded_frames = self.camera.discard_stale_frames(
                    live_capture_delay_s)
                log.info("实时解算: 已丢弃原始彩色/深度帧对 %d 组",
                         discarded_frames)
            frame = self.camera.capture_3d()
        if use_cached_snapshot:
            discarded_frames = 0
        camera_model = self.camera.camera_model(frame)

        matching = self.config.matching
        source_width = int(matching.get("source_image_width", 1920))
        source_height = int(matching.get("source_image_height", 1080))
        corners = scale_request_corners(
            request.target_corners_px,
            source_width, source_height,
            camera_model.width, camera_model.height)
        # 基座面板矩形（可选）：提供时目标中心深度以面板平面为准。
        panel_region_px = None
        if request.base_corners_px is not None:
            panel_region_px = scale_request_corners(
                request.base_corners_px,
                source_width, source_height,
                camera_model.width, camera_model.height)
        step = time.perf_counter()
        if tool_offsets.use_yolo:
            selection_mode = "yolo"
            batch = self.detector.process(
                frame, camera_model, request.task_id,
                panel_region_px=panel_region_px)
            log.info("检测: 模式=YOLO 目标数=%d 有效解算=%d 耗时=%.1fms",
                     len(batch.observations),
                     len(batch.valid_observations()),
                     (time.perf_counter() - step) * 1000.0)
            distance_scale = 0.5 * (
                camera_model.width / float(source_width) +
                camera_model.height / float(source_height))
            match = match_observation(
                batch.observations,
                corners,
                prefer_inside=bool(
                    matching.get("prefer_center_inside_polygon", True)),
                max_distance_px=float(
                    matching.get("max_match_distance_px", 300.0)) *
                distance_scale,
            )
            observation = match.observation
        else:
            request_center = np.mean(corners, axis=0)
            circle_settings = dict(
                matching.get("circle_refinement") or {})
            circle_enabled = bool(circle_settings.get("enabled", False))
            circle_match = None
            expected_color = "auto"
            if circle_enabled:
                distance_scale = 0.5 * (
                    camera_model.width / float(source_width) +
                    camera_model.height / float(source_height))
                expected_color = str(
                    tool_offsets.tools[selected_tool].get(
                        "target_color", "auto"))
                circle_match = find_nearest_circle_center(
                    np.asarray(frame.color), corners, circle_settings,
                    max_center_distance_px=float(circle_settings.get(
                        "max_center_distance_px", 250.0)) * distance_scale,
                    expected_color=expected_color)
                if circle_match is None and not bool(circle_settings.get(
                        "fallback_to_box_center", False)):
                    raise TaskError(
                        "NO_CIRCLE_TARGET",
                        "目标框附近没有找到满足评分和距离限制的圆，"
                        "本次解算已停止")

            if circle_match is None:
                selection_mode = "box_center"
                selected_center = request_center.copy()
                refined_radius = None
                refinement_metrics = None
                if circle_enabled:
                    log.warning(
                        "圆心拟合失败，按配置回退到原框中心: code=%s 工件=%s",
                        request.workpiece_code, selected_tool)
            else:
                selection_mode = "nearest_circle_center"
                selected_center = circle_match.center_px
                refined_radius = circle_match.radius_px
                log.info(
                    "颜色外圆拟合: code=%s 工件=%s 期望颜色=%s "
                    "识别颜色=%s 覆盖率=%.3f 融合=%s "
                    "边缘圆心=%s 最终圆心=%s 半径=%.1fpx",
                    request.workpiece_code, selected_tool, expected_color,
                    circle_match.color_name or "无/纯边缘",
                    circle_match.color_coverage,
                    circle_match.color_fusion_applied,
                    np.round(circle_match.edge_center_px, 2).tolist(),
                    np.round(circle_match.center_px, 2).tolist(),
                    circle_match.radius_px)
                refinement_metrics = {
                    "circle_score": circle_match.score,
                    "circle_edge_support": circle_match.edge_support,
                    "circle_radius_px": circle_match.radius_px,
                    "circle_edge_center_px": (
                        circle_match.edge_center_px.tolist()),
                    "circle_distance_to_request_center_px": (
                        circle_match.distance_to_request_center_px),
                    "circle_search_roi_xyxy_px": list(
                        circle_match.search_roi_xyxy_px),
                    "circle_candidate_count": circle_match.candidate_count,
                    "circle_accepted_candidate_count": (
                        circle_match.accepted_candidate_count),
                    "circle_selected_cluster_candidate_count": (
                        circle_match.selected_cluster_candidate_count),
                    "circle_color_name": circle_match.color_name,
                    "circle_color_score": circle_match.color_score,
                    "circle_color_coverage": circle_match.color_coverage,
                    "circle_color_center_px": (
                        None if circle_match.color_center_px is None
                        else circle_match.color_center_px.tolist()),
                    "circle_color_fusion_applied": (
                        circle_match.color_fusion_applied),
                    "circle_selection_rule": (
                        "expected_color_plus_nearest_cluster_then_"
                        "largest_valid_circle"),
                }
            batch = self.detector.process_box_center(
                frame, camera_model, request.task_id,
                target_region_px=corners,
                tool_class_name=selected_tool,
                panel_region_px=panel_region_px,
                refined_center_px=(
                    selected_center if circle_match is not None else None),
                refined_radius_px=refined_radius,
                refinement_metrics=refinement_metrics)
            observation = batch.observations[0]
            polygon = np.asarray(
                corners, dtype=np.float32).reshape(-1, 1, 2)
            center_inside = cv2.pointPolygonTest(
                polygon,
                (float(selected_center[0]), float(selected_center[1])),
                False) >= 0.0
            center_distance = float(np.linalg.norm(
                selected_center - request_center))
            match = MatchResult(
                observation=observation,
                request_center_px=request_center,
                detection_center_px=selected_center.copy(),
                distance_px=center_distance,
                center_inside_polygon=center_inside,
                ordered_corners_px=corners,
            )
            log.info(
                "检测: 模式=%s code=%s 对应工件=%s "
                "原框中心=%s 选中中心=%s 距离=%.1fpx "
                "有效解算=%d "
                "耗时=%.1fms（未执行YOLO推理）",
                selection_mode,
                request.workpiece_code,
                selected_tool,
                np.round(request_center, 2).tolist(),
                np.round(selected_center, 2).tolist(),
                center_distance,
                len(batch.valid_observations()),
                (time.perf_counter() - step) * 1000.0)
        for error_text in batch.errors:
            log.warning("检测告警: %s", error_text)
        if not observation.valid:
            log.error("解算失败: %s %s",
                      observation.error_code or "TARGET_GEOMETRY_FAILED",
                      observation.error_message or "三维几何解算失败")
            raise TaskError(
                observation.error_code or "TARGET_GEOMETRY_FAILED",
                observation.error_message or "目标三维几何解算失败")
        log.info("目标选择: 模式=%s 类别=%s 置信度=%.2f 距离=%.1fpx "
                 "中心在框内=%s",
                 selection_mode,
                 observation.class_name,
                 float(observation.confidence),
                 match.distance_px,
                 match.center_inside_polygon)
        pose = self.pose_solver.solve(
            observation, request.capture_tcp_mm_rpy_deg,
            tools_config=tool_offsets.tools)
        quality = dict(observation.quality_metrics or {})
        max_rms = float(self.config.plane.get("max_rms_mm", 0.0))
        rms = float(quality.get("plane_rms_mm", float("nan")))
        if max_rms > 0.0 and np.isfinite(rms) and rms > max_rms:
            log.error("平面RMS超标: RMS=%.3fmm > 阈值%.3fmm (深度源=%s)",
                      rms, max_rms, quality.get("center_depth_source"))
            raise TaskError(
                "PLANE_RMS_TOO_HIGH",
                f"平面拟合RMS={rms:.3f}mm超过阈值{max_rms:.3f}mm")
        depth_source = quality.get("center_depth_source")
        plane_dist = quality.get(
            "camera_to_plane_perpendicular_distance_m")
        ray_dist = quality.get("center_ray_plane_distance_m")
        log.info("几何质量: 深度源=%s 点数=%s 内点率=%s RMS=%.3fmm "
                 "相机到平面=%.1fmm 射线距离=%.1fmm",
                 depth_source,
                 quality.get("point_count"),
                 quality.get("plane_inlier_ratio"),
                 rms,
                 (float(plane_dist) * 1000.0
                  if plane_dist is not None else float("nan")),
                 (float(ray_dist) * 1000.0
                  if ray_dist is not None else float("nan")))
        log.info("v3.5.3求解: 目标模式=%s code=%s 类别=%s 工具=%s "
                 "光心参考距离=%.1fmm "
                 "工具偏移xyz=%s rpy=%s 配置sha256=%s 标准TCP=%s 最终TCP=%s",
                 selection_mode,
                 request.workpiece_code or "-",
                 observation.class_name,
                 pose["toolId"],
                 float(pose["standoffMm"]),
                 np.round(pose["standardToToolXyzMm"], 4).tolist(),
                 np.round(pose["standardToToolRpyDeg"], 4).tolist(),
                 tool_offsets.sha256[:12],
                 np.round(pose["standardTcpMmRpyDeg"], 3).tolist(),
                 np.round(pose["targetTcpMmRpyDeg"], 3).tolist())

        result = json_ready({
            "ok": True,
            "taskType": "solve_target_tcp_result",
            "taskId": request.task_id,
            "posePipelineVersion": 2,
            "alignmentMode": pose["alignmentMode"],
            "selectedClass": observation.class_name,
            "selectedClassId": observation.class_id,
            "selectedConfidence": observation.confidence,
            "toolId": pose["toolId"],
            "targetSelection": {
                "mode": selection_mode,
                "useYolo": tool_offsets.use_yolo,
                "requestedCode": request.workpiece_code,
                "configuredTool": (
                    selected_tool
                    if not tool_offsets.use_yolo else None),
                "centerSource": (
                    "yolo_detection_center"
                    if tool_offsets.use_yolo else (
                        "fitted_circle_center"
                        if circle_match is not None
                        else "request_box_center")),
            },
            "toolOffsetsHotReload": {
                "sourcePath": str(tool_offsets.source_path),
                "sha256": tool_offsets.sha256,
                "modifiedAtUnixS": tool_offsets.modified_at_unix_s,
                "requestedCode": request.workpiece_code,
                "selectedClass": observation.class_name,
                "configuredCode": (
                    tool_offsets.tools[observation.class_name].get("code")
                    if not tool_offsets.use_yolo else None),
                "standardToTool": {
                    "xyzMm": pose["standardToToolXyzMm"],
                    "rpyDeg": pose["standardToToolRpyDeg"],
                },
            },
            "targetTcpMmRpyDeg": pose["targetTcpMmRpyDeg"],
            "referenceFrame": pose["referenceFrame"],
            "poseReference": pose["poseReference"],
            "standoffMm": pose["standoffMm"],
            "liveCaptureDelaySApplied": live_capture_delay_s,
            "discardedFramePairsBeforeCapture": discarded_frames,
            "cameraReference": {
                "definition": "camera_origin_on_target_normal_standoff_only",
                "standoffMm": pose["standoffMm"],
                "tcpCorrectionMmRpyDeg": pose[
                    "tcpCorrectionMmRpyDeg"],
                "toolOffsetDefinition": pose["toolOffsetDefinition"],
                "inverseHandeyeTcpMmRpyDeg": pose[
                    "inverseHandeyeTcpMmRpyDeg"],
                "standardTcpMmRpyDeg": pose["standardTcpMmRpyDeg"],
                "toolTargetTcpMmRpyDeg": pose["toolTargetTcpMmRpyDeg"],
                "TBaseCameraReference": pose["TBaseCameraReference"],
                "TCameraTcpInverseHandeye": pose[
                    "TCameraTcpInverseHandeye"],
                "TBaseTcpInverseHandeye": pose[
                    "TBaseTcpInverseHandeye"],
                "TBaseTcpStandard": pose["TBaseTcpStandard"],
                "TStandardToolEffective": pose[
                    "TStandardToolEffective"],
                "TBaseTcpTarget": pose["TBaseTcpTarget"],
                "TBaseCameraTarget": pose["TBaseCameraTarget"],
            },
            "targetMatch": {
                "requestCornersPx": match.ordered_corners_px,
                "requestCenterPx": match.request_center_px,
                "detectionBboxXyxyPx": observation.bbox_2d,
                "detectionCenterPx": match.detection_center_px,
                "distancePx": match.distance_px,
                "centerInsidePolygon": match.center_inside_polygon,
            },
            "geometry": {
                "targetCenterBaseMm": pose["targetCenterBaseMm"],
                "surfaceNormalBase": pose["surfaceNormalBase"],
                "approachDirectionBase": pose["approachDirectionBase"],
                "centerDepthSource": quality.get("center_depth_source"),
                "planePointCount": quality.get("point_count"),
                "planeInlierRatio": quality.get("plane_inlier_ratio"),
                "planeRmsMm": quality.get("plane_rms_mm"),
                "cameraToPlanePerpendicularDistanceM": quality.get(
                    "camera_to_plane_perpendicular_distance_m"),
                "centerRayPlaneDistanceM": quality.get(
                    "center_ray_plane_distance_m"),
            },
        })

        if bool(self.config.system.get("save_point_cloud", False)):
            from ply_io import write_ply_xyz
            output_dir = self.artifacts.task_output_dir(request.task_id)
            ply_path = output_dir / "capture_point_cloud_mm.ply"
            write_ply_xyz(
                ply_path,
                np.asarray(frame.point_cloud)[..., :3].reshape(-1, 3))
            result["pointCloudPath"] = str(ply_path)
        saved = self.artifacts.save_result(
            request.task_id, result, batch.debug_overlay)
        result.update(saved)
        if use_cached_snapshot:
            # 成功响应生成后释放大图和点云；下一轮必须重新/snapshot。
            self._clear_http_snapshot()
        return result

    def _handle_impl(self, payload: Mapping[str, Any]) -> dict:
        task_type = str(payload.get("taskType") or "").strip()
        if task_type == "heartbeat":
            return {
                "ok": True, "taskType": "heartbeat_result",
                "taskId": str(payload.get("taskId") or ""),
                "timestamp": time.time(),
            }
        if task_type == "capture_annotation_image":
            return self._handle_annotation(payload)
        if task_type == "http_snapshot":
            return self._handle_http_snapshot(payload)
        if task_type == "solve_target_tcp":
            return self._handle_solve(payload)
        raise TaskError("UNSUPPORTED_TASK", f"不支持任务类型{task_type!r}")

    def handle_task(self, payload: Mapping[str, Any],
                    timeout_s: Optional[float] = None) -> dict:
        if not isinstance(payload, Mapping):
            return {
                "ok": False, "taskId": "", "errorCode": "INVALID_REQUEST",
                "message": "任务必须是JSON对象",
            }
        with self._state_lock:
            if self._closed:
                return {
                    "ok": False,
                    "taskId": str(payload.get("taskId") or ""),
                    "errorCode": "SERVICE_CLOSED",
                    "message": "视觉解算服务已关闭",
                }
        task_id = str(payload.get("taskId") or "")
        task_type = str(payload.get("taskType") or "")
        started = time.perf_counter()
        log.info("收到请求: taskType=%s taskId=%s", task_type or "?", task_id or "-")
        timeout = float(timeout_s or self.config.system.get("task_timeout_s", 90.0))
        future = self._executor.submit(self._handle_impl, dict(payload))
        try:
            result = future.result(timeout=timeout)
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            if bool(result.get("ok", False)):
                log.info("请求成功: taskType=%s taskId=%s 耗时=%.1fms",
                         task_type, task_id, elapsed_ms)
            else:
                log.error("请求失败: taskType=%s taskId=%s code=%s msg=%s "
                          "耗时=%.1fms", task_type, task_id,
                          result.get("errorCode"), result.get("message"),
                          elapsed_ms)
            return result
        except TimeoutError:
            log.error("任务超时: taskId=%s taskType=%s 超时=%.0fs",
                      task_id, task_type, timeout)
            return {
                "ok": False, "taskId": task_id,
                "errorCode": "TASK_TIMEOUT",
                "message": f"任务超过{timeout:g}秒未完成",
            }
        except TaskError as exc:
            log.error("任务失败: taskId=%s taskType=%s code=%s msg=%s",
                      task_id, task_type, exc.code, exc.message)
            return {
                "ok": False, "taskId": task_id,
                "errorCode": exc.code, "message": exc.message,
            }
        except Exception as exc:
            log.error("任务内部异常: taskId=%s taskType=%s",
                      task_id, task_type, exc_info=True)
            return {
                "ok": False, "taskId": task_id,
                "errorCode": "INTERNAL_ERROR", "message": str(exc),
            }

    def close(self) -> None:
        with self._state_lock:
            if self._closed:
                return
            self._closed = True
        log.info("服务关闭中……")
        try:
            self._clear_http_snapshot()
            self._executor.submit(self.camera.disconnect).result(timeout=30.0)
        finally:
            self._executor.shutdown(wait=True, cancel_futures=False)
        log.info("服务已关闭")

    def __enter__(self) -> "VisionToolTcpSolver":
        return self

    def __exit__(self, _exc_type, _exc, _traceback) -> None:
        self.close()
