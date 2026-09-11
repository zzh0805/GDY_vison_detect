# -*- coding: utf-8 -*-
"""对LabelMe图片离线回归无YOLO最大有效外圆算法。"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from vision_solver.circle_center_refiner import (  # noqa: E402
    find_nearest_circle_center,
)


def _rectangle_corners(points: list[list[float]]) -> np.ndarray:
    array = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    x1, y1 = np.min(array, axis=0)
    x2, y2 = np.max(array, axis=0)
    return np.array([
        [x1, y1], [x2, y1], [x2, y2], [x1, y2],
    ], dtype=np.float64)


def evaluate_image(image_path: Path, label: str, settings: dict,
                   output_dir: Path,
                   expected_color: str | None = None) -> dict:
    annotation_path = image_path.with_suffix(".json")
    annotation = json.loads(annotation_path.read_text(encoding="utf-8"))
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"无法读取图片: {image_path}")
    overlay = image.copy()
    targets = [
        shape for shape in annotation.get("shapes", [])
        if str(shape.get("label")) == label
    ]
    results = []
    for index, shape in enumerate(targets, start=1):
        corners = _rectangle_corners(shape["points"])
        request_center = np.mean(corners, axis=0)
        match = find_nearest_circle_center(
            image, corners, settings, expected_color=expected_color)
        cv2.polylines(
            overlay, [np.rint(corners).astype(np.int32)], True,
            (0, 255, 255), 2, cv2.LINE_AA)
        cv2.drawMarker(
            overlay, tuple(np.rint(request_center).astype(int)),
            (0, 255, 255), cv2.MARKER_CROSS, 16, 2, cv2.LINE_AA)
        if match is None:
            results.append({"index": index, "matched": False})
            cv2.putText(
                overlay, f"#{index} NO CIRCLE",
                tuple(np.rint(corners[0]).astype(int)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2,
                cv2.LINE_AA)
            continue
        fitted_center = np.rint(match.center_px).astype(int)
        edge_center = np.rint(match.edge_center_px).astype(int)
        cv2.circle(
            overlay, tuple(fitted_center), int(round(match.radius_px)),
            (0, 255, 0), 3, cv2.LINE_AA)
        cv2.drawMarker(
            overlay, tuple(fitted_center), (0, 255, 0),
            cv2.MARKER_CROSS, 20, 2, cv2.LINE_AA)
        if match.color_fusion_applied:
            cv2.drawMarker(
                overlay, tuple(edge_center), (255, 0, 255),
                cv2.MARKER_TILTED_CROSS, 16, 2, cv2.LINE_AA)
        cv2.arrowedLine(
            overlay, tuple(np.rint(request_center).astype(int)),
            tuple(fitted_center), (255, 0, 0), 2, cv2.LINE_AA,
            tipLength=0.25)
        cv2.putText(
            overlay,
            f"#{index} {match.color_name or 'edge'} r={match.radius_px:.1f}px",
            (int(corners[0, 0]), max(24, int(corners[0, 1]) - 8)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2,
            cv2.LINE_AA)
        results.append({
            "index": index,
            "matched": True,
            "request_center_px": request_center.tolist(),
            "fitted_center_px": match.center_px.tolist(),
            "edge_center_px": match.edge_center_px.tolist(),
            "radius_px": match.radius_px,
            "center_shift_px": (
                match.center_px - request_center).tolist(),
            "edge_support": match.edge_support,
            "candidate_count": match.candidate_count,
            "accepted_candidate_count": match.accepted_candidate_count,
            "selected_cluster_candidate_count": (
                match.selected_cluster_candidate_count),
            "color_name": match.color_name,
            "color_score": match.color_score,
            "color_coverage": match.color_coverage,
            "color_center_px": (
                None if match.color_center_px is None
                else match.color_center_px.tolist()),
            "color_fusion_applied": match.color_fusion_applied,
        })
    image_output_dir = output_dir / image_path.stem
    image_output_dir.mkdir(parents=True, exist_ok=True)
    overlay_path = image_output_dir / "color_outer_circle_overlay.jpg"
    result_path = image_output_dir / "color_outer_circle_result.json"
    cv2.imwrite(str(overlay_path), overlay)
    result = {
        "image": str(image_path),
        "label": label,
        "target_count": len(targets),
        "matched_count": sum(bool(item["matched"]) for item in results),
        "results": results,
        "overlay": str(overlay_path),
    }
    result_path.write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("images", nargs="+", type=Path)
    parser.add_argument("--label", default="1")
    parser.add_argument(
        "--expected-color", choices=("auto", "red", "green", "black"),
        default="auto")
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    workflow = yaml.safe_load(
        (PROJECT_ROOT / "config" / "workflow.yaml").read_text(
            encoding="utf-8"))
    settings = dict(
        workflow["target_matching"]["circle_refinement"])
    summaries = [
        evaluate_image(path.resolve(), args.label, settings,
                       args.output_dir.resolve(),
                       None if args.expected_color == "auto"
                       else args.expected_color)
        for path in args.images
    ]
    total = sum(item["target_count"] for item in summaries)
    matched = sum(item["matched_count"] for item in summaries)
    print(json.dumps({
        "images": len(summaries),
        "targets": total,
        "matched": matched,
        "details": summaries,
    }, ensure_ascii=False, indent=2))
    return 0 if matched == total else 1


if __name__ == "__main__":
    raise SystemExit(main())
