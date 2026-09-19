from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .config import AppConfig
from .contracts import Dimensions3D, RejectionReason, ScanView


@dataclass(frozen=True)
class GeometricEstimate:
    dimensions: Dimensions3D
    uncertainty: Dimensions3D
    confidence: float
    residual_m: float
    view_height_delta_m: float


def _robust_dimensions(points: np.ndarray, low: float, high: float) -> tuple[Dimensions3D, float]:
    if points.shape[0] < 4:
        raise ValueError("not enough points")
    xy = points[:, :2].astype(np.float32)
    hull = cv2.convexHull(xy)
    rectangle = cv2.minAreaRect(hull)
    angle = np.deg2rad(rectangle[2])
    axis_x = np.asarray([np.cos(angle), np.sin(angle)])
    axis_y = np.asarray([-np.sin(angle), np.cos(angle)])
    projected_x, projected_y = xy @ axis_x, xy @ axis_y
    x0, x1 = np.percentile(projected_x, [low, high])
    y0, y1 = np.percentile(projected_y, [low, high])
    z0, z1 = np.percentile(points[:, 2], [low, high])
    horizontal = sorted([float(x1 - x0), float(y1 - y0)], reverse=True)
    dimensions = Dimensions3D(horizontal[0], horizontal[1], float(z1 - z0))
    dx = np.minimum(np.abs(projected_x - x0), np.abs(projected_x - x1))
    dy = np.minimum(np.abs(projected_y - y0), np.abs(projected_y - y1))
    dz = np.minimum(np.abs(points[:, 2] - z0), np.abs(points[:, 2] - z1))
    residual = float(np.median(np.minimum(np.minimum(dx, dy), dz)))
    return dimensions, residual


def _bootstrap_uncertainty(points: np.ndarray, config: AppConfig, seed: int) -> Dimensions3D:
    estimator = config.estimator
    rng = np.random.default_rng(seed)
    capped = points
    if points.shape[0] > estimator.bootstrap_point_cap:
        capped = points[rng.choice(points.shape[0], estimator.bootstrap_point_cap, replace=False)]
    estimates = []
    sample_size = max(500, int(capped.shape[0] * 0.75))
    for _ in range(estimator.bootstrap_samples):
        sample = capped[rng.choice(capped.shape[0], sample_size, replace=True)]
        dimensions, _ = _robust_dimensions(sample, estimator.percentile_low, estimator.percentile_high)
        estimates.append(dimensions.as_array())
    values = np.asarray(estimates)
    spreads = np.percentile(values, 97.5, axis=0) - np.percentile(values, 2.5, axis=0)
    half_width = np.maximum(spreads / 2.0, 0.0005)
    return Dimensions3D(float(half_width[0]), float(half_width[1]), float(half_width[2]))


def estimate_geometry(views: list[ScanView], config: AppConfig, seed: int) -> tuple[GeometricEstimate | None, RejectionReason | None]:
    if len(views) < 2:
        return None, RejectionReason.INSUFFICIENT_VIEWS
    points = np.concatenate([view.points_tool_m for view in views], axis=0)
    if points.shape[0] < config.estimator.minimum_points:
        return None, RejectionReason.INSUFFICIENT_FOREGROUND
    dimensions, residual = _robust_dimensions(points, config.estimator.percentile_low, config.estimator.percentile_high)
    uncertainty = _bootstrap_uncertainty(points, config, seed)
    heights = []
    for view in views:
        if view.points_tool_m.shape[0] >= 50:
            z0, z1 = np.percentile(view.points_tool_m[:, 2], [config.estimator.percentile_low, config.estimator.percentile_high])
            heights.append(float(z1 - z0))
    height_delta = max(heights) - min(heights) if len(heights) >= 2 else float("inf")
    ranges = config.box_range
    values = dimensions.as_array()
    minimum = np.asarray([ranges.length_m[0], ranges.width_m[0], ranges.height_m[0]]) - 0.02
    maximum = np.asarray([ranges.length_m[1], ranges.width_m[1], ranges.height_m[1]]) + 0.02
    if np.any(values < minimum) or np.any(values > maximum):
        return None, RejectionReason.OUT_OF_RANGE
    if height_delta > config.estimator.max_view_height_delta_m:
        return None, RejectionReason.REGISTRATION_INCONSISTENT
    if np.any(uncertainty.as_array() > config.estimator.max_uncertainty_m):
        return None, RejectionReason.HIGH_UNCERTAINTY
    point_score = min(1.0, points.shape[0] / 20_000.0)
    residual_score = float(np.exp(-residual / config.estimator.cuboid_residual_scale_m))
    uncertainty_score = float(np.exp(-np.mean(uncertainty.as_array()) / config.estimator.max_uncertainty_m))
    consistency_score = float(np.exp(-height_delta / config.estimator.max_view_height_delta_m))
    confidence = float(np.clip(0.2 * point_score + 0.3 * residual_score + 0.3 * uncertainty_score + 0.2 * consistency_score, 0.0, 1.0))
    return GeometricEstimate(dimensions, uncertainty, confidence, residual, height_delta), None
