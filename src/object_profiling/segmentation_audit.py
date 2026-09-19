from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from pathlib import Path

import cv2
import numpy as np

from .checkpoint import MAXIMUM_BOX, MINIMUM_BOX, NOMINAL_BOX
from .config import AppConfig
from .environment import BoxSpec, ProfilingEnvironment
from .evaluation import GroundTruthRenderer, SegmentationMetrics, evaluate_segmentation
from .perception import segment_foreground
from .scanning import run_fixed_scan
from .sensors import RGBDSensor


MIN_INTERSECTION_OVER_UNION = 0.95
MAX_FALSE_TERMINAL_PIXELS = 200


@dataclass(frozen=True)
class SegmentationRecord:
    object_id: str
    pose_name: str
    metrics: SegmentationMetrics
    interior_pixels: int
    touches_border: bool
    rejection_reason: str | None
    valid: bool


def audit_box_segmentation(
    box_spec: BoxSpec,
    *,
    config: AppConfig | None = None,
    artifact_directory: Path | None = None,
) -> list[SegmentationRecord]:
    config = config or AppConfig()
    environment = ProfilingEnvironment.create(box_spec, config, attach_box=False)
    sensor = RGBDSensor(environment)
    truth_renderer = GroundTruthRenderer(environment)
    records: list[SegmentationRecord] = []

    def on_capture(pose, observation, backgrounds) -> None:
        truth = truth_renderer.masks()
        segmentation = segment_foreground(
            observation.depth_m, backgrounds.depth_for(pose.name), config.sensor
        )
        metrics = evaluate_segmentation(segmentation.mask, truth)
        records.append(
            SegmentationRecord(
                object_id=box_spec.object_id,
                pose_name=pose.name,
                metrics=metrics,
                interior_pixels=int(np.count_nonzero(segmentation.interior_mask)),
                touches_border=segmentation.touches_border,
                rejection_reason=segmentation.reason.value if segmentation.reason else None,
                valid=bool(
                    segmentation.reason is None
                    and metrics.intersection_over_union >= MIN_INTERSECTION_OVER_UNION
                    and metrics.false_terminal_pixels <= MAX_FALSE_TERMINAL_PIXELS
                ),
            )
        )
        if artifact_directory is not None:
            directory = artifact_directory / box_spec.object_id
            directory.mkdir(parents=True, exist_ok=True)
            name = pose.name.lower()
            cv2.imwrite(str(directory / f"{name}-observable-mask.png"), segmentation.mask.astype(np.uint8) * 255)
            cv2.imwrite(str(directory / f"{name}-truth-mask.png"), truth.box.astype(np.uint8) * 255)
            cv2.imwrite(
                str(directory / f"{name}-disagreement.png"),
                np.stack(
                    [
                        (truth.box & ~segmentation.mask).astype(np.uint8) * 255,
                        (truth.box & segmentation.mask).astype(np.uint8) * 255,
                        (segmentation.mask & ~truth.box).astype(np.uint8) * 255,
                    ],
                    axis=-1,
                ),
            )

    try:
        run_fixed_scan(environment, sensor, on_capture=on_capture)
    finally:
        sensor.close()
        truth_renderer.close()
    return records


def audit_segmentation_suite(*, artifact_directory: Path | None = None) -> dict:
    records = [
        record
        for box_spec in (MINIMUM_BOX, NOMINAL_BOX, MAXIMUM_BOX)
        for record in audit_box_segmentation(box_spec, artifact_directory=artifact_directory)
    ]
    return {
        "schema_version": 1,
        "checkpoint": "observable_segmentation",
        "valid": all(record.valid for record in records),
        "thresholds": {
            "minimum_intersection_over_union": MIN_INTERSECTION_OVER_UNION,
            "maximum_false_terminal_pixels": MAX_FALSE_TERMINAL_PIXELS,
        },
        "worst_intersection_over_union": min(
            record.metrics.intersection_over_union for record in records
        ),
        "worst_precision": min(record.metrics.precision for record in records),
        "worst_recall": min(record.metrics.recall for record in records),
        "worst_false_terminal_pixels": max(record.metrics.false_terminal_pixels for record in records),
        "worst_missed_box_pixels": max(record.metrics.missed_box_pixels for record in records),
        "records": [asdict(record) for record in records],
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Mide la segmentacion observable contra la mascara ground truth."
    )
    parser.add_argument("--artifacts", type=Path, help="Directorio opcional para mascaras y desacuerdos.")
    parser.add_argument("--output", type=Path, help="Ruta opcional para el informe JSON.")
    args = parser.parse_args()

    report = audit_segmentation_suite(artifact_directory=args.artifacts)
    payload = json.dumps(report, indent=2, sort_keys=True)
    print(payload)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload + "\n", encoding="utf-8")
    return 0 if report["valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
