from dataclasses import dataclass
import math
import struct
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class DynamicDiagnostics:
    enabled: bool
    pose_filter_enabled: bool
    map_filter_enabled: bool
    static_points: int
    unknown_points: int
    candidate_points: int
    confirmed_points: int
    dynamic_ratio: float
    effective_points: int
    information_ratio: float
    baseline_rank: int
    weighted_rank: int
    fallback_required: bool
    fallback_used: bool
    map_update_skipped: bool
    promoted_points: int
    temporal_voxels: int
    block_start: int
    block_end: int


class _CdrCursor:
    def __init__(self, payload: bytes) -> None:
        self.payload = payload
        self.offset = 4

    def align(self, size: int) -> None:
        self.offset += (-(self.offset - 4)) % size

    def read(self, fmt: str, alignment: int) -> int | float:
        self.align(alignment)
        size = struct.calcsize("<" + fmt)
        if self.offset + size > len(self.payload):
            raise ValueError("truncated CDR payload")
        value = struct.unpack_from("<" + fmt, self.payload, self.offset)[0]
        self.offset += size
        return value

    def read_bool(self) -> bool:
        return bool(self.read("B", 1))

    def skip_string(self) -> None:
        length = int(self.read("I", 4))
        if length < 1 or self.offset + length > len(self.payload):
            raise ValueError("invalid CDR string length")
        self.offset += length


def parse_dynamic_diagnostics_prefix(
    payload: bytes,
) -> DynamicDiagnostics | None:
    if len(payload) < 4 or payload[:2] not in (b"\x00\x01", b"\x00\x00"):
        return None
    cursor = _CdrCursor(payload)
    try:
        cursor.read("i", 4)
        cursor.read("I", 4)
        cursor.skip_string()
        cursor.read("Q", 8)
        cursor.read("d", 8)
        cursor.read("d", 8)
        common_counts = [int(cursor.read("I", 4)) for _ in range(5)]
        cursor.read("d", 8)
        block_start = cursor.offset
        raw_flags = payload[block_start:block_start + 3]
        if len(raw_flags) != 3 or any(value not in (0, 1) for value in raw_flags):
            return None
        enabled = cursor.read_bool()
        pose_enabled = cursor.read_bool()
        map_enabled = cursor.read_bool()
        static_points = int(cursor.read("I", 4))
        unknown_points = int(cursor.read("I", 4))
        candidate_points = int(cursor.read("I", 4))
        confirmed_points = int(cursor.read("I", 4))
        dynamic_ratio = float(cursor.read("d", 8))
        effective_points = int(cursor.read("I", 4))
        information_ratio = float(cursor.read("d", 8))
        baseline_rank = int(cursor.read("I", 4))
        weighted_rank = int(cursor.read("I", 4))
        fallback_required = cursor.read_bool()
        fallback_used = cursor.read_bool()
        map_update_skipped = cursor.read_bool()
        promoted_points = int(cursor.read("I", 4))
        temporal_voxels = int(cursor.read("Q", 8))
    except (ValueError, struct.error):
        return None

    downsampled_points = common_counts[2]
    classified_points = (
        static_points + unknown_points + candidate_points + confirmed_points
    )
    plausible = (
        classified_points <= max(downsampled_points * 2, downsampled_points + 100)
        and 0.0 <= dynamic_ratio <= 1.0
        and math.isfinite(information_ratio)
        and 0.0 <= information_ratio <= 2.0
        and 0 <= baseline_rank <= 18
        and 0 <= weighted_rank <= 18
    )
    if not plausible:
        return None
    return DynamicDiagnostics(
        enabled=enabled,
        pose_filter_enabled=pose_enabled,
        map_filter_enabled=map_enabled,
        static_points=static_points,
        unknown_points=unknown_points,
        candidate_points=candidate_points,
        confirmed_points=confirmed_points,
        dynamic_ratio=dynamic_ratio,
        effective_points=effective_points,
        information_ratio=information_ratio,
        baseline_rank=baseline_rank,
        weighted_rank=weighted_rank,
        fallback_required=fallback_required,
        fallback_used=fallback_used,
        map_update_skipped=map_update_skipped,
        promoted_points=promoted_points,
        temporal_voxels=temporal_voxels,
        block_start=block_start,
        block_end=cursor.offset,
    )


def strip_dynamic_diagnostics_block(
    payload: bytes,
    diagnostics: DynamicDiagnostics,
) -> bytes:
    return payload[:diagnostics.block_start] + payload[diagnostics.block_end:]


def summarize_dynamic_diagnostics(
    samples: Iterable[DynamicDiagnostics],
) -> dict[str, object]:
    values = list(samples)
    if not values:
        return {"available": False, "sample_count": 0}

    def array(field: str) -> np.ndarray:
        return np.asarray([getattr(value, field) for value in values], dtype=float)

    ratio = array("dynamic_ratio")
    information = array("information_ratio")
    confirmed = array("confirmed_points")
    return {
        "available": True,
        "sample_count": len(values),
        "enabled_fraction": float(np.mean(array("enabled"))),
        "dynamic_ratio_mean": float(np.mean(ratio)),
        "dynamic_ratio_p95": float(np.percentile(ratio, 95)),
        "dynamic_ratio_max": float(np.max(ratio)),
        "confirmed_points_max": int(np.max(confirmed)),
        "candidate_points_mean": float(np.mean(array("candidate_points"))),
        "candidate_points_max": int(np.max(array("candidate_points"))),
        "information_ratio_min": float(np.min(information)),
        "information_ratio_p05": float(np.percentile(information, 5)),
        "baseline_ranks": sorted(set(int(value.baseline_rank) for value in values)),
        "weighted_ranks": sorted(set(int(value.weighted_rank) for value in values)),
        "fallback_required_count": sum(value.fallback_required for value in values),
        "fallback_used_count": sum(value.fallback_used for value in values),
        "map_update_skipped_count": sum(value.map_update_skipped for value in values),
        "promoted_points_total": sum(value.promoted_points for value in values),
        "temporal_voxels_max": max(value.temporal_voxels for value in values),
    }
