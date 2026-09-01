from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from .bag_reader import BagData, read_bag, summarize_bag_streams
from .discovery import CaseLayout
from .trajectory import AlignmentResult, align_trajectories, detect_airborne_from_height
from .ulog_reader import analyze_ulog


def _landed_window(landed: list[tuple[float, bool]]) -> tuple[float, float]:
    if not landed:
        raise ValueError("no PX4 landed state is available")
    values = np.asarray(landed, dtype=float)
    airborne = np.flatnonzero(values[:, 1] == 0.0)
    if not len(airborne):
        raise ValueError("PX4 landed state contains no airborne interval")
    start_index = int(airborne[0])
    landing = np.flatnonzero(
        (values[:, 1] != 0.0) & (np.arange(len(values)) > start_index)
    )
    end_index = int(landing[0]) if len(landing) else int(airborne[-1])
    return float(values[start_index, 0]), float(values[end_index, 0])


def choose_airborne_window(bag: BagData) -> tuple[tuple[float, float], str]:
    height_window: tuple[float, float] | None = None
    landed_window: tuple[float, float] | None = None
    if bag.nokov is not None:
        try:
            height_window = detect_airborne_from_height(
                bag.nokov.time,
                bag.nokov.position[:, 2],
            )
        except ValueError:
            pass
    try:
        landed_window = _landed_window(bag.landed)
    except ValueError:
        pass
    if height_window and landed_window:
        intersection = (
            max(height_window[0], landed_window[0]),
            min(height_window[1], landed_window[1]),
        )
        if intersection[1] - intersection[0] >= 5.0:
            return intersection, "nokov_height_intersect_px4_landed"
    if height_window:
        return height_window, "nokov_height"
    if landed_window:
        return landed_window, "px4_landed"
    raise ValueError("cannot determine an airborne interval")


def _alignment_dict(result: AlignmentResult) -> dict[str, object]:
    return {
        "sample_count": result.sample_count,
        "time_offset_s": result.time_offset_s,
        "global_yaw_deg": float(np.rad2deg(result.global_yaw_rad)),
        "world_translation_xyz_m": result.world_translation_xyz_m.tolist(),
        "body_lever_xy_m": result.body_lever_xy_m.tolist(),
        "body_lever_norm_m": float(np.linalg.norm(result.body_lever_xy_m)),
        "scale": result.scale,
        "position": result.position,
        "horizontal": result.horizontal,
        "vertical": result.vertical,
        "drift_xyz_m": result.drift_xyz_m.tolist(),
        "drift_norm_m": result.drift_norm_m,
        "orientation_rms_deg": result.orientation_rms_deg.tolist(),
        "orientation_p95_deg": result.orientation_p95_deg.tolist(),
        "rpe_1s": result.rpe_1s,
        "rpe_5s": result.rpe_5s,
        "max_error_time": result.max_error_time,
    }


def _quality_warnings(case: dict[str, object]) -> list[str]:
    warnings: list[str] = []
    trajectory = case.get("trajectory")
    if isinstance(trajectory, dict) and trajectory.get("available") is not False:
        position = trajectory["position"]
        if position["p95_m"] > 0.05:
            warnings.append("trajectory_position_p95_over_5cm")
        if trajectory["drift_norm_m"] > 0.02:
            warnings.append("trajectory_drift_over_2cm")
    streams = case.get("streams")
    if isinstance(streams, dict):
        nokov = streams.get("nokov_timestamp")
        if isinstance(nokov, dict):
            if nokov["dt_gt_10ms_percent"] > 1.0:
                warnings.append("nokov_long_gap_fraction_over_1_percent")
            if nokov["dt_lt_1ms_percent"] > 1.0:
                warnings.append("nokov_burst_fraction_over_1_percent")
            if nokov["dt_max_ms"] > 50.0:
                warnings.append("nokov_max_gap_over_50ms")
        lio_lag = streams.get("lio_receive_lag_ms")
        if isinstance(lio_lag, dict) and lio_lag["p95"] > 40.0:
            warnings.append("lio_output_lag_p95_over_40ms")
        dynamic = streams.get("dynamic_diagnostics")
        if (
            case.get("dynamic_enabled")
            and isinstance(dynamic, dict)
            and dynamic.get("available")
        ):
            if dynamic["information_ratio_min"] < 0.35:
                warnings.append("dynamic_information_ratio_below_0_35")
            if dynamic["dynamic_ratio_max"] < 0.01:
                warnings.append("dynamic_scene_content_below_1_percent")
    ulog = case.get("ulog")
    if isinstance(ulog, dict) and ulog.get("available"):
        if ulog.get("file_corruption"):
            warnings.append("ulog_file_corruption")
        if not ulog.get("complete_flight"):
            warnings.append("ulog_does_not_cover_complete_flight")
    return warnings


def analyze_case(
    layout: CaseLayout,
    airborne_override: tuple[float, float] | None = None,
) -> dict[str, object]:
    bag = read_bag(layout.bag)
    if airborne_override is None:
        airborne_window, airborne_source = choose_airborne_window(bag)
    else:
        airborne_window, airborne_source = airborne_override, "cli_override"
    streams = summarize_bag_streams(bag, airborne_window)
    case: dict[str, object] = {
        "name": layout.name,
        "mode": layout.mode,
        "dynamic_enabled": layout.dynamic_enabled,
        "test_dir": str(layout.test_dir),
        "bag": str(layout.bag),
        "airborne_window": {
            "start_epoch_s": airborne_window[0],
            "end_epoch_s": airborne_window[1],
            "duration_s": airborne_window[1] - airborne_window[0],
            "source": airborne_source,
        },
        "streams": streams,
        "dynamic_diagnostics": streams["dynamic_diagnostics"],
        "scan_diagnostics": streams["scan_diagnostics"],
        "px4_ros": bag.px4_status,
        "warnings": list(bag.warnings),
    }
    if bag.nokov is not None:
        case["trajectory"] = _alignment_dict(
            align_trajectories(bag.lio, bag.nokov, airborne_window)
        )
    else:
        case["trajectory"] = {"available": False}
        case["warnings"].append("nokov_pose_missing")  # type: ignore[union-attr]
    if layout.ulog is not None:
        try:
            case["ulog"] = analyze_ulog(layout.ulog)
        except Exception as error:
            case["ulog"] = {
                "available": False,
                "path": str(layout.ulog),
                "error": str(error),
            }
            case["warnings"].append("ulog_analysis_failed")  # type: ignore[union-attr]
    else:
        case["ulog"] = {"available": False, "error": "ULog not found"}
        case["warnings"].append("ulog_missing")  # type: ignore[union-attr]
    case["warnings"].extend(_quality_warnings(case))  # type: ignore[union-attr]
    case["warnings"] = sorted(set(case["warnings"]))  # type: ignore[arg-type]
    return case


def analyze_session(
    root: Path,
    layouts: list[CaseLayout],
    airborne_overrides: dict[str, tuple[float, float]] | None = None,
) -> dict[str, object]:
    overrides = airborne_overrides or {}
    cases = [
        analyze_case(
            layout,
            overrides.get(layout.name, overrides.get(layout.mode)),
        )
        for layout in layouts
    ]
    return {
        "schema_version": 1,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "data_root": str(root),
        "cases": cases,
    }
