from collections import Counter
from pathlib import Path
from typing import Mapping

import numpy as np


def _stats(values: np.ndarray) -> dict[str, float]:
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not len(values):
        return {}
    return {
        "mean": float(np.mean(values)),
        "rmse": float(np.sqrt(np.mean(values * values))),
        "median": float(np.median(values)),
        "p95": float(np.percentile(values, 95)),
        "max": float(np.max(values)),
    }


def summarize_aid_source(
    data: Mapping[str, np.ndarray],
    mask: np.ndarray,
    kind: str,
) -> dict[str, object]:
    mask = np.asarray(mask, dtype=bool)
    summary: dict[str, object] = {
        "sample_count": int(np.count_nonzero(mask)),
        "fused_count": int(np.count_nonzero(np.asarray(data["fused"])[mask])),
        "rejected_count": int(np.count_nonzero(
            np.asarray(data["innovation_rejected"])[mask]
        )),
    }
    if not summary["sample_count"]:
        return summary
    if kind == "position":
        innovation = np.column_stack([
            data["innovation[0]"],
            data["innovation[1]"],
        ])[mask]
        norms = np.linalg.norm(innovation, axis=1)
        ratios = np.column_stack([
            data["test_ratio[0]"],
            data["test_ratio[1]"],
        ])[mask]
        summary.update({
            "innovation_norm_mean_m": float(np.mean(norms)),
            "innovation_norm_rmse_m": float(np.sqrt(np.mean(norms * norms))),
            "innovation_norm_p95_m": float(np.percentile(norms, 95)),
            "innovation_norm_max_m": float(np.max(norms)),
            "test_ratio_p95": float(np.percentile(np.max(ratios, axis=1), 95)),
            "test_ratio_max": float(np.max(ratios)),
        })
    else:
        innovation = np.abs(np.asarray(data["innovation"], dtype=float)[mask])
        ratio = np.asarray(data["test_ratio"], dtype=float)[mask]
        scale = 180.0 / np.pi if kind == "yaw" else 1.0
        unit = "deg" if kind == "yaw" else "m"
        summary.update({
            f"innovation_mean_{unit}": float(np.mean(innovation) * scale),
            f"innovation_p95_{unit}": float(np.percentile(innovation, 95) * scale),
            f"innovation_max_{unit}": float(np.max(innovation) * scale),
            "test_ratio_p95": float(np.percentile(ratio, 95)),
            "test_ratio_max": float(np.max(ratio)),
        })
    return summary


def _dataset(ulog: object, name: str, multi_id: int = 0) -> Mapping[str, np.ndarray]:
    for dataset in ulog.data_list:  # type: ignore[attr-defined]
        if dataset.name == name and dataset.multi_id == multi_id:
            return dataset.data
    raise KeyError(name)


def _time_mask(
    data: Mapping[str, np.ndarray],
    start_s: float,
    end_s: float,
) -> np.ndarray:
    time_s = np.asarray(data["timestamp"], dtype=float) / 1_000_000.0
    return (time_s >= start_s) & (time_s <= end_s)


def _airborne_window(ulog: object) -> tuple[float, float, bool]:
    data = _dataset(ulog, "vehicle_land_detected")
    time_s = np.asarray(data["timestamp"], dtype=float) / 1_000_000.0
    landed = np.asarray(data["landed"], dtype=bool)
    takeoff = np.flatnonzero(~landed)
    if not len(takeoff):
        raise ValueError("ULog contains no airborne vehicle_land_detected samples")
    start_index = int(takeoff[0])
    landing = np.flatnonzero(landed & (np.arange(len(landed)) > start_index))
    if len(landing):
        return float(time_s[start_index]), float(time_s[landing[0]]), True
    return (
        float(time_s[start_index]),
        float(ulog.last_timestamp) / 1_000_000.0,  # type: ignore[attr-defined]
        False,
    )


def _primary_instance(ulog: object) -> int:
    try:
        selector = _dataset(ulog, "estimator_selector_status")
    except KeyError:
        return 0
    values = [int(value) for value in selector["primary_instance"]]
    return Counter(values).most_common(1)[0][0]


def _boolean_counts(
    data: Mapping[str, np.ndarray],
    mask: np.ndarray,
    fields: list[str],
) -> dict[str, int]:
    return {
        field: int(np.count_nonzero(np.asarray(data[field], dtype=bool)[mask]))
        for field in fields
        if field in data
    }


def _counter_changes(
    data: Mapping[str, np.ndarray],
    mask: np.ndarray,
    fields: list[str],
) -> dict[str, int]:
    changes: dict[str, int] = {}
    for field in fields:
        if field not in data:
            continue
        values = np.asarray(data[field])[mask]
        changes[field] = int(np.count_nonzero(np.diff(values)))
    return changes


def analyze_ulog(path: Path) -> dict[str, object]:
    try:
        from pyulog import ULog
    except ImportError as error:
        raise RuntimeError("pyulog is required to analyze PX4 ULog files") from error

    ulog = ULog(str(path))
    air_start, air_end, complete_flight = _airborne_window(ulog)
    primary = _primary_instance(ulog)
    result: dict[str, object] = {
        "available": True,
        "path": str(path),
        "file_corruption": bool(ulog.file_corruption),
        "dropout_count": len(ulog.dropouts),
        "duration_s": float((ulog.last_timestamp - ulog.start_timestamp) / 1_000_000.0),
        "airborne_duration_s": air_end - air_start,
        "complete_flight": complete_flight,
        "primary_estimator_instance": primary,
    }

    for topic, label, kind in (
        ("estimator_aid_src_ev_pos", "ev_position", "position"),
        ("estimator_aid_src_ev_hgt", "ev_height", "height"),
        ("estimator_aid_src_ev_yaw", "ev_yaw", "yaw"),
    ):
        data = _dataset(ulog, topic, primary)
        result[label] = summarize_aid_source(
            data,
            _time_mask(data, air_start, air_end),
            kind,
        )

    flags = _dataset(ulog, "estimator_status_flags", primary)
    flags_mask = _time_mask(flags, air_start, air_end)
    flag_fields = [
        "cs_ev_pos", "cs_ev_hgt", "cs_ev_yaw", "cs_inertial_dead_reckoning",
        "reject_hor_pos", "reject_ver_pos", "reject_yaw", "cs_ev_yaw_fault",
        "cs_mag_fault", "cs_baro_fault", "cs_gnss_fault", "fs_bad_acc_clipping",
    ]
    result["estimator_flags"] = {
        "sample_count": int(np.count_nonzero(flags_mask)),
        "true_counts": _boolean_counts(flags, flags_mask, flag_fields),
    }

    status = _dataset(ulog, "estimator_status", primary)
    status_mask = _time_mask(status, air_start, air_end)
    result["estimator_status"] = {
        "sample_count": int(np.count_nonzero(status_mask)),
        "filter_fault_nonzero_count": int(np.count_nonzero(
            np.asarray(status["filter_fault_flags"])[status_mask]
        )),
        "health_nonzero_count": int(np.count_nonzero(
            np.asarray(status["health_flags"])[status_mask]
        )),
        "timeout_nonzero_count": int(np.count_nonzero(
            np.asarray(status["timeout_flags"])[status_mask]
        )),
        "reset_changes": _counter_changes(status, status_mask, [
            "reset_count_vel_ne", "reset_count_vel_d", "reset_count_pos_ne",
            "reset_count_pod_d", "reset_count_quat",
        ]),
        "horizontal_accuracy_m": _stats(
            np.asarray(status["pos_horiz_accuracy"])[status_mask]
        ),
        "vertical_accuracy_m": _stats(
            np.asarray(status["pos_vert_accuracy"])[status_mask]
        ),
    }

    local_position = _dataset(ulog, "vehicle_local_position")
    local_mask = _time_mask(local_position, air_start, air_end)
    local_count = int(np.count_nonzero(local_mask))
    result["local_position"] = {
        "sample_count": local_count,
        "valid_counts": _boolean_counts(local_position, local_mask, [
            "xy_valid", "z_valid", "v_xy_valid", "v_z_valid",
        ]),
        "dead_reckoning_count": int(np.count_nonzero(
            np.asarray(local_position["dead_reckoning"])[local_mask]
        )),
        "reset_changes": _counter_changes(local_position, local_mask, [
            "xy_reset_counter", "z_reset_counter", "vxy_reset_counter",
            "vz_reset_counter", "heading_reset_counter",
        ]),
        "eph_m": _stats(np.asarray(local_position["eph"])[local_mask]),
        "epv_m": _stats(np.asarray(local_position["epv"])[local_mask]),
    }
    heading_counter = np.asarray(local_position["heading_reset_counter"])[local_mask]
    heading_delta = np.asarray(local_position["delta_heading"])[local_mask]
    heading_time = np.asarray(local_position["timestamp"])[local_mask] / 1_000_000.0
    reset_indices = np.flatnonzero(np.diff(heading_counter) != 0) + 1
    result["heading_reset_events"] = [
        {
            "airborne_time_s": float(heading_time[index] - air_start),
            "delta_deg": float(np.rad2deg(heading_delta[index])),
        }
        for index in reset_indices
    ]

    visual = _dataset(ulog, "vehicle_visual_odometry")
    visual_mask = _time_mask(visual, air_start, air_end)
    timestamp = np.asarray(visual["timestamp"], dtype=np.int64)[visual_mask]
    timestamp_sample = np.asarray(
        visual["timestamp_sample"], dtype=np.int64
    )[visual_mask]
    sample_lag_ms = (timestamp - timestamp_sample) / 1_000.0
    result["visual_odometry"] = {
        "sample_count": int(len(timestamp)),
        "sample_lag_ms": _stats(sample_lag_ms),
        "negative_sample_lag_count": int(np.count_nonzero(sample_lag_ms < 0.0)),
    }

    vehicle_status = _dataset(ulog, "vehicle_status")
    vehicle_mask = _time_mask(vehicle_status, air_start, air_end)
    result["vehicle_status"] = {
        "sample_count": int(np.count_nonzero(vehicle_mask)),
        "failsafe_count": int(np.count_nonzero(
            np.asarray(vehicle_status["failsafe"])[vehicle_mask]
        )),
        "failure_detector_nonzero_count": int(np.count_nonzero(
            np.asarray(vehicle_status["failure_detector_status"])[vehicle_mask]
        )),
        "nav_states": sorted(set(
            int(value) for value in np.asarray(vehicle_status["nav_state"])[vehicle_mask]
        )),
    }
    return result
