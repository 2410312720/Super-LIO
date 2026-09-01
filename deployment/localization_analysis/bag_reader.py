from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from .diagnostics import (
    DynamicDiagnostics,
    parse_dynamic_diagnostics_prefix,
    strip_dynamic_diagnostics_block,
    summarize_dynamic_diagnostics,
)
from .trajectory import PoseSeries, timestamp_interval_metrics


@dataclass
class BagData:
    lio: PoseSeries
    nokov: PoseSeries | None
    landed: list[tuple[float, bool]]
    dynamic_ratio: np.ndarray
    dynamic_diagnostics: list[tuple[float, DynamicDiagnostics]]
    scan_diagnostics: list[dict[str, float]]
    px4_status: dict[str, object]
    warnings: list[str] = field(default_factory=list)


def _pose_row(message: Any, receive_time: float, odometry: bool) -> tuple[float, ...]:
    stamp = message.header.stamp
    header_time = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    pose = message.pose.pose if odometry else message.pose
    return (
        header_time,
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        receive_time,
    )


def _to_pose_series(rows: list[tuple[float, ...]]) -> PoseSeries:
    values = np.asarray(rows, dtype=float)
    return PoseSeries(
        time=values[:, 0],
        position=values[:, 1:4],
        quaternion_wxyz=values[:, 4:8],
        receive_time=values[:, 8],
    )


def _scan_message_metrics(message: Any) -> dict[str, float]:
    return {
        "match_fraction": float(message.match_fraction),
        "residual_abs_mean_m": float(message.residual_abs_mean_m),
        "residual_rmse_m": float(message.residual_rmse_m),
        "residual_p95_m": float(message.residual_p95_m),
        "residual_max_m": float(message.residual_max_m),
        "position_correction_m": float(message.scan_position_correction_m),
        "information_condition_number": float(message.information_condition_number),
        "information_rank": float(message.information_rank),
        "undistort_time_ms": float(message.undistort_time_ms),
        "downsample_time_ms": float(message.downsample_time_ms),
        "observe_time_ms": float(message.observe_time_ms),
        "map_update_time_ms": float(message.map_update_time_ms),
        "output_time_ms": float(message.output_time_ms),
        "total_time_ms": float(message.total_time_ms),
    }


def _deserialize_scan(
    payload: bytes,
    message_type: type,
    dynamic: DynamicDiagnostics | None,
    deserialize_message: Any,
) -> Any:
    type_has_dynamic = hasattr(message_type(), "dynamic_filter_enabled")
    if dynamic is not None and not type_has_dynamic:
        payload = strip_dynamic_diagnostics_block(payload, dynamic)
    return deserialize_message(payload, message_type)


def _summarize_px4_status(messages: dict[str, list[Any]]) -> dict[str, object]:
    result: dict[str, object] = {}
    estimator = messages.get("/fmu/out/estimator_status_flags", [])
    if estimator:
        fields = [
            "cs_ev_pos", "cs_ev_hgt", "cs_ev_yaw", "cs_inertial_dead_reckoning",
            "reject_hor_pos", "reject_ver_pos", "reject_yaw", "cs_ev_yaw_fault",
        ]
        result["estimator_flags"] = {
            "sample_count": len(estimator),
            "true_counts": {
                field: sum(bool(getattr(message, field)) for message in estimator)
                for field in fields
                if hasattr(estimator[0], field)
            },
        }
    status = messages.get("/fmu/out/vehicle_status_v1", [])
    if status:
        result["vehicle_status"] = {
            "sample_count": len(status),
            "failsafe_count": sum(bool(message.failsafe) for message in status),
            "failure_detector_nonzero_count": sum(
                int(message.failure_detector_status) != 0 for message in status
            ),
            "nav_states": sorted(set(int(message.nav_state) for message in status)),
        }
    local = messages.get("/fmu/out/vehicle_local_position_v1", [])
    if local:
        result["local_position"] = {
            "sample_count": len(local),
            "dead_reckoning_count": sum(bool(message.dead_reckoning) for message in local),
            "valid_counts": {
                field: sum(bool(getattr(message, field)) for message in local)
                for field in ("xy_valid", "z_valid", "v_xy_valid", "v_z_valid")
            },
            "reset_changes": {
                field: sum(
                    getattr(current, field) != getattr(previous, field)
                    for previous, current in zip(local, local[1:])
                )
                for field in (
                    "xy_reset_counter", "z_reset_counter", "vxy_reset_counter",
                    "vz_reset_counter", "heading_reset_counter",
                )
            },
        }
    vehicle_failsafe = messages.get("/fmu/out/failsafe_flags", [])
    if vehicle_failsafe:
        ignored_prefix = "mode_req_"
        fields = vehicle_failsafe[0].get_fields_and_field_types()
        active = {}
        for field_name in fields:
            if field_name == "timestamp" or field_name.startswith(ignored_prefix):
                continue
            count = sum(bool(getattr(message, field_name)) for message in vehicle_failsafe)
            if count:
                active[field_name] = count
        result["failsafe_flags_active"] = active
    return result


def read_bag(path: Path) -> BagData:
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise RuntimeError(
            "ROS 2 Python packages are required; source the workspace setup.bash"
        ) from error

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    wanted = {
        "/lio/robo/odom", "/Robot_3/pose", "/lio/dynamic_ratio",
        "/lio/diagnostics/scan", "/fmu/out/vehicle_land_detected",
        "/fmu/out/estimator_status_flags", "/fmu/out/failsafe_flags",
        "/fmu/out/vehicle_status_v1", "/fmu/out/vehicle_local_position_v1",
    }
    available = sorted(wanted.intersection(topic_types))
    reader.set_filter(rosbag2_py.StorageFilter(topics=available))
    message_types: dict[str, type | None] = {}
    warnings: list[str] = []
    for topic in available:
        try:
            message_types[topic] = get_message(topic_types[topic])
        except (ImportError, ModuleNotFoundError, AttributeError) as error:
            message_types[topic] = None
            if topic != "/lio/diagnostics/scan":
                warnings.append(f"cannot load {topic_types[topic]} for {topic}: {error}")

    lio_rows: list[tuple[float, ...]] = []
    nokov_rows: list[tuple[float, ...]] = []
    landed: list[tuple[float, bool]] = []
    dynamic_ratio: list[tuple[float, float]] = []
    dynamic_samples: list[tuple[float, DynamicDiagnostics]] = []
    scan_samples: list[dict[str, float]] = []
    px4_messages: dict[str, list[Any]] = {}
    scan_warning_emitted = False

    while reader.has_next():
        topic, payload, receive_ns = reader.read_next()
        receive_time = receive_ns * 1e-9
        if topic == "/lio/diagnostics/scan":
            dynamic = parse_dynamic_diagnostics_prefix(payload)
            if dynamic is not None:
                dynamic_samples.append((receive_time, dynamic))
            message_type = message_types[topic]
            if message_type is not None:
                try:
                    message = _deserialize_scan(
                        payload, message_type, dynamic, deserialize_message
                    )
                    scan_samples.append(_scan_message_metrics(message))
                except Exception as error:  # ROS type-support errors have varying types
                    if not scan_warning_emitted:
                        warnings.append(f"cannot deserialize scan diagnostics: {error}")
                        scan_warning_emitted = True
            continue

        message_type = message_types.get(topic)
        if message_type is None:
            continue
        message = deserialize_message(payload, message_type)
        if topic == "/lio/robo/odom":
            lio_rows.append(_pose_row(message, receive_time, True))
        elif topic == "/Robot_3/pose":
            nokov_rows.append(_pose_row(message, receive_time, False))
        elif topic == "/lio/dynamic_ratio":
            dynamic_ratio.append((receive_time, float(message.data)))
        elif topic == "/fmu/out/vehicle_land_detected":
            landed.append((receive_time, bool(message.landed)))
        elif topic.startswith("/fmu/out/"):
            px4_messages.setdefault(topic, []).append(message)

    if not lio_rows:
        raise ValueError(f"bag contains no /lio/robo/odom: {path}")
    return BagData(
        lio=_to_pose_series(lio_rows),
        nokov=_to_pose_series(nokov_rows) if nokov_rows else None,
        landed=landed,
        dynamic_ratio=np.asarray(dynamic_ratio, dtype=float).reshape(-1, 2),
        dynamic_diagnostics=dynamic_samples,
        scan_diagnostics=scan_samples,
        px4_status=_summarize_px4_status(px4_messages),
        warnings=warnings,
    )


def summarize_scan_diagnostics(samples: list[dict[str, float]]) -> dict[str, object]:
    if not samples:
        return {"available": False, "sample_count": 0}
    result: dict[str, object] = {"available": True, "sample_count": len(samples)}
    for field_name in samples[0]:
        values = np.asarray([sample[field_name] for sample in samples], dtype=float)
        result[field_name] = {
            "mean": float(np.mean(values)),
            "median": float(np.median(values)),
            "p95": float(np.percentile(values, 95)),
            "max": float(np.max(values)),
        }
    return result


def summarize_bag_streams(
    bag: BagData,
    airborne_window: tuple[float, float],
) -> dict[str, object]:
    start, end = airborne_window
    lio_mask = (bag.lio.time >= start) & (bag.lio.time <= end)
    lio_time = bag.lio.time[lio_mask]
    result: dict[str, object] = {
        "lio_timestamp": timestamp_interval_metrics(lio_time),
    }
    if bag.lio.receive_time is not None:
        lag_ms = (bag.lio.receive_time[lio_mask] - lio_time) * 1_000.0
        result["lio_receive_lag_ms"] = {
            "mean": float(np.mean(lag_ms)),
            "median": float(np.median(lag_ms)),
            "p95": float(np.percentile(lag_ms, 95)),
            "max": float(np.max(lag_ms)),
        }
    if bag.nokov is not None:
        nokov_mask = (bag.nokov.time >= start) & (bag.nokov.time <= end)
        result["nokov_timestamp"] = timestamp_interval_metrics(
            bag.nokov.time[nokov_mask]
        )
        if bag.nokov.receive_time is not None:
            receive_delta_ms = (
                bag.nokov.receive_time[nokov_mask] - bag.nokov.time[nokov_mask]
            ) * 1_000.0
            result["nokov_receive_lag_ms"] = {
                "mean": float(np.mean(receive_delta_ms)),
                "median": float(np.median(receive_delta_ms)),
                "p95_abs": float(np.percentile(np.abs(receive_delta_ms), 95)),
                "max_abs": float(np.max(np.abs(receive_delta_ms))),
            }
    dynamic = [
        sample for receive_time, sample in bag.dynamic_diagnostics
        if start <= receive_time <= end
    ]
    result["dynamic_diagnostics"] = summarize_dynamic_diagnostics(dynamic)
    if len(bag.dynamic_ratio):
        mask = (
            (bag.dynamic_ratio[:, 0] >= start)
            & (bag.dynamic_ratio[:, 0] <= end)
        )
        ratio = bag.dynamic_ratio[mask, 1]
        if len(ratio):
            result["dynamic_ratio"] = {
                "sample_count": len(ratio),
                "mean": float(np.mean(ratio)),
                "median": float(np.median(ratio)),
                "p95": float(np.percentile(ratio, 95)),
                "max": float(np.max(ratio)),
                "over_1_percent_count": int(np.count_nonzero(ratio > 0.01)),
            }
    result["scan_diagnostics"] = summarize_scan_diagnostics(bag.scan_diagnostics)
    return result
