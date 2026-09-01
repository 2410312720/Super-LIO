from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class PoseSeries:
    time: np.ndarray
    position: np.ndarray
    quaternion_wxyz: np.ndarray
    receive_time: np.ndarray | None = None


@dataclass(frozen=True)
class AlignmentResult:
    time_offset_s: float
    global_yaw_rad: float
    world_translation_xyz_m: np.ndarray
    body_lever_xy_m: np.ndarray
    scale: float
    sample_count: int
    position: dict[str, float]
    horizontal: dict[str, float]
    vertical: dict[str, float]
    drift_xyz_m: np.ndarray
    drift_norm_m: float
    orientation_rms_deg: np.ndarray
    orientation_p95_deg: np.ndarray
    rpe_1s: dict[str, float]
    rpe_5s: dict[str, float]
    max_error_time: float


def timestamp_interval_metrics(times: np.ndarray) -> dict[str, float | int]:
    times = np.asarray(times, dtype=float)
    if times.ndim != 1 or len(times) < 2:
        raise ValueError("at least two one-dimensional timestamps are required")
    intervals = np.diff(times)
    positive = intervals[intervals > 0.0]
    if not len(positive):
        raise ValueError("timestamps contain no positive intervals")
    duration = times[-1] - times[0]
    return {
        "sample_count": int(len(times)),
        "rate_hz": float((len(times) - 1) / duration) if duration > 0.0 else 0.0,
        "dt_median_ms": float(np.median(positive) * 1_000.0),
        "dt_p95_ms": float(np.percentile(positive, 95) * 1_000.0),
        "dt_max_ms": float(np.max(positive) * 1_000.0),
        "dt_lt_1ms_percent": float(np.mean(positive < 0.001) * 100.0),
        "dt_gt_10ms_percent": float(np.mean(positive > 0.010) * 100.0),
        "regressions": int(np.count_nonzero(intervals < 0.0)),
        "duplicates": int(np.count_nonzero(intervals == 0.0)),
    }


def _interpolate_columns(
    source_time: np.ndarray,
    values: np.ndarray,
    query_time: np.ndarray,
) -> np.ndarray:
    return np.column_stack([
        np.interp(query_time, source_time, values[:, column])
        for column in range(values.shape[1])
    ])


def _quaternion_to_rpy(quaternion_wxyz: np.ndarray) -> np.ndarray:
    quaternion = np.asarray(quaternion_wxyz, dtype=float)
    norms = np.linalg.norm(quaternion, axis=1)
    quaternion = quaternion / norms[:, None]
    w, x, y, z = quaternion.T
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.column_stack([roll, pitch, yaw])


def _wrap_angle(angle: np.ndarray) -> np.ndarray:
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def _downsample_indices(times: np.ndarray, period_s: float = 0.1) -> np.ndarray:
    selected = [0]
    next_time = times[0] + period_s
    for index in range(1, len(times)):
        if times[index] + 1e-9 >= next_time:
            selected.append(index)
            next_time = times[index] + period_s
    return np.asarray(selected, dtype=int)


def _fit_yaw_translation(
    source: np.ndarray,
    target: np.ndarray,
) -> tuple[float, np.ndarray, np.ndarray]:
    source_center = np.mean(source, axis=0)
    target_center = np.mean(target, axis=0)
    centered_source = source - source_center
    centered_target = target - target_center
    numerator = np.sum(
        centered_source[:, 0] * centered_target[:, 1]
        - centered_source[:, 1] * centered_target[:, 0]
    )
    denominator = np.sum(centered_source[:, :2] * centered_target[:, :2])
    yaw = float(np.arctan2(numerator, denominator))
    cosine, sine = np.cos(yaw), np.sin(yaw)
    rotation = np.array([
        [cosine, -sine, 0.0],
        [sine, cosine, 0.0],
        [0.0, 0.0, 1.0],
    ])
    translation = target_center - source_center @ rotation.T
    return yaw, translation, source @ rotation.T + translation


def _lever_design_matrix(yaw: np.ndarray) -> np.ndarray:
    cosine = np.cos(yaw)
    sine = np.sin(yaw)
    design = np.zeros((2 * len(yaw), 4), dtype=float)
    design[0::2, 0] = 1.0
    design[1::2, 1] = 1.0
    design[0::2, 2] = cosine
    design[0::2, 3] = -sine
    design[1::2, 2] = sine
    design[1::2, 3] = cosine
    return design


def _fit_linearized_yaw_and_lever(
    source: np.ndarray,
    target: np.ndarray,
    target_yaw: np.ndarray,
) -> tuple[float, np.ndarray]:
    cosine = np.cos(target_yaw)
    sine = np.sin(target_yaw)
    design = np.zeros((2 * len(source), 6), dtype=float)
    design[0::2, 0] = source[:, 0]
    design[0::2, 1] = -source[:, 1]
    design[0::2, 2] = 1.0
    design[0::2, 4] = cosine
    design[0::2, 5] = -sine
    design[1::2, 0] = source[:, 1]
    design[1::2, 1] = source[:, 0]
    design[1::2, 3] = 1.0
    design[1::2, 4] = sine
    design[1::2, 5] = cosine
    coefficients = np.linalg.lstsq(design, target[:, :2].reshape(-1), rcond=None)[0]
    yaw = float(np.arctan2(coefficients[1], coefficients[0]))
    lever_design = _lever_design_matrix(target_yaw)
    cosine_global, sine_global = np.cos(yaw), np.sin(yaw)
    rotated_xy = np.column_stack([
        cosine_global * source[:, 0] - sine_global * source[:, 1],
        sine_global * source[:, 0] + cosine_global * source[:, 1],
    ])
    rigid_coefficients = np.linalg.lstsq(
        lever_design,
        (target[:, :2] - rotated_xy).reshape(-1),
        rcond=None,
    )[0]
    predicted_xy = rotated_xy + (lever_design @ rigid_coefficients).reshape(-1, 2)
    z_translation = float(np.mean(target[:, 2] - source[:, 2]))
    predicted = np.column_stack([predicted_xy, source[:, 2] + z_translation])
    return yaw, predicted


def _fit_yaw_translation_and_lever(
    source: np.ndarray,
    target: np.ndarray,
    target_yaw: np.ndarray,
    initial_yaw: float,
) -> tuple[float, np.ndarray, np.ndarray, np.ndarray]:
    design = _lever_design_matrix(target_yaw)
    best: tuple[float, float, np.ndarray, np.ndarray] | None = None

    def evaluate(candidate_yaw: float) -> tuple[float, np.ndarray, np.ndarray]:
        cosine, sine = np.cos(candidate_yaw), np.sin(candidate_yaw)
        rotated_xy = np.column_stack([
            cosine * source[:, 0] - sine * source[:, 1],
            sine * source[:, 0] + cosine * source[:, 1],
        ])
        coefficients = np.linalg.lstsq(
            design,
            (target[:, :2] - rotated_xy).reshape(-1),
            rcond=None,
        )[0]
        predicted_xy = rotated_xy + (design @ coefficients).reshape(-1, 2)
        z_translation = float(np.mean(target[:, 2] - source[:, 2]))
        predicted = np.column_stack([predicted_xy, source[:, 2] + z_translation])
        residual = predicted - target
        return float(np.mean(np.sum(residual * residual, axis=1))), coefficients, predicted

    coarse = np.deg2rad(np.arange(-3.0, 3.0001, 0.05)) + initial_yaw
    for candidate in coarse:
        cost, coefficients, predicted = evaluate(float(candidate))
        if best is None or cost < best[0]:
            best = cost, float(candidate), coefficients, predicted
    assert best is not None
    fine = np.deg2rad(np.arange(-0.1, 0.1001, 0.002)) + best[1]
    for candidate in fine:
        cost, coefficients, predicted = evaluate(float(candidate))
        if cost < best[0]:
            best = cost, float(candidate), coefficients, predicted

    _, yaw, coefficients, predicted = best
    translation = np.array([
        coefficients[0],
        coefficients[1],
        np.mean(target[:, 2] - source[:, 2]),
    ])
    return yaw, translation, coefficients[2:4], predicted


def _error_summary(values: np.ndarray) -> dict[str, float]:
    values = np.asarray(values, dtype=float)
    return {
        "mean_m": float(np.mean(values)),
        "rmse_m": float(np.sqrt(np.mean(values * values))),
        "median_m": float(np.median(values)),
        "p95_m": float(np.percentile(values, 95)),
        "max_m": float(np.max(values)),
    }


def _relative_pose_error(
    times: np.ndarray,
    estimate: np.ndarray,
    reference: np.ndarray,
    interval_s: float,
) -> dict[str, float]:
    later = np.searchsorted(times, times + interval_s)
    valid = later < len(times)
    first = np.flatnonzero(valid)
    second = later[valid]
    estimate_delta = estimate[second] - estimate[first]
    reference_delta = reference[second] - reference[first]
    return _error_summary(np.linalg.norm(estimate_delta - reference_delta, axis=1))


def align_trajectories(
    lio: PoseSeries,
    nokov: PoseSeries,
    airborne_window: tuple[float, float],
    *,
    offset_limit_s: float = 0.15,
    offset_step_s: float = 0.0025,
) -> AlignmentResult:
    lio_time = np.asarray(lio.time, dtype=float)
    nokov_time = np.asarray(nokov.time, dtype=float)
    lio_position = np.asarray(lio.position, dtype=float)
    nokov_position = np.asarray(nokov.position, dtype=float)
    in_air = (lio_time >= airborne_window[0]) & (lio_time <= airborne_window[1])
    if np.count_nonzero(in_air) < 20:
        raise ValueError("airborne interval contains too few LIO poses")
    full_indices = np.flatnonzero(in_air)
    indices = full_indices[_downsample_indices(lio_time[full_indices])]
    source_time = lio_time[indices]
    source = lio_position[indices]
    nokov_rpy = _quaternion_to_rpy(nokov.quaternion_wxyz)
    unwrapped_nokov_rpy = np.unwrap(nokov_rpy, axis=0)

    best: tuple[float, float] | None = None
    offsets = np.arange(
        -offset_limit_s,
        offset_limit_s + offset_step_s / 2.0,
        offset_step_s,
    )
    for offset in offsets:
        query = source_time + offset
        valid = (query >= nokov_time[0]) & (query <= nokov_time[-1])
        if np.count_nonzero(valid) < 20:
            continue
        target = _interpolate_columns(nokov_time, nokov_position, query[valid])
        target_yaw = np.interp(
            query[valid],
            nokov_time,
            unwrapped_nokov_rpy[:, 2],
        )
        _, predicted = _fit_linearized_yaw_and_lever(
            source[valid],
            target,
            target_yaw,
        )
        cost = float(np.mean(np.sum((predicted - target) ** 2, axis=1)))
        if best is None or cost < best[0]:
            best = cost, float(offset)
    if best is None:
        raise ValueError("LIO and NOKOV trajectories do not overlap")

    offset = best[1]
    query = source_time + offset
    valid = (query >= nokov_time[0]) & (query <= nokov_time[-1])
    source_time = source_time[valid]
    source = source[valid]
    query = query[valid]
    target = _interpolate_columns(nokov_time, nokov_position, query)
    target_rpy = _interpolate_columns(nokov_time, unwrapped_nokov_rpy, query)
    initial_yaw, _ = _fit_linearized_yaw_and_lever(
        source,
        target,
        target_rpy[:, 2],
    )
    yaw, translation, lever, _ = _fit_yaw_translation_and_lever(
        source,
        target,
        target_rpy[:, 2],
        initial_yaw,
    )

    source_time = lio_time[full_indices]
    source = lio_position[full_indices]
    query = source_time + offset
    valid = (query >= nokov_time[0]) & (query <= nokov_time[-1])
    full_indices = full_indices[valid]
    source_time = source_time[valid]
    source = source[valid]
    query = query[valid]
    target = _interpolate_columns(nokov_time, nokov_position, query)
    target_rpy = _interpolate_columns(nokov_time, unwrapped_nokov_rpy, query)
    cosine, sine = np.cos(yaw), np.sin(yaw)
    rotated_xy = np.column_stack([
        cosine * source[:, 0] - sine * source[:, 1],
        sine * source[:, 0] + cosine * source[:, 1],
    ])
    rotated_lever = np.column_stack([
        np.cos(target_rpy[:, 2]) * lever[0]
        - np.sin(target_rpy[:, 2]) * lever[1],
        np.sin(target_rpy[:, 2]) * lever[0]
        + np.cos(target_rpy[:, 2]) * lever[1],
    ])
    predicted = np.column_stack([
        rotated_xy + translation[:2] + rotated_lever,
        source[:, 2] + translation[2],
    ])
    residual = predicted - target
    error = np.linalg.norm(residual, axis=1)
    horizontal_error = np.linalg.norm(residual[:, :2], axis=1)
    vertical_error = np.abs(residual[:, 2])

    target_lever = np.column_stack([
        np.cos(target_rpy[:, 2]) * lever[0]
        - np.sin(target_rpy[:, 2]) * lever[1],
        np.sin(target_rpy[:, 2]) * lever[0]
        + np.cos(target_rpy[:, 2]) * lever[1],
        np.zeros(len(target_rpy)),
    ])
    target_body_origin = target - target_lever
    centered_source = source - np.mean(source, axis=0)
    centered_target = target_body_origin - np.mean(target_body_origin, axis=0)
    rotation = np.array([
        [cosine, -sine, 0.0],
        [sine, cosine, 0.0],
        [0.0, 0.0, 1.0],
    ])
    rotated_centered = centered_source @ rotation.T
    scale = float(
        np.sum(rotated_centered * centered_target)
        / np.sum(rotated_centered * rotated_centered)
    )

    lio_rpy = _quaternion_to_rpy(lio.quaternion_wxyz)[full_indices]
    orientation_difference = _wrap_angle(target_rpy - np.unwrap(lio_rpy, axis=0))
    fixed_orientation = np.median(orientation_difference, axis=0)
    orientation_residual = _wrap_angle(orientation_difference - fixed_orientation)
    orientation_degrees = np.abs(np.rad2deg(orientation_residual))

    elapsed = source_time - source_time[0]
    first = residual[elapsed <= 3.0]
    last = residual[elapsed >= elapsed[-1] - 3.0]
    drift = np.mean(last, axis=0) - np.mean(first, axis=0)
    max_index = int(np.argmax(error))
    return AlignmentResult(
        time_offset_s=offset,
        global_yaw_rad=yaw,
        world_translation_xyz_m=translation,
        body_lever_xy_m=lever,
        scale=scale,
        sample_count=len(source),
        position=_error_summary(error),
        horizontal=_error_summary(horizontal_error),
        vertical=_error_summary(vertical_error),
        drift_xyz_m=drift,
        drift_norm_m=float(np.linalg.norm(drift)),
        orientation_rms_deg=np.sqrt(np.mean(orientation_degrees ** 2, axis=0)),
        orientation_p95_deg=np.percentile(orientation_degrees, 95, axis=0),
        rpe_1s=_relative_pose_error(source_time, predicted, target, 1.0),
        rpe_5s=_relative_pose_error(source_time, predicted, target, 5.0),
        max_error_time=float(source_time[max_index]),
    )


def detect_airborne_from_height(
    times: np.ndarray,
    heights: np.ndarray,
    *,
    rise_m: float = 0.25,
    ground_seconds: float = 8.0,
) -> tuple[float, float]:
    times = np.asarray(times, dtype=float)
    heights = np.asarray(heights, dtype=float)
    if times.ndim != 1 or heights.shape != times.shape or len(times) < 2:
        raise ValueError("times and heights must be equal one-dimensional arrays")
    initial = times <= times[0] + ground_seconds
    ground_height = float(np.median(heights[initial]))
    airborne_indices = np.flatnonzero(heights > ground_height + rise_m)
    if not len(airborne_indices):
        raise ValueError("unable to detect an airborne interval from height")

    split_points = np.flatnonzero(np.diff(airborne_indices) > 1) + 1
    segments = np.split(airborne_indices, split_points)
    segment = max(segments, key=len)
    return float(times[segment[0]]), float(times[segment[-1]])
