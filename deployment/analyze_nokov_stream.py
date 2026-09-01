#!/usr/bin/env python3

import argparse
import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Sequence

import numpy as np


NOKOV_TOPIC = "/Robot_3/pose"
BRIDGE_TOPIC = "/fmu/in/vehicle_visual_odometry"
PX4_LOCAL_POSITION_TOPIC = "/fmu/out/vehicle_local_position_v1"
PX4_ESTIMATOR_TOPIC = "/fmu/out/estimator_status_flags"
DYNAMIC_RATIO_TOPIC = "/lio/dynamic_ratio"


def _local_time(epoch_s: float) -> str:
    return datetime.fromtimestamp(epoch_s).astimezone().isoformat(timespec="milliseconds")


def summarize_timestamps(
    timestamps: Sequence[float],
    *,
    top_n: int = 10,
) -> dict[str, object]:
    values = np.asarray(timestamps, dtype=float)
    if values.ndim != 1 or len(values) < 2:
        return {"available": False, "sample_count": int(len(values))}

    intervals = np.diff(values)
    positive = intervals[intervals > 0.0]
    if not len(positive):
        return {
            "available": False,
            "sample_count": int(len(values)),
            "regressions": int(np.count_nonzero(intervals < 0.0)),
            "duplicates": int(np.count_nonzero(intervals == 0.0)),
        }

    duration = float(values[-1] - values[0])
    gap_counts = {
        "over_20ms": int(np.count_nonzero(positive > 0.020)),
        "over_50ms": int(np.count_nonzero(positive > 0.050)),
        "over_100ms": int(np.count_nonzero(positive > 0.100)),
        "over_200ms": int(np.count_nonzero(positive > 0.200)),
        "over_1s": int(np.count_nonzero(positive > 1.000)),
    }
    ranked_indices = sorted(
        (index for index, interval in enumerate(intervals) if interval > 0.0),
        key=lambda index: intervals[index],
        reverse=True,
    )[:top_n]
    top_gaps = [
        {
            "start_epoch_s": float(values[index]),
            "end_epoch_s": float(values[index + 1]),
            "start_local": _local_time(float(values[index])),
            "end_local": _local_time(float(values[index + 1])),
            "gap_ms": float(intervals[index] * 1_000.0),
        }
        for index in ranked_indices
    ]
    return {
        "available": True,
        "sample_count": int(len(values)),
        "duration_s": duration,
        "rate_hz": float((len(values) - 1) / duration) if duration > 0.0 else 0.0,
        "median_gap_ms": float(np.median(positive) * 1_000.0),
        "p95_gap_ms": float(np.percentile(positive, 95) * 1_000.0),
        "p99_gap_ms": float(np.percentile(positive, 99) * 1_000.0),
        "max_gap_ms": float(np.max(positive) * 1_000.0),
        "regressions": int(np.count_nonzero(intervals < 0.0)),
        "duplicates": int(np.count_nonzero(intervals == 0.0)),
        "gap_counts": gap_counts,
        "top_gaps": top_gaps,
    }


def parse_dds_monitor(
    text: str,
    recording_start_s: float,
    recording_end_s: float,
) -> dict[str, object]:
    current_time: float | None = None
    samples: list[tuple[float, int]] = []
    for line in text.splitlines():
        fields = line.split()
        if len(fields) >= 2 and fields[0] == "TIME":
            try:
                current_time = float(fields[1])
            except ValueError:
                current_time = None
        elif fields and fields[0] == "UdpRcvbufErrors" and current_time is not None:
            try:
                samples.append((current_time, int(fields[1])))
            except (IndexError, ValueError):
                continue

    if not samples:
        return {"available": False, "sample_count": 0}

    def value_at(epoch_s: float) -> int:
        before = [value for sample_time, value in samples if sample_time <= epoch_s]
        if before:
            return before[-1]
        return samples[0][1]

    recording_start_value = value_at(recording_start_s)
    recording_end_value = value_at(recording_end_s)
    return {
        "available": True,
        "sample_count": len(samples),
        "start_value": samples[0][1],
        "end_value": samples[-1][1],
        "total_delta": samples[-1][1] - samples[0][1],
        "recording_start_value": recording_start_value,
        "recording_end_value": recording_end_value,
        "recording_delta": recording_end_value - recording_start_value,
    }


def evaluate_flight_gate(
    case: dict[str, object],
    *,
    max_gap_ms: float,
) -> dict[str, object]:
    failures: list[str] = []
    incomplete = False
    threshold = f"{max_gap_ms:g}"

    nokov = case.get("nokov", {})
    if not isinstance(nokov, dict) or not nokov.get("available"):
        failures.append("nokov_unavailable")
    elif float(nokov["receive"]["max_gap_ms"]) > max_gap_ms:
        failures.append(f"nokov_max_gap_over_{threshold}ms")

    bridge = case.get("bridge", {})
    if not isinstance(bridge, dict) or not bridge.get("available"):
        failures.append("bridge_unavailable")
    elif float(bridge["receive"]["max_gap_ms"]) > max_gap_ms:
        failures.append(f"bridge_max_gap_over_{threshold}ms")

    px4 = case.get("px4", {})
    if not isinstance(px4, dict) or not px4.get("available"):
        failures.append("px4_status_unavailable")
        incomplete = True
    else:
        if int(px4.get("dead_reckoning_count", 0)) > 0:
            failures.append("px4_dead_reckoning_detected")
        reset_total = int(px4.get(
            "reset_total",
            sum(int(value) for value in px4.get("reset_changes", {}).values()),
        ))
        if reset_total > 0:
            failures.append("px4_position_reset_detected")

    hard_failures = [failure for failure in failures if failure != "px4_status_unavailable"]
    if hard_failures:
        result = "FAIL"
    elif incomplete:
        result = "INCOMPLETE"
    else:
        result = "PASS"
    return {"result": result, "failures": failures}


def _comparison_value(before: float, after: float) -> dict[str, float | None]:
    improvement = None
    if before != 0.0:
        improvement = (before - after) / abs(before) * 100.0
    return {
        "before": before,
        "after": after,
        "change": after - before,
        "improvement_percent": improvement,
    }


def compare_case_metrics(
    baseline: dict[str, object],
    candidate: dict[str, object],
) -> dict[str, object]:
    return {
        "baseline": baseline["name"],
        "candidate": candidate["name"],
        "nokov_max_gap_ms": _comparison_value(
            float(baseline["nokov"]["receive"]["max_gap_ms"]),
            float(candidate["nokov"]["receive"]["max_gap_ms"]),
        ),
        "bridge_max_gap_ms": _comparison_value(
            float(baseline["bridge"]["receive"]["max_gap_ms"]),
            float(candidate["bridge"]["receive"]["max_gap_ms"]),
        ),
        "dead_reckoning_count": _comparison_value(
            float(baseline["px4"].get("dead_reckoning_count", 0)),
            float(candidate["px4"].get("dead_reckoning_count", 0)),
        ),
        "reset_total": _comparison_value(
            float(baseline["px4"].get("reset_total", 0)),
            float(candidate["px4"].get("reset_total", 0)),
        ),
    }


def _parse_notes(path: Path) -> dict[str, str]:
    if not path.is_file():
        return {}
    values: dict[str, str] = {}
    for raw_line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if "=" not in raw_line:
            continue
        key, value = raw_line.split("=", 1)
        values[key.strip()] = value.strip()
    return values


def _resolve_case(path: Path) -> tuple[Path, Path]:
    resolved = path.expanduser().resolve()
    if resolved.is_file() and resolved.suffix == ".mcap":
        bag_dir = resolved.parent
    elif (resolved / "metadata.yaml").is_file():
        bag_dir = resolved
    else:
        bag_dir = resolved / "rosbag"
    if not (bag_dir / "metadata.yaml").is_file() or not list(bag_dir.glob("*.mcap")):
        raise ValueError(f"cannot find an MCAP rosbag below: {path}")
    case_dir = bag_dir.parent if bag_dir.name == "rosbag" else resolved
    return case_dir, bag_dir


def _safe_message_type(get_message: Any, type_name: str) -> tuple[type | None, str | None]:
    try:
        return get_message(type_name), None
    except (ImportError, ModuleNotFoundError, AttributeError) as error:
        return None, str(error)


def _summarize_dynamic_ratio(values: Sequence[float]) -> dict[str, object]:
    array = np.asarray(values, dtype=float)
    if not len(array):
        return {"available": False, "sample_count": 0}
    return {
        "available": True,
        "sample_count": int(len(array)),
        "mean_percent": float(np.mean(array) * 100.0),
        "p95_percent": float(np.percentile(array, 95) * 100.0),
        "max_percent": float(np.max(array) * 100.0),
        "over_0_05_percent_count": int(np.count_nonzero(array > 0.0005)),
    }


def analyze_case(path: Path, max_gap_ms: float) -> dict[str, object]:
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise RuntimeError(
            "ROS 2 Python packages are required; source /opt/ros/humble/setup.bash"
        ) from error

    case_dir, bag_dir = _resolve_case(path)
    notes = _parse_notes(case_dir / "test_notes.txt")
    warnings: list[str] = []
    recorded_test_id = notes.get("TEST_ID")
    if recorded_test_id and recorded_test_id != case_dir.name:
        warnings.append(
            f"directory name {case_dir.name} differs from TEST_ID {recorded_test_id}"
        )

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    wanted = {
        NOKOV_TOPIC,
        BRIDGE_TOPIC,
        PX4_LOCAL_POSITION_TOPIC,
        PX4_ESTIMATOR_TOPIC,
        DYNAMIC_RATIO_TOPIC,
    }
    available = sorted(wanted.intersection(topic_types))
    reader.set_filter(rosbag2_py.StorageFilter(topics=available))

    message_types: dict[str, type | None] = {}
    for topic in (NOKOV_TOPIC, DYNAMIC_RATIO_TOPIC, PX4_LOCAL_POSITION_TOPIC,
                  PX4_ESTIMATOR_TOPIC):
        if topic not in topic_types:
            message_types[topic] = None
            continue
        message_type, error = _safe_message_type(get_message, topic_types[topic])
        message_types[topic] = message_type
        if error:
            warnings.append(f"cannot load {topic_types[topic]} for {topic}: {error}")

    nokov_receive: list[float] = []
    nokov_header: list[float] = []
    nokov_invalid = 0
    bridge_receive: list[float] = []
    dynamic_ratio: list[float] = []
    all_receive: list[float] = []
    dead_reckoning_count = 0
    local_position_samples = 0
    estimator_dead_reckoning_count = 0
    estimator_samples = 0
    reset_fields = {
        "xy": "xy_reset_counter",
        "z": "z_reset_counter",
        "vxy": "vxy_reset_counter",
        "vz": "vz_reset_counter",
        "heading": "heading_reset_counter",
    }
    reset_changes = {name: 0 for name in reset_fields}
    previous_resets: dict[str, int] = {}

    while reader.has_next():
        topic, payload, receive_ns = reader.read_next()
        receive_time = receive_ns * 1e-9
        all_receive.append(receive_time)
        if topic == BRIDGE_TOPIC:
            bridge_receive.append(receive_time)
            continue

        message_type = message_types.get(topic)
        if message_type is None:
            continue
        message = deserialize_message(payload, message_type)
        if topic == NOKOV_TOPIC:
            nokov_receive.append(receive_time)
            stamp = message.header.stamp
            nokov_header.append(float(stamp.sec) + float(stamp.nanosec) * 1e-9)
            position = message.pose.position
            orientation = message.pose.orientation
            pose_values = (
                position.x, position.y, position.z,
                orientation.x, orientation.y, orientation.z, orientation.w,
            )
            quaternion_norm = math.sqrt(
                orientation.x ** 2 + orientation.y ** 2
                + orientation.z ** 2 + orientation.w ** 2
            )
            if not all(math.isfinite(value) for value in pose_values) or quaternion_norm < 1e-6:
                nokov_invalid += 1
        elif topic == DYNAMIC_RATIO_TOPIC:
            dynamic_ratio.append(float(message.data))
        elif topic == PX4_LOCAL_POSITION_TOPIC:
            local_position_samples += 1
            dead_reckoning_count += int(bool(message.dead_reckoning))
            for short_name, field_name in reset_fields.items():
                current = int(getattr(message, field_name))
                if short_name in previous_resets and previous_resets[short_name] != current:
                    reset_changes[short_name] += 1
                previous_resets[short_name] = current
        elif topic == PX4_ESTIMATOR_TOPIC:
            estimator_samples += 1
            estimator_dead_reckoning_count += int(
                bool(getattr(message, "cs_inertial_dead_reckoning", False))
            )

    if not all_receive:
        raise ValueError(f"no requested topics contain data: {bag_dir}")
    recording_start = min(all_receive)
    recording_end = max(all_receive)

    nokov_receive_summary = summarize_timestamps(nokov_receive)
    nokov_header_summary = summarize_timestamps(nokov_header)
    nokov: dict[str, object] = {
        "available": bool(nokov_receive_summary.get("available")),
        "receive": nokov_receive_summary,
        "header": nokov_header_summary,
        "invalid_pose_count": nokov_invalid,
    }
    if len(nokov_receive) == len(nokov_header) and nokov_receive:
        latency_ms = (np.asarray(nokov_receive) - np.asarray(nokov_header)) * 1_000.0
        nokov["latency_ms"] = {
            "median": float(np.median(latency_ms)),
            "p95_abs": float(np.percentile(np.abs(latency_ms), 95)),
            "max_abs": float(np.max(np.abs(latency_ms))),
        }

    bridge_summary = summarize_timestamps(bridge_receive)
    px4_available = (
        local_position_samples > 0
        and message_types.get(PX4_LOCAL_POSITION_TOPIC) is not None
    )
    reset_total = sum(reset_changes.values())
    px4 = {
        "available": px4_available,
        "local_position_sample_count": local_position_samples,
        "dead_reckoning_count": dead_reckoning_count,
        "dead_reckoning_percent": (
            dead_reckoning_count / local_position_samples * 100.0
            if local_position_samples else None
        ),
        "reset_changes": reset_changes,
        "reset_total": reset_total,
        "estimator_sample_count": estimator_samples,
        "estimator_dead_reckoning_count": estimator_dead_reckoning_count,
    }
    dds_path = case_dir / "dds_udp_monitor.log"
    dds = (
        parse_dds_monitor(
            dds_path.read_text(encoding="utf-8", errors="replace"),
            recording_start,
            recording_end,
        )
        if dds_path.is_file()
        else {"available": False, "sample_count": 0}
    )
    case: dict[str, object] = {
        "name": case_dir.name,
        "recorded_test_id": recorded_test_id,
        "test_dir": str(case_dir),
        "bag": str(bag_dir),
        "recording": {
            "start_epoch_s": recording_start,
            "end_epoch_s": recording_end,
            "duration_s": recording_end - recording_start,
        },
        "nokov": nokov,
        "bridge": {
            "available": bool(bridge_summary.get("available")),
            "receive": bridge_summary,
        },
        "px4": px4,
        "dynamic_ratio": _summarize_dynamic_ratio(dynamic_ratio),
        "dds_udp": dds,
        "warnings": warnings,
    }
    case["gate"] = evaluate_flight_gate(case, max_gap_ms=max_gap_ms)
    return case


def _format_number(value: object, digits: int = 2) -> str:
    if value is None:
        return "N/A"
    return f"{float(value):.{digits}f}"


def _render_case(case: dict[str, object], max_gap_ms: float) -> str:
    nokov = case["nokov"]
    bridge = case["bridge"]
    px4 = case["px4"]
    dynamic = case["dynamic_ratio"]
    dds = case["dds_udp"]
    receive = nokov.get("receive", {})
    header = nokov.get("header", {})
    bridge_receive = bridge.get("receive", {})
    resets = px4.get("reset_changes", {})
    lines = [
        f"\n=== {case['name']} ===",
        f"数据目录: {case['test_dir']}",
        f"录制时长: {_format_number(case['recording']['duration_s'])} s",
    ]
    if case.get("recorded_test_id") and case["recorded_test_id"] != case["name"]:
        lines.append(f"记录 TEST_ID: {case['recorded_test_id']}（与目录名不一致）")
    if nokov.get("available"):
        counts = receive["gap_counts"]
        latency = nokov.get("latency_ms", {})
        lines.extend([
            "NOKOV:",
            f"  样本/频率: {receive['sample_count']} / {_format_number(receive['rate_hz'])} Hz",
            f"  间隔 P95/P99/最大: {_format_number(receive['p95_gap_ms'], 3)} / "
            f"{_format_number(receive['p99_gap_ms'], 3)} / "
            f"{_format_number(receive['max_gap_ms'], 3)} ms",
            f"  >50/>100/>200 ms/>1 s: {counts['over_50ms']} / "
            f"{counts['over_100ms']} / {counts['over_200ms']} / {counts['over_1s']}",
            f"  源时间最大间隔: {_format_number(header.get('max_gap_ms'), 3)} ms",
            f"  延迟 median/P95abs/maxabs: {_format_number(latency.get('median'), 3)} / "
            f"{_format_number(latency.get('p95_abs'), 3)} / "
            f"{_format_number(latency.get('max_abs'), 3)} ms",
            f"  无效姿态: {nokov.get('invalid_pose_count', 0)}",
        ])
        lines.append("  最大断流:")
        for gap in receive["top_gaps"][:5]:
            lines.append(
                f"    {gap['start_local']} -> {gap['end_local']}  "
                f"{gap['gap_ms']:.3f} ms"
            )
    else:
        lines.append("NOKOV: 不可用")

    if bridge.get("available"):
        lines.append(
            f"PX4桥接: {_format_number(bridge_receive['rate_hz'])} Hz, "
            f"最大间隔 {_format_number(bridge_receive['max_gap_ms'], 3)} ms"
        )
    else:
        lines.append("PX4桥接: 不可用")

    if px4.get("available"):
        lines.append(
            f"PX4状态: dead-reckoning {px4['dead_reckoning_count']}/"
            f"{px4['local_position_sample_count']}, reset总数 {px4['reset_total']} "
            f"(xy={resets['xy']}, z={resets['z']}, vxy={resets['vxy']}, "
            f"vz={resets['vz']}, heading={resets['heading']})"
        )
    else:
        lines.append(
            "PX4状态: 不可用；请先 source 飞行工作空间 install/setup.bash 后重跑"
        )

    if dynamic.get("available"):
        lines.append(
            f"动态比例 mean/P95/max: {_format_number(dynamic['mean_percent'], 4)} / "
            f"{_format_number(dynamic['p95_percent'], 4)} / "
            f"{_format_number(dynamic['max_percent'], 4)} %"
        )
    if dds.get("available"):
        lines.append(
            f"UDP RcvbufErrors: 总增量 {dds['total_delta']}, "
            f"正式录制期间增量 {dds['recording_delta']}"
        )
    gate = case["gate"]
    failures = ", ".join(gate["failures"]) if gate["failures"] else "none"
    lines.append(
        f"严格门槛(max_gap<={max_gap_ms:g}ms): {gate['result']}  failures={failures}"
    )
    for warning in case.get("warnings", []):
        lines.append(f"WARNING: {warning}")
    return "\n".join(lines)


def _render_comparison(comparison: dict[str, object]) -> str:
    lines = [
        f"\n=== 对比: {comparison['baseline']} -> {comparison['candidate']} ===",
    ]
    labels = (
        ("NOKOV最大间隔(ms)", "nokov_max_gap_ms"),
        ("PX4桥接最大间隔(ms)", "bridge_max_gap_ms"),
        ("dead-reckoning样本", "dead_reckoning_count"),
        ("reset总数", "reset_total"),
    )
    for label, key in labels:
        value = comparison[key]
        improvement = value["improvement_percent"]
        improvement_text = "N/A" if improvement is None else f"{improvement:.2f}%"
        lines.append(
            f"{label}: {value['before']:.3f} -> {value['after']:.3f}, "
            f"改善 {improvement_text}"
        )
    return "\n".join(lines)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="离线分析并对比 Super-LIO 测试中的 NOKOV 断流质量。",
    )
    parser.add_argument(
        "cases",
        nargs="+",
        type=Path,
        metavar="TEST_DIR",
        help="一个或多个 run.sh 测试目录；第一个目录作为对比基线",
    )
    parser.add_argument(
        "--max-gap-ms",
        type=float,
        default=50.0,
        help="严格飞行门槛允许的最大 NOKOV/PX4桥接间隔（默认 50 ms）",
    )
    parser.add_argument(
        "--json",
        type=Path,
        dest="json_path",
        help="可选：把完整机器可读结果写入该 JSON 文件",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if not math.isfinite(args.max_gap_ms) or args.max_gap_ms <= 0.0:
        print("ERROR: --max-gap-ms must be a positive finite number", file=sys.stderr)
        return 2
    try:
        cases = [analyze_case(path, args.max_gap_ms) for path in args.cases]
        comparisons = [
            compare_case_metrics(cases[0], candidate)
            for candidate in cases[1:]
        ]
    except (RuntimeError, ValueError, OSError, KeyError) as error:
        print(f"ERROR: NOKOV analysis failed: {error}", file=sys.stderr)
        return 1

    for case in cases:
        print(_render_case(case, args.max_gap_ms))
    for comparison in comparisons:
        print(_render_comparison(comparison))

    result = {
        "schema_version": 1,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "max_gap_ms": args.max_gap_ms,
        "cases": cases,
        "comparisons": comparisons,
    }
    if args.json_path:
        args.json_path.parent.mkdir(parents=True, exist_ok=True)
        args.json_path.write_text(
            json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(f"\nJSON: {args.json_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
