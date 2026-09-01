#!/usr/bin/env python3

import argparse
import json
import math
import shutil
import statistics
import subprocess
import sys
import time
from pathlib import Path
from typing import Mapping, Sequence


def select_workspace(detected: Path, fallback: Path, override: str) -> Path:
    if override:
        return Path(override)
    if (detected / "install" / "ros2_px4_odometry").is_dir():
        return detected
    return fallback


def snapshot_file(source: Path, destination: Path) -> Path:
    destination.mkdir(parents=True, exist_ok=True)
    return Path(shutil.copy2(
        source,
        destination / source.name,
        follow_symlinks=True,
    ))


def clock_is_synchronized() -> bool:
    result = subprocess.run(
        ["timedatectl", "show", "-p", "NTPSynchronized", "--value"],
        check=False,
        capture_output=True,
        text=True,
        timeout=5,
    )
    return result.returncode == 0 and result.stdout.strip() == "yes"


def percentile(values: Sequence[float], fraction: float) -> float:
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(fraction * len(ordered)))]


def evaluate_nokov(payload: Mapping[str, object]) -> tuple[str, bool]:
    arrivals = [float(value) for value in payload["arrivals"]]  # type: ignore[index]
    stamps_ns = [int(value) for value in payload["stamps_ns"]]  # type: ignore[index]
    latencies_ms = [float(value) for value in payload["latencies_ms"]]  # type: ignore[index]
    invalid_samples = int(payload["invalid_samples"])
    sample_count = len(arrivals)
    if sample_count < 20:
        return f"samples={sample_count} result=FAIL failures=no_live_nokov", False
    duration = arrivals[-1] - arrivals[0]
    rate_hz = (sample_count - 1) / duration if duration > 0.0 else 0.0
    regressions = sum(curr < prev for prev, curr in zip(stamps_ns, stamps_ns[1:]))
    duplicates = sum(curr == prev for prev, curr in zip(stamps_ns, stamps_ns[1:]))
    zero_stamps = sum(stamp == 0 for stamp in stamps_ns)
    median_ms = statistics.median(latencies_ms)
    absolute_latencies = [abs(value) for value in latencies_ms]
    p95_abs_ms = percentile(absolute_latencies, 0.95)
    max_abs_ms = max(absolute_latencies)
    failures = []
    if rate_hz < 20.0:
        failures.append("low_rate")
    if regressions > 0:
        failures.append("timestamp_regression")
    if duplicates > max(1, len(stamps_ns) // 20):
        failures.append("duplicate_timestamps")
    if zero_stamps > 0:
        failures.append("zero_timestamp")
    if invalid_samples > 0:
        failures.append("invalid_pose")
    if median_ms < -250.0 or median_ms > 150.0:
        failures.append("median_latency")
    if p95_abs_ms > 250.0:
        failures.append("p95_latency")
    if max_abs_ms > 300.0:
        failures.append("max_latency")
    passed = not failures
    failure_text = "none" if passed else ",".join(failures)
    metrics = (
        f"samples={sample_count} rate_hz={rate_hz:.2f} "
        f"median_latency_ms={median_ms:.3f} p95_abs_latency_ms={p95_abs_ms:.3f} "
        f"max_abs_latency_ms={max_abs_ms:.3f} regressions={regressions} "
        f"duplicates={duplicates} zero_stamps={zero_stamps} "
        f"invalid_poses={invalid_samples} result={'PASS' if passed else 'FAIL'} "
        f"failures={failure_text}"
    )
    return metrics, passed


def evaluate_imu(payload: Mapping[str, object]) -> tuple[str, bool]:
    arrivals = [float(value) for value in payload["arrivals"]]  # type: ignore[index]
    stamps_ns = [int(value) for value in payload["stamps_ns"]]  # type: ignore[index]
    latencies_ms = [float(value) for value in payload["latencies_ms"]]  # type: ignore[index]
    sample_count = len(arrivals)
    if sample_count < 20:
        return f"samples={sample_count} result=FAIL failures=no_live_imu", False
    duration = arrivals[-1] - arrivals[0]
    rate_hz = (sample_count - 1) / duration if duration > 0.0 else 0.0
    regressions = sum(curr <= prev for prev, curr in zip(stamps_ns, stamps_ns[1:]))
    zero_stamps = sum(stamp == 0 for stamp in stamps_ns)
    median_ms = statistics.median(latencies_ms)
    absolute_latencies = [abs(value) for value in latencies_ms]
    p95_abs_ms = percentile(absolute_latencies, 0.95)
    max_abs_ms = max(absolute_latencies)
    failures = []
    if rate_hz < 50.0:
        failures.append("low_rate")
    if regressions > 0:
        failures.append("timestamp_regression")
    if zero_stamps > 0:
        failures.append("zero_timestamp")
    if median_ms < -5.0 or median_ms > 5.0:
        failures.append("median_latency")
    if p95_abs_ms > 10.0:
        failures.append("p95_latency")
    if max_abs_ms > 20.0:
        failures.append("max_latency")
    passed = not failures
    failure_text = "none" if passed else ",".join(failures)
    metrics = (
        f"samples={sample_count} rate_hz={rate_hz:.2f} "
        f"median_latency_ms={median_ms:.3f} p95_abs_latency_ms={p95_abs_ms:.3f} "
        f"max_abs_latency_ms={max_abs_ms:.3f} regressions={regressions} "
        f"zero_stamps={zero_stamps} result={'PASS' if passed else 'FAIL'} "
        f"failures={failure_text}"
    )
    return metrics, passed


def evaluate_rate(
    payload: Mapping[str, object],
    name: str,
    min_samples: int,
    min_rate_hz: float,
) -> tuple[str, bool]:
    arrivals = [float(value) for value in payload["arrivals"]]  # type: ignore[index]
    sample_count = len(arrivals)
    if sample_count < min_samples:
        return (
            f"stream={name} samples={sample_count} result=FAIL failures=no_live_stream",
            False,
        )
    duration = arrivals[-1] - arrivals[0]
    rate_hz = (sample_count - 1) / duration if duration > 0.0 else 0.0
    failures = []
    if rate_hz < min_rate_hz:
        failures.append("low_rate")
    passed = not failures
    failure_text = "none" if passed else ",".join(failures)
    metrics = (
        f"stream={name} samples={sample_count} rate_hz={rate_hz:.2f} "
        f"result={'PASS' if passed else 'FAIL'} failures={failure_text}"
    )
    return metrics, passed


def collect_imu() -> tuple[str, bool]:
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu

    rclpy.init()
    node = rclpy.create_node("flight_runner_imu_timestamp_gate")
    arrivals: list[float] = []
    stamps_ns: list[int] = []
    latencies_ms: list[float] = []

    def callback(msg: Imu) -> None:
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        arrivals.append(time.monotonic())
        stamps_ns.append(stamp_ns)
        latencies_ms.append((node.get_clock().now().nanoseconds - stamp_ns) / 1_000_000)

    subscription = node.create_subscription(
        Imu, "/livox/imu", callback, qos_profile_sensor_data)
    deadline = time.monotonic() + 5.0
    while len(arrivals) < 100 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_subscription(subscription)
    node.destroy_node()
    rclpy.shutdown()
    return evaluate_imu({
        "arrivals": arrivals,
        "stamps_ns": stamps_ns,
        "latencies_ms": latencies_ms,
    })


def collect_nokov() -> tuple[str, bool]:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init()
    node = rclpy.create_node("flight_runner_nokov_timestamp_gate")
    arrivals: list[float] = []
    stamps_ns: list[int] = []
    latencies_ms: list[float] = []
    invalid_samples = 0

    def callback(msg: PoseStamped) -> None:
        nonlocal invalid_samples
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        arrivals.append(time.monotonic())
        stamps_ns.append(stamp_ns)
        latencies_ms.append((node.get_clock().now().nanoseconds - stamp_ns) / 1_000_000)
        values = (
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w,
        )
        quaternion_norm = math.sqrt(sum(value * value for value in values[3:]))
        if (
            not all(math.isfinite(value) for value in values)
            or not 0.5 < quaternion_norm < 1.5
        ):
            invalid_samples += 1

    subscription = node.create_subscription(
        PoseStamped, "/Robot_3/pose", callback, qos_profile_sensor_data)
    deadline = time.monotonic() + 6.0
    while len(arrivals) < 120 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_subscription(subscription)
    node.destroy_node()
    rclpy.shutdown()
    return evaluate_nokov({
        "arrivals": arrivals,
        "stamps_ns": stamps_ns,
        "latencies_ms": latencies_ms,
        "invalid_samples": invalid_samples,
    })


def collect_rate(kind: str) -> tuple[str, bool]:
    import rclpy
    from rclpy.qos import qos_profile_sensor_data

    if kind == "lidar":
        from livox_ros_driver2.msg import CustomMsg as MessageType

        topic = "/livox/lidar"
        name = "livox_lidar"
        target_samples = 40
        deadline_seconds = 6.0
        min_samples = 20
        min_rate_hz = 5.0
    else:
        from px4_msgs.msg import VehicleStatus as MessageType

        topic = "/fmu/out/vehicle_status_v1"
        name = "px4_vehicle_status"
        target_samples = 10
        deadline_seconds = 8.0
        min_samples = 5
        min_rate_hz = 1.0

    rclpy.init()
    node = rclpy.create_node(f"flight_runner_{name}_gate")
    arrivals: list[float] = []

    def callback(_msg: object) -> None:
        arrivals.append(time.monotonic())

    subscription = node.create_subscription(
        MessageType, topic, callback, qos_profile_sensor_data)
    deadline = time.monotonic() + deadline_seconds
    while len(arrivals) < target_samples and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_subscription(subscription)
    node.destroy_node()
    rclpy.shutdown()
    return evaluate_rate(
        {"arrivals": arrivals}, name, min_samples, min_rate_hz)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)
    workspace = subparsers.add_parser("select-workspace")
    workspace.add_argument("--detected", type=Path, required=True)
    workspace.add_argument("--fallback", type=Path, required=True)
    workspace.add_argument("--override", default="")
    snapshot = subparsers.add_parser("snapshot-file")
    snapshot.add_argument("--source", type=Path, required=True)
    snapshot.add_argument("--destination", type=Path, required=True)
    subparsers.add_parser("clock-synchronized")
    evaluate = subparsers.add_parser("evaluate")
    evaluate.add_argument("kind", choices=("imu", "nokov", "rate"))
    evaluate.add_argument("--name", default="stream")
    evaluate.add_argument("--min-samples", type=int, default=1)
    evaluate.add_argument("--min-rate-hz", type=float, default=0.0)
    live = subparsers.add_parser("live")
    live.add_argument("kind", choices=("imu", "nokov", "lidar", "px4-status"))
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.command == "select-workspace":
        print(select_workspace(args.detected, args.fallback, args.override))
        return 0
    if args.command == "snapshot-file":
        print(snapshot_file(args.source, args.destination))
        return 0
    if args.command == "clock-synchronized":
        synchronized = clock_is_synchronized()
        print("yes" if synchronized else "no")
        return 0 if synchronized else 3
    if args.command == "evaluate":
        payload = json.load(sys.stdin)
        if args.kind == "imu":
            metrics, passed = evaluate_imu(payload)
        elif args.kind == "nokov":
            metrics, passed = evaluate_nokov(payload)
        else:
            metrics, passed = evaluate_rate(
                payload, args.name, args.min_samples, args.min_rate_hz)
        print(metrics)
        return 0 if passed else 3
    if args.command == "live":
        if args.kind == "imu":
            metrics, passed = collect_imu()
        elif args.kind == "nokov":
            metrics, passed = collect_nokov()
        else:
            metrics, passed = collect_rate(args.kind)
        print(metrics)
        return 0 if passed else 3
    raise AssertionError(f"Unhandled command: {args.command}")


if __name__ == "__main__":
    raise SystemExit(main())
