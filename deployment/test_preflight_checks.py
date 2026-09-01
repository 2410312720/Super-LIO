import json
import os
import subprocess
import sys
from pathlib import Path


CHECKER = Path(__file__).with_name("preflight_checks.py")


def run_checker(
    *args: str,
    payload: dict[str, object] | None = None,
    environ: dict[str, str] | None = None,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(CHECKER), *args],
        check=False,
        capture_output=True,
        input=json.dumps(payload) if payload is not None else None,
        text=True,
        env=environ,
    )


def test_complete_detected_overlay_is_selected_before_fallback(tmp_path: Path) -> None:
    detected = tmp_path / "merge_test"
    fallback = tmp_path / "ws_super_spline"
    (detected / "install" / "ros2_px4_odometry").mkdir(parents=True)
    fallback.mkdir()

    result = run_checker(
        "select-workspace",
        "--detected", str(detected),
        "--fallback", str(fallback),
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == str(detected)


def test_explicit_workspace_override_wins(tmp_path: Path) -> None:
    detected = tmp_path / "merge_test"
    fallback = tmp_path / "ws_super_spline"
    override = tmp_path / "manual_workspace"
    (detected / "install" / "ros2_px4_odometry").mkdir(parents=True)

    result = run_checker(
        "select-workspace",
        "--detected", str(detected),
        "--fallback", str(fallback),
        "--override", str(override),
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == str(override)


def test_incomplete_detected_overlay_falls_back(tmp_path: Path) -> None:
    detected = tmp_path / "super_lio_only"
    fallback = tmp_path / "ws_super_spline"
    detected.mkdir()
    fallback.mkdir()

    result = run_checker(
        "select-workspace",
        "--detected", str(detected),
        "--fallback", str(fallback),
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == str(fallback)


def test_snapshot_file_dereferences_symlink_and_can_be_refreshed(
    tmp_path: Path,
) -> None:
    source_dir = tmp_path / "source"
    install_dir = tmp_path / "install"
    snapshot_dir = tmp_path / "snapshot"
    source_dir.mkdir()
    install_dir.mkdir()
    snapshot_dir.mkdir()
    source_config = source_dir / "flight.yaml"
    installed_config = install_dir / "flight.yaml"
    source_config.write_text("profile: shadow\n", encoding="utf-8")
    installed_config.symlink_to(source_config)

    installed_result = run_checker(
        "snapshot-file",
        "--source", str(installed_config),
        "--destination", str(snapshot_dir),
    )
    source_result = run_checker(
        "snapshot-file",
        "--source", str(source_config),
        "--destination", str(snapshot_dir),
    )

    assert installed_result.returncode == 0, installed_result.stderr
    assert source_result.returncode == 0, source_result.stderr
    snapshot = snapshot_dir / "flight.yaml"
    assert snapshot.read_text(encoding="utf-8") == "profile: shadow\n"
    assert not snapshot.is_symlink()


def test_clock_gate_uses_ntp_synchronized_property(tmp_path: Path) -> None:
    fake_timedatectl = tmp_path / "timedatectl"
    fake_timedatectl.write_text(
        "#!/usr/bin/env bash\n"
        "if [[ \"$*\" == 'show -p NTPSynchronized --value' ]]; then\n"
        "  printf 'yes\\n'\n"
        "else\n"
        "  printf 'no\\n'\n"
        "fi\n",
        encoding="utf-8",
    )
    fake_timedatectl.chmod(0o755)
    environment = os.environ.copy()
    environment["PATH"] = f"{tmp_path}:{environment['PATH']}"

    result = run_checker("clock-synchronized", environ=environment)

    assert result.returncode == 0, result.stdout + result.stderr
    assert result.stdout.strip() == "yes"


def test_nokov_operational_future_offset_passes() -> None:
    payload = {
        "arrivals": [index / 180.0 for index in range(120)],
        "stamps_ns": [1_000_000_000 + index * 5_555_556 for index in range(120)],
        "latencies_ms": [-171.0] * 120,
        "invalid_samples": 0,
    }

    result = run_checker("evaluate", "nokov", payload=payload)

    assert result.returncode == 0, result.stdout + result.stderr
    assert "result=PASS" in result.stdout


def test_nokov_excessive_future_offset_fails() -> None:
    payload = {
        "arrivals": [index / 180.0 for index in range(120)],
        "stamps_ns": [1_000_000_000 + index * 5_555_556 for index in range(120)],
        "latencies_ms": [-350.0] * 120,
        "invalid_samples": 0,
    }

    result = run_checker("evaluate", "nokov", payload=payload)

    assert result.returncode == 3, result.stdout + result.stderr
    assert "median_latency" in result.stdout


def test_nokov_sustained_latency_tail_fails() -> None:
    payload = {
        "arrivals": [index / 180.0 for index in range(120)],
        "stamps_ns": [1_000_000_000 + index * 5_555_556 for index in range(120)],
        "latencies_ms": [-171.0] * 114 + [-280.0] * 6,
        "invalid_samples": 0,
    }

    result = run_checker("evaluate", "nokov", payload=payload)

    assert result.returncode == 3, result.stdout + result.stderr
    assert "p95_latency" in result.stdout


def test_nokov_single_extreme_latency_fails() -> None:
    payload = {
        "arrivals": [index / 180.0 for index in range(120)],
        "stamps_ns": [1_000_000_000 + index * 5_555_556 for index in range(120)],
        "latencies_ms": [-171.0] * 119 + [-310.0],
        "invalid_samples": 0,
    }

    result = run_checker("evaluate", "nokov", payload=payload)

    assert result.returncode == 3, result.stdout + result.stderr
    assert "max_latency" in result.stdout


def test_nokov_low_rate_fails() -> None:
    payload = {
        "arrivals": [index / 5.0 for index in range(120)],
        "stamps_ns": [1_000_000_000 + index * 200_000_000 for index in range(120)],
        "latencies_ms": [-171.0] * 120,
        "invalid_samples": 0,
    }

    result = run_checker("evaluate", "nokov", payload=payload)

    assert result.returncode == 3, result.stdout + result.stderr
    assert "low_rate" in result.stdout


def test_nokov_timestamp_regression_fails() -> None:
    stamps_ns = [1_000_000_000 + index * 5_555_556 for index in range(120)]
    stamps_ns[60] = stamps_ns[59] - 1
    payload = {
        "arrivals": [index / 180.0 for index in range(120)],
        "stamps_ns": stamps_ns,
        "latencies_ms": [-171.0] * 120,
        "invalid_samples": 0,
    }

    result = run_checker("evaluate", "nokov", payload=payload)

    assert result.returncode == 3, result.stdout + result.stderr
    assert "timestamp_regression" in result.stdout


def test_nokov_excessive_duplicate_timestamps_fail() -> None:
    stamps_ns = [1_000_000_000] * 8 + [
        1_000_000_000 + index * 5_555_556 for index in range(8, 120)
    ]
    payload = {
        "arrivals": [index / 180.0 for index in range(120)],
        "stamps_ns": stamps_ns,
        "latencies_ms": [-171.0] * 120,
        "invalid_samples": 0,
    }

    result = run_checker("evaluate", "nokov", payload=payload)

    assert result.returncode == 3, result.stdout + result.stderr
    assert "duplicate_timestamps" in result.stdout


def test_nokov_zero_timestamp_fails() -> None:
    stamps_ns = [0] + [1_000_000_000 + index * 5_555_556 for index in range(1, 120)]
    payload = {
        "arrivals": [index / 180.0 for index in range(120)],
        "stamps_ns": stamps_ns,
        "latencies_ms": [-171.0] * 120,
        "invalid_samples": 0,
    }

    result = run_checker("evaluate", "nokov", payload=payload)

    assert result.returncode == 3, result.stdout + result.stderr
    assert "zero_timestamp" in result.stdout


def test_nokov_invalid_pose_fails() -> None:
    payload = {
        "arrivals": [index / 180.0 for index in range(120)],
        "stamps_ns": [1_000_000_000 + index * 5_555_556 for index in range(120)],
        "latencies_ms": [-171.0] * 120,
        "invalid_samples": 1,
    }

    result = run_checker("evaluate", "nokov", payload=payload)

    assert result.returncode == 3, result.stdout + result.stderr
    assert "invalid_pose" in result.stdout


def test_nokov_too_few_samples_fails() -> None:
    payload = {
        "arrivals": [index / 180.0 for index in range(10)],
        "stamps_ns": [1_000_000_000 + index * 5_555_556 for index in range(10)],
        "latencies_ms": [-171.0] * 10,
        "invalid_samples": 0,
    }

    result = run_checker("evaluate", "nokov", payload=payload)

    assert result.returncode == 3, result.stdout + result.stderr
    assert "no_live_nokov" in result.stdout


def test_imu_board_profile_passes() -> None:
    payload = {
        "arrivals": [index / 200.0 for index in range(100)],
        "stamps_ns": [1_000_000_000 + index * 5_000_000 for index in range(100)],
        "latencies_ms": [1.5] * 100,
    }

    result = run_checker("evaluate", "imu", payload=payload)

    assert result.returncode == 0, result.stdout + result.stderr
    assert "result=PASS" in result.stdout


def test_imu_future_timestamps_fail() -> None:
    payload = {
        "arrivals": [index / 200.0 for index in range(100)],
        "stamps_ns": [1_000_000_000 + index * 5_000_000 for index in range(100)],
        "latencies_ms": [-100.0] * 100,
    }

    result = run_checker("evaluate", "imu", payload=payload)

    assert result.returncode == 3, result.stdout + result.stderr
    assert "median_latency" in result.stdout


def test_imu_sustained_latency_tail_fails() -> None:
    payload = {
        "arrivals": [index / 200.0 for index in range(100)],
        "stamps_ns": [1_000_000_000 + index * 5_000_000 for index in range(100)],
        "latencies_ms": [1.5] * 95 + [12.0] * 5,
    }

    result = run_checker("evaluate", "imu", payload=payload)

    assert result.returncode == 3, result.stdout + result.stderr
    assert "p95_latency" in result.stdout


def test_imu_single_extreme_latency_fails() -> None:
    payload = {
        "arrivals": [index / 200.0 for index in range(100)],
        "stamps_ns": [1_000_000_000 + index * 5_000_000 for index in range(100)],
        "latencies_ms": [1.5] * 99 + [25.0],
    }

    result = run_checker("evaluate", "imu", payload=payload)

    assert result.returncode == 3, result.stdout + result.stderr
    assert "max_latency" in result.stdout


def test_imu_low_rate_fails() -> None:
    payload = {
        "arrivals": [index / 20.0 for index in range(100)],
        "stamps_ns": [1_000_000_000 + index * 50_000_000 for index in range(100)],
        "latencies_ms": [1.5] * 100,
    }

    result = run_checker("evaluate", "imu", payload=payload)

    assert result.returncode == 3, result.stdout + result.stderr
    assert "low_rate" in result.stdout


def test_imu_non_monotonic_timestamp_fails() -> None:
    stamps_ns = [1_000_000_000 + index * 5_000_000 for index in range(100)]
    stamps_ns[60] = stamps_ns[59]
    payload = {
        "arrivals": [index / 200.0 for index in range(100)],
        "stamps_ns": stamps_ns,
        "latencies_ms": [1.5] * 100,
    }

    result = run_checker("evaluate", "imu", payload=payload)

    assert result.returncode == 3, result.stdout + result.stderr
    assert "timestamp_regression" in result.stdout


def test_imu_zero_timestamp_fails() -> None:
    payload = {
        "arrivals": [index / 200.0 for index in range(100)],
        "stamps_ns": [0] + [1_000_000_000 + index * 5_000_000 for index in range(1, 100)],
        "latencies_ms": [1.5] * 100,
    }

    result = run_checker("evaluate", "imu", payload=payload)

    assert result.returncode == 3, result.stdout + result.stderr
    assert "zero_timestamp" in result.stdout


def test_imu_too_few_samples_fails() -> None:
    payload = {
        "arrivals": [index / 200.0 for index in range(10)],
        "stamps_ns": [1_000_000_000 + index * 5_000_000 for index in range(10)],
        "latencies_ms": [1.5] * 10,
    }

    result = run_checker("evaluate", "imu", payload=payload)

    assert result.returncode == 3, result.stdout + result.stderr
    assert "no_live_imu" in result.stdout


def test_lidar_live_rate_passes() -> None:
    payload = {"arrivals": [index / 10.0 for index in range(40)]}

    result = run_checker(
        "evaluate", "rate",
        "--name", "livox_lidar",
        "--min-samples", "20",
        "--min-rate-hz", "5",
        payload=payload,
    )

    assert result.returncode == 0, result.stdout + result.stderr
    assert "result=PASS" in result.stdout


def test_lidar_low_rate_fails() -> None:
    payload = {"arrivals": [float(index) for index in range(40)]}

    result = run_checker(
        "evaluate", "rate",
        "--name", "livox_lidar",
        "--min-samples", "20",
        "--min-rate-hz", "5",
        payload=payload,
    )

    assert result.returncode == 3, result.stdout + result.stderr
    assert "low_rate" in result.stdout


def test_px4_status_too_few_samples_fails() -> None:
    payload = {"arrivals": [index / 2.0 for index in range(4)]}

    result = run_checker(
        "evaluate", "rate",
        "--name", "px4_vehicle_status",
        "--min-samples", "5",
        "--min-rate-hz", "1",
        payload=payload,
    )

    assert result.returncode == 3, result.stdout + result.stderr
    assert "no_live_stream" in result.stdout
