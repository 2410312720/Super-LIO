#!/usr/bin/env python3
"""Behavioral regression tests for run.sh dynamic-filter mode handling."""

from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


DEPLOYMENT_DIR = Path(__file__).resolve().parent
RUNNER = DEPLOYMENT_DIR / "run.sh"
MAIN_MARKER = '\ncase "${1:-}" in\n'


class RunDynamicModeTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp_dir = Path(tempfile.mkdtemp(prefix="superlio-run-test-"))
        source = RUNNER.read_text(encoding="utf-8")
        library, marker, _main = source.partition(MAIN_MARKER)
        self.assertEqual(marker, MAIN_MARKER, "run.sh main entry marker changed")
        self.library = self.temp_dir / "run_library.sh"
        self.library.write_text(library + "\n", encoding="utf-8")
        shutil.copy2(DEPLOYMENT_DIR / "preflight_checks.py", self.temp_dir)

    def tearDown(self) -> None:
        shutil.rmtree(self.temp_dir)

    def run_bash(self, body: str) -> subprocess.CompletedProcess[str]:
        environment = os.environ.copy()
        environment.update(
            {
                "FLIGHT_WORKSPACE": "/home/cat/ws_super_spline",
                "SUPER_LIO_OVERLAY": "/home/cat/ws_super_spline",
            }
        )
        return subprocess.run(
            ["bash", "-c", f'source "$1"\n{body}', "bash", str(self.library)],
            check=False,
            capture_output=True,
            text=True,
            env=environment,
        )

    def test_baseline_configures_dynamic_filter_as_enabled(self) -> None:
        result = self.run_bash(
            """
RUN_MODE=baseline
NOKOV_SHADOW_ENABLED=false
configure_mode
printf '%s\n' "$EXPECTED_DYNAMIC_ENABLE"
"""
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), "true")

    def test_baseline_waits_for_dynamic_ratio_when_filter_is_expected(self) -> None:
        result = self.run_bash(
            f"""
RUN_MODE=baseline
EXPECTED_DYNAMIC_ENABLE=true
ODOM_BRIDGE_PID=101
SUPER_LIO_PID=102
TEST_DIR={self.temp_dir!s}
kill() {{ return 0; }}
sleep() {{ SECONDS=$((SECONDS + 100)); }}
safe_ros_topic_list() {{
  printf '%s\n' /lio/imu/odom /lio/odom /lio/robo/odom \\
    /lio/diagnostics/scan /fmu/in/vehicle_visual_odometry
}}
wait_for_stack
"""
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Timed out waiting for baseline topics", result.stderr)

    def test_baseline_records_dynamic_ratio_when_filter_is_expected(self) -> None:
        capture = self.temp_dir / "rosbag_arguments.txt"
        result = self.run_bash(
            f"""
RUN_MODE=baseline
EXPECTED_DYNAMIC_ENABLE=true
TEST_DIR={self.temp_dir!s}
CAPTURE={capture!s}
nokov_is_required() {{ return 1; }}
append_dynamic_debug_topics() {{ return 0; }}
setsid() {{ printf '%s\n' "$@" >"$CAPTURE"; }}
sleep() {{ command sleep 0.1; }}
kill() {{ return 0; }}
start_rosbag
grep -Fx /lio/dynamic_ratio "$CAPTURE"
"""
        )
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_enabled_dynamic_filter_validates_pose_and_map_gates(self) -> None:
        result = self.run_bash(
            """
EXPECTED_DYNAMIC_ENABLE=true
parameter_must_match() { printf '%s|%s|%s\n' "$1" "$2" "$3"; }
validate_dynamic_parameters
"""
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(
            result.stdout.strip().splitlines(),
            [
                "/super_lio_node|lio.dynamic.enable|true",
                "/super_lio_node|lio.dynamic.filter_pose_update|true",
                "/super_lio_node|lio.dynamic.filter_map_update|true",
            ],
        )


if __name__ == "__main__":
    unittest.main()
