#!/usr/bin/env python3

import subprocess
import unittest
from pathlib import Path


RUN_SH = Path(__file__).with_name("run.sh")


def extract_shell_function(source: str, name: str) -> str:
    lines = source.splitlines()
    start = next(
        (index for index, line in enumerate(lines) if line.startswith(f"{name}() {{")),
        None,
    )
    if start is None:
        raise AssertionError(f"{name} is missing from {RUN_SH}")

    depth = 0
    body = []
    for line in lines[start:]:
        body.append(line)
        depth += line.count("{") - line.count("}")
        if depth == 0:
            return "\n".join(body)
    raise AssertionError(f"{name} is not a complete shell function")


class RunRecordingTopicsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        source = RUN_SH.read_text(encoding="utf-8")
        cls.functions = "\n\n".join(
            extract_shell_function(source, name)
            for name in (
                "runtime_boolean_parameter",
                "append_dynamic_debug_topics",
            )
        )

    def selected_topics(self, dynamic_enabled: bool, debug_enabled: bool) -> list[str]:
        dynamic_value = "True" if dynamic_enabled else "False"
        debug_value = "True" if debug_enabled else "False"
        harness = f"""
set -euo pipefail

die() {{
  printf 'ERROR: %s\\n' "$*" >&2
  return 1
}}

log() {{ :; }}

timeout() {{
  shift
  "$@"
}}

ros2() {{
  [[ "$1" == "param" && "$2" == "get" && "$3" == "/super_lio_node" ]]
  case "$4" in
    lio.dynamic.enable)
      printf 'Boolean value is: {dynamic_value}\\n'
      ;;
    lio.dynamic.publish_debug_clouds)
      printf 'Boolean value is: {debug_value}\\n'
      ;;
    *)
      return 2
      ;;
  esac
}}

{self.functions}

topics=(/base/topic)
append_dynamic_debug_topics topics
printf '%s\\n' "${{topics[@]}}"
"""
        result = subprocess.run(
            ["bash", "-c", harness],
            check=True,
            capture_output=True,
            text=True,
        )
        return result.stdout.splitlines()

    def test_records_debug_clouds_when_both_runtime_switches_are_enabled(self) -> None:
        self.assertEqual(
            self.selected_topics(dynamic_enabled=True, debug_enabled=True),
            [
                "/base/topic",
                "/lio/cloud_static",
                "/lio/cloud_dynamic",
                "/lio/cloud_unknown",
            ],
        )

    def test_omits_debug_clouds_when_debug_publishing_is_disabled(self) -> None:
        self.assertEqual(
            self.selected_topics(dynamic_enabled=True, debug_enabled=False),
            ["/base/topic"],
        )

    def test_omits_debug_clouds_when_dynamic_filter_is_disabled(self) -> None:
        self.assertEqual(
            self.selected_topics(dynamic_enabled=False, debug_enabled=True),
            ["/base/topic"],
        )


if __name__ == "__main__":
    unittest.main()
