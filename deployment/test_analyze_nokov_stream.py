#!/usr/bin/env python3

import importlib.util
import unittest
from pathlib import Path


SCRIPT_PATH = Path(__file__).with_name("analyze_nokov_stream.py")


def load_script_module():
    if not SCRIPT_PATH.is_file():
        raise AssertionError(f"analysis script is missing: {SCRIPT_PATH}")
    spec = importlib.util.spec_from_file_location("analyze_nokov_stream", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise AssertionError(f"cannot load analysis script: {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class StreamSummaryTest(unittest.TestCase):
    def test_counts_long_gaps_and_preserves_the_largest_gap_interval(self) -> None:
        module = load_script_module()

        summary = module.summarize_timestamps(
            [1000.0, 1000.005, 1000.065, 1001.265],
            top_n=2,
        )

        self.assertEqual(summary["sample_count"], 4)
        self.assertAlmostEqual(summary["max_gap_ms"], 1200.0)
        self.assertEqual(summary["gap_counts"], {
            "over_20ms": 2,
            "over_50ms": 2,
            "over_100ms": 1,
            "over_200ms": 1,
            "over_1s": 1,
        })
        self.assertEqual(len(summary["top_gaps"]), 2)
        self.assertAlmostEqual(summary["top_gaps"][0]["start_epoch_s"], 1000.065)
        self.assertAlmostEqual(summary["top_gaps"][0]["end_epoch_s"], 1001.265)

    def test_reports_timestamp_regressions_without_treating_them_as_gaps(self) -> None:
        module = load_script_module()

        summary = module.summarize_timestamps([10.0, 10.005, 10.004, 10.010])

        self.assertEqual(summary["regressions"], 1)
        self.assertEqual(summary["duplicates"], 0)
        self.assertAlmostEqual(summary["max_gap_ms"], 6.0)


class DdsMonitorTest(unittest.TestCase):
    def test_separates_startup_errors_from_recording_errors(self) -> None:
        module = load_script_module()
        monitor = """\
TIME 90.0 2026-09-01 10:00:00
UdpRcvbufErrors 10 0.0
TIME 100.0 2026-09-01 10:00:10
UdpRcvbufErrors 12 0.0
TIME 105.0 2026-09-01 10:00:15
UdpRcvbufErrors 20 0.0
TIME 110.0 2026-09-01 10:00:20
UdpRcvbufErrors 20 0.0
"""

        result = module.parse_dds_monitor(monitor, 100.0, 108.0)

        self.assertEqual(result["total_delta"], 10)
        self.assertEqual(result["recording_delta"], 8)
        self.assertEqual(result["recording_start_value"], 12)
        self.assertEqual(result["recording_end_value"], 20)


class FlightGateTest(unittest.TestCase):
    @staticmethod
    def case(max_gap_ms: float = 40.0) -> dict[str, object]:
        return {
            "nokov": {"available": True, "receive": {"max_gap_ms": max_gap_ms}},
            "bridge": {"available": True, "receive": {"max_gap_ms": 45.0}},
            "px4": {
                "available": True,
                "dead_reckoning_count": 0,
                "reset_changes": {"xy": 0, "z": 0, "vxy": 0, "vz": 0, "heading": 0},
            },
        }

    def test_passes_only_when_stream_and_px4_state_meet_the_strict_gate(self) -> None:
        module = load_script_module()

        gate = module.evaluate_flight_gate(self.case(), max_gap_ms=50.0)

        self.assertEqual(gate, {"result": "PASS", "failures": []})

    def test_fails_for_a_nokov_gap_over_the_configured_limit(self) -> None:
        module = load_script_module()

        gate = module.evaluate_flight_gate(self.case(126.6), max_gap_ms=50.0)

        self.assertEqual(gate["result"], "FAIL")
        self.assertEqual(gate["failures"], ["nokov_max_gap_over_50ms"])

    def test_marks_the_result_incomplete_when_px4_types_are_unavailable(self) -> None:
        module = load_script_module()
        case = self.case()
        case["px4"] = {"available": False}

        gate = module.evaluate_flight_gate(case, max_gap_ms=50.0)

        self.assertEqual(gate["result"], "INCOMPLETE")
        self.assertEqual(gate["failures"], ["px4_status_unavailable"])


class ComparisonTest(unittest.TestCase):
    def test_reports_max_gap_improvement_from_baseline_to_candidate(self) -> None:
        module = load_script_module()
        baseline = {
            "name": "TEST3",
            "nokov": {"receive": {"max_gap_ms": 9433.7}},
            "bridge": {"receive": {"max_gap_ms": 9433.2}},
            "px4": {"dead_reckoning_count": 728, "reset_total": 26},
        }
        candidate = {
            "name": "TEST5",
            "nokov": {"receive": {"max_gap_ms": 126.6}},
            "bridge": {"receive": {"max_gap_ms": 127.1}},
            "px4": {"dead_reckoning_count": 0, "reset_total": 0},
        }

        comparison = module.compare_case_metrics(baseline, candidate)

        self.assertEqual(comparison["baseline"], "TEST3")
        self.assertEqual(comparison["candidate"], "TEST5")
        self.assertAlmostEqual(
            comparison["nokov_max_gap_ms"]["improvement_percent"],
            98.658,
            places=3,
        )
        self.assertEqual(comparison["dead_reckoning_count"]["after"], 0)


if __name__ == "__main__":
    unittest.main()
