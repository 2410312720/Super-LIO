#!/usr/bin/env python3

import argparse
import json
import sys
from pathlib import Path
from typing import Sequence

from localization_analysis.analysis import analyze_session
from localization_analysis import discover_cases
from localization_analysis.report import write_reports


def parse_air_window_overrides(
    values: Sequence[str],
) -> dict[str, tuple[float, float]]:
    overrides: dict[str, tuple[float, float]] = {}
    for value in values:
        try:
            case_name, interval = value.split("=", 1)
            start_text, end_text = interval.split(",", 1)
            start, end = float(start_text), float(end_text)
        except ValueError as error:
            raise ValueError(
                f"invalid air window '{value}'; expected CASE=START,END"
            ) from error
        if not case_name:
            raise ValueError("air-window case name must not be empty")
        if end <= start:
            raise ValueError(f"air-window end must be greater than start: {value}")
        overrides[case_name] = (start, end)
    return overrides


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Analyze Super-LIO localization flight-test data.",
    )
    parser.add_argument("data_root", type=Path)
    parser.add_argument(
        "--discover-only",
        action="store_true",
        help="Print discovered test inputs as JSON without reading the bags.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Report directory (default: DATA_ROOT/analysis).",
    )
    parser.add_argument(
        "--air-window",
        action="append",
        default=[],
        metavar="CASE=START,END",
        help=(
            "Override an airborne epoch interval for a case name or mode; "
            "may be repeated."
        ),
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if not args.data_root.is_dir():
        print(
            f"ERROR: data root does not exist: {args.data_root}",
            file=sys.stderr,
        )
        return 2
    cases = discover_cases(args.data_root)
    if not cases:
        print(
            f"ERROR: no run.sh test cases found below: {args.data_root}",
            file=sys.stderr,
        )
        return 2
    if args.discover_only:
        print(json.dumps(
            {"cases": [case.to_dict() for case in cases]},
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
        ))
        return 0
    output_dir = args.output or args.data_root / "analysis"
    try:
        air_windows = parse_air_window_overrides(args.air_window)
        metrics = analyze_session(args.data_root, cases, air_windows)
        json_path, markdown_path = write_reports(metrics, output_dir)
    except (RuntimeError, ValueError, KeyError, OSError) as error:
        print(f"ERROR: localization analysis failed: {error}", file=sys.stderr)
        return 1

    print(f"Analyzed {len(cases)} case(s).")
    for case in metrics["cases"]:
        trajectory = case.get("trajectory", {})
        if trajectory and trajectory.get("available") is not False:
            position = trajectory["position"]
            print(
                f"  {case['name']}: position_rmse_cm={position['rmse_m'] * 100:.2f} "
                f"p95_cm={position['p95_m'] * 100:.2f} "
                f"drift_cm={trajectory['drift_norm_m'] * 100:.2f}"
            )
        for warning in case.get("warnings", []):
            print(f"  WARNING {case['name']}: {warning}")
    print(f"Markdown report: {markdown_path}")
    print(f"JSON metrics: {json_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
