import json
from pathlib import Path
from typing import Any


def _cm(value: float) -> str:
    return f"{value * 100.0:.2f}"


def _percent(value: float) -> str:
    return f"{value * 100.0:.3f}"


def render_markdown(metrics: dict[str, Any]) -> str:
    lines = [
        "# SuperLIO 定位质量分析",
        "",
        f"数据目录：`{metrics['data_root']}`  ",
        f"生成时间：`{metrics['generated_at']}`",
        "",
        "## 定位精度",
        "",
        "以下误差已经补偿时间偏移、世界坐标偏航/平移以及 NOKOV 与 LIO "
        "原点之间的固定机体系杆臂。",
        "",
        "| 测试 | 空中时长(s) | 中位(cm) | RMSE(cm) | P95(cm) | 最大(cm) | 漂移(cm) | 杆臂(cm) |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for case in metrics["cases"]:
        trajectory = case.get("trajectory", {})
        if not trajectory or trajectory.get("available") is False:
            continue
        position = trajectory["position"]
        lines.append(
            f"| {case['name']} ({case['mode']}) "
            f"| {case['airborne_window']['duration_s']:.1f} "
            f"| {_cm(position['median_m'])} "
            f"| {_cm(position['rmse_m'])} "
            f"| {_cm(position['p95_m'])} "
            f"| {_cm(position['max_m'])} "
            f"| {_cm(trajectory['drift_norm_m'])} "
            f"| {_cm(trajectory['body_lever_norm_m'])} |"
        )
    lines.extend(["", "### 轨迹对齐与姿态", ""])
    for case in metrics["cases"]:
        trajectory = case.get("trajectory", {})
        if not trajectory or trajectory.get("available") is False:
            continue
        rms = trajectory["orientation_rms_deg"]
        lines.extend([
            f"- **{case['name']}**：NOKOV-LIO 最优时间偏移 "
            f"`{trajectory['time_offset_s'] * 1000.0:+.1f} ms`，全局偏航 "
            f"`{trajectory['global_yaw_deg']:.2f}°`，尺度 `{trajectory['scale']:.5f}`；"
            f"姿态残差 RMS R/P/Y = `{rms[0]:.3f}° / {rms[1]:.3f}° / {rms[2]:.3f}°`。"
        ])
        lines.append(
            f"  - 1s/5s 相对位姿误差 P95 = "
            f"`{_cm(trajectory['rpe_1s']['p95_m'])} / "
            f"{_cm(trajectory['rpe_5s']['p95_m'])} cm`。"
        )

    lines.extend(["", "## NOKOV 时间质量", ""])
    for case in metrics["cases"]:
        nokov = case.get("streams", {}).get("nokov_timestamp")
        if not nokov:
            continue
        lines.append(
            f"- **{case['name']}**：平均频率 `{nokov['rate_hz']:.2f} Hz`，"
            f"`dt<1ms` 占 `{nokov['dt_lt_1ms_percent']:.2f}%`，"
            f"`dt>10ms` 占 `{nokov['dt_gt_10ms_percent']:.2f}%`，"
            f"最大间隔 `{nokov['dt_max_ms']:.2f} ms`，"
            f"回退 `{nokov['regressions']}`，重复 `{nokov['duplicates']}`。"
        )

    lines.extend(["", "## SuperLIO 实时输出", ""])
    for case in metrics["cases"]:
        streams = case.get("streams", {})
        timestamp = streams.get("lio_timestamp")
        lag = streams.get("lio_receive_lag_ms")
        if not timestamp:
            continue
        lag_text = (
            f"，接收延迟 P95 `{lag['p95']:.2f} ms`、最大 `{lag['max']:.2f} ms`"
            if lag else ""
        )
        scan = case.get("scan_diagnostics", {})
        scan_text = ""
        if scan.get("available") and scan.get("total_time_ms"):
            scan_text = (
                f"，扫描总耗时 P95 `{scan['total_time_ms']['p95']:.2f} ms`、"
                f"最大 `{scan['total_time_ms']['max']:.2f} ms`"
            )
        lines.append(
            f"- **{case['name']}**：`{timestamp['rate_hz']:.2f} Hz`，"
            f"最大输出间隔 `{timestamp['dt_max_ms']:.2f} ms`{lag_text}{scan_text}。"
        )

    lines.extend(["", "## PX4 EKF / ULog", ""])
    for case in metrics["cases"]:
        ulog = case.get("ulog", {})
        if not ulog.get("available"):
            lines.append(f"- **{case['name']}**：ULog 不可用：{ulog.get('error', 'unknown')}。")
            continue
        position = ulog["ev_position"]
        height = ulog["ev_height"]
        yaw = ulog["ev_yaw"]
        local = ulog["local_position"]
        vehicle = ulog["vehicle_status"]
        lines.append(
            f"- **{case['name']}**：ULog 空中覆盖 `{ulog['airborne_duration_s']:.1f} s`，"
            f"文件损坏=`{ulog['file_corruption']}`，完整飞行=`{ulog['complete_flight']}`；"
            f"EV 位置/高度/航向拒绝分别为 "
            f"`{position['rejected_count']} / {height['rejected_count']} / {yaw['rejected_count']}`，"
            f"dead reckoning `{local['dead_reckoning_count']}`，"
            f"failsafe `{vehicle['failsafe_count']}`。"
        )
        lines.append(
            f"  - 创新 P95：水平位置 "
            f"`{position.get('innovation_norm_p95_m', 0.0) * 100.0:.3f} cm`，"
            f"高度 `{height.get('innovation_p95_m', 0.0) * 100.0:.3f} cm`，"
            f"航向 `{yaw.get('innovation_p95_deg', 0.0):.3f}°`。"
        )
        px4_ros = case.get("px4_ros", {})
        ros_local = px4_ros.get("local_position", {})
        ros_status = px4_ros.get("vehicle_status", {})
        if ros_local and ros_status:
            reset_total = sum(ros_local.get("reset_changes", {}).values())
            lines.append(
                f"  - ROS bag 全时段：dead reckoning "
                f"`{ros_local.get('dead_reckoning_count', 0)}`，状态重置总数 "
                f"`{reset_total}`，failsafe `{ros_status.get('failsafe_count', 0)}`。"
            )
        if ulog.get("heading_reset_events"):
            event_text = ", ".join(
                f"t={event['airborne_time_s']:.1f}s Δyaw={event['delta_deg']:.2f}°"
                for event in ulog["heading_reset_events"]
            )
            lines.append(f"  - 航向重置：{event_text}。")

    lines.extend(["", "## 动态滤波", ""])
    for case in metrics["cases"]:
        dynamic = case.get("dynamic_diagnostics", {})
        ratio = case.get("streams", {}).get("dynamic_ratio", {})
        if not dynamic.get("available"):
            lines.append(f"- **{case['name']}**：无动态滤波诊断。")
            continue
        ratio_text = (
            f"独立 ratio 均值/P95/最大 = `{_percent(ratio['mean'])}% / "
            f"{_percent(ratio['p95'])}% / {_percent(ratio['max'])}%`；"
            if ratio else ""
        )
        lines.append(
            f"- **{case['name']}**：诊断 `{dynamic['sample_count']}` 帧，"
            f"{ratio_text}确认动态点最大 `{dynamic['confirmed_points_max']}`，"
            f"信息量比最小 `{dynamic['information_ratio_min']:.4f}`，"
            f"fallback required/used/map skipped = "
            f"`{dynamic['fallback_required_count']} / {dynamic['fallback_used_count']} / "
            f"{dynamic['map_update_skipped_count']}`。"
        )

    lines.extend(["", "## 告警", ""])
    warning_count = 0
    for case in metrics["cases"]:
        for warning in case.get("warnings", []):
            warning_count += 1
            lines.append(f"- **{case['name']}**：`{warning}`")
    if warning_count == 0:
        lines.append("- 无告警。")
    lines.append("")
    return "\n".join(lines)


def write_reports(metrics: dict[str, Any], output_dir: Path) -> tuple[Path, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / "metrics.json"
    markdown_path = output_dir / "localization_report.md"
    json_path.write_text(
        json.dumps(metrics, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    markdown_path.write_text(render_markdown(metrics), encoding="utf-8")
    return json_path, markdown_path
