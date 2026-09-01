#!/usr/bin/env bash

set -Eeuo pipefail
umask 0022

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly SCRIPT_DIR
REPO_ROOT="$(readlink -f "${SCRIPT_DIR}/..")"
readonly REPO_ROOT
DETECTED_OVERLAY="$(readlink -f "${REPO_ROOT}/../..")"
readonly DETECTED_OVERLAY
readonly PREFLIGHT_CHECKER="${SCRIPT_DIR}/preflight_checks.py"
[[ -r "$PREFLIGHT_CHECKER" ]] || {
  printf 'ERROR: Preflight checker is missing: %s\n' "$PREFLIGHT_CHECKER" >&2
  exit 1
}
command -v python3 >/dev/null 2>&1 || {
  printf 'ERROR: Required command not found: python3\n' >&2
  exit 1
}

# Override these paths when the runner is deployed to a differently named
# overlay. A complete overlay such as merge_test owns both packages; a
# Super-LIO-only overlay such as phase5 falls back to the CAT201 flight base.
FLIGHT_WORKSPACE="$(
  python3 "$PREFLIGHT_CHECKER" select-workspace \
    --detected "$DETECTED_OVERLAY" \
    --fallback /home/cat/ws_super_spline \
    --override "${FLIGHT_WORKSPACE:-}"
)"
readonly FLIGHT_WORKSPACE
readonly SUPER_LIO_OVERLAY="${SUPER_LIO_OVERLAY:-${DETECTED_OVERLAY}}"
readonly DATA_ROOT="${DATA_ROOT:-${SUPER_LIO_OVERLAY}/test_data}"
readonly NOKOV_TOPIC="/Robot_3/pose"
readonly LIO_ODOM_TOPIC="/lio/robo/odom"
readonly VISUAL_ODOM_TOPIC="/fmu/in/vehicle_visual_odometry"
readonly LIVOX_INTERFACE="${LIVOX_INTERFACE:-eth1}"
readonly LIVOX_HOST_CIDR="${LIVOX_HOST_CIDR:-192.168.1.5/24}"
readonly LIVOX_SENSOR_IP="${LIVOX_SENSOR_IP:-192.168.1.139}"
readonly MIN_FREE_KIB="${MIN_FREE_KIB:-$((10 * 1024 * 1024))}"

if [[ -r /etc/ros/ros_env ]]; then
  # shellcheck source=/dev/null
  source /etc/ros/ros_env
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-66}"

ACTION="run"
RUN_MODE=""
TEST_ID=""
TEST_DIR=""
SUPER_LIO_CONFIG_NAME=""
SUPER_LIO_CONFIG=""
BRIDGE_CONFIG_NAME=""
BRIDGE_CONFIG=""
EXPECTED_CONTROL_TOPIC=""
EXPECTED_DYNAMIC_ENABLE=""
NOKOV_SHADOW_ENABLED="false"
NOKOV_ROLE="disabled"

ODOM_BRIDGE_PID=""
SUPER_LIO_PID=""
BAG_PID=""
VMSTAT_PID=""
DDS_MONITOR_PID=""
PTP_CAPTURE_PID=""
PTP_WRAPPER_PID=""
CLEANUP_DONE=0

usage() {
  cat <<'EOF'
Usage:
  run.sh baseline TEST_ID [--nokov-shadow]
  run.sh shadow TEST_ID
  run.sh --check-only baseline [--nokov-shadow]
  run.sh --check-only shadow
  run.sh --help

Modes:
  baseline  PX4 control source is Super-LIO /lio/robo/odom. Dynamic filtering
            is disabled by livox_360.yaml. Add --nokov-shadow to validate and
            record NOKOV /Robot_3/pose as a shadow-only reference.

  shadow    PX4 control source is NOKOV /Robot_3/pose. Super-LIO runs the full
            dynamic pose/map pipeline with livox_360_dynamic_shadow_nokov.yaml,
            but generic_odometry_node must not subscribe to /lio/robo/odom.

Examples:
  ./deployment/run.sh --check-only baseline
  ./deployment/run.sh baseline superlio_baseline_hover_01
  ./deployment/run.sh --check-only baseline --nokov-shadow
  ./deployment/run.sh baseline superlio_nokov_shadow_hover_01 --nokov-shadow
  ./deployment/run.sh --check-only shadow
  ./deployment/run.sh shadow dynamic_nokov_shadow_hover_01

The runner owns the complete data-test lifecycle:
  preflight -> PX4 odometry bridge -> Super-LIO -> topology isolation checks
  -> MCAP/system/PTP recording -> one Ctrl+C -> clean flush and report.

Before shadow mode or baseline --nokov-shadow, start exactly one NOKOV
publisher for /Robot_3/pose. The --nokov-shadow option is valid only with
baseline; it never routes NOKOV data into PX4.
Do not separately start generic_odometry.launch.py, Livox_mid360.launch.py,
real_super_lio_*.launch.py, ros2 bag record, a planner, or Offboard control.

This script never arms the aircraft and never starts a planner or Offboard.
EOF
}

log() {
  printf '[%s] %s\n' "$(date '+%F %T')" "$*"
}

warn() {
  printf '[%s] WARN: %s\n' "$(date '+%F %T')" "$*" >&2
}

die() {
  printf '[%s] ERROR: %s\n' "$(date '+%F %T')" "$*" >&2
  exit 1
}

require_command() {
  command -v "$1" >/dev/null 2>&1 || die "Required command not found: $1"
}

service_must_be_active() {
  local service_name="$1"
  systemctl is-active --quiet "$service_name" || \
    die "Service is not active: ${service_name}"
}

safe_ros_topic_list() {
  timeout 12 ros2 topic list 2>/dev/null
}

topic_must_exist() {
  local topic_name="$1"
  local topic_snapshot="$2"
  grep -Fxq "$topic_name" <<<"$topic_snapshot" || \
    die "Required ROS topic is missing: ${topic_name}"
}

publisher_count() {
  local topic_name="$1"
  local output_file="$2"
  local info_text

  info_text="$(timeout 12 ros2 topic info -v "$topic_name" 2>&1)" || {
    printf '%s\n' "$info_text" >"$output_file"
    die "Unable to inspect publishers for ${topic_name}"
  }
  printf '%s\n' "$info_text" >"$output_file"
  awk '/Publisher count:/ {print $3; exit}' <<<"$info_text"
}

process_count() {
  local pattern="$1"
  local matches
  matches="$(pgrep -af "$pattern" 2>/dev/null || true)"
  if [[ -z "$matches" ]]; then
    printf '0\n'
  else
    wc -l <<<"$matches" | awk '{$1=$1; print}'
  fi
}

process_must_not_exist() {
  local pattern="$1"
  local description="$2"
  local matches
  matches="$(pgrep -af "$pattern" 2>/dev/null || true)"
  if [[ -n "$matches" ]]; then
    printf '%s\n' "$matches" >&2
    die "Already running: ${description}"
  fi
}

check_no_existing_test_stack() {
  process_must_not_exist '/super_lio_node([[:space:]]|$)' 'Super-LIO node'
  process_must_not_exist '/odom_to_px4_node([[:space:]]|$)' 'PX4 odometry bridge'
  process_must_not_exist 'ros2 launch .*real_super_lio_.*\.launch\.py' \
    'real_super_lio launch'
  process_must_not_exist 'ros2 launch .*real_neural_traj\.launch\.py' \
    'real_neural_traj.launch.py'
  process_must_not_exist 'ros2 bag record' 'ROS 2 bag recorder'

  if pgrep -x ptp4l >/dev/null 2>&1 || pgrep -x phc2sys >/dev/null 2>&1; then
    pgrep -af 'ptp4l|phc2sys' >&2 || true
    die "ptp4l/phc2sys conflicts with the configured ptpd service"
  fi
}

check_ptp_exchange() {
  local output_file="${1:-}"
  local capture_text

  capture_text="$(
    sudo -n timeout 8 tcpdump -ni "$LIVOX_INTERFACE" -c 20 -tttt -vv \
      'udp port 319 or udp port 320' 2>&1 || true
  )"

  if [[ -n "$output_file" ]]; then
    printf '%s\n' "$capture_text" >"$output_file"
  fi

  grep -Eqi 'sync msg' <<<"$capture_text" || \
    die "PTP Sync packet was not observed on ${LIVOX_INTERFACE}"
  grep -Eqi 'delay req msg' <<<"$capture_text" || \
    die "MID360 Delay_Req packet was not observed on ${LIVOX_INTERFACE}"
  grep -Eqi 'delay resp msg' <<<"$capture_text" || \
    die "PTP Delay_Resp packet was not observed on ${LIVOX_INTERFACE}"
}

check_imu_timestamp_latency() {
  python3 "$PREFLIGHT_CHECKER" live imu
}

check_nokov_stream() {
  python3 "$PREFLIGHT_CHECKER" live nokov
}

check_lidar_stream() {
  python3 "$PREFLIGHT_CHECKER" live lidar
}

check_px4_status_stream() {
  python3 "$PREFLIGHT_CHECKER" live px4-status
}

check_lio_odometry_stream() {
  python3 - <<'PY'
import math
import statistics
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

rclpy.init()
node = rclpy.create_node("flight_runner_lio_odometry_gate")
arrivals = []
latencies_ms = []
stamps_ns = []
invalid_count = 0

def callback(msg):
    global invalid_count
    now_ns = node.get_clock().now().nanoseconds
    stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
    arrivals.append(time.monotonic())
    stamps_ns.append(stamp_ns)
    latencies_ms.append((now_ns - stamp_ns) / 1_000_000)
    pose = msg.pose.pose
    twist = msg.twist.twist
    values = (
        pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w,
        twist.linear.x, twist.linear.y, twist.linear.z,
        msg.pose.covariance[0], msg.pose.covariance[7],
        msg.pose.covariance[14], msg.pose.covariance[21],
        msg.pose.covariance[28], msg.pose.covariance[35],
    )
    q_norm = math.sqrt(
        pose.orientation.x ** 2 + pose.orientation.y ** 2
        + pose.orientation.z ** 2 + pose.orientation.w ** 2)
    if (
        msg.header.frame_id != "world"
        or stamp_ns == 0
        or not all(math.isfinite(value) for value in values)
        or not 0.5 < q_norm < 1.5
        or any(value <= 0.0 for value in values[-6:])
    ):
        invalid_count += 1

subscription = node.create_subscription(
    Odometry, "/lio/robo/odom", callback, qos_profile_sensor_data)
deadline = time.monotonic() + 8.0

while len(arrivals) < 200 and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)

node.destroy_node()
rclpy.shutdown()

if len(arrivals) < 40:
    print(f"samples={len(arrivals)} result=FAIL reason=no_live_lio_odometry")
    sys.exit(2)

duration = arrivals[-1] - arrivals[0]
rate_hz = (len(arrivals) - 1) / duration if duration > 0 else 0.0
regressions = sum(curr <= prev for prev, curr in zip(stamps_ns, stamps_ns[1:]))
median_ms = statistics.median(latencies_ms)
p95_ms = sorted(latencies_ms)[
    min(len(latencies_ms) - 1, int(0.95 * len(latencies_ms)))]
maximum_ms = max(latencies_ms)
print(
    f"samples={len(arrivals)} rate_hz={rate_hz:.2f} "
    f"median_latency_ms={median_ms:.3f} p95_latency_ms={p95_ms:.3f} "
    f"max_latency_ms={maximum_ms:.3f} regressions={regressions} "
    f"invalid_samples={invalid_count}")

if (
    rate_hz < 40.0
    or regressions > 0
    or invalid_count > 0
    or median_ms < -20.0
    or median_ms > 80.0
    or p95_ms > 100.0
    or maximum_ms > 150.0
):
    sys.exit(3)
PY
}

nokov_is_required() {
  [[ "$NOKOV_ROLE" != "disabled" ]]
}

configure_mode() {
  case "$RUN_MODE" in
    baseline)
      SUPER_LIO_CONFIG_NAME="livox_360.yaml"
      BRIDGE_CONFIG_NAME="super_lio_config.yaml"
      EXPECTED_CONTROL_TOPIC="$LIO_ODOM_TOPIC"
      EXPECTED_DYNAMIC_ENABLE="false"
      if [[ "$NOKOV_SHADOW_ENABLED" == "true" ]]; then
        NOKOV_ROLE="shadow"
      fi
      ;;
    shadow)
      SUPER_LIO_CONFIG_NAME="livox_360_dynamic_shadow_nokov.yaml"
      BRIDGE_CONFIG_NAME="nokov_mocap_config.yaml"
      EXPECTED_CONTROL_TOPIC="$NOKOV_TOPIC"
      EXPECTED_DYNAMIC_ENABLE="true"
      NOKOV_ROLE="control"
      ;;
    *)
      die "Mode must be baseline or shadow"
      ;;
  esac

  SUPER_LIO_CONFIG="${SUPER_LIO_OVERLAY}/install/super_lio/share/super_lio/config/${SUPER_LIO_CONFIG_NAME}"
  BRIDGE_CONFIG="${FLIGHT_WORKSPACE}/install/ros2_px4_odometry/share/ros2_px4_odometry/config/${BRIDGE_CONFIG_NAME}"
}

base_preflight() {
  local output_dir="${1:-}"
  local topic_snapshot
  local imu_metrics
  local lidar_metrics
  local px4_status_metrics
  local source_metrics=""
  local source_type
  local source_count
  local visual_count
  local free_kib
  local scratch_dir

  log "Running ${RUN_MODE} service, PTP, timestamp, package and topology preflight"

  service_must_be_active ptp-sync.service
  service_must_be_active livox-mid360.service
  service_must_be_active microxrc-agent.service

  ip -brief address show "$LIVOX_INTERFACE" | grep -Fq "$LIVOX_HOST_CIDR" || \
    die "${LIVOX_INTERFACE} does not have ${LIVOX_HOST_CIDR}"
  ip route get "$LIVOX_SENSOR_IP" | grep -Fq "dev ${LIVOX_INTERFACE}" || \
    die "MID360 route is not using ${LIVOX_INTERFACE}"

  check_no_existing_test_stack

  free_kib="$(df -Pk "$SUPER_LIO_OVERLAY" | awk 'NR == 2 {print $4}')"
  [[ "$free_kib" =~ ^[0-9]+$ ]] || die "Unable to determine free disk space"
  (( free_kib >= MIN_FREE_KIB )) || \
    die "Free space is below $((MIN_FREE_KIB / 1024)) MiB"

  [[ -r "$SUPER_LIO_CONFIG" ]] || \
    die "Installed Super-LIO config is missing: ${SUPER_LIO_CONFIG}"
  [[ -r "$BRIDGE_CONFIG" ]] || \
    die "Installed PX4 bridge config is missing: ${BRIDGE_CONFIG}"

  [[ "$(ros2 pkg prefix super_lio)" == \
    "${SUPER_LIO_OVERLAY}/install/super_lio" ]] || \
    die "super_lio is not resolved from ${SUPER_LIO_OVERLAY}"
  [[ "$(ros2 pkg prefix ros2_px4_odometry)" == \
    "${FLIGHT_WORKSPACE}/install/ros2_px4_odometry" ]] || \
    die "ros2_px4_odometry is not resolved from ${FLIGHT_WORKSPACE}"

  topic_snapshot="$(safe_ros_topic_list)" || die "Unable to query ROS 2 topics"
  topic_must_exist /livox/imu "$topic_snapshot"
  topic_must_exist /livox/lidar "$topic_snapshot"
  topic_must_exist /fmu/out/vehicle_status_v1 "$topic_snapshot"

  if [[ -n "$output_dir" ]]; then
    scratch_dir="$output_dir"
  else
    scratch_dir="$(mktemp -d)"
  fi

  if nokov_is_required; then
    topic_must_exist "$NOKOV_TOPIC" "$topic_snapshot"
    source_type="$(timeout 12 ros2 topic type "$NOKOV_TOPIC" 2>/dev/null || true)"
    [[ "$source_type" == "geometry_msgs/msg/PoseStamped" ]] || \
      die "${NOKOV_TOPIC} type is ${source_type:-unknown}, expected PoseStamped"
    source_count="$(publisher_count \
      "$NOKOV_TOPIC" "${scratch_dir}/preflight_nokov_publishers.txt")"
    [[ "$source_count" -eq 1 ]] || \
      die "Expected one NOKOV publisher, found ${source_count:-unknown}"
    if [[ -n "$output_dir" && "$NOKOV_ROLE" == "control" ]]; then
      cp -a "${scratch_dir}/preflight_nokov_publishers.txt" \
        "${scratch_dir}/preflight_control_publishers.txt"
    fi
  fi

  visual_count="$(publisher_count \
    "$VISUAL_ODOM_TOPIC" "${scratch_dir}/preflight_visual_odom_publishers.txt")"
  [[ "${visual_count:-0}" -eq 0 ]] || \
    die "PX4 visual odometry already has ${visual_count} publisher(s)"

  check_ptp_exchange "${scratch_dir}/preflight_ptp.txt"

  imu_metrics="$(check_imu_timestamp_latency)" || {
    printf '%s\n' "$imu_metrics" >&2
    die "Livox IMU timestamp latency gate failed"
  }
  log "Livox IMU: ${imu_metrics}"

  lidar_metrics="$(check_lidar_stream)" || {
    printf '%s\n' "$lidar_metrics" >&2
    die "Livox lidar live-stream gate failed"
  }
  log "Livox lidar: ${lidar_metrics}"

  px4_status_metrics="$(check_px4_status_stream)" || {
    printf '%s\n' "$px4_status_metrics" >&2
    die "PX4 vehicle status live-stream gate failed"
  }
  log "PX4 status: ${px4_status_metrics}"

  if nokov_is_required; then
    source_metrics="$(check_nokov_stream)" || {
      printf '%s\n' "$source_metrics" >&2
      die "NOKOV rate/timestamp/pose gate failed"
    }
    log "NOKOV (${NOKOV_ROLE}): ${source_metrics}"
  fi

  if [[ -n "$output_dir" ]]; then
    printf '%s\n' "$topic_snapshot" | sort >"${output_dir}/preflight_topics.txt"
    printf '%s\n' "$imu_metrics" >"${output_dir}/preflight_imu_metrics.txt"
    printf '%s\n' "$lidar_metrics" >"${output_dir}/preflight_lidar_metrics.txt"
    printf '%s\n' "$px4_status_metrics" >"${output_dir}/preflight_px4_status_metrics.txt"
    if [[ -n "$source_metrics" ]]; then
      printf '%s\n' "$source_metrics" >"${output_dir}/preflight_nokov_metrics.txt"
      if [[ "$NOKOV_ROLE" == "control" ]]; then
        printf '%s\n' "$source_metrics" >"${output_dir}/preflight_control_metrics.txt"
      fi
    fi
  else
    rm -rf -- "${scratch_dir:?}"
  fi

  if ! python3 "$PREFLIGHT_CHECKER" clock-synchronized >/dev/null; then
    warn "System NTP is not synchronized; relative sensor timestamp gates passed"
  fi

  log "${RUN_MODE} preflight checks passed"
}

copy_if_readable() {
  local source_path="$1"
  local destination_dir="$2"
  if [[ -r "$source_path" ]]; then
    python3 "$PREFLIGHT_CHECKER" snapshot-file \
      --source "$source_path" \
      --destination "$destination_dir" \
      >/dev/null
  fi
}

save_configuration_snapshot() {
  local runner_path
  local bridge_source_config
  local source_super_config
  runner_path="$(readlink -f "${BASH_SOURCE[0]}")"
  bridge_source_config="${FLIGHT_WORKSPACE}/src/px4_ros2_ctrl/navigation/ros2_px4_odometry/config/${BRIDGE_CONFIG_NAME}"
  source_super_config="${SUPER_LIO_OVERLAY}/src/Super-LIO/src/super_lio/config/${SUPER_LIO_CONFIG_NAME}"

  mkdir -p "${TEST_DIR}/config" "${TEST_DIR}/ros_logs"
  copy_if_readable "$SUPER_LIO_CONFIG" "${TEST_DIR}/config"
  copy_if_readable "$source_super_config" "${TEST_DIR}/config"
  copy_if_readable "$bridge_source_config" "${TEST_DIR}/config"
  copy_if_readable \
    "${FLIGHT_WORKSPACE}/src/px4_ros2_ctrl/navigation/ros2_px4_odometry/launch/generic_odometry.launch.py" \
    "${TEST_DIR}/config"
  copy_if_readable \
    "${SUPER_LIO_OVERLAY}/src/Super-LIO/src/super_lio/launch/Livox_mid360.launch.py" \
    "${TEST_DIR}/config"
  cp -a "$runner_path" "${TEST_DIR}/config/"
  cp -a "$PREFLIGHT_CHECKER" "${TEST_DIR}/config/"

  {
    date -Ins
    uname -a
    timedatectl status
    systemctl show ptp-sync.service livox-mid360.service microxrc-agent.service \
      -p Id -p ActiveState -p MainPID -p NRestarts -p ExecMainStatus --no-pager
    ip -brief address
    ip route
    ip -s -s link show "$LIVOX_INTERFACE"
    nstat -az
    ss -uapnm
    sysctl net.core.rmem_default net.core.rmem_max
    df -h "$SUPER_LIO_OVERLAY"
    free -h
  } >"${TEST_DIR}/preflight_system.txt" 2>&1

  {
    for repository in \
      "${SUPER_LIO_OVERLAY}/src/Super-LIO" \
      "${FLIGHT_WORKSPACE}/src/px4_ros2_ctrl"; do
      printf '\nREPOSITORY=%s\n' "$repository"
      git -C "$repository" rev-parse HEAD 2>/dev/null || true
      git -C "$repository" status --short 2>/dev/null || true
      git -C "$repository" diff --stat 2>/dev/null || true
    done

    printf '\nPACKAGE_PREFIXES\n'
    for package_name in super_lio ros2_px4_odometry; do
      printf '%s=' "$package_name"
      ros2 pkg prefix "$package_name"
    done

    printf '\nARTIFACT_SHA256\n'
    sha256sum \
      "$(readlink -f "${SUPER_LIO_OVERLAY}/install/super_lio/lib/super_lio/super_lio_node")" \
      "$SUPER_LIO_CONFIG" \
      "$(readlink -f "${FLIGHT_WORKSPACE}/install/ros2_px4_odometry/lib/ros2_px4_odometry/odom_to_px4_node")" \
      "$BRIDGE_CONFIG" \
      "$runner_path" \
      "$PREFLIGHT_CHECKER"
  } >"${TEST_DIR}/source_state.txt"

  cat >"${TEST_DIR}/test_notes.txt" <<EOF
TEST_ID=${TEST_ID}
RUN_MODE=${RUN_MODE}
START_TIME=$(date -Ins)
ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
CONTROL_ODOMETRY=${EXPECTED_CONTROL_TOPIC}
SUPER_LIO_CONFIG=${SUPER_LIO_CONFIG_NAME}
SUPER_LIO_DYNAMIC_ENABLE=${EXPECTED_DYNAMIC_ENABLE}
NOKOV_ROLE=${NOKOV_ROLE}
NOKOV_SHADOW_ENABLED=${NOKOV_SHADOW_ENABLED}
SUPER_LIO_OVERLAY=${SUPER_LIO_OVERLAY}
SUPER_LIO_COMMIT=$(git -C "${SUPER_LIO_OVERLAY}/src/Super-LIO" rev-parse HEAD)
FLIGHT_MODE=POSCTL
PLANNER=DISABLED
OFFBOARD=DISABLED
PTP=ptpd_master_${LIVOX_INTERFACE}
ULOG=PENDING
VIDEO=PENDING
OBSERVATION=
EOF
}

wait_for_stack() {
  local deadline=$((SECONDS + 90))
  local topic_snapshot=""
  local required_topic
  local all_ready
  local required_topics=(
    /lio/imu/odom
    /lio/odom
    /lio/robo/odom
    /lio/diagnostics/scan
    "$VISUAL_ODOM_TOPIC"
  )

  if nokov_is_required; then
    required_topics+=("$NOKOV_TOPIC")
  fi

  if [[ "$RUN_MODE" == "shadow" ]]; then
    required_topics+=(/lio/dynamic_ratio)
  fi

  log "Waiting for ${RUN_MODE} bridge and Super-LIO topics"
  while (( SECONDS < deadline )); do
    kill -0 "$ODOM_BRIDGE_PID" >/dev/null 2>&1 || \
      die "PX4 odometry bridge exited before topics became ready"
    kill -0 "$SUPER_LIO_PID" >/dev/null 2>&1 || \
      die "Super-LIO exited before topics became ready"

    topic_snapshot="$(safe_ros_topic_list || true)"
    all_ready=1
    for required_topic in "${required_topics[@]}"; do
      if ! grep -Fxq "$required_topic" <<<"$topic_snapshot"; then
        all_ready=0
        break
      fi
    done
    if (( all_ready == 1 )); then
      printf '%s\n' "$topic_snapshot" | sort >"${TEST_DIR}/topics.txt"
      log "Required topics are present"
      return 0
    fi
    sleep 1
  done
  die "Timed out waiting for ${RUN_MODE} topics"
}

parameter_must_match() {
  local node_name="$1"
  local parameter_name="$2"
  local expected_text="$3"
  local output
  output="$(timeout 12 ros2 param get "$node_name" "$parameter_name" 2>&1)" || \
    die "Unable to read ${node_name} ${parameter_name}"
  grep -Fqi "$expected_text" <<<"$output" || \
    die "${node_name} ${parameter_name} is not ${expected_text}: ${output}"
}

validate_topology() {
  local visual_count
  local trajectory_count
  local offboard_count
  local super_lio_count
  local bridge_count
  local nokov_count
  local lio_metrics

  timeout 15 ros2 node list | sort >"${TEST_DIR}/nodes.txt"

  super_lio_count="$(process_count '/super_lio_node([[:space:]]|$)')"
  bridge_count="$(process_count '/odom_to_px4_node([[:space:]]|$)')"
  [[ "$super_lio_count" -eq 1 ]] || \
    die "Expected one Super-LIO process, found ${super_lio_count}"
  [[ "$bridge_count" -eq 1 ]] || \
    die "Expected one PX4 odometry bridge, found ${bridge_count}"

  visual_count="$(publisher_count \
    "$VISUAL_ODOM_TOPIC" "${TEST_DIR}/visual_odom_publishers.txt")"
  [[ "$visual_count" -eq 1 ]] || \
    die "Expected one PX4 visual odometry publisher, found ${visual_count:-unknown}"
  grep -Fq 'Node name: generic_odometry_node' \
    "${TEST_DIR}/visual_odom_publishers.txt" || \
    die "PX4 visual odometry publisher is not generic_odometry_node"

  if nokov_is_required; then
    nokov_count="$(publisher_count \
      "$NOKOV_TOPIC" "${TEST_DIR}/nokov_publishers.txt")"
    [[ "$nokov_count" -eq 1 ]] || \
      die "Expected one NOKOV publisher, found ${nokov_count:-unknown}"

    timeout 12 ros2 topic info -v "$NOKOV_TOPIC" \
      >"${TEST_DIR}/nokov_endpoints.txt" 2>&1 || \
      die "Unable to inspect ${NOKOV_TOPIC} endpoints"

    if [[ "$NOKOV_ROLE" == "control" ]]; then
      grep -Fq 'Node name: generic_odometry_node' \
        "${TEST_DIR}/nokov_endpoints.txt" || \
        die "generic_odometry_node is not consuming ${NOKOV_TOPIC}"
    elif grep -Fq 'Node name: generic_odometry_node' \
        "${TEST_DIR}/nokov_endpoints.txt"; then
      die "generic_odometry_node consumes ${NOKOV_TOPIC}; NOKOV shadow isolation failed"
    fi
  fi

  timeout 12 ros2 topic info -v "$LIO_ODOM_TOPIC" \
    >"${TEST_DIR}/lio_robo_odom_endpoints.txt" 2>&1 || \
    die "Unable to inspect ${LIO_ODOM_TOPIC} endpoints"

  if [[ "$RUN_MODE" == "shadow" ]]; then
    if grep -Fq 'Node name: generic_odometry_node' \
      "${TEST_DIR}/lio_robo_odom_endpoints.txt"; then
      die "generic_odometry_node consumes ${LIO_ODOM_TOPIC}; shadow isolation failed"
    fi
  else
    grep -Fq 'Node name: generic_odometry_node' \
      "${TEST_DIR}/lio_robo_odom_endpoints.txt" || \
      die "generic_odometry_node is not consuming ${LIO_ODOM_TOPIC}"
  fi

  parameter_must_match \
    /generic_odometry_node odometry_topic "$EXPECTED_CONTROL_TOPIC"
  parameter_must_match \
    /super_lio_node lio.dynamic.enable "$EXPECTED_DYNAMIC_ENABLE"
  if [[ "$RUN_MODE" == "shadow" ]]; then
    parameter_must_match /generic_odometry_node message_type pose_stamped
    parameter_must_match /super_lio_node lio.dynamic.filter_pose_update true
    parameter_must_match /super_lio_node lio.dynamic.filter_map_update true
  else
    parameter_must_match /generic_odometry_node message_type odometry
  fi

  trajectory_count="$(publisher_count \
    /fmu/in/trajectory_setpoint "${TEST_DIR}/trajectory_publishers.txt")"
  [[ "${trajectory_count:-0}" -eq 0 ]] || \
    die "Unexpected trajectory_setpoint publisher detected"
  offboard_count="$(publisher_count \
    /fmu/in/offboard_control_mode "${TEST_DIR}/offboard_publishers.txt")"
  [[ "${offboard_count:-0}" -eq 0 ]] || \
    die "Unexpected offboard_control_mode publisher detected"

  lio_metrics="$(check_lio_odometry_stream)" || {
    printf '%s\n' "$lio_metrics" >&2
    die "Super-LIO odometry rate/timestamp/covariance gate failed"
  }
  printf '%s\n' "$lio_metrics" >"${TEST_DIR}/lio_odometry_metrics.txt"
  log "Super-LIO odometry: ${lio_metrics}"

  timeout 15 ros2 param dump /super_lio_node \
    >"${TEST_DIR}/super_lio_runtime_params.yaml" 2>&1 || \
    warn "Unable to dump /super_lio_node parameters"
  timeout 15 ros2 param dump /generic_odometry_node \
    >"${TEST_DIR}/odom_converter_runtime_params.yaml" 2>&1 || \
    warn "Unable to dump /generic_odometry_node parameters"

  case "$NOKOV_ROLE" in
    control)
      log "Topology verified: NOKOV is the sole PX4 odometry source; Super-LIO is shadow-only"
      ;;
    shadow)
      log "Topology verified: Super-LIO is the sole PX4 odometry source; NOKOV is shadow-only"
      ;;
    disabled)
      log "Topology verified: Super-LIO is the sole PX4 odometry source"
      ;;
  esac
}

start_monitors() {
  log "Starting vmstat, DDS UDP and PTP monitors"

  setsid stdbuf -oL vmstat -t 1 >"${TEST_DIR}/vmstat.log" 2>&1 &
  VMSTAT_PID=$!

  setsid stdbuf -oL bash -c '
    while true; do
      date "+TIME %s.%N %F %T"
      ss -uapnm
      nstat -az | grep -E "UdpRcvbufErrors|UdpInErrors|IpInDiscards" || true
      sleep 1
    done
  ' >"${TEST_DIR}/dds_udp_monitor.log" 2>&1 &
  DDS_MONITOR_PID=$!

  # shellcheck disable=SC2024
  sudo -n sh -c '
    printf "%s\n" "$$" >"$1"
    exec /usr/bin/tcpdump -Z cat -ni "$2" -s 256 -U \
      -w "$3" "udp port 319 or udp port 320"
  ' sh "${TEST_DIR}/ptp_capture.pid" "$LIVOX_INTERFACE" \
    "${TEST_DIR}/ptp.pcap" >"${TEST_DIR}/ptp_capture.log" 2>&1 &
  PTP_WRAPPER_PID=$!

  for _ in {1..30}; do
    if [[ -s "${TEST_DIR}/ptp_capture.pid" ]]; then
      PTP_CAPTURE_PID="$(<"${TEST_DIR}/ptp_capture.pid")"
      break
    fi
    sleep 0.1
  done

  [[ "$PTP_CAPTURE_PID" =~ ^[0-9]+$ ]] || die "Unable to determine tcpdump PID"
  [[ "$(ps -p "$PTP_CAPTURE_PID" -o comm= | awk '{$1=$1; print}')" == "tcpdump" ]] || \
    die "PTP capture process did not start correctly"
}

start_rosbag() {
  local topics=(
    /livox/imu
    /livox/lidar
    /lio/imu/odom
    /lio/odom
    /lio/robo/odom
    /lio/diagnostics/scan
    /tf
    /tf_static
    "$VISUAL_ODOM_TOPIC"
    /fmu/out/vehicle_odometry
    /fmu/out/vehicle_local_position_v1
    /fmu/out/vehicle_attitude
    /fmu/out/vehicle_angular_velocity
    /fmu/out/estimator_status_flags
    /fmu/out/timesync_status
    /fmu/out/sensor_combined
    /fmu/out/manual_control_setpoint
    /fmu/out/vehicle_control_mode
    /fmu/out/vehicle_status_v1
    /fmu/out/vehicle_land_detected
    /fmu/out/failsafe_flags
    /fmu/out/battery_status_v1
    /fmu/out/position_setpoint_triplet
    /fmu/in/trajectory_setpoint
    /fmu/in/offboard_control_mode
    /fmu/in/vehicle_command
    /fmu/out/vehicle_command_ack
    /rosout
    /parameter_events
  )

  if nokov_is_required; then
    topics+=("$NOKOV_TOPIC")
  fi
  if [[ "$RUN_MODE" == "shadow" ]]; then
    topics+=(/lio/dynamic_ratio)
  fi

  log "Starting MCAP recorder"
  setsid ros2 bag record \
    -s mcap \
    --compression-mode none \
    --max-cache-size 268435456 \
    --max-bag-duration 300 \
    -o "${TEST_DIR}/rosbag" \
    "${topics[@]}" >"${TEST_DIR}/rosbag_record.log" 2>&1 &
  BAG_PID=$!

  sleep 5
  kill -0 "$BAG_PID" >/dev/null 2>&1 || \
    die "ROS 2 bag recorder exited during startup"
  date -Ins >"${TEST_DIR}/recording_started.txt"
}

stop_process_group() {
  local pid="$1"
  local graceful_seconds="$2"
  local signal_name="${3:-INT}"
  local counter

  [[ -n "$pid" ]] || return 0
  ps -p "$pid" -o pid= >/dev/null 2>&1 || {
    wait "$pid" 2>/dev/null || true
    return 0
  }

  kill -s "$signal_name" -- "-${pid}" 2>/dev/null || \
    kill -s "$signal_name" "$pid" 2>/dev/null || \
    sudo -n kill -s "$signal_name" -- "-${pid}" 2>/dev/null || \
    sudo -n kill -s "$signal_name" "$pid" 2>/dev/null || true

  for ((counter = 0; counter < graceful_seconds * 2; counter++)); do
    ps -p "$pid" -o pid= >/dev/null 2>&1 || break
    sleep 0.5
  done

  if ps -p "$pid" -o pid= >/dev/null 2>&1; then
    warn "Process group ${pid} did not stop after ${graceful_seconds}s; sending TERM"
    kill -s TERM -- "-${pid}" 2>/dev/null || \
      kill -s TERM "$pid" 2>/dev/null || \
      sudo -n kill -s TERM -- "-${pid}" 2>/dev/null || \
      sudo -n kill -s TERM "$pid" 2>/dev/null || true
    sleep 2
  fi

  if ps -p "$pid" -o pid= >/dev/null 2>&1; then
    warn "Process group ${pid} still running; sending KILL"
    kill -s KILL -- "-${pid}" 2>/dev/null || \
      kill -s KILL "$pid" 2>/dev/null || \
      sudo -n kill -s KILL -- "-${pid}" 2>/dev/null || \
      sudo -n kill -s KILL "$pid" 2>/dev/null || true
  fi
  wait "$pid" 2>/dev/null || true
}

stop_ptp_capture() {
  local counter
  if [[ "$PTP_CAPTURE_PID" =~ ^[0-9]+$ ]] && \
      ps -p "$PTP_CAPTURE_PID" -o pid= >/dev/null 2>&1; then
    kill -s TERM "$PTP_CAPTURE_PID" 2>/dev/null || \
      sudo -n kill -s TERM "$PTP_CAPTURE_PID" 2>/dev/null || true
    for ((counter = 0; counter < 20; counter++)); do
      ps -p "$PTP_CAPTURE_PID" -o pid= >/dev/null 2>&1 || break
      sleep 0.25
    done
    if ps -p "$PTP_CAPTURE_PID" -o pid= >/dev/null 2>&1; then
      warn "tcpdump did not stop after 5s; sending KILL"
      kill -s KILL "$PTP_CAPTURE_PID" 2>/dev/null || \
        sudo -n kill -s KILL "$PTP_CAPTURE_PID" 2>/dev/null || true
    fi
  fi
  if [[ -n "$PTP_WRAPPER_PID" ]]; then
    wait "$PTP_WRAPPER_PID" 2>/dev/null || true
  fi
}

save_postflight_snapshot() {
  date -Ins >"${TEST_DIR}/recording_stopped.txt"
  {
    date -Ins
    ip -s -s link show "$LIVOX_INTERFACE"
    nstat -az
    ss -uapnm
    df -h "$SUPER_LIO_OVERLAY"
    free -h
  } >"${TEST_DIR}/postflight_system.txt" 2>&1

  # shellcheck disable=SC2024
  sudo -n journalctl -b -o short-iso-precise \
    -u ptp-sync.service \
    -u livox-mid360.service \
    -u microxrc-agent.service \
    --no-pager >"${TEST_DIR}/system_services.log" 2>&1 || true

  if [[ -d "${TEST_DIR}/rosbag" ]]; then
    ros2 bag info "${TEST_DIR}/rosbag" \
      >"${TEST_DIR}/rosbag_info.txt" 2>&1 || true
    du -sh "${TEST_DIR}/rosbag" \
      >"${TEST_DIR}/rosbag_size.txt" 2>&1 || true
  fi

  if [[ -s "${TEST_DIR}/dds_udp_monitor.log" ]]; then
    awk '
      /^TIME / {sample_time = $2}
      /UdpRcvbufErrors/ {
        if (!seen) {first = $2; first_time = sample_time; seen = 1}
        last = $2; last_time = sample_time
      }
      END {
        if (seen) {
          printf "UdpRcvbufErrors_start=%d\n", first
          printf "UdpRcvbufErrors_end=%d\n", last
          printf "UdpRcvbufErrors_delta=%d\n", last - first
          printf "monitor_start_epoch=%s\n", first_time
          printf "monitor_end_epoch=%s\n", last_time
        } else {
          print "UdpRcvbufErrors_unavailable=1"
        }
      }
    ' "${TEST_DIR}/dds_udp_monitor.log" >"${TEST_DIR}/dds_drop_summary.txt"
  fi

  sudo -n chown -R "$(id -u):$(id -g)" "$TEST_DIR" 2>/dev/null || true
}

cleanup() {
  local original_status=$?

  trap - EXIT INT TERM HUP
  set +e
  if (( CLEANUP_DONE == 1 )); then
    exit "$original_status"
  fi
  CLEANUP_DONE=1

  if [[ -n "$TEST_DIR" && -d "$TEST_DIR" ]]; then
    log "Stopping recorder first so MCAP can flush cleanly"
  fi

  stop_process_group "$BAG_PID" 30 INT
  stop_ptp_capture
  stop_process_group "$DDS_MONITOR_PID" 5 TERM
  stop_process_group "$VMSTAT_PID" 5 TERM
  stop_process_group "$SUPER_LIO_PID" 20 INT
  stop_process_group "$ODOM_BRIDGE_PID" 20 INT

  if [[ -n "$TEST_DIR" && -d "$TEST_DIR" ]]; then
    save_postflight_snapshot
    if (( original_status == 0 )); then
      printf 'COMPLETE %s\n' "$(date -Ins)" >"${TEST_DIR}/test_status.txt"
      log "Test recording finalized: ${TEST_DIR}"
    else
      printf 'ABORTED exit_status=%d %s\n' "$original_status" "$(date -Ins)" \
        >"${TEST_DIR}/test_status.txt"
      warn "Test stopped with status ${original_status}; partial data kept"
    fi
  fi
  exit "$original_status"
}

case "${1:-}" in
  --help|-h)
    usage
    exit 0
    ;;
  --check-only)
    (( $# == 2 || $# == 3 )) || {
      usage >&2
      die "--check-only requires one mode and optionally --nokov-shadow"
    }
    ACTION="check"
    RUN_MODE="$2"
    if (( $# == 3 )); then
      [[ "$3" == "--nokov-shadow" ]] || die "Unknown option: $3"
      NOKOV_SHADOW_ENABLED="true"
    fi
    shift "$#"
    ;;
  baseline|shadow)
    (( $# == 2 || $# == 3 )) || {
      usage >&2
      die "$1 requires one TEST_ID and optionally --nokov-shadow"
    }
    RUN_MODE="$1"
    TEST_ID="$2"
    if (( $# == 3 )); then
      [[ "$3" == "--nokov-shadow" ]] || die "Unknown option: $3"
      NOKOV_SHADOW_ENABLED="true"
    fi
    shift "$#"
    ;;
  "")
    usage >&2
    exit 2
    ;;
  *)
    usage >&2
    die "Unknown mode or option: $1"
    ;;
esac

(( $# == 0 )) || die "Unexpected arguments: $*"
if [[ "$NOKOV_SHADOW_ENABLED" == "true" && "$RUN_MODE" != "baseline" ]]; then
  die "--nokov-shadow is valid only with baseline mode"
fi
configure_mode

if [[ "$ACTION" == "run" ]]; then
  [[ "$TEST_ID" =~ ^[A-Za-z0-9][A-Za-z0-9._-]{0,79}$ ]] || \
    die "TEST_ID must contain only letters, numbers, dot, underscore or dash"
fi

[[ -r /opt/ros/humble/setup.bash ]] || die "ROS Humble setup is missing"
[[ -r "${FLIGHT_WORKSPACE}/install/setup.bash" ]] || \
  die "Flight workspace install/setup.bash is missing"
[[ -r "${SUPER_LIO_OVERLAY}/install/setup.bash" ]] || \
  die "Super-LIO overlay install/setup.bash is missing"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${FLIGHT_WORKSPACE}/install/setup.bash"
# shellcheck disable=SC1091
source "${SUPER_LIO_OVERLAY}/install/setup.bash"
set -u

for command_name in \
  awk bash cp date df du free git grep ip journalctl mktemp nstat pgrep ps \
  python3 readlink rm ros2 setsid sha256sum ss stdbuf sudo systemctl tcpdump \
  timeout timedatectl vmstat wc; do
  require_command "$command_name"
done

sudo -v

if [[ "$ACTION" == "check" ]]; then
  base_preflight
  log "Check-only completed; no bridge, Super-LIO, recorder or planner was started"
  exit 0
fi

TEST_DIR="${DATA_ROOT}/${RUN_MODE}/${TEST_ID}"
[[ ! -e "$TEST_DIR" ]] || die "Test directory already exists: ${TEST_DIR}"
mkdir -p "$TEST_DIR"
export ROS_LOG_DIR="${TEST_DIR}/ros_logs"
mkdir -p "$ROS_LOG_DIR"

trap cleanup EXIT
trap 'log "Stop requested; finalizing all records"; exit 0' INT TERM
trap 'warn "Terminal disconnected; finalizing records"; exit 129' HUP

base_preflight "$TEST_DIR"
save_configuration_snapshot

log "Starting PX4 odometry bridge with ${BRIDGE_CONFIG_NAME}"
setsid stdbuf -oL -eL ros2 launch \
  ros2_px4_odometry generic_odometry.launch.py \
  config_file:="$BRIDGE_CONFIG_NAME" \
  >"${TEST_DIR}/odometry_bridge.log" 2>&1 &
ODOM_BRIDGE_PID=$!

log "Starting Super-LIO with ${SUPER_LIO_CONFIG_NAME}"
setsid stdbuf -oL -eL ros2 launch \
  super_lio Livox_mid360.launch.py \
  rviz:=false \
  config_file:="$SUPER_LIO_CONFIG" \
  >"${TEST_DIR}/super_lio.log" 2>&1 &
SUPER_LIO_PID=$!

wait_for_stack
validate_topology
start_monitors
start_rosbag

log "Recording is active: ${TEST_DIR}"
log "PX4 control source: ${EXPECTED_CONTROL_TOPIC}"
if [[ "$RUN_MODE" == "shadow" ]]; then
  log "Super-LIO dynamic output is shadow-only and recorded under /lio/*"
elif [[ "$NOKOV_SHADOW_ENABLED" == "true" ]]; then
  log "NOKOV output is shadow-only and recorded under ${NOKOV_TOPIC}"
fi
log "Use POSCTL only. This runner does not arm or start Offboard."
log "After disarm, wait 15 seconds and press Ctrl+C once."

while kill -0 "$BAG_PID" >/dev/null 2>&1; do
  sleep 1
done

wait "$BAG_PID" || true
die "ROS 2 bag recorder stopped unexpectedly"
