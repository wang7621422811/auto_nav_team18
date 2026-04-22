#!/usr/bin/env bash

set -euo pipefail

# Usage:
#   ./scripts/record_gps_debug.sh [label]
#
# Purpose:
#   Record the key GPS-navigation debug topics into a timestamped directory
#   using ros2 bag, plus a few lightweight one-shot snapshots.
#
# Behavior:
#   - sources ROS + workspace
#   - creates artifacts/logs/gps_debug_<timestamp>[_label]
#   - records key topics into a rosbag directory
#   - saves one-shot snapshots for params, nodes, and topics
#   - optionally records /scan if RECORD_SCAN=true
#   - packs the directory into a .tar.gz file on exit

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
LABEL="${1:-}"
RECORD_SCAN="${RECORD_SCAN:-false}"

PIDS=()
ARCHIVE_CREATED="false"
CLEANUP_DONE="false"
STOPPING="false"
BAG_PID=""
LOG_DIR=""
ARCHIVE_PATH=""
BAG_DIR=""

timestamp() {
    date +"%Y%m%d_%H%M%S"
}

iso_timestamp() {
    date +"%Y-%m-%dT%H:%M:%S%z"
}

sanitize_label() {
    local raw="$1"
    printf "%s" "${raw}" | tr -cs '[:alnum:]_-' '_'
}

source_workspace() {
    set +u
    if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
        source "/opt/ros/${ROS_DISTRO}/setup.bash"
    else
        echo "ROS setup not found: /opt/ros/${ROS_DISTRO}/setup.bash" >&2
        exit 1
    fi

    if [[ ! -f "${ROOT_DIR}/install/setup.bash" ]]; then
        echo "Workspace setup not found: ${ROOT_DIR}/install/setup.bash" >&2
        exit 1
    fi
    source "${ROOT_DIR}/install/setup.bash"
    set -u
}

write_snapshot() {
    local outfile="$1"
    shift

    {
        echo "# command: $*"
        "$@"
    } >"${outfile}" 2>&1 || true
}

start_background_snapshot() {
    local outfile="$1"
    shift

    (
        write_snapshot "${outfile}" "$@"
    ) &
    PIDS+=("$!")
}

start_rosbag_record() {
    local -a topics=(
        /control_mode
        /deadman_ok
        /emergency_stop
        /fix
        /nav/status
        /nav/odom
        /waypoint/current
        /waypoint/status
        /local_target
        /gap/local_target
        /weave/local_target
        /cmd_vel_auto
        /cmd_vel_safe
    )

    if [[ "${RECORD_SCAN}" == "true" ]]; then
        topics+=(/scan)
    fi

    printf "%s\n" "${topics[@]}" >"${LOG_DIR}/recorded_topics.txt"

    echo "[record] rosbag -> ${BAG_DIR}"
    {
        echo "# started_at: $(iso_timestamp)"
        echo "# command: ros2 bag record --output ${BAG_DIR} <topics>"
        printf "# topics:\n"
        printf "#   %s\n" "${topics[@]}"
    } >"${LOG_DIR}/rosbag_record.txt"

    ros2 bag record --output "${BAG_DIR}" "${topics[@]}" \
        >>"${LOG_DIR}/rosbag_record.txt" 2>&1 &
    BAG_PID="$!"
}

create_archive() {
    if [[ "${ARCHIVE_CREATED}" == "true" ]]; then
        return
    fi

    tar -czf "${ARCHIVE_PATH}" -C "$(dirname "${LOG_DIR}")" "$(basename "${LOG_DIR}")"
    ARCHIVE_CREATED="true"
    echo "[archive] ${ARCHIVE_PATH}"
}

cleanup() {
    local pid

    if [[ "${CLEANUP_DONE}" == "true" ]]; then
        return
    fi
    CLEANUP_DONE="true"

    if [[ -n "${BAG_PID}" ]] && kill -0 "${BAG_PID}" 2>/dev/null; then
        echo "[stop] Stopping rosbag recorder..."
        kill -INT "${BAG_PID}" 2>/dev/null || true
        wait "${BAG_PID}" 2>/dev/null || true
    fi

    for pid in "${PIDS[@]:-}"; do
        if kill -0 "${pid}" 2>/dev/null; then
            kill "${pid}" 2>/dev/null || true
        fi
    done
    wait "${PIDS[@]:-}" 2>/dev/null || true

    echo "[stop] Creating archive..."
    create_archive
}

handle_signal() {
    if [[ "${STOPPING}" == "true" ]]; then
        return
    fi
    STOPPING="true"
    echo "[stop] Ctrl+C received. Finalizing recording, please wait..."
    trap - INT TERM
    cleanup
    exit 130
}

main() {
    local run_id
    local safe_label=""

    source_workspace

    run_id="$(timestamp)"
    if [[ -n "${LABEL}" ]]; then
        safe_label="$(sanitize_label "${LABEL}")"
        LOG_DIR="${ROOT_DIR}/artifacts/logs/gps_debug_${run_id}_${safe_label}"
    else
        LOG_DIR="${ROOT_DIR}/artifacts/logs/gps_debug_${run_id}"
    fi
    ARCHIVE_PATH="${LOG_DIR}.tar.gz"
    BAG_DIR="${LOG_DIR}/rosbag"

    mkdir -p "${LOG_DIR}"

    trap cleanup EXIT
    trap handle_signal INT TERM

    echo "[hint] Stop recording with Ctrl+C once. Wait for the archive line, then the shell will return."

    {
        echo "run_id=${run_id}"
        echo "label=${LABEL}"
        echo "record_scan=${RECORD_SCAN}"
        echo "root_dir=${ROOT_DIR}"
        echo "ros_distro=${ROS_DISTRO}"
        echo "started_at=$(iso_timestamp)"
    } >"${LOG_DIR}/session_info.txt"

    write_snapshot "${LOG_DIR}/nodes.txt" ros2 node list
    write_snapshot "${LOG_DIR}/topics.txt" ros2 topic list -t
    write_snapshot "${LOG_DIR}/path_follower_waypoints_file.txt" ros2 param get /path_follower waypoints_file
    write_snapshot "${LOG_DIR}/path_follower_use_gps.txt" ros2 param get /path_follower use_gps

    start_background_snapshot "${LOG_DIR}/control_mode_once.txt" ros2 topic echo --once /control_mode
    start_background_snapshot "${LOG_DIR}/nav_status_once.txt" ros2 topic echo --once /nav/status
    start_background_snapshot "${LOG_DIR}/nav_odom_once.txt" ros2 topic echo --once /nav/odom
    start_background_snapshot "${LOG_DIR}/waypoint_current_once.txt" ros2 topic echo --once /waypoint/current
    start_background_snapshot "${LOG_DIR}/waypoint_status_once.txt" ros2 topic echo --once /waypoint/status

    start_rosbag_record

    cat <<EOF
[ready] GPS debug recording started
[log_dir] ${LOG_DIR}
[bag_dir] ${BAG_DIR}
[archive] ${ARCHIVE_PATH}

Run your field test now. Keep this terminal open.
Press Ctrl+C here after the test finishes.
Then send me the generated .tar.gz file.
EOF

    while true; do
        sleep 1
    done
}

main "$@"
