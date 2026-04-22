#!/usr/bin/env bash

set -euo pipefail

# Usage:
#   ./scripts/start_all_services.sh [full|test|bench]
#
# Modes:
#   full  - build, then start full stack with current/default flags
#   test  - build, then start full stack with GPS + NMEA GPS + camera enabled
#   bench - build, then start bringup + teleop + navigation_bench

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:-full}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

USE_GPS="${USE_GPS:-false}"
USE_CAMERA="${USE_CAMERA:-true}"
USE_NMEA_GPS="${USE_NMEA_GPS:-}"
USE_SIM_TIME="${USE_SIM_TIME:-false}"
GAMEPAD="${GAMEPAD:-ps4}"
JOY_DEV="${JOY_DEV:-/dev/input/js0}"
SERIAL_PORT="${SERIAL_PORT:-/dev/ttyUSB0}"
GPS_PORT="${GPS_PORT:-/dev/ttyACM0}"
GPS_BAUD="${GPS_BAUD:-9600}"
GPS_FRAME_ID="${GPS_FRAME_ID:-gps}"
WAYPOINTS_FILE="${WAYPOINTS_FILE:-}"

PIDS=()

resolve_mode_defaults() {
    if [[ "${MODE}" == "test" ]]; then
        USE_GPS="true"
        USE_CAMERA="true"
        if [[ -z "${USE_NMEA_GPS}" ]]; then
            USE_NMEA_GPS="true"
        fi
        return
    fi

    if [[ -z "${USE_NMEA_GPS}" ]]; then
        USE_NMEA_GPS="${USE_GPS}"
    fi
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

cleanup() {
    local pid
    for pid in "${PIDS[@]:-}"; do
        if kill -0 "${pid}" 2>/dev/null; then
            kill "${pid}" 2>/dev/null || true
        fi
    done
}

start_launch() {
    local name="$1"
    shift

    echo "[start] ${name}"
    "$@" &
    PIDS+=("$!")
    sleep 2
}

build_workspace() {
    echo "[build] colcon build --symlink-install"
    colcon build --symlink-install
}

print_summary() {
    echo "[mode] ${MODE}"
    echo "[gps] ${USE_GPS}"
    echo "[nmea_gps] ${USE_NMEA_GPS}"
    echo "[camera] ${USE_CAMERA}"
}

main() {
    if [[ "${MODE}" != "full" && "${MODE}" != "test" && "${MODE}" != "bench" ]]; then
        echo "Usage: ./scripts/start_all_services.sh [full|test|bench]" >&2
        exit 1
    fi

    resolve_mode_defaults
    trap cleanup EXIT INT TERM

    source_workspace
    build_workspace
    source_workspace
    print_summary

    start_launch \
        "bringup" \
        ros2 launch auto_nav bringup.launch.py \
        serial_port:="${SERIAL_PORT}" \
        use_gps:="${USE_GPS}" \
        use_nmea_gps:="${USE_NMEA_GPS}" \
        use_camera:="${USE_CAMERA}" \
        gps_port:="${GPS_PORT}" \
        gps_baud:="${GPS_BAUD}" \
        gps_frame_id:="${GPS_FRAME_ID}" \
        joy_dev:="${JOY_DEV}" \
        gamepad:="${GAMEPAD}" \
        use_sim_time:="${USE_SIM_TIME}"

    start_launch \
        "teleop" \
        ros2 launch auto_nav teleop.launch.py \
        gamepad:="${GAMEPAD}" \
        joy_dev:="${JOY_DEV}" \
        use_sim_time:="${USE_SIM_TIME}"

    if [[ "${MODE}" == "bench" ]]; then
        start_launch \
            "navigation_bench" \
            ros2 launch auto_nav navigation_bench.launch.py \
            use_sim_time:="${USE_SIM_TIME}" \
            log_filename:="navigation_bench.log"
    else
        if [[ -n "${WAYPOINTS_FILE}" ]]; then
            start_launch \
                "navigation" \
                ros2 launch auto_nav navigation.launch.py \
                use_sim_time:="${USE_SIM_TIME}" \
                use_gps:="${USE_GPS}" \
                waypoints_file:="${WAYPOINTS_FILE}"
        else
            start_launch \
                "navigation" \
                ros2 launch auto_nav navigation.launch.py \
                use_sim_time:="${USE_SIM_TIME}" \
                use_gps:="${USE_GPS}"
        fi

        start_launch \
            "perception" \
            ros2 launch auto_nav perception.launch.py \
            use_sim_time:="${USE_SIM_TIME}"

        start_launch \
            "mission" \
            ros2 launch auto_nav mission.launch.py \
            use_sim_time:="${USE_SIM_TIME}" \
            use_gps:="${USE_GPS}"
    fi

    echo "[ready] services started, Ctrl+C to stop all"
    wait
}

main "$@"
