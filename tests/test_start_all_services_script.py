from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "start_all_services.sh"


def test_start_all_script_exists_and_is_documented() -> None:
    content = SCRIPT.read_text(encoding="utf-8")

    assert SCRIPT.is_file()
    assert "#!/usr/bin/env bash" in content
    assert "Usage:" in content
    assert "[full|bench]" in content


def test_start_all_script_launches_full_stack_in_order() -> None:
    content = SCRIPT.read_text(encoding="utf-8")

    bringup_index = content.index("bringup.launch.py")
    teleop_index = content.index("teleop.launch.py")
    navigation_index = content.index("navigation.launch.py")
    perception_index = content.index("perception.launch.py")
    mission_index = content.index("mission.launch.py")

    assert bringup_index < teleop_index < navigation_index < perception_index < mission_index


def test_start_all_script_supports_bench_mode() -> None:
    content = SCRIPT.read_text(encoding="utf-8")

    assert "navigation_bench.launch.py" in content
    assert 'if [[ "${MODE}" == "bench" ]]' in content


def test_start_all_script_defaults_to_real_world_outdoor_mode() -> None:
    content = SCRIPT.read_text(encoding="utf-8")

    assert 'USE_GPS="${USE_GPS:-true}"' in content
    assert 'USE_CAMERA="${USE_CAMERA:-true}"' in content
    assert 'WAYPOINTS_FILE="${WAYPOINTS_FILE:-${ROOT_DIR}/config/waypoints_real_gps.yaml}"' in content
    assert 'if [[ "${MODE}" != "bench" ]]; then' in content
    assert 'WAYPOINTS_FILE="${GPS_WAYPOINTS_FILE}"' in content


def test_start_all_script_forces_real_outdoor_stack_in_full_mode() -> None:
    content = SCRIPT.read_text(encoding="utf-8")

    assert 'USE_GPS="true"' in content
    assert 'USE_CAMERA="true"' in content
    assert 'USE_SIM_TIME="false"' in content
    assert 'USE_NMEA_GPS="${USE_GPS}"' in content


def test_start_all_script_enables_imu_and_passes_phidget_args() -> None:
    content = SCRIPT.read_text(encoding="utf-8")

    assert 'USE_IMU="${USE_IMU:-true}"' in content
    assert 'IMU_SERIAL="${IMU_SERIAL:--1}"' in content
    assert 'IMU_HUB_PORT="${IMU_HUB_PORT:-0}"' in content
    assert 'use_imu:="${USE_IMU}"' in content
    assert 'imu_serial:="${IMU_SERIAL}"' in content
    assert 'imu_hub_port:="${IMU_HUB_PORT}"' in content


def test_start_all_script_rejects_local_waypoints_in_gps_mode() -> None:
    content = SCRIPT.read_text(encoding="utf-8")

    assert 'GPS_WAYPOINTS_FILE="${ROOT_DIR}/config/waypoints_real_gps.yaml"' in content
    assert 'LOCAL_WAYPOINTS_FILE="${ROOT_DIR}/config/waypoints_data.yaml"' in content
    assert 'if [[ "${USE_GPS}" == "true" ]] && { [[ "${WAYPOINTS_FILE}" == "${LOCAL_WAYPOINTS_FILE}" ]] || [[ "${WAYPOINTS_FILE}" == *"/waypoints_data.yaml" ]]; }; then' in content
    assert 'GPS outdoor mode refuses local waypoint file' in content


def test_start_all_script_relaxes_nounset_while_sourcing_ros() -> None:
    content = SCRIPT.read_text(encoding="utf-8")

    source_workspace_start = content.index("source_workspace() {")
    set_plus_u = content.index("\n    set +u\n", source_workspace_start)
    ros_setup = content.index(
        '\n        source "/opt/ros/${ROS_DISTRO}/setup.bash"\n',
        source_workspace_start,
    )
    workspace_setup = content.index(
        '\n    source "${ROOT_DIR}/install/setup.bash"\n',
        source_workspace_start,
    )
    set_minus_u = content.index("\n    set -u\n", source_workspace_start)

    assert source_workspace_start < set_plus_u < ros_setup < workspace_setup < set_minus_u
