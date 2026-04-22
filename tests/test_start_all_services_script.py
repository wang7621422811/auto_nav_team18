from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "start_all_services.sh"


def test_start_all_script_exists_and_is_documented() -> None:
    content = SCRIPT.read_text(encoding="utf-8")

    assert SCRIPT.is_file()
    assert "#!/usr/bin/env bash" in content
    assert "Usage:" in content
    assert "[full|test|bench]" in content


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


def test_start_all_script_has_real_world_test_mode() -> None:
    content = SCRIPT.read_text(encoding="utf-8")

    assert 'if [[ "${MODE}" == "test" ]]' in content
    assert 'USE_GPS="true"' in content
    assert 'USE_CAMERA="true"' in content
    assert 'USE_NMEA_GPS="true"' in content


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
