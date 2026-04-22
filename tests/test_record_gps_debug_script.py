from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "record_gps_debug.sh"


def test_record_script_exists_and_is_documented() -> None:
    content = SCRIPT.read_text(encoding="utf-8")

    assert SCRIPT.is_file()
    assert "#!/usr/bin/env bash" in content
    assert "Usage:" in content
    assert "./scripts/record_gps_debug.sh [label]" in content
    assert "ros2 bag" in content
    assert "RECORD_SCAN" in content


def test_record_script_uses_rosbag_for_key_navigation_topics() -> None:
    content = SCRIPT.read_text(encoding="utf-8")

    assert "start_rosbag_record()" in content
    assert "ros2 bag record --output" in content
    assert "/control_mode" in content
    assert "/deadman_ok" in content
    assert "/emergency_stop" in content
    assert "/fix" in content
    assert "/nav/status" in content
    assert "/nav/odom" in content
    assert "/waypoint/current" in content
    assert "/waypoint/status" in content
    assert "/local_target" in content
    assert "/gap/local_target" in content
    assert "/weave/local_target" in content
    assert "/cmd_vel_auto" in content
    assert "/cmd_vel_safe" in content
    assert 'topics+=(/scan)' in content


def test_record_script_saves_snapshots_and_archive() -> None:
    content = SCRIPT.read_text(encoding="utf-8")

    assert 'write_snapshot "${LOG_DIR}/nodes.txt" ros2 node list' in content
    assert 'write_snapshot "${LOG_DIR}/topics.txt" ros2 topic list -t' in content
    assert 'write_snapshot "${LOG_DIR}/path_follower_waypoints_file.txt" ros2 param get /path_follower waypoints_file' in content
    assert 'write_snapshot "${LOG_DIR}/path_follower_use_gps.txt" ros2 param get /path_follower use_gps' in content
    assert 'start_background_snapshot "${LOG_DIR}/control_mode_once.txt" ros2 topic echo --once /control_mode' in content
    assert 'start_background_snapshot "${LOG_DIR}/nav_status_once.txt" ros2 topic echo --once /nav/status' in content
    assert 'tar -czf "${ARCHIVE_PATH}"' in content


def test_record_script_waits_for_manual_stop_and_interrupts_rosbag() -> None:
    content = SCRIPT.read_text(encoding="utf-8")

    assert 'kill -INT "${BAG_PID}"' in content
    assert 'wait "${BAG_PID}"' in content
    assert "while true; do" in content
    assert "sleep 1" in content
    assert "trap cleanup EXIT" in content
    assert "trap handle_signal INT TERM" in content
    assert "exit 130" in content
    assert 'echo "[stop] Ctrl+C received. Finalizing recording, please wait..."' in content
    assert 'echo "[stop] Stopping rosbag recorder..."' in content
    assert 'echo "[stop] Creating archive..."' in content
    assert 'if [[ "${CLEANUP_DONE}" == "true" ]]; then' in content
    assert 'if [[ "${STOPPING}" == "true" ]]; then' in content
