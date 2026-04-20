```
仓库结构
repo/
  teleop/
    joy_mapper_node.py
    mode_manager_node.py
    cmd_gate_node.py
    gamepad_watchdog_node.py

  navigation/
    waypoint_provider.py
    geo_localizer.py
    local_planner.py
    path_follower.py
    obstacle_guard.py
    gap_planner.py
    weave_planner.py
    final_approach.py

  perception/
    cone_detector.py
    object_detector.py
    shape_classifier.py
    distance_estimator.py
    photo_capture.py
    color_calibrator.py

  mission/
    mission_controller.py
    journey_logger.py
    summary_generator.py
    mission_events.py

  config/
    joystick.yaml
    robot.yaml
    waypoints.yaml
    camera.yaml
    lidar.yaml
    mission.yaml
    sim.yaml
    real.yaml

  launch/
    bringup.launch.py
    teleop.launch.py
    navigation.launch.py
    mission.launch.py
    sim.launch.py

  tests/
    test_teleop.py
    test_waypoints.py
    test_perception.py
    test_mission_fsm.py

  artifacts/
    photos/
    logs/
    summaries/

```

接口: 不要一编写一边该接口

```
控制相关 topic
/joy
/control_mode：MANUAL / AUTO / PAUSED / ABORTED
/cmd_vel_manual
/cmd_vel_auto
/cmd_vel_safe
/deadman_ok
/emergency_stop
传感器相关 topic
/odom
/imu/data
/scan
/camera/color/image_raw
/camera/depth/image_raw
导航/任务 topic
/waypoint/current
/waypoint/status
/marker/detection
/object/detection
/mission/state
/journey/event
最小 TF
odom -> base_link
base_link -> laser
base_link -> camera_link
如果保留 GPS 输入，再加 map/local_origin
```

