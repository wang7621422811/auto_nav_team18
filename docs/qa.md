# QA Log

## 2026-04-22

### 问题
收到带 `/scan` 的室外 `AUTO` 日志 `gps_debug_20260422_114851_gps_outdoor_spin.zip`。用户描述为“原地左右打转”。需要明确判断是 LiDAR 局部规划导致，还是 IMU 航向方向本身与车体前向相反。

### 日志核对
- `control_mode_once.txt`：`AUTO`
- `waypoint_status_once.txt`：`NAVIGATING`
- `nav_status_once.txt`：`GPS_READY`
- `path_follower_waypoints_file.txt`：`/root/workspace/auto_nav_team18/config/waypoints_real_gps.yaml`
- `recorded_topics.txt`：本次已包含 `/scan`
- `nodes.txt`：`/outdoor_pose_fuser`、`/phidgets_spatial`、`/sick_scan` 都在线

### 关键分析
- 快照时：
- raw `/odom` 朝向约 `39.1°`
- `/nav/odom` 朝向约 `-144.6°`
- `/imu/data` 朝向约 `-142.5°`
- 当前 waypoint bearing 约 `42.8°`
- 这说明：
- raw `/odom` 本来已经基本朝向 waypoint，误差只有约 `-4.9°`
- 但 `outdoor_pose_fuser` 一旦采用 IMU yaw，`/nav/odom` 会被翻到几乎相反方向，与 waypoint 的夹角约 `172.6°`
- 同时快照中的角速度：
- `/odom.twist.angular.z ≈ -0.873 rad/s`
- `/nav/odom.twist.angular.z ≈ -0.866 rad/s`
- 线速度几乎为零
- 这与现场“原地左右打转”高度一致。

### 结论
- 这次根因已经可以明确：
- **Phidget Spatial 的 yaw 参考方向相对 `base_link` 反了约 180°，导致 `outdoor_pose_fuser` 用 IMU yaw 后把导航朝向翻到 waypoint 反方向。**
- 因此这不是：
- 启动模式错误；
- `waypoints_data.yaml` 回退；
- 也不是当前证据下最主要的 LiDAR 局部避障问题。

### 修改
- 在 `auto_nav/navigation/outdoor_pose_fuser.py` 中新增参数：
- `imu_yaw_offset_deg`
- IMU 回调现在会先把四元数 yaw 加上该偏置，再作为 fused yaw 使用。
- 在 `config/gps.yaml` 中将：
- `imu_yaw_offset_deg: 180.0`
- 并注明当前真实机器上 Phidget Spatial 的安装朝向与 `base_link` 反向。
- 在 `tests/test_outdoor_pose_fuser.py` 中补充：
- IMU yaw offset 应用于融合姿态前的单元测试。

### 测试
- 执行：
- `python3 -m pytest tests/test_outdoor_pose_fuser.py -q`
- 结果：`8 passed in 0.04s`
- 执行：
- `python3 -m pytest tests/test_start_all_services_script.py tests/test_tf_tree.py -q`
- 结果：`21 passed in 0.02s`

### 结论补充
- 这次修复不是简单地禁用 IMU，而是保留 IMU 航向链路，同时按当前实机安装方向补 180° 修正。
- 这样既能继续利用 IMU 抑制 GPS/odom 漂移，又能避免导航朝向被整体翻转到目标点反方向。

### 问题
收到新的室外 `AUTO` 日志 `gps_debug_20260422_113116.zip`。用户反馈“车乱窜，而且一直在转向”。需要判断这次是否仍然是启动模式错误，还是已经进入真实 GPS 模式后控制本身出现振荡。

### 日志核对
- `control_mode_once.txt`：`AUTO`
- `waypoint_status_once.txt`：`NAVIGATING`
- `nav_status_once.txt`：`GPS_READY`
- `path_follower_waypoints_file.txt`：`/root/workspace/auto_nav_team18/config/waypoints_real_gps.yaml`
- `nodes.txt`：有 `/outdoor_pose_fuser`，没有 `/odom_tf_broadcaster`
- `nav_odom_once.txt`：`/nav/odom` 正常发布

### 关键分析
- 这次与之前 `gps_debug_20260422_111249.zip` 不同，启动模式已经正确：
- 真正进入了 GPS 模式；
- 真正用了 `waypoints_real_gps.yaml`；
- `outdoor_pose_fuser` 正常在线；
- `nav_status=GPS_READY`。
- 快照中：
- `nav_odom` 位置约 `(-16.82, -6.61)`
- 当前 waypoint 约 `(5.21, 19.48)`
- `nav_odom` 朝向约 `51.3°`
- 从当前 `nav_odom` 指向 waypoint 的 bearing 约 `49.8°`
- 两者误差仅约 `-1.5°`。
- 也就是说，在快照这一刻，机器人全局朝向其实**已经大致对准 waypoint**，并不存在“静态上完全反着 180°”这种情况。

### 结论
- 这次问题**不是**“又跑错到本地 waypoint 模式”。
- 但从现有包里也**不能直接证明**是 IMU yaw 符号反了，因为静态快照时 heading 与 waypoint bearing 是基本一致的。
- 更像的情况是：
- 运行过程中 `/cmd_vel_auto`、`/cmd_vel_safe` 或 `/local_target` / `/gap/local_target` 在跳变；
- 或 GPS / 融合位置在过程中有抖动，导致控制器反复重新找角度；
- 或局部规划层在持续发布偏转目标。

### 证据缺口
- 这份包虽然录了 `/cmd_vel_auto`、`/cmd_vel_safe`、`/local_target`、`/gap/local_target`，但当前本地环境没有直接解析 MCAP 的 ROS 2 工具链，无法从时间序列里恢复“到底是谁在持续下转向命令”。
- 同时这次 `record_scan=false`，没有 `/scan`，所以无法判断是否是 LiDAR 局部避障在主导转向。
- `imu_data_once.txt` 也没有成功抓到本次快照，因此无法用快照交叉验证这一刻的 IMU yaw。

### 下一步建议
- 下一次 outdoor `AUTO` 复测必须开：
- `RECORD_SCAN=true`
- 并保留 `/cmd_vel_auto`、`/cmd_vel_safe`、`/local_target`、`/gap/local_target`、`/weave/local_target`
- 这样才能明确区分：
- 是 `path_follower` 自己在发转向；
- 还是 `gap_planner` / `local_planner` 在持续把目标拉向侧面；
- 还是 `obstacle_guard` / safety gate 在改写速度。

### 问题
用户明确要求启动脚本默认行为改成“真机室外 GPS 模式”，不再依赖额外传参，也不允许默认落回测试/仿真 waypoint。目标是：
- 不传参数时默认启动 GPS；
- 默认启动 IMU；
- 默认启动相机；
- 默认使用 `waypoints_real_gps.yaml`；
- 默认拒绝 `waypoints_data.yaml`。

### 修改
- 更新 `scripts/start_all_services.sh`：
- `USE_GPS` 默认值从 `false` 改为 `true`
- `WAYPOINTS_FILE` 默认值改为 `config/waypoints_real_gps.yaml`
- 去掉原来的 `test` 模式依赖，不再要求用户通过 `test` 模式才能进入真机 GPS 启动链路
- `full` 模式现在固定代表“室外真机模式”
- 只有 `bench` 模式才关闭 GPS，并进入 bench 联调链路
- `resolve_mode_defaults()` 中对非 `bench` 模式强制：
- `USE_GPS=true`
- `USE_CAMERA=true`
- `USE_SIM_TIME=false`
- `WAYPOINTS_FILE=waypoints_real_gps.yaml`
- 继续保留保护逻辑：GPS 模式下一旦检测到 `waypoints_data.yaml`，立即报错退出。
- 更新 `tests/test_start_all_services_script.py`：
- 改为校验脚本默认就是 outdoor 真机模式；
- 校验 usage 现在是 `[full|bench]`；
- 校验 `full` 模式下强制使用 `waypoints_real_gps.yaml`；
- 校验 GPS 模式下拒绝本地 waypoint 的逻辑。
- 更新 `docs/gps_nav_test_plan.md`：
- 启动命令改为默认直接执行 `./scripts/start_all_services.sh`
- bench 联调改为 `./scripts/start_all_services.sh bench`

### 测试
- 执行：
- `python3 -m pytest tests/test_start_all_services_script.py tests/test_tf_tree.py -q`
- 结果：`21 passed in 0.03s`

### 结论
- 现在用户直接运行：
- `./scripts/start_all_services.sh`
- 就会默认进入：
- 真机 GPS
- 真机 IMU
- 真机相机
- `waypoints_real_gps.yaml`
- 这样可以从启动脚本层面减少再次误跑到 `waypoints_data.yaml` / `odom_tf_broadcaster` 模式的概率。

### 问题
收到室外 `AUTO` 日志 `gps_debug_20260422_111249.zip`。用户反馈“方向完全相反，而且一直转向”，需要判断是 IMU 融合方向错误，还是系统再次错误地跑到了室内/本地 waypoint 模式。

### 日志核对
- `control_mode_once.txt`：`AUTO`
- `waypoint_status_once.txt`：`NAVIGATING`
- `nodes.txt`：运行的是 `/odom_tf_broadcaster`，**不是** `/outdoor_pose_fuser`
- `nav_odom_once.txt`：`/nav/odom` 实际上没有发布
- `nav_status_once.txt`：`/nav/status` 实际上没有发布
- `path_follower_waypoints_file.txt`：`/root/workspace/auto_nav_team18/install/auto_nav/share/auto_nav/config/waypoints_data.yaml`
- `waypoint_current_once.txt`：当前目标点是 `(-11.0, -11.0)`，这是本地 `odom` 平面里的仿真/室内 waypoint，不是真实 GPS waypoint

### 关键分析
- 这份日志最重要的证据不是 IMU，而是：
- `bringup` 侧启动了 `odom_tf_broadcaster`
- 没有启动 `outdoor_pose_fuser`
- `path_follower` 读到的是 `waypoints_data.yaml`
- `waypoint/current` 的 frame 也是本地 `odom`
- 这意味着系统此时**根本没在跑 GPS 室外模式**，而是在拿真实机器人去追一个本地平面目标 `(-11, -11)`。
- 对真实 outdoor 机器人来说，这种目标当然会表现成：
- 朝向完全不对；
- 一直原地找角度；
- 转向像“完全相反”。
- 因为它不是在跟 GPS waypoint 对齐，而是在跟一个错误坐标系下的虚拟 waypoint 对齐。

### 结论
- 这份 `gps_debug_20260422_111249.zip` **不能说明 IMU 方向错了**；
- 它说明的是：**这次启动流程又回到了错误模式，没有进入真实 GPS outdoor 导航。**
- 因此这次“完全反方向、一直转向”的直接根因是：
- `use_gps` 没有真正生效；
- `outdoor_pose_fuser` 没有启动；
- `path_follower` 读取了 `waypoints_data.yaml` 而不是 `waypoints_real_gps.yaml`。

### 修复/检查建议
- 现场必须先确认以下状态，再允许开始 outdoor AUTO 测试：
- `ros2 node list | grep outdoor_pose_fuser`
- `ros2 node list | grep odom_tf_broadcaster`
- `ros2 param get /path_follower waypoints_file`
- `ros2 topic echo /nav/status --once`
- 正确状态应当是：
- 有 `/outdoor_pose_fuser`
- 没有 `/odom_tf_broadcaster`
- `waypoints_file` 指向 `waypoints_real_gps.yaml`
- `/nav/status` 能发布，例如 `WAITING_FOR_FIX` / `GPS_READY`

### 问题
收到一份手动驾驶日志 `gps_debug_20260422_104703.zip`。用户反馈“robot 无法推得动”，需要判断这是 IMU/GPS 融合异常，还是底盘本身处于上电抱死/受控状态，导致“手推测试”本身不能作为融合验证手段。

### 日志核对
- `control_mode_once.txt`：`MANUAL`
- `waypoint_status_once.txt`：`IDLE`
- `nav_status_once.txt`：`ODOM_IMU_ONLY`
- `path_follower_waypoints_file.txt`：仍指向 `waypoints_real_gps.yaml`
- `nodes.txt`：`/phidgets_spatial`、`/phidgets_spatial_container`、`/outdoor_pose_fuser` 都已在线
- `imu_data_once.txt`：`/imu/data` 已有真实四元数和角速度
- `odom_once.txt`：原始底盘 odom 几乎静止，位置约 `x=0.001m`，角速度很小
- `nav_odom_once.txt`：位置仍几乎静止，但姿态已不再等于 raw `/odom`

### 关键分析
- 这次记录时系统明确处于 `MANUAL`，`waypoint/status=IDLE`，因此它**不是**一份“自动导航左右摇摆”的日志，而是一份“手动/静止状态下底层姿态融合”的日志。
- `nav_status=ODOM_IMU_ONLY` 说明当时没有有效 GPS fix 参与，`outdoor_pose_fuser` 正在走“轮速位置 + IMU 航向”的退化模式。
- 从快照四元数计算：
- raw `/odom` yaw 约为 `0°`
- `/imu/data` yaw 约为 `-159.9°`
- `/nav/odom` yaw 约为 `-159.1°`
- 这说明 `outdoor_pose_fuser` 此时已经在使用 IMU yaw，而且 `/nav/odom` 与 IMU 只差约 `0.8°`，链路是通的。
- 同时，`/odom` 几乎没有位移变化，说明记录期间机器人本体基本没发生可观运动。结合“推不动”的现场描述，更符合：
- Pioneer 底盘上电后电机/传动处于受控或抱死状态；
- 用户没有通过手柄实际驱动车体产生可观运动；
- 因此这份包**不能用来判断前进/转弯控制好坏**，只能用来确认 IMU 已经接入并被融合。

### 结论
- 这份手动日志证明了一件好事：
- **Phidget Spatial 3/3/3 已经成功接入，`/nav/odom` 航向已经跟随 IMU。**
- 但它也说明：
- **“手推机器人”不是当前这台上电 Pioneer 的有效测试方法。**
- 因为底盘几乎没动，日志不足以评估自动导航摇摆问题，也不足以检验 `odom` 与 IMU 在真实运动中的符号/方向关系。

### 下一步建议
- 之后的验证必须改成“手柄实际驾驶”，不要再用手推：
- 原地静止 5 到 10 秒；
- 手柄直线前进 2 到 3 米；
- 停住；
- 原地左转约 90°；
- 停住；
- 再原地右转约 90° 回来。
- 并继续用 `record_gps_debug.sh` 录包。
- 这样才能检查：
- `/cmd_vel_safe` 是否真的下发到底盘；
- `/odom` 的线位移/角位移是否与手柄动作一致；
- `/imu/data` 的 yaw 增减方向是否和 `/odom` 一致；
- 若两者符号相反，再决定是改 IMU 安装朝向、TF，还是在融合里做 yaw 反向修正。

### 问题
需要把真实的 Phidget Spatial 3/3/3 IMU 正式接入项目，而不是只在 `outdoor_pose_fuser` 里预留 `/imu/data` 订阅接口。此前实机上虽然能看到 `/imu`、`/imu/data` 这样的 topic 名字，但没有对应的 IMU publisher 节点上线，导致 GPS 室外导航实际上一直退化为纯轮速/odom 航向。

### 现有实现核对
- 仓库原先只有 IMU 消费端，没有 IMU 启动链路：
- `auto_nav/navigation/outdoor_pose_fuser.py` 会订阅 `imu_topic` 并读取四元数 yaw；
- `config/gps.yaml` 把 `imu_topic` 设为 `/imu/data`；
- 但 `launch/bringup.launch.py` 之前没有任何 `phidgets_spatial` 或其他 IMU driver 节点。
- 真实机器日志也验证了这一点：
- `ros2 topic list | grep imu` 能看到 `/imu`、`/imu/data`；
- 但 `ros2 node list | grep -E "imu|phidget|spatial|ekf"` 为空；
- 因此这些 topic 只是接口名存在，不代表有真实消息流。

### 修改
- 在 `launch/bringup.launch.py` 新增 IMU bringup 参数与节点：
- 新增 `use_imu`、`imu_serial`、`imu_hub_port` launch argument；
- 新增 `_imu_nodes()`，使用 `rclcpp_components` 容器加载 `phidgets_spatial` 的 `phidgets::SpatialRosI` 组件；
- 将官方默认 `/imu/data_raw` 直接 remap 成项目统一接口 `/imu/data`，避免下游节点继续分叉；
- 保留 `gps.yaml` 中的 `/imu/data` 作为系统唯一 IMU topic 入口。
- 新增 `config/imu.yaml`：
- 固化 Phidget Spatial 3/3/3 的运行参数；
- 默认 `use_orientation: true`，让驱动直接输出可被 `outdoor_pose_fuser` 使用的姿态四元数；
- 统一 `frame_id: imu_link`。
- 扩展真实机器人静态 TF：
- `config/robot.yaml` 与 `config/real.yaml` 新增 `imu_x / imu_y / imu_z`；
- `auto_nav/robot_extrinsics.py` 扩展为同时解析 laser / camera / imu 三套外参；
- `launch/bringup.launch.py` 的 `_sensor_static_tf()` 现在额外发布 `base_link -> imu_link`。
- 更新启动脚本：
- `scripts/start_all_services.sh` 新增 `USE_IMU`、`IMU_SERIAL`、`IMU_HUB_PORT`；
- 默认 `USE_IMU=true`，并在 bringup 启动时自动传给 `bringup.launch.py`，避免每次现场手动补 IMU 命令。
- 更新运行依赖：
- `package.xml` 增加 `phidgets_spatial` 与 `rclcpp_components` 运行依赖。

### 测试
- 新增/更新 `tests/test_tf_tree.py`：
- 校验 bringup 现在会发布 `tf_base_to_imu`；
- 校验 `_imu_nodes()` 能正确创建 `phidgets_spatial` 容器，并把 `/imu/data_raw` 规范化到 `/imu/data`。
- 更新 `tests/test_start_all_services_script.py`：
- 校验启动脚本默认启用 IMU，并把 `imu_serial` / `imu_hub_port` 传入 bringup。
- 本次执行：
- `python3 -m pytest tests/test_tf_tree.py tests/test_start_all_services_script.py tests/test_outdoor_pose_fuser.py -q`
- 结果：`27 passed in 0.11s`
- `python3 -m pytest tests/test_waypoints.py -q`
- 结果：`58 passed in 0.18s`

### 结论
- 这次修改后，项目层面已经真正具备了：
- 启动 Phidget Spatial 3/3/3；
- 统一把真实 IMU 数据接到 `/imu/data`；
- 给 IMU 提供 `imu_link` TF；
- 在 `start_all_services.sh` 真机链路里默认带起 IMU。
- 也就是说，后续现场只要机器人镜像里已安装 `phidgets_spatial`，并且设备映射进容器，`outdoor_pose_fuser` 就不再是“空等一个不存在的 IMU”。

### 剩余风险
- 当前是在本地仓库层面完成了集成和单测，尚未在你的真机容器里实测 `phidgets_spatial` 包、USB 权限和设备枚举是否完全匹配。
- `imu_x / imu_y / imu_z` 目前先给了近似默认值 `(0.00, 0.00, 0.23)`；若实机安装位置明显偏离，需要用卷尺实测后修正。
- 若 Phidget 驱动在你的镜像里使用的参数名与当前 Jazzy 版本存在差异，现场可能需要按实际安装版本微调 `config/imu.yaml`；但系统集成点和项目接口已经统一好了。

### 问题
需要确认当前 `AUTO` 模式下的避障链路是否已经真正具备以下行为：遇到障碍时绕开障碍物，绕开后再重新对正 waypoint / 目标角度，并继续直线行驶过去。

### 现有实现核对
- `auto_nav/navigation/gap_planner.py` 在每帧 LiDAR 中先检查“目标方向是否足够通畅”：
- 若 waypoint bearing 对应的窗口足够清晰，`_goal_direction_range()` 会直接返回该方向的可用距离，随后 `_scan_cb()` 直接按 `goal_angle` 发布 `/gap/local_target`。
- 若目标方向被挡住，`_find_gaps()` + `_score_and_pick()` 会从所有可通行 gap 中选一个，并且 `_guided_gap_angle()` 不会盲目取 gap 中心，而是尽量把目标角度往 waypoint bearing 拉回，只在超出 gap 边界时才裁到边缘。
- `auto_nav/navigation/weave_planner.py` 对 1→2 waypoint 的 weaving 段使用同样的“朝 waypoint 回拉”策略，同时额外限制 corridor，避免为了绕障直接绕出锥桶走廊。
- `auto_nav/navigation/local_planner.py` 只在 `/waypoint/status == NAVIGATING` 时转发局部目标；当局部目标过期时，停止转发，让 `path_follower` 回退到全局 waypoint。
- `auto_nav/navigation/path_follower.py` 在 `_do_navigating()` 中优先跟随新鲜且位于前向扇区内的 `/local_target`；否则直接回到全局 `waypoint`。
- 同文件 `_drive_toward()` 在 heading error 过大时会先原地转向，只有角度进入阈值内后才恢复线速度，因此“重新对正后再直行”的控制意图是明确存在的。

### 测试核对
- `tests/test_navigation_step3.py` 已覆盖：
- `GapPlannerNode._guided_gap_angle()` 会把目标角度往 waypoint bearing 拉回；
- `GapPlannerNode._goal_direction_range()` 在目标方向清晰时会优先直接朝 waypoint；
- `WeavePlannerNode` 也有对应的 guided-gap / clear-goal 用例。
- `tests/test_waypoints.py` 已覆盖：
- `PathFollowerNode` 会拒绝后向 `local_target`，避免局部规划把车带到目标背面；
- `_drive_toward()` 对大角误差会原地旋转，不会在未对正时继续向前冲。
- 本次执行：
- `python3 -m pytest tests/test_navigation_step3.py tests/test_waypoints.py -q`
- 结果：`99 passed in 0.12s`

### 结论
- 从当前代码实现看，`AUTO` 模式的避障链路**基本已经按“绕障 -> 重新朝 waypoint 拉回 -> 对正后继续前进”这个思路实现**。
- 更具体地说，它不是单纯急停，而是：
- LiDAR 先尝试判断目标方向是否恢复通畅；
- 未恢复时走 gap / weave 局部目标；
- 恢复后重新发布朝 waypoint bearing 的局部目标；
- 若局部目标消失或过期，`path_follower` 还会进一步退回全局 waypoint 跟踪。
- 所以从代码逻辑上判断，这一行为是“有实现”的。

### 剩余风险
- 当前证据主要来自单元测试和静态代码核对，还**没有**看到“真实 scan 场景下先被挡住、再绕过去、最后稳定回到目标直线”的端到端仿真/实机测试记录。
- `obstacle_guard.py` 主要保证安全急停；如果场景里一度找不到有效 gap，系统更可能是“停住等待 / 退回 waypoint 驱动后再被 estop 卡住”，而不是保证一定能连续绕出。
- 因此更准确的表述应是：
- **实现框架已经具备，回拉目标角度的关键逻辑也在，但是否‘已经弄好到现场稳定可用’还需要补一条端到端验证证据。**

### 问题
已确认 `outdoor_pose_fuser` 只做平移对齐、没有做航向/坐标轴对齐，需要修复 GPS ENU 与 odom/yaw 参考系不一致导致的系统性偏航。

### 解决
- 修改 `auto_nav/navigation/outdoor_pose_fuser.py`：
- 首个有效 GPS fix 不再把 `/nav/odom` 锁在原始 odom 平移坐标里，而是把融合位置锚定到 waypoint 共用的 ENU 平面；
- 新增基于“GPS 位移方向 vs raw odom 位移方向”的 heading 对齐逻辑，估计 `odom -> ENU` 的固定旋转偏角；
- 在偏角建立后，把 wheel odom 的位置增量旋转到 ENU 再用于 `/nav/odom` 连续传播；
- 同时把发布出去的 yaw 也旋转到与 ENU 一致的参考系，保证 `path_follower` 的 `atan2(dy, dx)` 与机器人当前 yaw 使用同一坐标系。
- 更新 `config/gps.yaml`，新增：
- `heading_alignment_min_dist_m`
- `heading_alignment_alpha`
- 补充 `tests/test_outdoor_pose_fuser.py`：
- 首个 fix 后 `/nav/odom` 进入 ENU 坐标；
- GPS 恢复后的平滑回归；
- 关键新增用例：当 raw odom 轨迹相对 ENU 旋转 90° 时，`outdoor_pose_fuser` 能学习这个偏角，并把后续位置传播与姿态一起旋回 ENU。

### 结论
- 当前 bug 已按“统一到 ENU 平面 + 自动学习 odom/ENU 航向偏角”的方向修复。
- 这能直接消除“waypoint 在 ENU，但 `/nav/odom` yaw 不在同一参考系”导致的固定单侧偏航。
- 现场仍需注意：如果 IMU 自身存在额外磁航向漂移，`use_imu_yaw` 可能仍需要按实测切换回 odom orientation。

### 问题
需要核查这样一个判断是否成立：当前实机“总是偏一个方向，怎么调都不顺”的现象，根因可能不是 GPS 精度，而是 `outdoor_pose_fuser` 只做了 GPS ENU 到 odom 的平移对齐，没有做航向/坐标轴旋转对齐，导致 `path_follower` 用 yaw 算 heading error 时存在固定参考系偏差。

### 现有实现核对
- `auto_nav/navigation/geo_localizer.py` 明确把坐标定义为本地 ENU：`x=east, y=north`。
- `auto_nav/navigation/outdoor_pose_fuser.py` 在 `_fix_cb()` 中把 GPS fix 转成 ENU 后，只做了启动点平移对齐：
- `target_x = odom_init_x + (east_m - gps_init_e)`
- `target_y = odom_init_y + (north_m - gps_init_n)`
- `auto_nav/navigation/outdoor_pose_fuser.py` 没有保存任何“odom/IMU yaw 相对 ENU 的初始偏置”，也没有对 GPS 增量做二维旋转变换。
- 同文件 `_resolve_orientation()` 只是直接选择 IMU yaw 或原始 odom orientation 作为 `/nav/odom` 的姿态来源。
- `auto_nav/navigation/path_follower.py` 在 `_heading_error_to()` 中用 `atan2(dy, dx)` 得到目标方向，再直接减去 `self._robot_yaw`。
- 这说明控制器默认假设：
- 位置 `(x, y)` 所在坐标系；
- 当前 `yaw` 的零点与正方向；
- 二者是同一个参考系。

### 测试覆盖核对
- `tests/test_outdoor_pose_fuser.py` 目前覆盖了：
- 首个 GPS fix 的平移对齐；
- 对齐后的 ENU 位置投影；
- GPS 丢失后的 odom/IMU fallback；
- TF 与 `/nav/odom` 一致性。
- 但没有任何测试验证：
- IMU yaw 是否已经与 ENU 对齐；
- odom yaw 与 ENU 是否存在固定偏角；
- 若存在固定偏角，`outdoor_pose_fuser` 是否会补偿这个角度；
- 在存在 yaw 偏置时，`path_follower` 的 heading error 是否会稳定错误。

### 结论
- 这个判断基本成立，且证据强度较高。
- 目前实现里，`outdoor_pose_fuser` 的“对齐”确实只有位置平移，没有航向旋转对齐。
- `path_follower` 又明确依赖 `(x, y)` 与 `yaw` 共享同一参考系来计算 heading error。
- 因此，只要 IMU yaw 或原始 odom yaw 的零点不是 ENU 东北坐标系的同一参考，机器人就会表现出“持续向一侧偏”“参数怎么调都差一个固定角度”的系统性偏航。
- 所以当前更像是“坐标系/航向参考不一致”的实现缺口，而不是单纯“GPS 精度不够”。

### 建议
- 主方案仍应保留 `GPS + odom` 融合，不建议直接切成正式的纯 GPS 控制。
- 下一步优先做两件事：
- 1. 现场记录机器人静止朝向、直线推车方向与 `/nav/odom` yaw 的关系，确认是否存在固定 yaw 偏角；
- 2. 给 `outdoor_pose_fuser` 增加“启动时 yaw 相对 ENU 的旋转对齐”设计，或显式改成一个已经保证 ENU 对齐的 heading 来源。
- 如果只是为了快速诊断，可临时做纯 GPS 对照分支，但它更适合排障，不适合作为最终方案。

### 问题
真实户外测试里，GPS waypoint 导航持续出现明显偏差，需要评估后续应优先走“纯 GPS 导航”还是“GPS + odom 融合导航”方案。

### 现有实现核对
- 当前仓库并不是纯 GPS 导航，而是“GPS waypoint 转本地 ENU + GPS/IMU/odom 融合位姿 + 平面 waypoint 跟踪”：
- `auto_nav/navigation/waypoint_provider.py` 会把 `type: gps` 的 waypoint 转成局部 `(x, y)`；
- `auto_nav/navigation/outdoor_pose_fuser.py` 把 `/fix`、`/imu/data`、`/odom` 融成 `/nav/odom`；
- `launch/navigation.launch.py` 在 `use_gps:=true` 时把导航节点的 `/odom` remap 到 `/nav/odom`；
- `auto_nav/navigation/path_follower.py` 最终仍按平面距离和 heading error 控制，不直接按经纬度闭环。

### 关键风险判断
- 当前 `outdoor_pose_fuser.py` 只做了 GPS ENU 与 odom 的“平移对齐”：
- `target_x = odom_init_x + (gps_e - gps_init_e)`
- `target_y = odom_init_y + (gps_n - gps_init_n)`
- 但没有做“航向/坐标轴旋转对齐”。
- 这意味着 waypoint 坐标是 ENU 东北系，而机器人控制里使用的 yaw 却直接来自 IMU 或原始 odom orientation；如果这个 yaw 零点不是同一 ENU 参考，`path_follower.py` 计算 heading error 时就会产生系统性偏差。
- 现场表现上会像：
- 机器人始终朝一侧偏；
- 明明 waypoint 在前方，却持续向左或向右修不过来；
- 位置看起来大致合理，但朝向控制总是“差一个固定角度”。

### 方案评估
- 纯 GPS 导航：
- 优点：实现链路最短，容易排除 odom 融合逻辑的干扰；如果只是想验证 GPS 点位和大方向是否正确，能快速做最小闭环。
- 缺点：更新慢、抖动大、短距离误差明显；在 8 到 15 米级 waypoint 测试里很容易出现左右摆动、停不稳、最终到点误差大；更不适合当前任务里靠近 marker 的阶段。
- 结论：可作为“诊断模式”或 A/B 对照，不适合作为最终正式方案。
- GPS + odom 融合导航：
- 优点：更符合课程目标“GPS 负责粗导航，闭环不能依赖 GPS 精度”；位置连续，局部控制更平滑，也更容易和 LiDAR / final approach 共存。
- 缺点：前提是坐标系对齐必须正确；一旦 yaw 参考系错了，融合后反而会稳定地朝错误方向走。
- 结论：仍应作为主方案，但必须优先修正“ENU 与 yaw 的参考系一致性”问题，否则继续调 `gps_position_alpha`、`k_angular`、`k_linear` 的收益会很有限。

### 建议
- 不建议直接切到“纯 GPS 导航”作为正式路线。
- 建议优先按下面顺序排查和修正：
- 1. 先验证 `config/gps.yaml` 与 `config/waypoints_real_gps.yaml` 的 origin 一致；
- 2. 现场静态检查 `/nav/odom` 的位置方向和真实推车方向是否一致；
- 3. 单独检查 `/nav/odom` 的 yaw 是否与 ENU 方向一致；
- 4. 如果 yaw 与 ENU 不一致，优先补“启动时旋转对齐”或改成更可靠的 heading 来源；
- 5. 只有在需要做最小链路排障时，才临时做一个纯 GPS 控制分支做对照测试。

### 结论
- 当前偏差问题更像“坐标系/航向参考不一致”，不只是“GPS 不准”。
- 项目的最终推荐路线仍然是“GPS + odom 融合导航”，但需要把融合从“只做平移对齐”升级到“位置 + 航向参考一致”的版本。

### 问题
根据课程任务要求，需要判断当前仓库到底已经完成了哪些内容，哪些只是部分实现，哪些还没有形成可交付闭环。

### 结论
- 任务 1「按 waypoint 依次行驶并返回起点」：**部分完成**
- 任务 2「到达 waypoint 后拍 marker 照片并保持 marker 在机器人右侧」：**部分完成，且主闭环存在接口错位风险**
- 任务 3「1→2 waypoint 间 weaving through cones」：**部分完成**
- 任务 4「识别 waypoint 附近彩色物体形状、拍照并计算与 marker 距离」：**部分完成**
- 任务 5「任务完成后输出 journey summary」：**已基本完成**
- 任务 6「使用 LiDAR 避障」：**部分完成**
- 任务 7「蓝牙手柄 + X 自动 / O 手动 + dead-man」：**大体完成，但存在手柄映射与题面不完全一致的问题**

### 依据
- 导航主链已存在：
  - `auto_nav/navigation/waypoint_provider.py`
  - `auto_nav/navigation/path_follower.py`
  - `auto_nav/navigation/final_approach.py`
- 任务层已存在：
  - `auto_nav/mission/mission_controller.py`
  - `auto_nav/mission/journey_logger.py`
  - `auto_nav/mission/summary_generator.py`
- 感知层已存在：
  - `auto_nav/perception/cone_detector.py`
  - `auto_nav/perception/object_detector.py`
  - `auto_nav/perception/distance_estimator.py`
  - `auto_nav/perception/photo_capture.py`
- 手柄/安全层已存在：
  - `auto_nav/teleop/joy_mapper_node.py`
  - `auto_nav/teleop/mode_manager_node.py`
  - `auto_nav/teleop/gamepad_watchdog_node.py`
  - `auto_nav/teleop/cmd_gate_node.py`
- launch 集成已存在：
  - `launch/bringup.launch.py`
  - `launch/navigation.launch.py`
  - `launch/perception.launch.py`
  - `launch/mission.launch.py`

### 关键判断
- 任务 1：
  - `PathFollowerNode` 已实现 `HOME -> WP[0] -> ... -> HOME` 的状态机。
  - `tests/test_waypoints.py` 大部分 waypoint / homing 逻辑单测可通过。
  - 但真实 GPS waypoint 样例 `config/waypoints_real_gps.yaml` 当前只保留了 1 个 waypoint，和“multiple waypoints”场景不一致。
- 任务 2：
  - `PhotoCaptureNode` 能在 `FINAL_APPROACH:<idx>` 时保存 marker / object / annotated 图片。
  - `FinalApproachController` 与 `PathFollowerNode` 已实现“marker 留在机器人右侧”的通过点计算逻辑。
  - 但 `cone_detector.py` 把 `/marker/detection` 发布为 `camera_link` 下的 bearing/range 编码，而 `path_follower.py` 与 `obstacle_guard.py` 把同一消息当作 `odom` 坐标下的 marker `(x,y)` 直接使用；这说明“检测到 cone -> 生成右侧绕行 pass point”的真实接口很可能还没有完全打通。
- 任务 3：
  - `weave_planner.py`、`gap_planner.py`、`local_planner.py` 已实现 1→2 段 weaving / gap-following 逻辑，并由 `/navigation/segment == "1"` 激活。
  - `tests/test_navigation_step3.py` 单独运行可通过。
  - 但这仍主要证明规划逻辑存在，不能等价证明真实机器人已在未知间距 cones 中稳定穿行。
- 任务 4：
  - `object_detector.py` 已检测彩色目标并做形状分类。
  - `distance_estimator.py` 已计算 marker 与 object 的 3-D / fallback 距离。
  - `photo_capture.py` 已保存 object 图片。
  - 但 MissionController 当前订阅的是 `/object/detection`，没有直接消费 `/perception/distance` 结果；summary 中使用的是 object detection 里的 `range_m`，不等于题目要求的“object 到 waypoint marker 的距离”已经稳定进入任务闭环。
- 任务 5：
  - `journey_logger.py` 与 `summary_generator.py` 已支持 JSONL 日志和 Markdown/JSON summary 输出。
  - `tests/test_journey_summary.py` 单独运行可通过。
- 任务 6：
  - `obstacle_guard.py` 已实现前向安全扇区急停，`cmd_gate_node.py` 也会统一裁决速度输出。
  - 但它当前是“急停/限行”层，不等于完整的动态避障导航能力已经在实机验证完成。
- 任务 7：
  - `joy_mapper`、`mode_manager`、`gamepad_watchdog`、`cmd_gate` 已形成 X/AUTO、O/MANUAL、dead-man、掉线保护、安全门的基础闭环。
  - `tests/test_teleop.py` 单独运行可通过。
  - 但 `config/gamepad/switch_pro.yaml` 当前把 `btn_auto_mode` 配成了 `B`、`btn_manual_mode` 配成了 `A`，与题目明确要求的 `X -> AUTO`、`O -> MANUAL` 不一致；只有 `config/gamepad/ps4.yaml` 才符合题面语义。

### 测试现状
- `PYTHONPATH=. pytest -q tests/test_teleop.py` → 通过
- `PYTHONPATH=. pytest -q tests/test_navigation_step3.py` → 通过
- `PYTHONPATH=. pytest -q tests/test_perception.py` → 通过
- `PYTHONPATH=. pytest -q tests/test_journey_summary.py` → 通过
- `PYTHONPATH=. pytest -q tests/test_waypoints.py` → 失败 1 项：
  - `config/waypoints_real_gps.yaml` 当前只有 1 个 waypoint，但测试要求仓库 GPS 样例至少有 2 个 waypoint
- `PYTHONPATH=. pytest -q tests/test_mission_fsm.py` → 收集阶段失败：
  - 该测试文件内的 ROS stub 没有补 `nav_msgs.msg.Odometry`，与当前 `mission_controller.py` 的导入不匹配
- 多个测试文件合并一次性运行时还会出现 stub 互相污染问题，说明测试隔离性仍不够稳

### 结论补充
- 当前仓库已经具备课程任务的大部分模块和分层架构，不是从 0 开始。
- 但从“课程交付是否完成”的标准看，最稳妥的判断应是：
  - **已基本完成：任务 5、任务 7（PS4 配置前提下）**
  - **部分完成：任务 1、2、3、4、6**
- 其中最需要优先补强的是：
  - marker 坐标接口统一
  - object-to-marker distance 进入任务闭环和 summary
  - 实际 waypoint 数据与测试样例一致
  - mission_fsm 测试文件修复
  - 实机/仿真的端到端验证证据

### 问题
真实机器上在容器内执行 `ros2 launch auto_nav navigation.launch.py use_gps:=true` 时，`home_pose_recorder`、`path_follower`、`obstacle_guard`、`gap_planner`、`weave_planner`、`local_planner` 全部在节点构造阶段退出，日志统一报：
- `Failed to find a free participant index for domain 0`
- `rmw_cyclonedds_cpp: rmw_create_node: failed to create domain`
- `rclpy._rclpy_pybind11.RCLError: error creating node`

### 根因判断
- 这批节点都在 `super().__init__(...)` 时失败，还没进入各自的业务逻辑，因此问题不在 `gap_planner.py`、`path_follower.py` 等导航代码本身。
- `navigation.launch.py` 只是正常启动 6 个独立 ROS 2 进程；项目代码里没有额外改写 `RMW_IMPLEMENTATION`、`ROS_DOMAIN_ID` 或 CycloneDDS participant 策略。
- 该报错更符合 CycloneDDS 在 `domain 0` 上无法再分配新的 participant index，通常意味着：
- 当前 domain 上已经有过多 ROS 2 进程残留；
- 或容器/主机里存在未清理的旧 launch、僵尸节点、其他 ROS 2 服务仍占着 participant；
- 或运行环境额外注入了 CycloneDDS 配置，把可用 participant index 范围压得很小。

### 现有实现核对
- `launch/navigation.launch.py` 会启动 6 个导航层节点：
- `home_pose_recorder`
- `path_follower`
- `obstacle_guard`
- `gap_planner`
- `weave_planner`
- `local_planner`
- 这些节点全部是独立进程；每个进程启动时都要向 DDS domain 申请 participant。
- 从仓库代码看，不存在“某一个节点参数错了导致只有它自己崩”的迹象；本次是中间件初始化阶段的统一失败。

### 建议现场先检查
- 在容器里看是否已有残留 ROS 2 进程：
- `ps -ef | grep -E 'ros2|auto_nav|aria|oakd|sick|lakibeam'`
- 看当前还能否枚举 ROS 图：
- `ros2 node list`
- 检查 domain 和 middleware 环境变量：
- `printenv | grep -E 'ROS_DOMAIN_ID|RMW_IMPLEMENTATION|CYCLONEDDS'`
- 如果怀疑是旧 launch 没清干净，先确认是否已经同时开着多套 `bringup/teleop/navigation/perception/mission`。
- 如果容器里已经有很多旧节点，先停掉旧进程后再重启 `navigation.launch.py`；否则每次重试都会继续消耗 participant 名额。

### 结论
- 这次故障优先按“运行环境 / DDS participant 资源耗尽”处理，不建议先改导航代码。
- 下一步应先在目标容器里确认：
- 当前到底跑着多少 ROS 2 进程；
- `ROS_DOMAIN_ID` 是否都在 `0`；
- 是否存在旧 launch 残留或外部 CycloneDDS 配置。

### 问题
仓库测试已经要求存在 `scripts/start_all_services.sh`，但当前仓库里 `scripts/` 目录为空；现场又需要一个尽量简单的一键脚本，先编译，再按顺序启动完整服务，并确保 `gps`、`camera` 都能一起拉起。

### 结论
- 已新增 `scripts/start_all_services.sh`。
- 脚本默认先执行 `colcon build --symlink-install`，再按顺序启动服务。
- 提供 `full | test | bench` 三种模式，其中 `test` 模式默认启用 `GPS + NMEA GPS + camera`，更适合真实机器测试。

### 解决
- 新增 `scripts/start_all_services.sh`：
- `source_workspace()` 中按已有测试要求，在 source ROS 环境前临时 `set +u`，完成后恢复 `set -u`。
- 启动顺序固定为：
- `bringup.launch.py`
- `teleop.launch.py`
- `navigation.launch.py`
- `perception.launch.py`
- `mission.launch.py`
- `bench` 模式下改为启动 `navigation_bench.launch.py`，方便室内 dry-run。
- 支持通过环境变量覆盖常见参数，如 `USE_GPS`、`USE_NMEA_GPS`、`USE_CAMERA`、`SERIAL_PORT`、`GPS_PORT`、`WAYPOINTS_FILE`。
- `Ctrl+C` 时通过 `trap` 统一结束脚本拉起的所有子进程。

### 使用方法
- 默认完整启动：`./scripts/start_all_services.sh`
- 真实测试，默认打开 GPS 和相机：`./scripts/start_all_services.sh test`
- 室内 bench：`./scripts/start_all_services.sh bench`
- 指定 GPS 串口：`GPS_PORT=/dev/ttyUSB1 ./scripts/start_all_services.sh test`

### 验证
- `bash -n scripts/start_all_services.sh`
- `pytest tests/test_start_all_services_script.py`

### 问题
项目目前需要依次手动执行 `bringup.launch.py`、`teleop.launch.py`、`navigation.launch.py`、`perception.launch.py`、`mission.launch.py`，现场启动步骤过多，不适合一键拉起完整服务栈。

### 结论
- 已新增仓库级脚本 `scripts/start_all_services.sh`，默认用 `full` 模式按既有依赖顺序一次性启动完整任务链路。
- 脚本同时支持 `bench` 模式，只启动 `bringup + teleop + navigation_bench`，方便室内抬车 dry-run。
- 脚本不会改动现有 launch 架构，只是把现有 `ros2 launch auto_nav ...` 入口统一编排，并把日志分别写到 `artifacts/logs/start_all_<timestamp>/`。

### 解决
- 新增 `scripts/start_all_services.sh`：
- 启动前自动 source `/opt/ros/.../setup.bash` 与工作区 `install/setup.bash`。
- 默认按顺序后台拉起 `bringup -> teleop -> navigation -> perception -> mission`。
- 用环境变量覆盖常见启动参数，例如 `USE_GPS`、`USE_CAMERA`、`GAMEPAD`、`JOY_DEV`、`SERIAL_PORT`、`WAYPOINTS_FILE`。
- 通过 `trap` 统一清理子进程，`Ctrl+C` 时能够一并停止所有 launch。
- 任一 launch 提前退出时，脚本会主动结束其余服务，避免系统处于半启动状态。
- 新增 `tests/test_start_all_services_script.py`，校验脚本存在、包含 `full|bench` 模式，并且完整栈的 launch 顺序与项目文档一致。

### 使用方法
- 完整任务：`./scripts/start_all_services.sh`
- 室内 bench：`./scripts/start_all_services.sh bench`
- 例如开启 GPS：`USE_GPS=true ./scripts/start_all_services.sh`
- 例如关闭相机：`USE_CAMERA=false ./scripts/start_all_services.sh`

### 影响
- 现场 bringup 不再需要连续手动敲 5 条 `ros2 launch` 命令。
- 日志按 launch 分文件保存，便于排查是哪个子系统启动失败。
- 现有单独 launch 文件仍可继续独立使用，兼容原有分层调试流程。

### 问题
现场测试时不仅要起完整任务链路，还必须把 `camera`、`gps` 等真实硬件相关服务一起启动；仅靠默认 `full` 模式再额外拼环境变量，操作上仍然容易漏掉。

### 结论
- 已把一键脚本补成 `full | test | bench` 三种模式。
- 新增 `test` 模式后，`./scripts/start_all_services.sh test` 会默认启用 `camera + GPS + NMEA GPS + 完整任务链路`，更贴合真实场地测试。
- 同时单独增加 `USE_NMEA_GPS` 环境变量，避免把 GPS 串口驱动是否启动硬绑定到 `USE_GPS`。

### 解决
- 在 `scripts/start_all_services.sh` 中新增 `resolve_mode_defaults()`：
- `test` 模式强制 `USE_GPS=true`、`USE_CAMERA=true`，并在未显式覆盖时默认 `USE_NMEA_GPS=true`。
- `full` / `bench` 模式下，如果没有单独指定 `USE_NMEA_GPS`，则保持与 `USE_GPS` 一致。
- 启动摘要里增加 `GPS enabled / NMEA GPS / Camera` 状态打印，现场更容易确认本次到底起了哪些硬件。
- 在 `tests/test_start_all_services_script.py` 中补充 `test` 模式断言，确保脚本后续修改时不会把这组默认值删掉。

### 使用方法
- 真实测试推荐：`./scripts/start_all_services.sh test`
- 如果 GPS 串口设备名不同：`GPS_PORT=/dev/ttyUSB1 ./scripts/start_all_services.sh test`
- 如果只想保留融合但不启串口驱动：`USE_NMEA_GPS=false ./scripts/start_all_services.sh test`

### 问题
在某些机器上执行一键脚本时，`/opt/ros/jazzy/setup.bash` 会立刻报错：`AMENT_TRACE_SETUP_FILES: unbound variable`，导致所有服务还没开始启动就退出。

### 根因
- `scripts/start_all_services.sh` 在最开头启用了 `set -euo pipefail`。
- ROS 2 的 `setup.bash` 在部分发行版/安装组合下，会读取像 `AMENT_TRACE_SETUP_FILES` 这类可选环境变量，即使它们并未预先定义。
- 在 `set -u` 打开的情况下，shell 读取未定义变量会直接报 `unbound variable`，所以问题发生在 source ROS 环境阶段，而不是 launch 阶段。

### 解决
- 在 `source_workspace()` 里，source `/opt/ros/.../setup.bash` 和工作区 `install/setup.bash` 前临时执行 `set +u`。
- 两段环境脚本 source 完成后再恢复 `set -u`，这样后续脚本逻辑仍然保持严格模式。
- 在 `tests/test_start_all_services_script.py` 里补充顺序断言，防止后续又把 `set -u` 提前恢复到 ROS setup 之前。

### 影响
- `./scripts/start_all_services.sh test` 在 Jazzy 环境下不会再因为 `AMENT_TRACE_SETUP_FILES` 未定义而提前崩掉。
- 脚本其他部分仍然保留 `nounset` 检查，不会把整体错误检查放松到不可控。

## 2026-04-21

### 问题
`odom_tf_broadcaster` 在这个项目中什么作用？

### 结论
`odom_tf_broadcaster` 的作用是把底盘驱动发布的 `/odom` 里的位姿，转成标准 TF 里的 `odom -> base_link` 变换并发布到 `/tf`。

### 作用背景
- 项目架构要求最小 TF 链必须包含 `odom -> base_link -> laser` 和 `odom -> base_link -> camera_link`。
- 课程提供的 `ariaNode` 会发布 `/odom`，但不保证同时发布对应的 TF。
- 因此项目在 `bringup.launch.py` 中单独启动 `odom_tf_broadcaster`，补齐这段 TF 链。

### 具体行为
- 订阅 `/odom`。
- 读取 `Odometry.header.frame_id` 作为父坐标系。
- 读取 `Odometry.child_frame_id` 作为子坐标系。
- 把 `pose.pose.position` 和 `pose.pose.orientation` 复制到 `TransformStamped`。
- 通过 `tf2_ros.TransformBroadcaster` 发布到 `/tf`。
- 如果 `frame_id` 或 `child_frame_id` 为空，就跳过并报警告，避免发布非法 TF。

### 在本项目中的意义
- 让 RViz 和依赖 TF 的组件能得到标准的机器人位姿树。
- 让 `base_link -> laser`、`base_link -> camera_link` 这两个静态 TF 能挂接到统一的 `odom` 根上。
- 它本身不负责定位、建图、路径规划或控制，只负责把已有里程计位姿“桥接”为 TF。

### 问题
怎么单独看 `/camera/color/image_raw` 有没有数据？

### 结论
最直接的方法是先看这个 topic 是否存在，再看发布频率，最后看一帧消息或直接开图像窗口。

### 推荐命令
- `ros2 topic list | grep /camera/color/image_raw`
- `ros2 topic hz /camera/color/image_raw`
- `ros2 topic echo /camera/color/image_raw --once`
- `ros2 run image_view image_view image:=/camera/color/image_raw`

### 判断方法
- `topic list` 能看到 `/camera/color/image_raw`：说明话题已注册。
- `topic hz` 有持续输出频率：说明确实在持续发图像。
- `topic echo --once` 能打印出一条 `sensor_msgs/msg/Image`：说明至少收到过一帧。
- `image_view` 能显示实时画面：说明数据内容正常，不只是空 topic。

### 问题
`config/waypoints_data.yaml` 里的 `origin` 需要替换吗？真实场景里机器人的初始位置不固定。

### 结论
- 当前 `config/waypoints_data.yaml` 里的 waypoint 全部是 `type: xy`，所以这份文件现在不需要 `origin`。
- 只有当 waypoint 使用 `type: gps` 时，才必须提供 `origin`。
- 这个 `origin` 不是“机器人每次启动的位置”，而是“这块场地统一的 GPS 参考点”。
- 因此如果你的机器人每次起点不固定，不需要每次跟着机器人起点去改 `origin`。

### 依据
- `config/waypoints_data.yaml` 当前条目是 `type: xy`，不是 `type: gps`。
- `WaypointProvider` 只有在读取到 `type: gps` waypoint 时，才会要求 YAML 里存在 `origin`，并用它把 `lat/lon` 转成平面 ENU 坐标。
- 项目的 GPS 模式设计里已经明确处理了“机器人不是从 origin 点启动”的情况：首个有效 GPS fix 会对齐当前 odom 起点，而不是要求机器人起点等于 `origin`。

### 实际使用建议
- 如果你继续用 `config/waypoints_data.yaml` 这种本地平面坐标文件：不用改 `origin`，直接维护 `x/y` 即可。
- 如果你改用真实 GPS waypoint 文件，例如 `config/waypoints_real_gps.yaml`：需要把其中的 `origin` 改成你比赛场地附近一个固定参考点，并且保持它和 `config/gps.yaml` 里的 `gps_origin_lat/lon` 一致。
- 这个参考点应当对整块场地固定，不应随着机器人每次起始摆放位置变化。

### 问题
更正：上一个问题指的是 `config/waypoints_real_gps.yaml`。这份文件里的 `origin` 需要随机器人每次真实起始位置变化而替换吗？

### 结论
- 需要保留 `origin`，因为这份文件里的 waypoint 全部是 `type: gps`。
- 但不需要随着机器人每次起始位置变化去替换 `origin`。
- `origin` 应该是比赛场地附近一个固定的 GPS 参考点，用来把所有 waypoint 统一转换到同一个 ENU 平面坐标系。
- 机器人每次从哪里开机、起步点是否固定，这件事由 `outdoor_pose_fuser` 的“首个有效 GPS fix 对齐当前 odom”逻辑处理，不是靠改 `origin` 处理。

### 依据
- `config/waypoints_real_gps.yaml` 在文件开头声明了 `origin`，且后续 waypoint 全部是 `type: gps`。
- `outdoor_pose_fuser` 启动时会使用 `gps_origin_lat/lon` 建立 GPS 到 ENU 的统一参考。
- 在收到首个有效 GPS fix 后，节点会记录当时的 GPS ENU 与当前 odom 位姿，再按下面的平移关系对齐：
- `target_x = odom_init_x + (gps_e - gps_init_e)`
- `target_y = odom_init_y + (gps_n - gps_init_n)`
- 所以系统设计本身就允许机器人不是从 `origin` 点启动。

### 实际建议
- 你第一次去真实场地时，选一个固定参考点填进 `config/waypoints_real_gps.yaml` 的 `origin`。
- 同时把 `config/gps.yaml` 里的 `gps_origin_lat` / `gps_origin_lon` 改成完全相同的值。
- 之后只要场地不变、waypoint 仍在同一片区域，就不要因为机器人换了起点而改 `origin`。
- 只有在“场地整体换了一个地方”或“你决定重新定义整套 waypoint 的参考系”时，才需要重新设置 `origin`。

### 问题
这里说的“选一个固定参考点”具体指什么？

### 结论
- 固定参考点指的是：你在真实场地里选一个不会随着机器人每次摆放位置变化的地理位置，用它的 GPS 经纬度作为整套 waypoint 的共同原点。
- 这个点可以是场地里的某个角点、路边某个井盖中心、地面某个长期可复现的标记点，或者你自己定义并记录下来的一个基准位置。
- 它不要求是机器人起点，也不要求机器人每次都经过这个点。

### 这个点要满足什么条件
- 位置固定：每次去同一块场地都能指向同一个物理位置。
- 容易复现：下次测试时你还能大致找到它。
- 离任务区域不要太远：最好就在 waypoint 活动区域附近，避免 ENU 换算基点离任务区太远。
- 和所有 waypoint 使用同一参考：同一批 waypoint、实时 GPS 融合都必须共用这个参考点。

### 推荐怎么选
- 最稳妥的是选“场地里一个长期不变、容易描述的位置”，例如场地西南角、停车线交点、围栏立柱旁某个地砖角点。
- 如果没有明显地标，也可以直接选“第一个 waypoint 附近的一个固定点”作为 `origin`，但不要把 `origin` 直接写成“机器人当天的出发点”，除非你的出发点本身就是固定场地标记。
- 工程上不必追求这个点在几厘米级完全精确，关键是整套配置保持一致。

### 不要这样理解
- 不是“机器人开机时所在位置”。
- 不是“每次 mission 的起点”。
- 不是“必须放在 waypoint 1 上”。
- 它本质上只是 GPS 转本地平面坐标时使用的统一基准点。

## 2026-04-22

### 问题
运行 `ros2 launch auto_nav perception.launch.py use_sim_time:=false` 后，`cone_detector` 正常打印 `ready`，但把橙色 cone 放到摄像头前也没有任何识别结果。

### 根因
- `launch/perception.launch.py` 只启动感知处理节点：`cone_detector`、`object_detector`、`distance_estimator`、`photo_capture`。
- 它不会启动 OAK-D 相机驱动；相机节点是在 `launch/bringup.launch.py` 里通过 `oakd_camera` 启动的，且默认由 `use_camera:=true` 控制。
- 因此如果现场只启动了 `perception.launch.py`，`cone_detector` 虽然会显示 `ready`，但实际上可能根本没有收到 `/camera/color/image_raw`，自然不会发布 `/marker/detection` 或 `/marker/bbox`。
- 即使相机已经启动，当前默认阈值仍要求目标大致满足：
- HSV 色相在 `H=5~25`、饱和度 `S>=120`、亮度 `V>=80`
- 轮廓面积 `400~80000 px`
- 包围框纵横比 `h/w` 在 `0.4~4.0`
- 所以过暗、过小、颜色偏红/偏黄的 cone 也会被过滤。

### 解决
- 先确认相机数据源真的存在：
- `ros2 topic list | grep /camera/color/image_raw`
- `ros2 topic hz /camera/color/image_raw`
- `ros2 topic echo /marker/bbox`
- 正确启动顺序应至少包含：`ros2 launch auto_nav bringup.launch.py use_camera:=true`，再启动 `ros2 launch auto_nav perception.launch.py use_sim_time:=false`
- 为了避免后续误判，在 `auto_nav/perception/cone_detector.py` 中新增了无图像输入诊断：节点启动后如果一直收不到 `/camera/color/image_raw`，会定期明确报警。
- 同时在 `launch/perception.launch.py` 增加启动提示，直接说明这里不会启动相机驱动。

### 影响
- 以后如果只开了感知节点、没开相机驱动，终端会明确提示“没有收到 `/camera/color/image_raw`”，不会再只有 `ready` 这种误导性信息。
- 如果相机话题存在但仍无检测，下一步就应针对 HSV 阈值和实际光照调 `config/camera.yaml` 里的 `cone_detector` 参数，而不是继续怀疑 launch。

### 验证
- 启动后 5~10 秒内如果没有相机数据，应看到 `No frames received on /camera/color/image_raw yet...` 警告。
- 相机正常时，`ros2 topic hz /camera/color/image_raw` 应持续输出频率。
- 把橙色 cone 放入画面后，`ros2 topic echo /marker/bbox` 应出现 JSON 检测结果；如果没有，再继续调 HSV 和面积阈值。

### 问题
PS4 手柄上只按 `R2` 时 deadman 可以触发，但只按 `L2` 不生效；需求是 `L2` 或 `R2` 任意一个按下都应该允许行驶，松开两者才停车。

### 根因
- 现场抓取 `/joy` 后确认 `axes[4]` 和 `axes[5]` 都会随 trigger 变化。
- 旧版 `GamepadWatchdogNode` 仅支持单个 `axis_deadman`，因此即使 `L2` 和 `R2` 都是合法 deadman 输入，也只能识别其中一个轴。

### 解决
- 在 `auto_nav/teleop/gamepad_watchdog_node.py` 中新增 `deadman_axis_secondary` 参数，允许监听第二个 deadman 轴。
- 轴式 deadman 现在采用“主轴 OR 次轴”的组合逻辑：只要任意一个 trigger 满足阈值方向，就发布 `/deadman_ok=True`。
- 在 `config/gamepad/ps4.yaml` 中把 `deadman_axis_secondary` 设为 `4`，这样 `R2(axis 5)` 或 `L2(axis 4)` 任一按下都能作为 deadman。
- 在 `tests/test_watchdog.py` 和 `tests/test_teleop.py` 中补充次轴触发测试。

### 影响
- PS4 现在支持 `L2` / `R2` 双 deadman。
- 两个 trigger 都松开时仍然会立即停车，安全约束不变。
- Switch Pro 的按钮式 deadman 仍按原逻辑工作，不受影响。

### 验证
- 已运行：`python3 -m pytest tests/test_watchdog.py tests/test_teleop.py`
- 结果：见本次修改后的测试输出。

### 问题
PS4 手柄实机上出现了 deadman 逻辑反向：放开 R2 时机器人会进入 AUTO 并尝试行驶，按下 R2 反而立刻 `AUTO -> PAUSED`，与“必须按住 deadman 才能走”的要求相反。

### 根因
- 实机抓取 `/joy` 后确认，当前 PS4 驱动的 trigger 轴行为是“松开=+1，按下=-1”。
- 旧版 `GamepadWatchdogNode` 固定使用 `axis_value > deadman_threshold` 判断 deadman 是否按下，默认假设“按下时轴值更大”。
- `config/gamepad/ps4.yaml` 里的注释和阈值假设也是按“fully pressed ≈ +1”写的，因此在这台实机上判定方向正好反了。

### 解决
- 在 `auto_nav/teleop/gamepad_watchdog_node.py` 中新增 `deadman_axis_pressed_high` 参数：
- `true` 表示“按下时轴值应高于阈值”；
- `false` 表示“按下时轴值应低于阈值”。
- 在 `config/gamepad/ps4.yaml` 中把 PS4 deadman 配置改为：
- `axis_deadman: 5`
- `deadman_threshold: 0.0`
- `deadman_axis_pressed_high: false`
- 同步修正 PS4 配置文件顶部的轴位说明，避免把 L2/R2 写反。
- 在 `tests/test_watchdog.py` 和 `tests/test_teleop.py` 中补充“pressed-low”场景测试，覆盖这类驱动映射。

### 影响
- PS4 现在会恢复为“按住 R2 才走，松开 R2 立即停”。
- Switch Pro 这类按钮式 deadman 不受影响。
- 以后如果换一套驱动、trigger 方向又变了，只需要改参数，不需要改代码。

### 验证
- 已运行：`python3 -m pytest tests/test_watchdog.py tests/test_teleop.py`
- 结果：见本次修改后的测试输出。

### 问题
实机继续出现 `ObstacleGuard: obstacle at 0.07m~0.22m angle≈-5°` 的急停日志，但现场确认机器人前方没有真实障碍物，`tf2_echo base_link laser` 也显示 `base_link -> laser = (0.20, 0.00, 0.281)`、姿态为零。

### 根因
- 用前扇区在线监视脚本连续观察 `/scan` 后发现，绝大多数帧的 `front_min` 都在约 `2.8m @ +15°`，但会偶发单帧掉到 `0.054m @ -5.0°`，并与 `ObstacleGuard` 急停日志同步。
- 这说明当前不是“前方一直有个固定障碍”，而是 LiDAR 在前扇区偶发产生了单点超近距离毛刺；可能来自车体边缘、地面反射或驱动近距噪声。
- 旧版 `ObstacleGuardNode` 对前扇区采用“单根 ray 小于阈值就急停”的策略，过于敏感，单帧孤立尖峰就足以把 `/emergency_stop` 拉高。
- 仓库里虽然已经有 `scan_min_range_m: 0.25` 参数，但实机日志仍出现 `0.18m`、`0.07m` 触发，说明运行环境很可能没用上最新代码/参数；即便该参数生效，单点毛刺只要落在 `0.25m~0.5m` 之间依然会误触发。

### 解决
- 在 `auto_nav/navigation/obstacle_guard.py` 中把判障逻辑改为“连续命中”：
- 新增 `min_hit_rays` 参数，要求前扇区内至少有指定数量的相邻 hit ray 连续落在 `safety_distance_m` 内，才触发急停。
- 保留并继续使用 `scan_min_range_m`，先过滤掉机器人本体盲区内的超近距离回波，再做连续命中判断。
- 在 `config/lidar.yaml` 中将 `min_hit_rays` 默认设为 `3`，让单根或双根毛刺不再触发 `/emergency_stop`。
- 在 `tests/test_navigation_step3.py` 中增加三类测试：单根尖峰不触发、连续三根近点触发、`scan_min_range_m` 过滤超近距离尖峰。

### 影响
- 真实障碍通常会占据多个相邻扫描点，仍会触发急停。
- 单帧单点毛刺不再直接把机器人锁死，AUTO 模式鲁棒性会明显提升。
- 如果实机仍然误停，下一步应继续检查 LiDAR 安装角度、支架遮挡和驱动原始数据，但安全层不会再被 1-ray 尖峰轻易击穿。

### 验证
- 已运行：`python3 -m pytest tests/test_navigation_step3.py`
- 结果：见本次修改后的测试输出。

### 问题
把机器人拿到室外并切换 Wi‑Fi 后，`oakd_camera` 持续打印：
`OAK-D error: Failed to connect to device, error message: X_LINK_DEVICE_ALREADY_IN_USE`

### 根因
- 这个错误不是“Wi‑Fi 配置直接影响 OAK-D 通信”，而是 DepthAI 在打开设备时发现同一台 OAK-D 仍被别的进程占用。
- 本项目的相机节点 `auto_nav/perception/oakd_camera.py` 会在循环里直接调用 `dai.Device(...)`；如果旧的 `oakd_camera`、另一个 ROS launch、`image_view`、调试脚本，或宿主机上其他使用 DepthAI 的程序没有完全退出，新的节点就会反复拿到 `X_LINK_DEVICE_ALREADY_IN_USE`。
- 现场“换 Wi‑Fi 后才出现”通常只是时序上的相关：例如网络切换时重启了一部分 ROS 节点，但旧的相机进程还留在原终端、tmux、Docker 容器或宿主机上，没有把 USB/XLink 句柄释放干净。

### 解决
- 先确保只保留一个相机驱动实例，不要同时启动多个 `bringup.launch.py`、多个 `oakd_camera`，也不要再开其他直接访问 OAK-D 的脚本。
- 停掉所有相关 ROS 进程后，等待 2~3 秒让 DepthAI 释放设备；如果仍然报同样错误，重新插拔 OAK-D USB 线后再启动。
- 建议按下面顺序恢复：
- `Ctrl+C` 停掉当前相机相关 launch；
- 检查是否还有残留的 `ros2 launch auto_nav bringup.launch.py`、`oakd_camera`、DepthAI 调试脚本或旧容器实例；
- 确认没有残留实例后，再重新执行 `ros2 launch auto_nav bringup.launch.py use_camera:=true`；
- 启动成功后，用 `ros2 topic hz /camera/color/image_raw` 确认图像流已经恢复。

### 现场判断方法
- 如果是“设备占用”，日志会在 `Connecting to OAK-D...` 后立刻报 `X_LINK_DEVICE_ALREADY_IN_USE`，且持续重试。
- 如果是“完全没识别到设备/USB 断开”，通常会出现找不到设备、枚举失败，或根本没有 `OAK-D connected: <mxid>`。
- 如果只是感知节点没结果，但相机是好的，那么 `/camera/color/image_raw` 仍然会持续有频率，不会出现这个 XLink 占用错误。

### 验证
- 重新启动后应看到一次 `Connecting to OAK-D...`，随后打印 `OAK-D connected: <mxid>`。
- `ros2 topic hz /camera/color/image_raw` 和 `ros2 topic hz /camera/depth/image_raw` 应持续输出频率。
- 若仍然失败，优先排查“第二个相机进程/第二个容器/宿主机脚本未退出”，而不是继续改 Wi‑Fi 参数。

### 问题
在目标机上直接执行：
`python3 -c "import depthai as dai; print(dai.Device.getAllAvailableDevices())"`
返回 `[]`，表现为“完全检测不到相机”。

### 根因
- 这和 `X_LINK_DEVICE_ALREADY_IN_USE` 不同；`[]` 表示 DepthAI 在设备枚举阶段就没有发现任何 OAK-D，问题发生在“打开设备之前”。
- 因为仓库内的 `auto_nav/perception/oakd_camera.py` 最终也是通过 `dai.Device(...)` 打开相机，所以当 `getAllAvailableDevices()` 已经是空列表时，继续重启 ROS 节点通常没有意义，应该先排查底层 USB/设备可见性。
- 现场最常见原因是：USB 线松动或只接成供电未接数据、插在不稳定的 HUB/前置口、容器没有透传 `/dev/bus/usb`、相机异常掉电后未重新枚举，或系统层根本没有识别到 Luxonis 设备。

### 解决
- 先在目标机上确认系统是否能看到 USB 设备，而不是先改 ROS launch：
- `lsusb | grep -i -E 'luxonis|movidius|intel'`
- `dmesg | tail -n 50`
- 如果 `lsusb` 里也没有相机，优先重新插拔 OAK-D，换一根支持数据的 USB 3.0 线，并直连 NUC 主机 USB 口，不要先经过 HUB。
- 如果宿主机 `lsusb` 能看到，但容器/当前环境里 `depthai` 仍返回 `[]`，就检查是否做了 USB 透传；Docker 场景至少要确认 `/dev/bus/usb` 已映射进去，且容器权限足够访问该设备。
- 如果重新插拔后偶尔恢复、过一会又消失，要继续排查供电不足、线材接触不良或端口不稳定，而不是只重启 `oakd_camera`。

### 现场判断方法
- `getAllAvailableDevices() == []`：系统/当前运行环境根本没枚举到 OAK-D，先查 USB、线材、容器透传。
- `getAllAvailableDevices()` 非空，但启动时报 `X_LINK_DEVICE_ALREADY_IN_USE`：设备存在，但被别的进程占用。
- `getAllAvailableDevices()` 非空，`oakd_camera` 也能连上，但 `/camera/color/image_raw` 没数据：再回到 ROS topic、参数和图像处理链路排查。

### 验证
- 目标机上 `python3 -c "import depthai as dai; print(dai.Device.getAllAvailableDevices())"` 应至少返回一个设备对象，而不是 `[]`。
- 随后再启动 `ros2 launch auto_nav bringup.launch.py use_camera:=true`，应能看到 `OAK-D connected: <mxid>`。

### 问题
实机日志里 `ObstacleGuard` 持续报前方 `0.07m ~ 0.15m` 的障碍并反复把 `/emergency_stop` 拉高，表现为按住 deadman 后 `PathFollower` 会提示 `resuming mission`，但机器人仍然不往前走。

### 根因
- 从日志顺序看，`/deadman_ok` 与 `AUTO -> PAUSED -> AUTO` 切换是正常的；真正阻止底盘输出的是 `ObstacleGuardNode` 发布的 `/emergency_stop=True`。
- 这类日志说明 LiDAR 在机器人正前方约 `-7°` 持续看到一个非常近的回波；在当前 `safety_distance_m=0.5` 的配置下，安全层必须拦截前进命令，所以“按住 deadman 也不能往前走”本身符合安全逻辑。
- 另外 `auto_nav/teleop/cmd_gate_node.py` 的急停分支存在实现错误：前面先 `return _ZERO`，导致后面的 `_reverse_escape_command()` 永远不可达。结果是即使切回 `MANUAL`，操作员也无法按住 deadman 后倒车把机器人从误检区或贴近障碍的状态里脱出来。

### 解决
- 修正 `auto_nav/teleop/cmd_gate_node.py` 的急停分支，让 `/emergency_stop=True` 时进入 `_reverse_escape_command()`：
- `MANUAL` 模式下只允许 `linear.x < 0` 的纯倒车命令通过；
- 前进和转向仍然被屏蔽；
- `AUTO` 模式下仍然保持完全停车，不允许自动硬顶着障碍前进。
- 在 `docs/qa.md` 记录这次排查结论，便于后续 report 说明“deadman 正常，但被 LiDAR safety gate 抢占”。

### 影响
- 当 LiDAR 继续检测到前方近障碍时，AUTO 依旧不会前进，这符合当前安全设计。
- 操作员现在可以切回 `MANUAL`，按住 deadman 后用纯倒车把机器人从障碍前方退出，再重新进入 AUTO。
- 如果实机上这些 `0.07m ~ 0.15m` 回波不是外部障碍而是车体/支架反射，后续还需要单独校正 LiDAR 安装、TF 或增加更精细的自车屏蔽参数；这次修改没有放宽前向安全阈值。

### 验证
- 已运行：`python3 -m pytest tests/test_teleop.py`
- 结果：见本次修改后的测试输出，重点覆盖 `test_estop_allows_manual_reverse_escape`

### 实机验证建议
- 在 AUTO 模式按住 deadman，确认只要 `/emergency_stop=True` 就不会前进。
- 切到 MANUAL，按住 deadman 并给一个小的倒车指令，确认机器人可以后退脱困。
- 保持 `MANUAL + deadman`，给前进指令或原地转向，确认急停激活期间仍然不会放行。
- 如果倒车脱困后日志仍持续报 `0.07m ~ 0.15m angle≈-7°`，优先排查 LiDAR 是否扫到保险杠、线缆、安装支架或地面近距离反射。

### 问题
行驶时必须按住 deadman 才能运动，这个约束需要同时作用于手动和自动；同时要取消手柄三角按键作为急停，无论手动还是自动都不应由这个按键触发 `/emergency_stop`。当前实现没有满足这两个要求。

### 根因
- `auto_nav/teleop/cmd_gate_node.py` 只在 `AUTO` 模式检查 `deadman_ok`，`MANUAL` 模式会直接放行 `/cmd_vel_manual`，所以手动驾驶不需要持续按住 deadman。
- `auto_nav/teleop/mode_manager_node.py` 把 `/joy/btn_emergency` 做成了锁存式急停，按一次就进入 `ABORTED` 并发布 `/emergency_stop=True`。
- `config/gamepad/ps4.yaml` 和 `config/gamepad/switch_pro.yaml` 仍然把三角键/X 键映射到了 `btn_emergency_stop`。
- `auto_nav/teleop/joy_mapper_node.py` 对负数按钮索引没有做保护；如果把 `btn_emergency_stop` 设成 `-1` 代表禁用，旧代码会错误读取 `buttons[-1]`，导致“已禁用”的急停映射仍有误触发风险。

### 解决
- 在 `auto_nav/teleop/cmd_gate_node.py` 中把 deadman 检查提升为手动、自动共用前置条件：只要 `deadman_ok == False`，统一输出零速度到 `/cmd_vel_safe`。
- 在 `auto_nav/teleop/mode_manager_node.py` 中移除手柄急停按钮的订阅和锁存逻辑，只保留 `MANUAL / AUTO / PAUSED` 三种手柄控制模式。
- 在 `config/gamepad/ps4.yaml` 与 `config/gamepad/switch_pro.yaml` 中把 `btn_emergency_stop` 改为 `-1`，明确禁用手柄急停按键映射。
- 在 `auto_nav/teleop/joy_mapper_node.py` 中补上 `0 <= idx < len(msg.buttons)` 的边界判断，使 `-1` 真正表示“禁用按钮”。

### 影响
- 现在无论是手动还是自动，只要松开 deadman，`/cmd_vel_safe` 都会立刻归零。
- 三角键/对应映射键不再通过手柄链路触发急停。
- LiDAR 安全急停 `ObstacleGuardNode -> /emergency_stop` 仍然保留，不受这次修改影响。

### 验证
- 已运行：`python3 -m pytest tests/test_teleop.py tests/test_teleop_launch.py`
- 结果：`35 passed`

### 实机验证建议
- 手动模式下推动摇杆但不按 deadman，确认机器人不动。
- 手动模式下按住 deadman 再推动摇杆，确认机器人可以运动；松开 deadman 后立即停车。
- 自动模式下触发导航，确认只有按住 deadman 才会继续行驶，松开即停。
- 按三角键（或 Switch Pro 对应原急停键）时，确认不会进入 `ABORTED`，也不会单独触发 `/emergency_stop`。

### 问题
把树下某个固定点定义成 `origin` 的原理是什么？

### 结论
- 原理是先选一个固定 GPS 参考点作为局部坐标系的零点，再把所有经纬度都换算成“相对这个零点向东多少米、向北多少米”。
- 这样导航层就不必直接处理 `lat/lon`，而是处理更适合运动控制的平面 `x/y` 坐标。
- 只要所有 waypoint 和实时 GPS 都使用同一个 `origin`，它选在树下、路边、角点，本质上都一样；差别只在于是否容易复现。

### 具体原理
- GPS 给的是全球坐标：`lat/lon`。
- 机器人导航更适合用局部平面坐标：例如 `x = 向东多少米`，`y = 向北多少米`。
- 所以系统先选一个 `origin(lat0, lon0)` 作为参考。
- 对任意一个 GPS 点 `(lat, lon)`，先计算它和 `origin` 的纬度差、经度差。
- 再把这个差值按地球半径近似换算成米：
- `north_m ≈ (lat - lat0) * R`
- `east_m ≈ (lon - lon0) * R * cos(lat0)`
- 这样就得到该点相对 `origin` 的局部 ENU 坐标。

### 为什么 `origin` 不必等于机器人起点
- `origin` 只负责定义“整张局部地图的零点”。
- 机器人实际从哪里启动，是另一个问题。
- 项目里的 `outdoor_pose_fuser` 会在拿到首个有效 GPS fix 时，把当时 GPS 位置对齐到当前 odom 起点，所以机器人可以在场地任意位置启动。
- 因此 `origin` 是地图参考，启动点是定位初始化，两者职责不同。

### 为什么选树下也可以
- 因为系统不关心这个参考点有没有特殊物理意义，只关心它是不是固定且全程一致。
- 树下、角点、路沿交点都可以，只要你以后还能用同一个点重新定义同一套坐标系。

### 问题
`ros2 run tf2_ros tf2_echo odom base_link` 刚启动时先打印一次
`Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist`，
但紧接着就开始持续输出 `odom -> base_link` 变换，这正常吗？

### 结论
- 这是正常现象，前提是它后面很快就开始稳定输出变换。
- 你给出的日志里，报错只出现在第一行，随后每秒都能看到连续的平移和姿态数据，说明 `odom -> base_link` TF 已经正常存在。
- 这种首条 `invalid frame` 往往只是 `tf2_echo` 启动瞬间比 TF 发布器更早检查了一次缓存，当时 `/tf` 里还没收到第一帧对应坐标系。

### 为什么会这样
- `tf2_echo` 启动后会立刻调用一次 `canTransform()`。
- 如果这一刻 TF buffer 里还没有 `odom -> base_link`，就会先打印一次“frame does not exist”。
- 只要 `odom_tf_broadcaster` 或 `outdoor_pose_fuser` 紧接着开始往 `/tf` 发这段变换，后续输出就会恢复正常。
- 这和“TF 长期缺失”不是一回事；真正异常的情况是它持续报错、一直等不到后续变换。

### 结合这次现场输出的判断
- 第一条报错后，后面连续得到：
- 平移约 `(-0.420, 0.014, 0.000)` 到 `(-0.422, 0.001, 0.000)`
- 姿态保持接近单位四元数，说明当前朝向基本稳定
- 这表示机器人位姿 TF 已经在持续发布，导航链路所需的最小 TF 并没有缺失。

### 什么时候才算不正常
- `tf2_echo` 一直只打印 `Invalid frame ID` 或 `Lookup would require extrapolation`，长时间没有任何有效变换。
- `/nav/odom` 或 `/odom` 明明有数据，但 `/tf` 里始终没有 `odom -> base_link`。
- RViz 中机器人坐标系持续丢失，或依赖 TF 的节点反复报 transform unavailable。

### 现场建议
- 先把这个首条报错视为“启动瞬时现象”，不要单独因为这一条就判定系统异常。
- 真正该看的，是 2~3 秒后 `tf2_echo` 是否持续刷新。
- 你现在这份输出可以判定为正常，可以继续下一步联调，例如观察 `/nav/status`、短距离推车检查 `/nav/odom`、再做小范围 AUTO 测试。

### 问题
室外 AUTO 启动后，机器人开始频繁左右转向，但几乎不向前走，像是在原地来回找方向。这通常是哪一层出了问题？

### 结论
- 这个现象通常不是单一“底盘坏了”，而是导航目标方向在左右来回跳，或者机器人估计朝向与真实朝向不一致，导致 `PathFollower` 一直认为“还没对准目标”。
- 现有实现里最常见的三类原因是：
- `PathFollower` 收到的目标点在机器人侧后方，heading error 接近 `±90°` 甚至反复换符号；
- `LocalPlanner` 转发的 `/local_target` 在左侧和右侧之间跳变，导致 `/cmd_vel_auto.angular.z` 正负来回切；
- `/nav/odom` 的 yaw 不稳定、冻结或方向错误，导致同一个目标点被解释成“忽左忽右”。

### 代码依据
- `auto_nav/navigation/path_follower.py` 的 `_drive_toward()` 使用：
- `angular_vel = k_angular * heading_err`
- `linear_vel = k_linear * dist * cos(heading_err)^2`
- 这意味着一旦 heading error 接近直角，线速度会被压得很低，看起来就会“只转不走”。
- 如果 heading error 符号在正负之间切换，机器人就会表现为左右来回摆头。

### 现有实现下最可能的触发链
- 目标坐标有问题：
- `config/waypoints_real_gps.yaml` 的 waypoint 相对 `origin` 算出来方向不符合现场摆放，或机器人初始朝向与 waypoint 方向差太大。
- 局部目标抖动：
- `gap_planner` / `weave_planner` 根据 `/scan` 选缝，如果近场存在车体反射、侧边空旷过大、或左右评分接近，就可能让 `/gap/local_target` 左右切换。
- 姿态源有问题：
- GPS 模式下导航节点订阅的是 `/nav/odom`；如果 `outdoor_pose_fuser` 的 yaw 来源不稳定，`PathFollower` 计算出来的 heading error 就会跟着抖。

### 最快判别方法
- 先看自动命令本身是不是在左右翻：
- `ros2 topic echo /cmd_vel_auto`
- 如果 `linear.x` 很小，而 `angular.z` 在正负之间频繁切换，说明是导航层目标方向在跳。
- 再看局部目标是不是在跳：
- `ros2 topic echo /local_target`
- `ros2 topic echo /gap/local_target`
- 如果 x/y 一会儿偏左一会儿偏右，问题在 LiDAR 规划侧，不在底盘。
- 再看当前大目标是否本来就在侧后方：
- `ros2 topic echo /waypoint/current --once`
- 结合 `ros2 topic echo /nav/odom --once` 对比，如果 waypoint 相对机器人明显在侧面甚至身后，原地转向是预期行为，只是配置/摆放还没对齐。
- 最后看姿态是否稳定：
- 连续执行 `ros2 topic echo /nav/odom`
- 如果位置变化很小但 orientation 的 yaw 在抖，或真实车头已转动但 yaw 几乎不变，就要先处理姿态源。

### 现场建议顺序
- 第一步先停用复杂因素，只保留单个 waypoint，距离放近到 5~8 m。
- 第二步确认 waypoint 在机器人正前方大致方向，而不是侧后方。
- 第三步观察 `/cmd_vel_auto`：
- 若角速度单边持续为正或负，只是还没转到目标，先别急着判故障。
- 若角速度正负高速来回切，继续看 `/local_target` 是否左右跳。
- 第四步如果 `/local_target` 在跳，优先怀疑 LiDAR gap/weave 规划，而不是 GPS waypoint 本身。
- 第五步如果 `/local_target` 不跳，但 `/nav/odom` yaw 异常，就先修 GPS/odom/IMU 融合。

### 本项目当前更值得优先怀疑的点
- 现有 `PathFollower` 没有 heading deadband，且线速度按 `cos^2(heading_err)` 衰减；因此只要目标方位在左右两侧切换，就会表现得非常明显。
- GPS 模式下又额外引入了 `/nav/odom` 和 `/local_target` 两层中间量，所以现场首先应抓这两个 topic，而不是只盯着底盘不走。

### 问题
室外 GPS 模式下，设置 waypoint 并进入 AUTO 后，机器人频繁左右转向但不稳定向前。抓现场 topic 后看到：
- `/cmd_vel_auto.angular.z` 在 `+1.0` 和 `-1.0` 之间切换；
- `/local_target` 在机器人前后左右大幅跳变；
- `/waypoint/current` 本身在远处前方，大方向并没有问题。

### 根因
- 这次主问题不在 GPS waypoint，而在局部目标接管链路。
- `PathFollowerNode` 在 `NAVIGATING` 状态下，只要 `/local_target` 新鲜就优先跟随它。
- 现场抓到的 `/local_target` 有不少点落在机器人后半平面；但旧版 `_drive_toward()` 用的是：
- `linear_vel ∝ cos(heading_err)^2`
- 这个公式在目标位于车后方时仍会给出正的前进速度，因为 `cos(pi)^2 = 1`。
- 结果就是：当 LiDAR 规划器在车后左侧和车后右侧之间来回切目标时，`PathFollower` 一边给正前进速度，一边把角速度打到 `±max_angular_vel`，现场看起来就像“左右摆头、不稳定向前”。

### 解决
- 在 `auto_nav/navigation/path_follower.py` 中新增 `local_target_max_heading_deg` 参数，默认 `85°`。
- `NAVIGATING` 状态下，只有当 `/local_target` 仍位于机器人前向扇区内时，才允许它覆盖全局 waypoint；否则直接回退到原始 waypoint 跟随。
- 同时修正 `_drive_toward()` 的前进速度整形：
- 旧逻辑：`forward_factor = cos(heading_err)^2`
- 新逻辑：`forward_factor = max(0, cos(heading_err))^2`
- 这样目标在车后方时，机器人只允许原地转向对齐，不再错误地下发正向线速度。
- 在 `config/waypoints.yaml` 中补充 `local_target_max_heading_deg` 参数配置。
- 在 `tests/test_waypoints.py` 中补充两个回归测试：
- 后方 `/local_target` 应回退到全局 waypoint；
- 目标在车后方时 `_drive_toward()` 不应给正向线速度。

### 影响
- 当 LiDAR 规划器偶发给出车后方的局部目标时，机器人不再被带着左右乱摆。
- GPS 大目标仍可继续驱动机器人朝远处 waypoint 前进，不会被一个明显错误的 `/local_target` 抢走控制权。
- 如果后续还观察到 `/local_target` 本身持续左右跳，说明 LiDAR 规划层还可以继续优化，但至少路径跟随器不再放大这个问题。

### 验证
- 已运行：`python3 -m pytest tests/test_waypoints.py -k 'rear_local_target_falls_back_to_waypoint or drive_toward_rear_target_turns_in_place or navigating_far_from_wp_publishes_cmd or homing_completes_at_home or coarse_arrived_with_marker_starts_final_approach'`
- 结果：`5 passed`
- 已运行：`python3 -m pytest tests/test_navigation_step3.py`
- 结果：`35 passed`

### 备注
- 当前完整 `tests/test_waypoints.py` 全量运行仍会被现场配置影响：你把 `config/waypoints_real_gps.yaml` 临时改成了单 waypoint，仓库里一个“样例文件至少 2 个 waypoint”的旧测试会失败。
- 这不是本次修复引入的问题，而是现场调试配置与仓库旧测试假设不一致。

### 问题
进一步现场联调时发现，即使 `/waypoint/current` 已经正确指向远处目标点，机器人仍然会整体向 waypoint 左侧偏着走，而不是逐步收敛到该坐标。现场抓到：
- `/waypoint/current ≈ (1.23, 14.46)`；
- 而局部目标 `/local_target` / `/gap/local_target` 会围绕机器人当前位置持续给出“朝当前车头前方”的点；
- 结果机器人在无遮挡空场里沿着当前朝向前进，逐渐把与 waypoint 的横向误差越拉越大。

### 根因
- 这次问题在 `GapPlannerNode` / `WeavePlannerNode` 的目标角选择，而不在 waypoint 坐标本身。
- 旧实现先选一个 gap，然后直接用 `gap.center_angle` 发布局部目标。
- 在“几乎全空旷”的场景里，LiDAR 会看到一个覆盖整个前方的大 gap；它的中心角接近 `0 rad`，也就是“车头正前方”。
- 这样即使真实 waypoint 明明在左前方或右前方，只要场地没有障碍，局部规划仍会不断把 `/local_target` 设成“沿当前朝向直走”。
- `PathFollower` 又会优先跟随新鲜的 `/local_target`，所以机器人就会偏着走，而不是向 waypoint 收敛。

### 解决
- 在 `auto_nav/navigation/gap_planner.py` 中新增 `_guided_gap_angle()`：
- 不再盲目使用 `gap.center_angle`；
- 而是先取当前 `angle_to_waypoint()`；
- 如果 waypoint 方向位于该 gap 内，就直接朝 waypoint 方向发局部目标；
- 如果 waypoint 方向落在 gap 外，就把角度 clamp 到最近的 gap 边界。
- 在 `auto_nav/navigation/weave_planner.py` 中同步采用相同策略，避免编织段仍然固定朝 gap 中心。
- 这样局部规划仍然遵守“不能穿过障碍”的约束，但在无遮挡或大开口场景里，会尽量向全局 waypoint 收敛，而不是只图当前朝向前进。
- 在 `tests/test_navigation_step3.py` 中补充回归测试：
- gap 覆盖 waypoint 方向时，应返回 waypoint 方向；
- waypoint 超出 gap 时，应钳制到最近边界；
- weave_planner 也要有相同的 waypoint-bias 行为。

### 影响
- 在开阔场地里，机器人不再因为“全场只有一个大 gap”而被局部规划强行带着直行跑偏。
- GPS / 全局 waypoint 的引导作用会重新体现出来。
- 如果后续仍有轻微横向漂移，下一步应检查 `/nav/odom` 朝向和轮式里程计比例，而不是继续怀疑 waypoint 转换。

### 验证
- 已运行：`python3 -m pytest tests/test_navigation_step3.py`
- 结果：`38 passed`
- 已运行：`python3 -m pytest tests/test_waypoints.py -k 'rear_local_target_falls_back_to_waypoint or drive_toward_rear_target_turns_in_place'`
- 结果：`2 passed`

### 问题
修正“沿当前车头直走而不是朝 waypoint 收敛”之后，室外又出现新的极限环：机器人会左右摇摆、缓慢前进，`/cmd_vel_auto.angular.z` 在正负之间切换，像是在左右抢方向。

### 根因
- 这次问题说明局部规划仍然在不该接管时接管。
- 旧版 `gap_planner` / `weave_planner` 的流程是：
- 每帧都先做 gap 选择；
- 只要能找到 gap，就发布 `/local_target`；
- `PathFollower` 又总是优先跟随新鲜的 `/local_target`。
- 即便 waypoint 方向本身根本没有被障碍挡住，局部规划也会继续参与竞争。
- 在真实 LiDAR 数据里，只要 waypoint bearing 附近有一点点边缘噪声、局部近距反射或左右 gap 评分接近，局部规划就可能在左/右两个候选方向之间切换，最终表现为车身左右摇摆。

### 解决
- 在 `auto_nav/navigation/gap_planner.py` 中新增 waypoint-bearing 直通判定：
- 先检查 waypoint 方向附近一个小角窗是否“足够清楚”；
- 如果该窗口内的 free-ray 比例达到阈值，就直接沿 waypoint bearing 发布 `/local_target`；
- 只有当 waypoint bearing 真的被挡住时，才进入原有 gap 竞争。
- 在 `auto_nav/navigation/weave_planner.py` 中同步加入相同逻辑，避免编织段也出现同类摆动。
- 新增参数并写入 `config/lidar.yaml`：
- `goal_clear_window_deg: 12.0`
- `goal_clear_min_fraction: 0.8`
- 同时补充 `tests/test_navigation_step3.py`：
- waypoint bearing 清楚时，应返回 direct-goal range；
- waypoint bearing 被挡住时，应拒绝直通、继续走 gap 逻辑；
- weave_planner 同样覆盖 clear-bearing 场景。

### 影响
- 如果真正没有障碍挡在 waypoint 前方，局部规划不再左右抢舵，机器人会稳定朝 waypoint 收敛。
- 只有在 direct bearing 确实被障碍占住时，gap/weave 才会接管。
- 这相当于把局部规划从“默认总在线竞争”改成了“必要时才接管”，更符合室外大空地导航。

### 验证
- 已运行：`python3 -m pytest tests/test_navigation_step3.py`
- 结果：`41 passed`
- 已运行：`python3 -m pytest tests/test_waypoints.py -k 'rear_local_target_falls_back_to_waypoint or drive_toward_rear_target_turns_in_place'`
- 结果：`2 passed`

### 问题
在继续外场测试前，需要做一次完整的链路检查和算法检查，确认当前版本还有哪些已知风险，并明确每次试车要记录哪些参数，方便后续定位问题。

### 当前链路检查结论
- `bringup.launch.py` 的 GPS 模式链路已经打通：
- `nmea_navsat_driver` 可选自动启动；
- `outdoor_pose_fuser` 负责 `/nav/odom`、`/nav/status` 和 TF；
- `navigation.launch.py` / `mission.launch.py` 已把 GPS 模式下的 `/odom` 消费者 remap 到 `/nav/odom`。
- `navigation.launch.py` 里 `path_follower` 的参数加载顺序已修正为先 `robot.yaml` 后 `waypoints.yaml`，避免全局底盘限速把导航专用参数覆盖掉。
- `PathFollower` 当前已具备三层保护：
- 拒绝使用车后方的 `/local_target`；
- 大角度误差时先原地转向，不再边大角度转边高速前冲；
- 只在足够对准目标后才恢复前进速度。
- `GapPlanner` / `WeavePlanner` 当前已具备两层约束：
- gap 内优先朝 waypoint bearing，而不是盲目取 gap 中心；
- 如果 waypoint bearing 本身足够清楚，则直接沿 waypoint bearing 发 `/local_target`，只有被挡住时才进入 gap 竞争。

### 当前剩余风险
- 最大剩余风险不是 waypoint 坐标，而是 `/nav/odom` 朝向是否稳定。
- 如果朝向抖动或方向与真实车头不一致，`PathFollower` 会把同一个 waypoint 解释成左右不同方向，表现仍然会是摇摆。
- 第二个风险是 LiDAR 原始数据噪声仍可能让“goal bearing clear / not clear”的判定在临界值附近来回切换；虽然现在比之前稳，但外场仍需用 topic 记录确认。
- 第三个风险是现场配置文件现在只保留了 1 个 GPS waypoint；这不影响当前单点联调，但会让仓库里一个假设“GPS 示例文件至少含 2 个 waypoint”的旧测试失败。这个是配置与旧测试假设不一致，不是导航算法错误。

### 回归验证
- 已运行：`python3 -m pytest tests/test_navigation_step3.py tests/test_tf_tree.py`
- 结果：全部通过
- 已运行：`python3 -m pytest tests/test_navigation_step3.py tests/test_waypoints.py tests/test_tf_tree.py`
- 结果：除 `tests/test_waypoints.py::test_repo_real_gps_waypoints_file_loads` 外，其余全部通过
- 失败原因：当前 `config/waypoints_real_gps.yaml` 被现场调试改成了单 waypoint，旧测试仍要求 `>= 2` 个 waypoint

### 外场必须记录的参数
- 每次试车，至少记录下面 8 类数据：
- 1. 机器人初始摆放：
- 机头朝向的大致方位；
- waypoint 相对机器人是在正前、左前还是右前；
- 起步点附近是否有树干、墙、金属杆、围栏等会影响 LiDAR 的物体。
- 2. 当前配置快照：
- `config/gps.yaml` 里的 `gps_origin_lat/lon`
- `config/waypoints_real_gps.yaml` 的 `origin`
- 当前启用的 waypoint 名称和坐标
- `config/lidar.yaml` 中 `goal_clear_window_deg`、`goal_clear_min_fraction`
- `config/waypoints.yaml` 中 `max_linear_vel`、`k_angular`、`k_linear`、`rotate_in_place_angle_deg`
- 3. 定位与姿态：
- `ros2 topic echo /nav/odom --once`
- 连续观察 `/nav/odom` 时，位置是否连续、orientation/yaw 是否稳定
- 4. 全局目标：
- `ros2 topic echo /waypoint/current --once`
- 记录 waypoint 是否和你实际希望去的位置一致
- 5. 局部目标：
- `ros2 topic echo /local_target`
- 如果还要更细分，额外看：
- `ros2 topic echo /gap/local_target`
- `ros2 topic echo /weave/local_target`
- 6. 导航输出命令：
- `ros2 topic echo /cmd_vel_auto`
- 重点记录：
- `linear.x` 是否长期被打满
- `angular.z` 是否在正负之间切换
- 摆头时切换周期大概是多少
- 7. 安全层状态：
- `ros2 topic echo /emergency_stop`
- `ros2 topic echo /deadman_ok`
- `ros2 topic echo /control_mode`
- 防止把“摇摆”误判成导航问题，实际却是安全层在抢控制
- 8. GPS 状态：
- `ros2 topic echo /nav/status`
- `ros2 topic echo /fix --once`
- 记录当时是 `GPS_READY` 还是 `GPS_LOST`，以及 fix 是否稳定

### 建议的最小记录模板
- 时间：
- waypoint 名称：
- 机器人初始朝向：
- `/waypoint/current`：
- `/nav/odom` 初值：
- `/local_target` 现象：
- `/cmd_vel_auto` 现象：
- `/nav/status`：
- `/emergency_stop`：
- 现场障碍情况：
- 最终现象：
- 备注：

### 问题
现场切到 `AUTO` 后完全没有运动反应，但手柄模式切换日志是正常的。

### 根因
- 从现场终端日志可以直接看到关键报错：
- `PathFollower: parameter waypoints_file is empty — pass it via launch or set in waypoints.yaml`
- 随后又出现：
- `PathFollower: waypoints list is empty — mission complete immediately`
- `PathFollower: IDLE -> DONE`
- 这说明问题不在手柄、不在 `AUTO/PAUSED` 模式切换本身，而在 `PathFollower` 启动时根本没有拿到 waypoint 文件，所以它一进入 AUTO 就立刻判定“任务已完成”，自然不会发导航速度。

### 结论
- `AUTO` 没反应的直接原因是：`path_follower` 当前运行实例的 waypoint 列表为空。
- 换句话说，当前机器人上实际运行的 `navigation.launch.py` / 安装产物，没有把 `waypoints_file` 正确传进 `path_follower`。

### 高概率场景
- 机器人上还在跑旧的 install 产物，未重新 `colcon build` / `source install/setup.bash`
- 启动命令不是当前仓库里的 `navigation.launch.py`
- 启动时 workspace 没有 source 到你刚改过的 `auto_nav`
- 或者直接单独跑了 `path_follower`，没经过 launch 注入 `waypoints_file`

### 现场应对
- 先不要继续调控制参数，先修 waypoint 文件注入问题。
- 最直接的兜底启动方式是显式传绝对路径：
- `ros2 launch auto_nav navigation.launch.py use_gps:=true waypoints_file:=/root/workspace/auto_nav_team18/config/waypoints_real_gps.yaml`
- 如果这样恢复正常，再回头查 install 里的 launch 是否还是旧版本。

### 问题
已经 `source install/setup.bash`，且 `ros2 pkg prefix auto_nav` 返回的是当前工作区的 install 路径，但 `ros2 param get /path_follower waypoints_file` 仍然是空字符串，`/waypoint/current` 也没有输出。

### 根因
- 这说明“环境已经指向当前工作区”这一点是正常的，但“当前正在运行的 `/path_follower` 进程”并没有拿到 waypoint 参数。
- 典型原因不是 package 没找到，而是：
- 正在运行的 `navigation.launch.py` 是更早启动的旧进程，未重启；
- 或者启动 `navigation.launch.py` 时没有显式传入 `waypoints_file`，同时运行实例也没有从 launch 自动拿到该参数；
- 因此 `source` 本身不会自动更新已在运行中的 ROS 节点参数，必须停掉旧进程再重启。

### 结论
- `ros2 pkg prefix auto_nav = /root/workspace/auto_nav_team18/install/auto_nav` 说明包路径没问题。
- 当前 blocker 是：老的 `/path_follower` 还在运行，且它的 `waypoints_file` 为空。
- 所以必须先停掉当前 navigation 相关进程，再按带显式 `waypoints_file` 的命令重新启动。

### 问题
显式执行 `ros2 param get /path_follower waypoints_file` 后，返回仍然是空字符串；随后 `ros2 topic echo /waypoint/current --once` 一直没有消息。

### 结论
- 这说明当前运行中的 `/path_follower` 进程仍然没有成功加载 waypoint 文件。
- 在这个状态下继续测 `AUTO` 没有意义，因为导航节点没有任何目标点可发布。

### 直接判断
- 如果 `waypoints_file` 为空，那么无论 GPS、LiDAR、deadman、控制器参数怎么调，`path_follower` 都会保持“无任务可执行”状态。
- 所以当前问题仍然是“导航进程启动参数未生效”，不是控制算法问题。

### 现场建议
- 先停掉所有 `navigation.launch.py` / `path_follower` 相关终端和残留进程。
- 重新 `colcon build --packages-select auto_nav`，然后 `source install/setup.bash`。
- 用显式绝对路径重新启动 `navigation.launch.py`，再立刻检查：
- `ros2 param get /path_follower waypoints_file`
- `ros2 topic echo /waypoint/current --once`

### 问题
进一步排查发现：
- `ps -ef` 显示 `path_follower` 进程的命令行已经带了
  `-p waypoints_file:=/root/workspace/auto_nav_team18/config/waypoints_real_gps.yaml`
- 但 `ros2 param get /path_follower waypoints_file` 仍然返回空字符串
- 同时 `install/auto_nav/share/auto_nav/config/waypoints.yaml` 通过软链接指向源目录配置，且其中仍是：
- `waypoints_file: ""`

### 结论
- 当前 install 配置和 source 配置是联动的，install 目录没有独立副本；直接改 source 里的 `config/waypoints.yaml` 即可影响运行时配置。
- 既然运行时 `-p waypoints_file:=...` 覆盖在这台机器上没有体现在节点参数里，就不要继续依赖 CLI override。
- 最稳妥的办法是把绝对路径直接写进 `config/waypoints.yaml`，然后重启节点。

### 容易误判的一点
- `/waypoint/current` 不是节点一启动就发布。
- `PathFollower` 只有在收到 `/control_mode = AUTO`、进入 mission start 后，才会调用 `_set_target_to_wp()` 并发布 `/waypoint/current`。
- 所以“`/waypoint/current --once` 没输出”只有在 `waypoints_file` 已确认正确、且 AUTO 已真正触发后，才有排查价值。

### 问题
现场日志里切到 `AUTO` 后，任务很快进入导航，但机器人表现为“自动模式没有正常反应”，随后 mission 被中止。

### 根因
- 这份日志说明自动模式实际上已经启动，不是没进 AUTO。证据包括：
- `Mode: MANUAL → AUTO`
- `Mission: AUTO_READY → NAVIGATE_TO_WAYPOINT`
- `PathFollower: IDLE → NAVIGATING`
- 中间频繁出现 `AUTO ↔ PAUSED` 是 deadman 正常行为：`/deadman_ok` 失效时 `mode_manager` 会把 `AUTO` 切到 `PAUSED`，恢复后再切回 `AUTO`。
- 真正阻止机器人前进的是 `ObstacleGuardNode` 的急停。日志里连续出现：
- `ObstacleGuard: obstacle at 0.49m angle=29.3°`
- `ObstacleGuard: obstacle at 0.50m angle=-49.0°`
- `ObstacleGuard: obstacle at 0.39m angle=7.3°`
- 当前 `safety_distance_m=0.5`，这些回波都已经落入急停区，因此 `/emergency_stop=True` 会被拉高，`cmd_gate` 会拦截 `/cmd_vel_safe`。
- mission 随后的 `Mission: NAVIGATE_TO_WAYPOINT → ABORTED` 也与急停逻辑一致，表示任务被安全层中止，而不是导航节点没工作。

### 结论
- 这份日志里“自动模式没有反应”并不是模式切换异常，而是“自动模式已进入，但立刻被 LiDAR 安全层拦住”。
- 从安全逻辑上看，这个表现是正常的；从任务执行结果看，这意味着前方近距离障碍检测当前阻止了自动驾驶继续前进。

### 现场建议
- 先确认机器人前方约 `0.39m ~ 0.50m`、`-49° ~ +29°` 范围内是否真的有障碍。
- 如果现场看起来是空的，优先排查 LiDAR 是否扫到车体、保险杠、支架、线缆或地面近距离反射。
- 同时开三个 topic 对照看，最容易分辨根因：
- `ros2 topic echo /control_mode`
- `ros2 topic echo /deadman_ok`
- `ros2 topic echo /emergency_stop`
- 只要 `/emergency_stop` 为 `True`，即使已经在 `AUTO` 且 deadman 恢复，底盘也不会继续前进。

### 问题
需要为当前项目整理一份“户外真实 GPS 导航测试流程”，用于真实场地按步骤执行和记录结果。

### 处理
- 先复查了项目要求、架构和目标，确认 GPS 在本项目中用于室外粗导航，不作为最终到点真值。
- 检查了现有启动入口 `scripts/start_all_services.sh`，确认 `test` 模式会自动启用 `USE_GPS=true` 和 `USE_NMEA_GPS=true`，适合作为户外实测主入口。
- 检查了现有 GPS 配置文件：
- `config/gps.yaml`
- `config/waypoints_real_gps.yaml`
- 确认测试流程必须强调 `origin` 一致性、单 waypoint 先行、`GPS_READY` 先确认、`/emergency_stop` 与 `waypoints_file` 两类常见 blocker 的排查顺序。
- 重写了 `docs/gps_nav_test_plan.md`，改成按现场执行顺序展开的中文流程，覆盖：
- 静态检查
- GPS 就绪检查
- 单 waypoint 户外导航
- GPS 丢失恢复
- 多 waypoint 连续导航
- 记录模板
- 常见失败现象与排查优先级

### 结果
- 已新增一份可直接执行的户外真实 GPS 导航测试流程文档：
- `docs/gps_nav_test_plan.md`

### 说明
- 这次修改只涉及文档整理，没有改动导航、融合或控制代码，因此没有新增自动化测试可运行。
- 该流程默认使用：
- `USE_GPS=true USE_NMEA_GPS=true ./scripts/start_all_services.sh test`
- 如果现场 launch 参数覆盖不稳定，文档中也给出了显式 `WAYPOINTS_FILE=...` 的启动方式。

### 建议执行顺序
- 第一次外场只做 Level A 到 Level C，不要直接做多点路线。
- 单点通过至少 2 次后，再做 GPS 丢失恢复和多 waypoint 测试。

### 问题
机器人在接近较远 waypoint 的过程中，开始左右摇摆，像是在左右抢方向。这种现象现场应该怎么测试和定位？

### 结论
- 这类问题不要只盯着“机器人在晃”，要把它拆成三层分别看：
- 全局目标是否合理：`/waypoint/current`
- 局部目标是否左右跳：`/local_target`、`/gap/local_target`、`/weave/local_target`
- 机器人姿态是否稳定：`/nav/odom`
- 如果 `angular.z` 正负快速切换，而 `/local_target` 也在左右跳，优先怀疑局部规划层；
- 如果 `/local_target` 基本稳定，但 `/nav/odom` 的 yaw 在抖，优先怀疑 GPS/odom/IMU 融合；
- 如果 waypoint 本身就在车侧后方，原地转向或轻微摆头可能只是目标配置问题，不一定是控制器坏了。

### 现场测试方法
- 测试前把场景简化：
- 只保留 1 个 waypoint；
- waypoint 距离控制在约 `8m ~ 15m`；
- 尽量放在机器人正前方大方向，不要一开始就侧后方；
- 暂时不要做多点、不要叠加复杂障碍。

- 测试时同时抓下面几个 topic：
- `ros2 topic echo /waypoint/current`
- `ros2 topic echo /nav/odom`
- `ros2 topic echo /local_target`
- `ros2 topic echo /gap/local_target`
- `ros2 topic echo /weave/local_target`
- `ros2 topic echo /cmd_vel_auto`
- `ros2 topic echo /emergency_stop`

### 判定顺序
- 先看 `/cmd_vel_auto`
- 如果 `linear.x` 很小，而 `angular.z` 在正负之间快速切换，说明导航层正在左右抢方向。
- 再看 `/local_target`
- 如果局部目标在左前、右前、甚至车后方之间来回跳，问题重点在局部规划，不是底盘。
- 再看 `/nav/odom`
- 如果机器人实际朝向变化不大，但 yaw 数值明显抖动或方向反常，问题重点在姿态源。
- 最后对照 `/waypoint/current`
- 如果 waypoint 大方向本来就不在机器人前方，那么先修 waypoint/origin/起始朝向配置，再谈控制问题。

### 建议测试流程
1. 原地静止 10 秒，只看 `/nav/odom`
- 目的：排除姿态源静态抖动。
- 通过标准：机器人不动时，yaw 不应明显来回跳。

2. 手动低速直行 3 到 5 米，只看 `/nav/odom`
- 目的：确认姿态和位姿更新方向基本正确。
- 通过标准：前进时位置连续变化，朝向变化合理。

3. 进入 AUTO，单 waypoint 测试，只看 `/cmd_vel_auto` 和 `/local_target`
- 目的：判断是不是局部目标在抢控制。
- 通过标准：`/local_target` 总体朝 waypoint 收敛，不应左右大跳。

4. 摇摆一出现，立刻记录 10 到 20 秒现象
- 记录：
- `/cmd_vel_auto.angular.z` 是否正负切换
- `/local_target` 是否左右跳
- `/nav/odom` yaw 是否同步抖动
- `/emergency_stop` 是否被拉高

5. 如果摇摆明显，立即切回 MANUAL
- 不要让它长时间在错误状态下继续跑，避免把安全层、避障层和路径层的问题混在一起。

### 最小记录模板
- 时间：
- waypoint 名称：
- waypoint 大致方向：
- 摇摆开始距离目标约多少米：
- `/cmd_vel_auto` 现象：
- `/local_target` 现象：
- `/nav/odom` yaw 现象：
- `/emergency_stop`：
- 现场是否有障碍：
- 最终判断更像哪一层问题：

### 实用建议
- 第一次不要追求“直接修好”，先追求“把摇摆归类清楚”。
- 只要能判断它是 `/local_target` 抖动、`/nav/odom` yaw 抖动、还是 waypoint 配置错误，后续改代码才不会乱改。

### 问题
现场测试时需要一条命令就能开始记录 GPS 导航相关日志，测试完成后直接把日志包发回来，不希望手工开很多 `ros2 topic echo`。

### 处理
- 新增脚本 `scripts/record_gps_debug.sh`，用于一键记录 GPS 导航调试日志。
- 脚本会自动：
- source ROS 和当前 workspace；
- 创建 `artifacts/logs/gps_debug_<timestamp>` 目录；
- 记录关键 topic：
- `/control_mode`
- `/deadman_ok`
- `/emergency_stop`
- `/fix`
- `/nav/status`
- `/nav/odom`
- `/waypoint/current`
- `/waypoint/status`
- `/local_target`
- `/gap/local_target`
- `/weave/local_target`
- `/cmd_vel_auto`
- `/cmd_vel_safe`
- 额外保存一次性快照：
- `ros2 node list`
- `ros2 topic list -t`
- `/path_follower` 的 `waypoints_file`
- `/path_follower` 的 `use_gps`
- 退出时自动打包成 `.tar.gz`，方便直接回传分析。
- 另外补充了 `tests/test_record_gps_debug_script.py`，校验脚本存在、关键 topic 没漏、归档逻辑存在。

### 问题
拿到一份室内录制包后发现，大多数关键日志文件都没有真正录到消息，而是报：
- `Failed to find a free participant index for domain 0`

### 根因
- 旧版录制脚本为每个 topic 单独起一个 `ros2 topic echo`；
- 在机器人容器里同时开这么多 CLI 订阅器，会额外消耗大量 CycloneDDS participant；
- 结果是 recorder 自己把 DDS 资源打满，反而录不到关键导航链路。

### 修复
- 将 `scripts/record_gps_debug.sh` 重构为 `ros2 bag record` 方案；
- 用单个 rosbag 进程统一录制关键 topic，避免同时起十几个 `ros2 topic echo`；
- 保留少量一次性快照：
- `ros2 node list`
- `ros2 topic list -t`
- `/path_follower` 关键参数
- 少量 `--once` topic 快照
- 退出时对 rosbag 发送 `SIGINT`，确保 bag 正常落盘后再归档。

### 结果
- 新版脚本更适合真实机器人容器环境；
- 你仍然只需要一条命令开始录制，结束后发一个 `.tar.gz` 包回来；
- 但底层录制方式改成了更稳的 rosbag。

### 问题
切换到 rosbag 版本后，现场按 `Ctrl+C` 时会看到已经打印出 `[archive]`，但脚本没有退出，终端里持续出现很多 `^C`。

### 根因
- 脚本原先使用：
- `trap cleanup EXIT INT TERM`
- `INT` 信号触发后虽然执行了 `cleanup`，但没有 `exit`；
- 主循环仍然是 `while true; do sleep 1; done`，所以会继续回到循环等待。
- 结果表现为：
- bag 已经停了；
- 归档已经生成了；
- 但脚本本身还没退出，看起来像“无法关闭”。

### 修复
- 将 `EXIT` 和 `INT/TERM` 分开处理：
- `trap cleanup EXIT`
- `trap handle_signal INT TERM`
- 在 `handle_signal()` 里执行 `cleanup` 后显式 `exit 130`。

### 结果
- 现在按一次 `Ctrl+C` 的预期行为是：
- 停止 rosbag；
- 归档日志；
- 直接返回 shell。

### 问题
外场测试时虽然人和机器人都在室外，但日志里发现导航实际加载的是：
- `config/waypoints_data.yaml`
而不是：
- `config/waypoints_real_gps.yaml`

### 风险
- 这会导致“人在室外，但导航仍在跑本地平面 waypoint”；
- 后续抓到的摇摆日志会混淆“真实 GPS 室外问题”和“本地 waypoint/非 GPS 模式问题”；
- 不符合项目里“室外真实 GPS 测试”的目标。

### 修复
- 在 `scripts/start_all_services.sh` 中增加强制逻辑：
- 只要 `USE_GPS=true`，且 `WAYPOINTS_FILE` 未显式指定，就自动设置为 `config/waypoints_real_gps.yaml`
- 同时拒绝 `config/waypoints_data.yaml` 在 GPS 模式下被使用
- `test` 模式下额外强制：
- `USE_GPS=true`
- `USE_NMEA_GPS=true`
- `USE_SIM_TIME=false`
- 在 `launch/navigation.launch.py` 中增加第二层防呆：
- 若 `use_gps=true` 且传入的是 `waypoints_data.yaml`，直接抛错，不允许悄悄进入错误模式

### 结果
- 现在外场 GPS 启动链路默认只会走真实 GPS waypoint 文件；
- 即使有人手工传入 `waypoints_data.yaml`，GPS 模式下也会被明确拒绝；
- 这样后续录到的外场日志才可判定为真正的 GPS 室外导航数据。

### 问题
新的外场日志包显示：系统已经跑在真实 GPS 室外链路上，但机器人仍会“前进一点点，然后持续左右摇摆”，且不是单纯完全不走。

### 证据
- `path_follower_waypoints_file.txt` 指向：
- `config/waypoints_real_gps.yaml`
- `nav_status_once.txt` 为：
- `GPS_READY`
- `control_mode_once.txt` 为：
- `AUTO`
- `waypoint_status_once.txt` 为：
- `NAVIGATING`
- 这说明该包确实是外场 GPS 模式下的有效导航记录，不再是误跑本地 waypoint 文件。

### 日志结论
- `/deadman_ok` 在摇摆阶段持续为 `True`，不是 deadman 丢失导致停走停走。
- `/emergency_stop` 在摇摆开始时为 `False`，后段才有 `True`，所以急停不是初始根因。
- `/nav/status` 在摇摆开始阶段保持 `GPS_READY`，不是 GPS 丢失后才开始出问题。
- `/cmd_vel_auto` 和 `/cmd_vel_safe` 前段确实会给 `linear.x = 0.4`，说明系统最开始允许向前推进。
- 之后 `/cmd_vel_auto.angular.z` 正负方向切换很多次，`/cmd_vel_safe` 也同步切换，说明摆头是导航层自己在输出，而不是安全层单独改写。
- `/local_target` 与 `/gap/local_target` 轨迹本身没有出现那种“左右大幅反复跳边”的特征；至少从 odom 坐标看，没有出现频繁左右跨侧的简单模式。

### 判断
- 这次外场摇摆的主嫌疑不再是：
- `waypoints_data.yaml` 误用
- deadman 抖动
- 初始急停
- 后方 local target 抢控制
- 更值得优先怀疑的是：
- 机器人姿态估计与局部目标/全局目标之间的相对方位解释不稳定
- 换句话说，问题更像“目标没有明显乱跳，但控制器对目标方向的判断在来回改判”。

### 当前优先级
1. 优先怀疑 `/nav/odom` 的 yaw / 朝向解释链路
2. 其次怀疑 GPS/odom 融合后的朝向与真实车头不一致
3. 再其次才是局部规划本身在特殊位置造成的抖动

### 下一步建议
- 后续日志脚本应继续保留：
- `/nav/odom`
- `/local_target`
- `/gap/local_target`
- `/cmd_vel_auto`
- `/cmd_vel_safe`
- 并建议额外加入：
- `/odom`
- `/imu/data`
- 这样可以直接判断是融合姿态问题，还是 path_follower / local planner 的目标解释问题。

### 问题
用户进一步询问：当前外场导航里，“轮速/odom 有没有融合 IMU”，以及当前摇摆是否可能由融合导致。

### 代码结论
- 原始 `/odom` 不是本仓库自己和 IMU 融合出来的结果；导航栈把它当作“底盘原始 odom 输入”使用。
- 真正的室外融合发生在 `auto_nav/navigation/outdoor_pose_fuser.py`：
- 位置：
- 先使用原始 `/odom` 的位移增量推进 `nav_x/nav_y`
- 再用 GPS fix 通过 `gps_position_alpha` 做位置回拉
- 朝向：
- 若 `config/gps.yaml` 中 `use_imu_yaw=true`，则优先使用 `/imu/data` 的 yaw
- 否则退回使用原始 `/odom` 里的 yaw
- 然后叠加 odom-to-ENU 的 `heading_offset`
- 因此：外场 GPS 模式下，`/nav/odom` 的朝向确实会受到 IMU 影响。

### 判断
- 是的，当前外场融合链路里，IMU 很可能会影响最终导航朝向；
- 如果 IMU yaw 本身有漂移、跳变、磁干扰，或者 heading offset 学偏，就可能表现为“前进一点后左右摆头”。
- 所以“融合导致的”是一个高优先级怀疑方向，不是次要因素。

### 处理
- 为了下一包能直接验证这一点，更新了 `scripts/record_gps_debug.sh`：
- rosbag 录制新增：
- `/odom`
- `/imu/data`
- 一次性快照新增：
- `imu_data_once.txt`
- `odom_once.txt`

### 结果
- 下一次外场录包后，可以直接对比：
- 原始 `/odom`
- 原始 `/imu/data`
- 融合后的 `/nav/odom`
- 从而判断是 IMU 源、原始 odom，还是 outdoor_pose_fuser 的融合逻辑在放大摆头。

### 问题
补录 `/odom` 和 `/imu/data` 后，新的外场日志需要确认：当前摇摆是否真的是 IMU 融合导致。

### 结论
- 这次包里 `/imu/data` 话题被列入录制，但 `imu_data_once.txt` 没有任何消息；
- 说明本次测试里虽然 topic 名存在于图上，但并没有实际收到 `/imu/data` 消息；
- 因此这次外场运行时，`outdoor_pose_fuser` 很可能没有拿到 IMU yaw，而是退回使用原始 `/odom` 朝向。

### 依据
- `config/gps.yaml` 中 `use_imu_yaw=true`
- 但 `auto_nav/navigation/outdoor_pose_fuser.py` 的实现是：
- 有 IMU yaw 就优先用 IMU yaw
- 没有 IMU yaw 就退回 `_last_raw_yaw`
- 这次快照里：
- `imu_data_once.txt` 为空
- `odom_once.txt` 与 `nav_odom_once.txt` 的 orientation 完全一致
- 同时 twist 中的 `linear.x` / `angular.z` 也一致
- 这说明在该时刻，融合器并没有在朝向上明显改写原始 odom。

### 判断
- 所以“本次外场摇摆主要由 IMU 融合导致”这个说法，目前证据不足；
- 更准确地说，这次更像是：
- IMU 数据没有真正进入融合链路
- 导航主要在用原始 odom yaw
- 因此下一步应优先排查：
- 为什么 `/imu/data` 没有实际消息
- 原始 `/odom` 朝向本身是否稳定
- GPS 融合在无 IMU yaw 时是否还在放大 heading offset 误差

### 问题
这个项目怎么在 `rviz2` 里看 bag？应该配置哪些 topic？

### 现有基础
- 仓库已经自带 RViz 配置文件：
- `auto_nav/simulation/sim_bringup/rviz_config.rviz`
- 这个配置默认打开了：
- `TF`
- `RobotModel`
- `LaserScan` → `/scan`
- `Odometry` → `/odom`
- `Image` → `/camera/color/image_raw`
- `Fixed Frame` 设为：
- `odom`

### 最小可视化链路
- 如果 bag 是本地/仿真 odom 模式，优先保证这些 topic 在包里：
- `/tf`
- `/odom`
- `/scan`
- `/camera/color/image_raw`
- `/camera/color/camera_info`
- 如果 bag 是 GPS 模式，导航节点实际可能使用：
- `/nav/odom`
- `/nav/status`
- 但 RViz 的 `Fixed Frame` 仍建议先用：
- `odom`
- 因为项目的 TF 发布仍是 `odom -> base_link`。

### 建议在 RViz2 里加的显示项
- 必开：
- `TF`：看 `odom -> base_link -> laser/camera_link` 是否完整。
- `Odometry`：topic 选 `/odom`；若回放的是 GPS 模式融合结果，可再加一个 `Odometry` 显示 `/nav/odom` 做对比。
- `LaserScan`：topic 选 `/scan`。
- `Image`：topic 选 `/camera/color/image_raw`。
- 建议加：
- `Pose` 或 `Marker` 类暂时不是首选，因为项目里的 `/marker/detection` 是 `geometry_msgs/PoseStamped`，更适合用 `Pose` 显示观察 cone 检测位置。
- 再加 4 个 `Pose` 显示做导航调试：
- `/waypoint/current`
- `/local_target`
- `/gap/local_target`
- `/weave/local_target`

### 推荐录包 topic 清单
- 如果你是为了“回放时能看清机器人、雷达、相机、导航中间量”，建议至少录：
- `/tf`
- `/tf_static`
- `/odom`
- `/scan`
- `/camera/color/image_raw`
- `/camera/color/camera_info`
- `/control_mode`
- `/deadman_ok`
- `/emergency_stop`
- `/cmd_vel_auto`
- `/cmd_vel_safe`
- `/waypoint/current`
- `/waypoint/status`
- `/navigation/segment`
- `/local_target`
- `/gap/local_target`
- `/weave/local_target`
- `/mission/home_pose`
- 如果你跑的是 GPS 室外模式，再额外录：
- `/fix`
- `/imu/data`
- `/nav/odom`
- `/nav/status`
- 如果你还要看 cone / perception，补上：
- `/marker/detection`
- `/marker/bbox`
- `/object/detection`
- `/perception/distance`

### 实际使用方法
- 先回放 bag：
- `ros2 bag play <你的bag目录> --clock`
- 再开 RViz：
- `rviz2 -d install/auto_nav/share/auto_nav/simulation/sim_bringup/rviz_config.rviz`
- 如果你还没 `colcon build`，也可以直接用源码里的配置：
- `rviz2 -d auto_nav/simulation/sim_bringup/rviz_config.rviz`
- 若回放 GPS bag，建议在现有配置基础上手动再加一个 `Odometry` 显示到 `/nav/odom`，以及 4 个 `Pose` 显示到：
- `/waypoint/current`
- `/local_target`
- `/gap/local_target`
- `/weave/local_target`

### 判断标准
- `TF` 树正常时，应能看到至少：
- `odom -> base_link`
- `base_link -> laser`
- `base_link -> camera_link`
- `LaserScan` 能贴着机器人前方展开，说明 `/scan` 和 TF 对齐基本正常。
- `Odometry(/odom 或 /nav/odom)` 轨迹能连续移动，说明定位消息在回放。
- `Image(/camera/color/image_raw)` 能出图，说明相机 topic 录进去了。
- `Pose(/local_target /gap/local_target /weave/local_target)` 能看到时，才适合继续查“为什么车会左右摇摆”这类导航问题。
