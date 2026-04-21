# QA Log

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
GPS waypoint 已支持，但机器人当前位置仍然只来自原始 `/odom`，会导致 GPS waypoint 与机器人位姿不在同一坐标系；同时导航节点如果读 `/nav/odom`，TF 还来自 `/odom`，会出现 TF 不一致。

### 结论
按 `docs/steps/gps_function.md` 新增 `outdoor_pose_fuser`，在 GPS 模式下负责：
- 将 `/fix` 转成 ENU；
- 用首个有效 GPS fix 与当前 `/odom` 建立启动对齐；
- 轻量融合 GPS 位置与 wheel odom；
- 发布 `/nav/odom`；
- 发布与 `/nav/odom` 一致的 `odom -> base_link` TF。

### 这次改动
- 新增 `auto_nav/navigation/outdoor_pose_fuser.py`。
- 新增 `config/gps.yaml`，集中保存 GPS 融合参数。
- 新增 `config/waypoints_real_gps.yaml`，提供室外 GPS waypoint 样例。
- 修改 `launch/bringup.launch.py`，增加 `use_gps` 开关。
- 修改 `launch/navigation.launch.py` 与 `launch/mission.launch.py`，在 GPS 模式下把相关 `/odom` 输入 remap 到 `/nav/odom`。
- 修改 `setup.py` 注册 `outdoor_pose_fuser` 可执行入口。
- 新增 `tests/test_outdoor_pose_fuser.py` 与 `tests/test_gps_launch.py`。
- 扩展 `tests/test_waypoints.py`、`tests/test_tf_tree.py`。

### 关键实现
- `outdoor_pose_fuser` 首次拿到有效 fix 时，记录：
  - `gps_init_e/gps_init_n`
  - `odom_init_x/odom_init_y`
- 后续 GPS 点统一做平移对齐：
  - `aligned_x = odom_init_x + (gps_e - gps_init_e)`
  - `aligned_y = odom_init_y + (gps_n - gps_init_n)`
- GPS 正常时，输出 `raw_odom + alpha * (aligned_gps - raw_odom)` 的平滑位置。
- GPS 超时或无效时，退化回原始 odom 位置，同时保留 IMU yaw。
- GPS 模式下不再启用 `odom_tf_broadcaster`，避免双重 TF 源。

### 补充说明
- 为了让 GPS 机器人位姿与 GPS waypoint 使用同一个 ENU 参考，本次在 `config/gps.yaml` 中增加了 `gps_origin_lat/gps_origin_lon`。这一点是对计划的必要补充，否则 `/fix` 无法转换到与 waypoint 相同的平面坐标系。
