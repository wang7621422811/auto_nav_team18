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
