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
