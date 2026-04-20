项目目标：
为 Pioneer 3-AT 机器人实现一个不依赖 Nav2 的自主任务系统。任务包括：
1. 依次访问多个 waypoint，并最终返回起点；
2. 每个 waypoint 有一个橙色 traffic cone，机器人到达 1-2m 内要拍照，并保证该 marker 留在机器人右侧；
3. 第 1 和第 2 个 waypoint 之间需要 weaving through cones；
4. 每个 waypoint 附近有一个额外彩色物体，需要识别颜色/形状、拍照，并计算它到 waypoint marker 的距离；
5. 任务结束输出 journey summary；
6. 使用 LiDAR 避开静态和动态障碍；
7. 使用蓝牙手柄：X 启用自动模式，O 启用手动模式；自动模式下后扳机是 dead-man，松开必须停车。

硬件/软件约束：
- Ubuntu 24.04 + ROS2
- 底盘驱动优先复用 ariaNode / AriaCoda
- 相机为 OAK-D V2，优先复用 depthai-ros
- LiDAR 为 SICK 或 Lakibeam
- 不使用 Nav2
- GPS 输入可保留，但闭环控制不能依赖 GPS 精度
- 所有底盘运动命令都必须经过唯一的 cmd_gate/safety gate
- 优先 Python 实现，性能不够再局部改 C++

工程约束：
- 只完成当前指定 Step，不要一次性写全系统
- 所有按钮/轴/阈值/速度限制都必须参数化
- 每个新节点都要给出 launch、yaml、测试方法
- 能复用现成 ROS2 包就不要重造
- 代码必须可运行，不要只给伪代码