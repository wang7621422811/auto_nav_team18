工程约束：

- 只完成当前指定 Step，不要一次性写全系统
- 所有按钮/轴/阈值/速度限制都必须参数化
- 每个新节点都要给出 launch、yaml、测试方法
- 能复用现成 ROS2 包就不要重造
- 代码必须可运行，不要只给伪代码

当前要实现的 Step：
先做环境和基础 bringup（建议补这一阶段）

虽然你给的正式顺序从 Step 1 开始，但实际开发前一定要先做这个。

目标
让底盘、LiDAR、相机、手柄、odom、tf 全部“活起来”。

AI agent 任务

写 bringup.launch.py
接入 ariaNode
接入 LiDAR 驱动
接入 depthai-ros
接入 joy
确认 /odom /scan /joy /camera/* 都在发
记录 mission 开始时的 home pose

完成标准

机器人能在低速手动控制下移动
可以看到 odom 和 lidar
手柄断连时系统能检测到
相机能保存一张测试图

特别提醒
PDF 已经说明开发环境在机器人端走 Docker、没有 sudo，所以 bringup 阶段就要把设备路径、串口、LiDAR IP、相机参数全部做成配置项。SICK 和 Lakibeam 的 IP 也不要硬编码死

输出格式必须包含：并且写入到下面交付内容：

1. 设计说明
2. 修改的文件列表
3. 完整代码
4. launch 和 yaml
5. 测试方法
6. 完成标准
7. 当前遗留风险

交付内容：
Step 0 交付总结

1. 设计说明

三层架构（安全 → 导航 → 任务）从这里开始搭底座。Step 0 只做硬件 bringup，不引入任何控制逻辑：

唯一运动出口：bringup.launch.py 把底盘订阅设为 /cmd_vel_safe，后续 Step 1 的 cmd_gate_node 才会填充它。
LiDAR 二选一：通过 OpaqueFunction 在 launch 时按 lidar_type 参数选 sick_scan_xd 或 lakibeam_ros2，IP 均来自 YAML。
无硬编码：所有设备路径、IP、轴/按钮索引、速度限制、超时值都在 config/*.yaml，real.yaml 覆盖部署差异。
2. 修改/新增文件
文件	说明
package.xml
ament_python ROS2 包声明，依赖 joy / teleop_twist_joy / ros2aria / depthai_ros_driver
setup.py / setup.cfg
安装 launch、config；注册两个 entry_points
resource/auto_nav
ament 标记文件
config/joystick.yaml
轴/按钮/dead-man/超时，全参数化
config/robot.yaml
串口、速度限制、TF 偏移
config/lidar.yaml
SICK / Lakibeam IP、安全距离
config/camera.yaml
OAK-D 分辨率、帧率、输出目录
config/real.yaml
部署覆盖（唯一需要改 IP/串口的地方）
config/mission.yaml
waypoint 到达半径、日志路径（后续 Step 用）
auto_nav/teleop/gamepad_watchdog_node.py
手柄断连检测 → /joy_connected /deadman_ok
auto_nav/mission/home_pose_recorder.py
首条 odom 记为 home pose → /mission/home_pose + YAML
launch/bringup.launch.py
接入 ariaNode / LiDAR / camera / joy / static TF / watchdog / recorder
launch/teleop.launch.py
手动驾驶层（joy + teleop_twist_joy → /cmd_vel_manual）
tests/test_watchdog.py
14 个 unit test（dead-man / 断连 / 发布）
tests/test_home_pose_recorder.py
8 个 unit test（只录一次 / YAML 内容 / 错误处理）
artifacts/{photos,logs,summaries}/.gitkeep
产物目录骨架
3. 新增 Topic / Service / Param
Topic	类型	发布者
/odom
nav_msgs/Odometry
ros2aria（remapped）
/scan
sensor_msgs/LaserScan
sick_scan_xd / lakibeam
/joy
sensor_msgs/Joy
joy_node
/camera/color/image_raw
sensor_msgs/Image
depthai_ros_driver
/camera/depth/image_raw
sensor_msgs/Image
depthai_ros_driver
/joy_connected
std_msgs/Bool
gamepad_watchdog（latched）
/deadman_ok
std_msgs/Bool
gamepad_watchdog（latched）
/mission/home_pose
geometry_msgs/PoseStamped
home_pose_recorder（latched）
/cmd_vel_safe
geometry_msgs/Twist
← 底盘订阅此 topic（Step 1 填充）
TF: odom → base_link（ros2aria）、base_link → laser、base_link → camera_link（static）

1. 启动方式

# 在机器人 Docker 容器内，source ROS2 和工作空间后：

cd ~/workspace/auto_nav_team18  
colcon build --packages-select auto_nav  
source install/setup.bash

# 默认 SICK LiDAR

ros2 launch auto_nav bringup.launch.py

# 切换到 Lakibeam / 覆盖 IP

ros2 launch auto_nav bringup.launch.py   
    lidar_type:=lakibeam   
    lakibeam_ip:=192.168.1.200   
    serial_port:=/dev/ttyUSB1

# 仅手动遥控层

ros2 launch auto_nav teleop.launch.py
5. 测试方法
单元测试（无需硬件）：

python3 -m pytest tests/ -v        # 18 passed
在机器人上验证（完成标准）：

# 确认所有 topic 在发

ros2 topic list | grep -E '/odom|/scan|/joy|/camera|/joy_connected|/deadman_ok'

# 检查 odom 帧率

ros2 topic hz /odom

# 检查 LiDAR

ros2 topic hz /scan

# 拍一张测试图

ros2 run image_transport republish raw < /camera/color/image_raw   
    & ros2 run image_view image_view image:=/camera/color/image_raw

# 断开手柄，确认 /joy_connected 变 False

ros2 topic echo /joy_connected --once

# 检查 home pose YAML

cat artifacts/logs/home_pose.yaml
6. 完成标准（对应 step_0.md）
标准	实现
手动低速移动
teleop.launch.py 接通 joy → /cmd_vel_manual
可见 odom 和 lidar
ros2aria → /odom，SICK/Lakibeam → /scan
手柄断连检测
gamepad_watchdog_node，1 s 超时 → /joy_connected=False
相机测试图
depthai_ros_driver → /camera/color/image_raw
Home pose 记录
home_pose_recorder → artifacts/logs/home_pose.yaml
7. 遗留风险
ros2aria 包名未确认：市面上有 ros2aria / RosAria / p2os 多个变体，需在机器人端 ros2 pkg list | grep aria 确认实际包名，再调整 package.xml 和 bringup.launch.py 的 package= 字段。
Lakibeam ROS2 包：lakibeam_ros2 为推测包名，需检查 Benewake 官方仓库确认 executable 名称。
/cmd_vel_safe 暂无发布者：Step 0 底盘不会动；Step 1 的 cmd_gate_node 才接通这条链路。
串口权限：Docker 内需确认 /dev/ttyUSB* 已 passthrough（--device 参数），否则 ariaNode 连不上底盘。
depthai_ros_driver remapping：不同版本的 driver topic 前缀可能不同，需在目标机器上用 ros2 topic list 确认后调整 remappings。