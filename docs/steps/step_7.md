Step 7：🔄 仿真 → 真机迁移调试（config/）

这是工程项目最容易被低估的部分。

建议顺序

仿真环境先跑通 teleop + 单 waypoint
rosbag 回放测试 perception
真机低速手动
真机单 waypoint
真机 waypoint + 避障
真机 weaving
真机完整 mission

需要区分 sim / real 参数

速度上限
加速度限制
LiDAR 安全距离
图像分辨率
颜色阈值
串口/IP
机器人宽度和最小转弯参数

实机调试建议

初始最大线速度不要超过 0.2–0.3 m/s
final approach 时再降速
RViz 尽量放到地面站，不要长期在车上跑重可视化
每次实测都录 rosbag2

课程资料已经提醒过：机器人端 RAM 是有限的，RViz 吃内存，最好放地面站；同时开发环境默认走 Docker。这个建议很值得遵守。

6. 关键算法细节，AI agent 很容易忽略的点
6.1 不要把 GPS 当“到点真值”

GPS 可以用来：

接收 waypoint
粗略估算方向
生成初始 local frame

但不能用来：

判断是否已经在 cone 1–2 米内
判断 marker 是否在右侧
计算 object 到 marker 的距离

这些都必须交给局部坐标 + 视觉/深度。

6.2 marker 在右侧，不是“碰巧右侧”

必须显式做 final approach offset。
不要指望普通避障自然满足这条要求。

6.3 目标 cone 要从障碍急停里剔除

否则你永远接近不到 waypoint marker。

6.4 weaving 必须有“走廊约束”

否则局部规划很可能直接绕过整片锥桶区。

6.5 手柄映射必须配置化

DualShock clone 的按钮编号经常和网上教程不一致。

6.6 Mission 失败时也要可恢复

建议所有失败都分为两类：

hard fail：安全问题，直接 abort
soft fail：识别失败、图片不理想，记录 unknown，继续 mission