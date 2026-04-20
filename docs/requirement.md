先定总架构：三层闭环，不用 Nav2

你的系统不要做成“一个巨大的主程序”，而要做成三层：

安全层
负责模式切换、dead-man、手柄掉线保护、障碍急停、速度限幅。
这一层优先级最高，所有 /cmd_vel 最终都必须经过它。
导航层
负责 waypoint 管理、局部路径生成、LiDAR 避障、1→2 号 waypoint 之间的 weaving/slalom、最终接近 waypoint marker。
任务层
负责状态机、拍照、物体识别、距离计算、日志、summary 输出。

一句话概括就是：

传感器/手柄/底盘驱动
        ↓
     安全仲裁层
        ↓
   导航与感知执行层
        ↓
    Mission Controller
2. AI agent 必须遵守的开发规则

这部分很重要，

规则 A：一次只做一个“可运行子闭环”

不要让 AI 一次生成全系统。正确顺序是：

手柄安全闭环
单 waypoint 去回
waypoint + LiDAR 避障
1→2 waypoint weaving
waypoint marker 拍照
彩色物体识别 + 距离
完整 mission + summary
规则 B：所有运动命令必须走一个唯一出口

AI agent 不允许让多个节点直接往底盘发 /cmd_vel。
必须只保留一个最终出口，例如：

/cmd_vel_manual
/cmd_vel_auto
/cmd_vel_safe ← 唯一送到底盘
规则 C：参数化，不要硬编码

这些都必须放到 config/*.yaml：

手柄按钮/轴映射
速度上限
dead-man 阈值
waypoint 到达半径
cone 橙色阈值
彩色物体颜色阈值
LiDAR 安全距离
串口名、LiDAR IP
规则 D：先复用，再自研

优先复用：

底盘：给定的 ariaNode
手柄：joy
手动控制：teleop_twist_joy 可先用，再在外面加安全层
相机：depthai-ros
TF：标准 tf2
规则 E：每一步都必须产出 5 样东西

AI agent 每轮输出必须包含：

改了哪些文件
新增了哪些 topic / service / param
怎么启动
怎么测试
当前还剩什么风险



