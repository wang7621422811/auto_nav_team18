Step 1：🎮 Gamepad 安全控制（teleop/）

这是第一优先级。没有这个，后面的自动化都不安全。

目标

X：进入自动模式
O：进入手动模式，并关闭自动模式
自动模式下：后扳机作为 dead-man，松开立即停车
手动模式下：摇杆能直接控制前后左右
手柄掉线：立即停车

手柄使用：ps4手柄，但是需要配置，不要写死，可以兼容其他手柄比如swtich pro

建议实现
拆成 4 个节点，不要写成一个巨型脚本：

joy_mapper_node.py
负责把不同手柄映射统一成你的内部语义。
因为 DualShock clone 很可能按键编号和标准 DS4 不一样，不要写死按钮编号。
mode_manager_node.py
监听 X/O，发布当前模式。
建议支持：
MANUAL
AUTO_READY
AUTO_PAUSED
MANUAL_OVERRIDE
gamepad_watchdog_node.py
监控 /joy 时间戳。比如 0.5 秒没新消息就认为断连。
cmd_gate_node.py
这是最关键的节点。输入：
手动命令
自动命令
模式
dead-man 状态
obstacle stop 状态
输出唯一 /cmd_vel_safe

推荐逻辑

if joystick_lost:
    publish zero
elif mode == MANUAL:
    pass manual cmd
elif mode == AUTO and deadman_ok and not emergency_stop:
    pass auto cmd
else:
    publish zero

完成标准

按 O 后，摇杆能开车
按 X 后，手动指令失效，切自动
自动模式不按 dead-man l,r绝对不能动
松开 dead-man 200ms 内停车
手柄断连立即停车

---
```
# PS4（默认，不加参数也是 ps4）
ros2 launch auto_nav teleop.launch.py gamepad:=ps4

# Switch Pro
ros2 launch auto_nav teleop.launch.py gamepad:=switch_pro

# bringup 同样支持
ros2 launch auto_nav bringup.launch.py gamepad:=switch_pro
```