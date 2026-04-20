Step 5：📋 Mission Controller 状态机（mission/）

这一步决定系统是不是“工程化”。

建议状态机

IDLE
MANUAL
AUTO_READY
NAVIGATE_TO_WAYPOINT
WEAVING_SEGMENT
FINAL_APPROACH
CAPTURE_MARKER
SEARCH_OBJECT
CAPTURE_OBJECT
RETURN_HOME
AUTO_PAUSED_DEADMAN
MANUAL_OVERRIDE
ABORTED
COMPLETE

推荐转换逻辑
E
O → MANUAL
X → AUTO_READY
AUTO_READY → NAVIGATE_TO_WAYPOINT
当前段是 1→2 → WEAVING_SEGMENT
到 waypoint 粗半径 → FINAL_APPROACH
成功到 marker → CAPTURE_MARKER
然后 SEARCH_OBJECT
识别完毕 → 下一个 waypoint
全部结束 → RETURN_HOME
home 到达 → COMPLETE
dead-man 松开 → AUTO_PAUSED_DEADMAN
障碍过近 → 停车并等待清除

一个很实用的策略
物体不在当前相机视野内时，不要立刻判失败。
进入 SEARCH_OBJECT 后，执行一个小范围搜索动作，例如：

原地左右摆头/转向
或慢速小弧线扫描 5–8 秒

这样对“物体在 waypoint 附近但不正对前方”的情况非常有帮助。

完成标准

状态机不乱跳
手动覆盖后不会继续偷偷跑自动
松开 dead-man 能暂停自动
再按住 dead-man 可以继续当前 mission
任务结束后能返回 home 并进入 COMPLETE