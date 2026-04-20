Step 3：🚗 路径跟踪 + LiDAR 避障 + weaving（navigation/）

这是本项目最难的核心。

3.1 普通路段：局部规划 + 避障

建议算法

跟踪器：Pure Pursuit 或 Stanley
局部避障：自己实现一个轻量版本的 gap follower / VFH-lite
安全层：前方扇区最小距离急停

推荐拆分

local_planner.py：给出局部目标点
gap_planner.py：根据 /scan 选一条可通行 gap
obstacle_guard.py：做安全停障，不参与路径评分

gap 评分建议

score =
  w_goal      * 与当前 waypoint 方向一致性
+ w_clearance * gap 宽度/安全裕度
+ w_progress  * 朝前推进程度
- w_offset    * 偏离当前航段中心线
- w_smooth    * 转向变化过大惩罚
3.2 1st → 2nd waypoint 之间的 weaving

这里不能只是“绕开锥桶”，否则机器人可能直接绕到锥桶区外面去。

建议专门做 weave_planner.py
只在 segment_id == 1_to_2 时启用：

把 WP1 -> WP2 这段定义成一条走廊
LiDAR 只重点考虑这条走廊内的锥桶和障碍
gap 选择时强烈惩罚“离开走廊过远”的方案
这样机器人会在走廊里穿行，而不是整个绕开

非常关键的一点
最终接近 waypoint marker 时，LiDAR 会把目标锥桶也当障碍。
所以你必须做一个target whitelist：

当前 mission 知道哪个 cone 是“目标 marker”
final approach 模式里，把这个 marker 对应的 LiDAR cluster 从急停判定里剔除
其他障碍仍然保留

否则机器人永远无法进入 1–2 米内。

完成标准

普通障碍不会撞
有人/车/墙进入前方安全区会停车
1→2 waypoint 之间能够在锥桶走廊内前进，而不是绕远路
最终接近目标 marker 时不会被目标 marker 自己“误急停”