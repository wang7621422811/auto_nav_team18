Step 2：🗺️ Waypoint 导航框架（navigation/）

这里要处理一个现实问题：你任务书写的是 GPS waypoints，但课程资料又说 GPS 不适合作为主定位。我的建议是做一个双输入接口：

外部输入可以是经纬度 waypoint
内部控制一律转成局部坐标 map/local_origin

目标

读取 waypoints
按顺序访问
最后返回起点
记录当前 waypoint index
为后续 final approach 预留接口

建议实现

waypoint_provider.py
支持两种 waypoint：
lat/lon
x/y
geo_localizer.py
如果输入是 GPS，经纬度转局部 ENU / local frame。
但不要拿 GPS 直接判断 1–2m 到点。
path_follower.py
全局路径不用 nav2，也不需要复杂 global planner。
直接把路径定义成：起点 -> WP1 -> WP2 -> ... -> HOME 的折线段。
waypoint arrival 两阶段逻辑
粗到达：距离 nominal waypoint 小于 2–3 米
精到达：识别到橙色 marker 后，切 final approach

关键设计：marker 永远在机器人右侧
这条要求不能靠“临场避障”顺便做到，必须显式实现。

建议在 final_approach.py 中这样做：

已知 marker 位置 p_marker
已知机器人到 marker 的入射方向 u
取左法向 n_left
设置通过点
p_pass = p_marker + pass_offset * n_left

机器人去追这个 p_pass，就会从 marker 左边经过，因此 marker 留在机器人右侧。

完成标准

能加载 waypoint 列表
能自动走到单个 waypoint 再返回 home
切到下一个 waypoint 的逻辑正确
home pose 由 mission 开始时记录，而不是程序启动时记录