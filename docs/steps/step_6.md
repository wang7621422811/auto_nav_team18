Step 6：📊 Journey Summary 日志（mission/）

这是最后展示效果的关键，不要放到最后一天做。

建议做两个节点

journey_logger.py
summary_generator.py

journey_logger 记录

start/end time
每个 waypoint 的到达时间
走过的总里程
dead-man 触发次数
障碍急停次数
每个 waypoint 的 marker/object 图片路径
识别出的颜色、形状、距离
是否成功 return home

summary_generator 输出

summary.json
summary.md

建议 Markdown 模板：

Mission Result: SUCCESS / PARTIAL / FAIL
Total Time:
Total Distance:
Dead-man Stops:
Emergency Stops:

Waypoint 1:
- Marker photo:
- Object photo:
- Object color:
- Object shape:
- Distance to marker:

Waypoint 2:
...

完成标准

mission 结束自动生成 summary
summary 里能定位到照片
出现失败项时也能输出部分结果，而不是空白