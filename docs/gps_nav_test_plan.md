# GPS 户外真实导航测试流程

## 1. 目标

本流程用于真实机器人、真实 GPS、真实户外环境下的 GPS 导航测试。目标不是一次性跑完整 mission，而是按风险从低到高逐步验证：

1. GPS 是否稳定出 fix；
2. `outdoor_pose_fuser` 是否进入 `GPS_READY`；
3. 导航节点是否正确读取 GPS waypoint；
4. 机器人是否能完成单点粗导航；
5. GPS 丢失后系统是否能安全恢复；
6. 最后才做多 waypoint 连续导航。

注意：根据项目目标，GPS 只负责室外粗导航，不能把 GPS 精度直接当作最终到点真值。最终接近、避障和停车仍要结合 odom、LiDAR 和任务状态判断。

---

## 2. 测试前提

开始真实户外测试前，先满足以下条件：

- `colcon build --symlink-install` 已成功；
- 室内或 bench 联调已通过，不要直接跳到户外；
- 手柄工作正常，`AUTO` / `MANUAL` / deadman 已经验证过；
- LiDAR、IMU、GPS、底盘都能正常上线；
- 当前场地允许低速试车，并预留人工接管空间；
- 至少 2 人在场：1 人盯机器人和急停，1 人盯终端和 topic。

---

## 3. 现场前检查

### 3.1 场地要求

- 开阔区域，GPS 天线尽量无遮挡；
- 地面尽量平整，避免一开始就把 LiDAR 近距离反射误判成障碍；
- 单点测试时，前方 10 到 15 米范围尽量无行人、车辆和明显遮挡；
- 首次实测不要直接上复杂路径，先做单 waypoint。

### 3.2 配置检查

先检查两个文件中的 origin 必须一致：

- [config/gps.yaml](/Users/weibin/workspace/University/AUTO4508/workspace/auto_nav_team18/config/gps.yaml)
- [config/waypoints_real_gps.yaml](/Users/weibin/workspace/University/AUTO4508/workspace/auto_nav_team18/config/waypoints_real_gps.yaml)

重点确认：

- `gps_origin_lat`
- `gps_origin_lon`
- `origin.lat`
- `origin.lon`

如果这两处不一致，实时 GPS 和 waypoint 会落在不同坐标系，现场必然表现异常。

### 3.3 waypoint 准备规则

单点测试阶段建议只保留 1 个 waypoint：

- 距离起点约 8 到 15 米；
- 不要太近，否则 GPS 抖动会掩盖导航行为；
- 不要太远，否则第一次试车定位问题成本太高；
- waypoint 前方和两侧尽量开阔，避免把“避障触发”误判成“GPS 导航失败”。

多点测试阶段再恢复完整 waypoint 列表。

---

## 4. 测试分级

真实户外测试按下面顺序执行，禁止跳步：

1. Level A：静态检查，不动车；
2. Level B：GPS 就绪检查，不进 AUTO；
3. Level C：单 waypoint 低风险导航；
4. Level D：GPS 丢失恢复；
5. Level E：多 waypoint 连续导航。

只要某一级失败，就先停在该级定位问题，不要继续后面的测试。

---

## 5. Level A：静态检查

### 5.1 启动命令

推荐使用项目已有脚本启动整套服务：

```bash
cd /Users/weibin/workspace/University/AUTO4508/workspace/auto_nav_team18
./scripts/start_all_services.sh
```

默认行为已经固定为：

- 启动 GPS；
- 启动 IMU；
- 启动相机；
- 使用 `config/waypoints_real_gps.yaml`；
- 拒绝 `waypoints_data.yaml`。

只有在 bench/室内联调时，才使用：

```bash
cd /Users/weibin/workspace/University/AUTO4508/workspace/auto_nav_team18
./scripts/start_all_services.sh bench
```

### 5.2 必查 topic

分别开终端观察：

```bash
ros2 topic echo /control_mode
ros2 topic echo /deadman_ok
ros2 topic echo /emergency_stop
ros2 topic echo /fix
ros2 topic echo /nav/status
ros2 topic echo /nav/odom
```

### 5.3 通过条件

- `/fix` 持续有消息；
- `/nav/odom` 持续更新；
- `/control_mode` 能在手动切换时变化；
- deadman 按下/松开时 `/deadman_ok` 有变化；
- 未进 AUTO 时 `/emergency_stop` 应保持合理状态，不能无故常亮；
- 机器人静止时，`/nav/odom` 不应大幅跳变。

### 5.4 失败时先查什么

- `/fix` 没消息：先查 GPS 设备、串口、天线、`GPS_PORT`；
- `/nav/odom` 没消息：先查 `outdoor_pose_fuser` 是否启动；
- `/emergency_stop=True` 常亮：优先查 LiDAR 是否扫到车体、支架、线缆或近距离地面反射；
- `/control_mode` 正常但后续不动：不要先怀疑控制器，先看 waypoint 是否真的加载成功。

---

## 6. Level B：GPS 就绪检查

这一阶段仍然不让机器人跑，只确认 GPS 导航链路是否 ready。

### 6.1 观察状态

```bash
ros2 topic echo /nav/status
ros2 topic echo /fix
ros2 topic echo /nav/odom
```

### 6.2 通过条件

- 启动后 30 秒内进入 `GPS_READY`；
- `GPS_READY` 后，轻推机器人或缓慢移动时，`/nav/odom` 的位置变化方向基本合理；
- 没有明显的坐标突跳；
- 机器人静止时，位置漂移可接受，不出现大幅来回跳。

### 6.3 失败判定

以下任一现象都不要进入单点导航：

- 长时间停留在 `GPS_ALIGNING`；
- 在无遮挡环境下仍频繁 `GPS_READY` / `GPS_LOST` 来回切；
- `/nav/odom` 坐标方向与实际移动方向明显不一致；
- 刚拿到 GPS fix 就出现数米以上突跳。

---

## 7. Level C：单 waypoint 户外导航

### 7.1 测试设置

- `config/waypoints_real_gps.yaml` 只保留 1 个 waypoint；
- waypoint 距离起点约 8 到 15 米；
- 起步前机器人朝向尽量大致对准目标前方，不要一开始就背向 waypoint；
- 测试人员全程可人工接管。

### 7.2 测试动作

1. 保持 `MANUAL`，确认周围无障碍；
2. 观察 `/nav/status` 已进入 `GPS_READY`；
3. 按规则切到 `AUTO`；
4. 按住 deadman，允许机器人开始导航；
5. 全程同时观察：

```bash
ros2 topic echo /waypoint/current
ros2 topic echo /waypoint/status
ros2 topic echo /nav/odom
ros2 topic echo /cmd_vel_auto
ros2 topic echo /emergency_stop
```

### 7.3 预期行为

- 进入 `AUTO` 后不是立刻 `DONE`；
- `/waypoint/current` 有当前目标输出；
- `/cmd_vel_auto` 有非零控制命令；
- 机器人总体朝 waypoint 前进，而不是原地长时间左右摆头；
- 接近目标后进入 `COARSE_ARRIVED` 或后续接近状态；
- 最终到达该单点测试目标并停止。

### 7.4 现场中止条件

出现以下任一情况，立即切回手动或急停：

- 机器人朝错误方向持续加速；
- 原地连续左右大角度摆动，3 到 5 秒内没有收敛；
- `/emergency_stop=True` 持续拉高；
- heading 明显异常，机器人转向与目标方向矛盾；
- GPS 漂移导致目标方向明显跳变。

### 7.5 单点通过标准

- 成功进入 `AUTO`；
- 成功读取并执行该 waypoint；
- 机器人可持续向目标推进，不是只会转向；
- 无非预期急停；
- 最终单点任务完成，且过程可复现至少 2 次。

---

## 8. Level D：GPS 丢失恢复测试

只在单点导航已经稳定通过后再做。

### 8.1 测试动作

在单 waypoint 导航过程中，短时间遮挡 GPS 天线，然后恢复。

### 8.2 观察项

```bash
ros2 topic echo /nav/status
ros2 topic echo /nav/odom
ros2 topic echo /cmd_vel_auto
```

### 8.3 通过标准

- 状态出现 `GPS_READY -> GPS_LOST -> GPS_READY` 的合理切换；
- GPS 丢失期间机器人行为可控，不出现剧烈跳变；
- 恢复后 `/nav/odom` 不出现明显突跳；
- 不因 GPS 短暂丢失直接导致程序崩溃或控制失稳。

---

## 9. Level E：多 waypoint 连续导航

只在单点与 GPS 丢失恢复都通过后再做。

### 9.1 测试设置

- 恢复完整的 `config/waypoints_real_gps.yaml`；
- 先用 2 个 waypoint 验证，再逐步增加，不建议第一次就全量；
- 确认每个 waypoint 周围有足够转向空间。

### 9.2 观察项

```bash
ros2 topic echo /waypoint/current
ros2 topic echo /waypoint/status
ros2 topic echo /nav/status
ros2 topic echo /emergency_stop
```

### 9.3 通过标准

- waypoint 能按顺序切换；
- 每一段都能稳定朝目标前进；
- 中间没有无故 mission abort；
- 最终完成设定路线。

---

## 10. 推荐记录模板

每次真实外场测试至少记录下面内容，便于后续写 report 和复盘：

- 测试时间：
- 测试地点：
- 天气和天空遮挡情况：
- waypoint 配置文件：
- origin 配置：
- 机器人初始朝向：
- `/nav/status` 状态变化：
- `/waypoint/current`：
- `/waypoint/status`：
- `/nav/odom` 现象：
- `/cmd_vel_auto` 现象：
- `/emergency_stop`：
- deadman 是否正常：
- 实际机器人行为：
- 是否通过：
- 失败现象：
- 当场处理动作：
- 结论：

---

## 11. 常见问题与判定

### 11.1 切到 AUTO 完全不动

优先检查：

- `waypoints_file` 是否为空；
- `/waypoint/current` 是否有输出；
- `/emergency_stop` 是否为 `True`；
- deadman 是否真的有效。

### 11.2 一直左右摇头但不前进

优先检查：

- `/nav/odom` 的 yaw 是否稳定；
- `/local_target` 或局部目标是否在跳；
- waypoint 与 origin 是否配置错误；
- GPS 模式下 heading 是否与真实朝向一致。

### 11.3 明明场地是空的却总急停

优先检查：

- LiDAR 是否扫到车体；
- 是否扫到离地太近的结构；
- 安全距离阈值是否过于保守；
- 近距离反射噪声是否持续进入急停区域。

---

## 12. 最低通过门槛

在当前项目阶段，建议把“GPS 户外导航通过”定义为：

1. 静态链路检查通过；
2. `GPS_READY` 能稳定出现；
3. 单 waypoint 户外导航连续 2 次成功；
4. GPS 短时丢失恢复可接受；
5. 至少 2 个 waypoint 的连续导航可完成。

达到这 5 条后，再继续做更长路线、多障碍、marker 与 mission 联调。
