# 用新手眼结果测试原单 AprilTag 目标

`pregrasp_move` 默认读取 `/home/jetson/charuco_handeye_result_filtered.json`。
其变换是 `6_Link <- camera_optical_frame`，平移约 `[40.433, -48.519, -3.166] mm`，
四元数 xyzw 为 `[-0.00035287, 0.71858406, 0.01712150, 0.69522923]`。

抓取观测目标继续使用 **tag36h11 ID=1、外边长 20 mm 的单 AprilTag**。
手眼标定所用 ChArUco 板的 tag 长度 21 mm 和板原点均不作为抓取目标参数。
程序使用当前图像观测目标，并计算：

```text
T_base_tag = T_base_6_Link @ T_6_Link_camera @ T_camera_tag
```

没有硬编码旧棋盘或旧 tag 在基座下的位置。相机分辨率和 K/D 内参必须与标定时一致。

## 启动条件

相机图像、相机内参、真实关节反馈和 robot_state_publisher 应已启动。
机器人姿态应停稳，相机看到原单 AprilTag。首次观察和规划均不发送运动命令。

```bash
cd /home/jetson/jetcobot_colcon_ws
source install/setup.bash
```

实机测试采用下面三个终端（每个终端先执行上面的 `source`）。相机节点也应保持运行。
**这套流程不要同时启动 `demo.launch.py`**：当前 demo 使用
`mock_components/GenericSystem` 和 `joint_state_broadcaster` 发布模拟关节状态，
会与真实反馈混在 `/joint_states` 上。已运行 demo 时先在它的终端按 Ctrl+C 停止。

终端 1：读取机械臂串口中的真实六轴角度，持续发布 `/joint_states`：

```bash
ros2 run jetcobot_moveit local_joint_camera_control --speed 15
```

该程序启动后即每 0.1 秒读取 `get_angles()` 并发布，不需要按发送键。
第六轴按现有模型加 45° 后发布。测试定位时保持机械臂停稳；`s` 或空格才会发送滑块角度。

终端 2：订阅 `/joint_states`，计算并发布各连杆 TF（它本身不读取串口或发布关节状态）：

```bash
ros2 launch jetcobot_moveit rsp.launch.py
```

终端 3：启动 MoveIt，提供规划、FK 和碰撞检查：

```bash
ros2 launch jetcobot_moveit move_group.launch.py
```

各节点已经运行时不用重复启动。用下面命令确认关节状态只有一个发布者，
且节点名为 `local_joint_camera_control`：

```bash
ros2 topic info /joint_states --verbose
ros2 topic echo /joint_states --once
```

数据流是：机械臂串口 → `local_joint_camera_control` → `/joint_states`
→ `robot_state_publisher` → `/tf`；MoveIt 同时订阅真实 `/joint_states`。

也可以不运行终端 1 的 GUI：
如果 GUI 已关闭且串口没有被其它进程占用，下面的观察/规划命令可以加
`--own-serial`，程序会只读取机械臂角度并发布 `/joint_states`。
此时 `/joint_states` 的唯一发布者是 `pregrasp_move`。
不要同时让两个程序占用 `/dev/ttyUSB0`。执行已规划路径时 `pregrasp_move --execute`
会自行读取并发布真实反馈，所以执行前须关闭终端 1 的 GUI。

## 先看目标位置

```bash
ros2 run jetcobot_moveit pregrasp_move \
  --tag-id 1 --marker-length 0.020 \
  --link jiazhua_Link --height 0.20 \
  --observe-only
```

程序收集 8～10 帧新观测，使用图像时间戳对应的末端 TF，打印 tag 基座坐标及
高位接近目标坐标。它检查图像/内参匹配、重投影误差、tag 位置抖动和末端运动。
若目标距基座原点超过 URDF 连杆长度之和，就直接报告不可达。
位于这个长度界限内并不保证存在满足姿态、关节限位和碰撞约束的解。

## 规划并验证高位接近

```bash
ros2 run jetcobot_moveit pregrasp_move \
  --tag-id 1 --marker-length 0.020 \
  --link jiazhua_Link --height 0.20 --nearby --pitch 0.3
```

这里保留之前高位接近试验的参数：`jiazhua_Link` 原点位于 tag 中心上方
200 mm（基座 Z 方向），工具姿态使用 `--pitch 0.3` rad 的显式设置。
它是到目标上方的试验，不是夹爪指尖接触 tag 或自动闭合夹爪。
可省略 `--pitch` 保持观测时的工具姿态。`--height` 至少为 0.15 m。

程序先求附近 IK，再请求 MoveIt 只规划，检查加密后的路径碰撞与 FK。
默认报告为 `/home/jetson/pregrasp_charuco_handeye_plan.json`，不覆盖以前的
`pregrasp_plan.json`，也不改写任何标定文件。

| 报告字段 | 含义 |
| --- | --- |
| `calibration_file`、`calibration_sha256` | 这次规划使用的手眼文件及校验值 |
| `target_spec` | 实际检测的单 tag ID、家族与尺寸 |
| `tag_position_m` | 新观测的单 tag 中心基座坐标 |
| `target_position_m` | 所选 URDF link 的高位接近目标 |
| `within_reach_bound` | 是否位于连杆总长度的必要界限内 |
| `status=planned`、`path_validated=true` | IK、MoveIt、路径碰撞和 FK 检查已通过 |
| `motion_sent=false` | 观察和规划没有发送实机运动 |

没有检测到目标、目标超界、IK 或碰撞检查失败时不能执行该报告。
每次开始新的观测会先使旧路径失效，避免误用上一次的规划。

## 执行已验证的高位路径

在规划后 180 秒内、机器人及 tag 未移动时，关闭占用串口的 GUI，运行：

```bash
ros2 run jetcobot_moveit pregrasp_move --execute
```

执行默认使用 SDK 速度 5；可用 `--execute --speed 10` 显式改为速度 10，
允许范围 1～15。碰撞检查仍使用每轴步长 <=1° 的加密路径；串口下发时仅合并
同一直线、同一前进方向上的点，名义指令步长 <=3°，所有转折点及反向点保留。
这样避免把每个不足 1° 的点都当成独立停靠动作。实际反馈落后时进一步拆分指令，
确保下一个目标与当前反馈每轴相差 <=3.75°，原有 >4° 停止条件继续生效。
报告的 `command_waypoint_indices` 是合并后的名义指令索引，
`waypoint_feedback` 记录实际执行的索引（可能因反馈落后而增加拆分点）。

每个执行指令要求控制器停止、相邻反馈稳定，并连续两次满足
每轴误差 <2°，才进入下一点。若控制器已停止而误差仍 >=2°，最多重发两次同一目标，
不会跳过该点；总等待仍限制在 12 秒，任一轴超差 >4° 立即停止。
报告中的 `waypoint_feedback` 保存各点的指令、反馈、各轴误差、运动状态和重试次数。
速度 5 下短距离指令未达标时可以试速度 10，但不能保证消除机械死区、回差或负载问题。
执行失败后应从实际姿态重新观察并规划；已执行过的报告不能直接重试。
到终点后，还会用真实反馈角度调用 MoveIt FK；末端模型位置与目标相差 >6 mm 时
报告停止，不能仅凭关节角 <2° 就报告到达。这个 FK 检查使用 URDF 模型，
不等同于外部测量的实物位置精度。

它只执行通过检查的高位路径，
不会下降或闭合夹爪。报告中的 `execution_status` 与最终关节反馈用于判断是否到位。
环境检查目前包含模型自碰撞及相对 tag 高度约束；桌面、周围物体没有实测碰撞模型。
`jiazhua_Link` 是模型坐标原点，夹爪指尖 TCP 尚未标定，所以不能将其等同于实际抓取点。
