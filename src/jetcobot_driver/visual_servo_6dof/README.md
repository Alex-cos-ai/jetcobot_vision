# JetCobot 六轴颜色跟随

识别夹爪相机中的红、绿、蓝色区域。相机随目标转向，使目标回到画面中心。
图像尺寸决定沿目标视线靠近或后退的速度。MoveIt Servo 将相机运动转换成六关节目标。

## 已定位的“不跟随”原因

- 原模拟初始姿态六个关节全为零。Servo 实际返回状态 `2`，日志为
  `Very close to a singularity, emergency stop`，因此一直输出停止轨迹。
- 原来的控制器使用 `mock_components/GenericSystem`，未连接机械臂串口。
- 原 LQR 的加速度模型和 Twist 速度接口不一致，而且相机轴向靠名称假设。

现已加入弯肘仿真姿态、匹配本机尺寸的有限奇异点阈值、速度输入 LQR、
已有手眼标定变换，以及默认禁止运动的真机串口桥接节点。
没有关闭碰撞和奇异点保护。全伸直姿态仍然会触发停机。

## 构建与公共环境

构建一次：

```bash
cd /home/jetson/jetcobot_colcon_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select jetcobot_driver --symlink-install
```

每个新终端先执行：

```bash
cd /home/jetson/jetcobot_colcon_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

同一种节点只启动一次。下面的“仿真”和“真机”选一套，不能同时运行。
切换前在旧终端中按 Ctrl+C 关闭 demo、Servo 和颜色跟随节点。
不要同时运行 `sync_plan`、`sync_plan_nx`、其它串口控制器或多个关节反馈发布者。

## 相机：两套流程通用

终端 1：

```bash
ros2 launch jetcobot_driver wrist_camera.launch.py
```

已通过本 launch 启动相机时保留现有节点即可。请退出旧的 `ros2 run usb_cam ...`。
本机设备为 `/dev/video0`，使用
YUYV、640×480、30 fps，加载已有内参
`/home/jetson/.ros/camera_info/wrist_camera.yaml`。
图像和内参的帧名均为 `camera_optical_frame`。

## 仿真：验证 RViz 模型响应

终端 2：使用新的弯肘初始姿态，替换原来的 `demo.launch.py`：

```bash
ros2 launch jetcobot_driver visual_servo_demo.launch.py
```

终端 3：

```bash
ros2 launch jetcobot_driver moveit_servo.launch.py
```

终端 4：

```bash
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger '{}'
ros2 launch jetcobot_driver visual_servo.launch.py
```

模拟控制器只移动 RViz 中的模型，不能带动物理相机。
这套流程用于验证颜色识别和运动响应，不能作为实物视觉闭环的验证。

## 真机：真实反馈与串口执行

画面同时有多个色块时，默认 `target_colour:=auto` 跟随最大色块。要固定跟蓝色：

```bash
ros2 launch jetcobot_driver hardware_visual_servo.launch.py start_camera:=true target_colour:=blue
```

也可将 `blue` 改成 `red` 或 `green`。请先在调试图像上确认目标，再启用实物运动。

先关闭上述仿真节点，保留终端 1 的相机。
本次读到的真机姿态也接近伸直；首次跟随前，先使用已有手动工具调整为弯肘观察姿态：

```bash
ros2 run jetcobot_moveit local_joint_camera_control --speed 10
```

通过界面调整姿态并使相机朝向目标后，关闭手动工具，释放 `/dev/ttyUSB0`。
不要把仿真初始关节角直接当成真机运动指令。

终端 2：启动真实反馈、TF、MoveIt、Servo、颜色跟随和 RViz：

```bash
ros2 launch jetcobot_driver hardware_visual_servo.launch.py
```

如果相机也尚未启动，可以用以下命令一并启动，代替终端 1：

```bash
ros2 launch jetcobot_driver hardware_visual_servo.launch.py start_camera:=true
```

串口桥接默认**只读取真实关节角，不发送运动命令**。
`/joint_states` 应只有 `servo_serial_bridge` 一个发布者。还要确认能收到六个关节位置，
此时桥接状态为 `feedback_only_motion_disabled`。

```bash
ros2 topic info /joint_states --verbose
ros2 topic echo /joint_states --once
ros2 topic echo /servo_serial_bridge/status --once
```

终端 3：启用 Servo，检查识别和运动保护状态：

```bash
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger '{}'
ros2 topic echo /visual_servo_6dof/status --once
ros2 topic echo /servo_node/status --once
```

颜色状态应为 `tracking_red`、`tracking_green` 或 `tracking_blue`。
`waiting_for_timestamped_tool_tf` 表示图像尚未配到同一时刻的真实工具姿态。
不要额外启动 `visual_servo.launch.py`，真机 launch 已包含该节点。

Servo 状态含义：`0` 正常，`1`/`6` 奇异点附近减速，`2` 奇异点停机，
`3` 碰撞附近减速，`4` 碰撞停机，`5` 关节限位停机。
状态 `2` 需要先退出这套流程，使用手动工具调整实际姿态后重启，
不能通过不断提高阈值来绕过。

确认当前识别及姿态后，以下命令开始允许实物运动：

```bash
ros2 service call /servo_serial_bridge/enable_motion std_srvs/srv/SetBool '{data: true}'
```

必须返回 `success: true`。如果旧版本返回 `Requires real feedback only; other publishers=[]`，
空列表表示没有检测到其他关节发布者，该次请求因缺少有效的实物反馈被拒绝。
新版会显示具体原因。`No valid hardware joint feedback` 表示最近一次串口读取失败或还在等待首帧，
`Hardware joint feedback is stale` 表示最后一帧已过期，
`Conflicting /joint_states publishers` 会列出需要关闭的额外发布者。

停止实物跟随：

```bash
ros2 service call /servo_serial_bridge/enable_motion std_srvs/srv/SetBool '{data: false}'
```

关闭硬件 launch 也会在启用过运动时发送停止指令。
串口故障、指令超时、Servo 停机或目标关节超差后，运动开关自动关闭，
须排除原因后重新启用。

## 看图与状态

```bash
ros2 run rqt_image_view rqt_image_view
```

选择 `/visual_servo_6dof/debug_image`，或原图 `/wrist_camera/image_raw`。
检查命令按需单独运行；持续显示的命令按 Ctrl+C 退出：

```bash
ros2 topic echo /visual_servo_6dof/status
ros2 topic echo /servo_node/status
ros2 topic echo /servo_serial_bridge/status
ros2 topic echo /joint_states --once
```

## 自查相机时间同步

保持完整真机 launch 运行，在另一个已加载工作空间的终端执行：

```bash
ros2 run jetcobot_driver check_camera_sync
```

此命令只读采样约 6 秒。它显示图像和关节反馈发布者、帧数、时间戳差、图像时刻的工具 TF
匹配情况、跟踪状态，以及启用运动服务是否存在。命令本身不改变机械臂的运动状态。

`时间戳差 = 接收时的 ROS 时间 - 消息 header.stamp`。图像需在 0～350 ms 内，
这个值还受驱动时间戳是否准确影响，不能直接当成相机的物理延迟。
本机已修正相机的只读联调通常约 40～70 ms；负载和曝光变化会影响结果。
调试图像顶部也显示跟踪状态和时间戳差，检测框并不代表该图像通过了时效检查。

也可逐项检查：

```bash
ros2 topic info /wrist_camera/image_raw --verbose
ros2 topic delay /wrist_camera/image_raw
ros2 topic hz /wrist_camera/image_raw
ros2 topic info /joint_states --verbose
ros2 service list | rg 'enable_motion|start_servo'
fuser -v /dev/video0 /dev/ttyUSB0
```

`delay` 和 `hz` 会持续显示，按 Ctrl+C 退出。图像有帧率并不能证明时间戳正常。
此处测量的是发布者到这个命令的表现；若跟踪器自身负载高，可能还会有额外处理延迟。

| 现象 | 如何处理 |
| --- | --- |
| 发布者仍是 `usb_cam`，时间戳差约 0.7～0.8 秒 | 本机旧驱动有时间戳换算错误。退出旧相机，使用 `wrist_camera.launch.py` 或真机 launch 的 `start_camera:=true`。 |
| 发布者超过一个，或出现两个跟踪器 | 退出旧 launch 和单独启动的跟踪器，只保留一套真机流程。 |
| 没有关节数据，`enable_motion` 服务也不存在 | 串口桥接没有运行。检查启动报错，退出占用串口的手动工具后重启真机 launch。 |
| 图像时间戳差正常，但一直等工具 TF | 检查真实关节反馈、`robot_state_publisher` 和 TF 链；真机各节点 `use_sim_time` 应为 `false`。 |
| 新相机的时间戳差也持续增大、到达率明显下降 | 退出重复的图像节点，检查 CPU/USB 负载、曝光和消息堆积，再测量。 |
| 图像来自未来，或重启后偏移跳变 | 检查各节点是否使用同一时钟；跨主机时检查系统时钟同步，真机勿混用仿真时钟。 |

先关闭运动再排查：

```bash
ros2 service call /servo_serial_bridge/enable_motion std_srvs/srv/SetBool '{data: false}'
```

服务不存在时检查桥接进程和串口占用。退出冲突进程后，仅启动：

```bash
ros2 launch jetcobot_driver hardware_visual_servo.launch.py start_camera:=true target_colour:=blue
```

启动现在会检查设备占用；相机或串口桥接退出时也会结束该套流程。
恢复后先运行诊断、确认 `tracking_blue`，再启动 Servo 并显式启用运动。

## 模型、参数及实际限制

状态有三个量。前两个是从画面中心射线转到目标射线的角度误差，最后一个是由尺寸估计的距离误差。
两条射线由相机内参和去畸变后的像素位置计算，目标位置使用画面的几何中心。
例如目标偏右且大小合适时，控制器让相机朝右转，目标会往画面中心移。

```text
e = [绕两个瞄准轴的角度误差, 估计距离 - 期望距离]
u = [绕两个瞄准轴的角速度, 沿目标视线的线速度]
e_dot ≈ -u
A = I, B = -dt I
u = -K e
```

将 LQR 输出映射成相机的线速度和角速度，再按手眼标定转换到 `6_Link`。
转动时会补偿相机与 `6_Link` 的位置差，使镜头尽量绕相机自身中心转动。
Servo 根据实时关节角求六轴运动。控制器等待与图像时间匹配的工具 TF 后才使用该帧。
默认读取 `/home/jetson/charuco_handeye_result_filtered.json`，并检查
相机 K/D、分辨率和标定一致。该标定文件由现有 ChArUco 流程生成。

- 六轴机械臂执行相机的瞄准和远近调节。控制器使用颜色块中心和大小，目标自身的三维姿态需要额外的观测。
- 图像尺寸用于同一物体的相对远近控制，不是独立的绝对深度测量。
  更换物体大小或改变其朝向会影响距离估计。
- `visual_servo.yaml` 默认最大平移速度 0.01 m/s，角速度 0.04 rad/s。
- 真机串口以 10 Hz 读取真实角度并发送最近的单点 Servo 目标，SDK 速度 10，
  每轴目标与真实反馈的差不超过 0.035 rad（约 2°）。不接收完整规划轨迹。
  它是为 Servo 提供的专用串口接口，不实现 RViz 普通规划执行所需的
  `FollowJointTrajectory` action；SDK 也不提供硬实时速度控制。
- 桥接保留模型第六轴的 45° 偏置，发送前执行逆转换。
- MoveIt 使用已有碰撞模型，尚不包含实测的桌面和周围物体模型。

需要清空上一帧的误差和滤波历史时，先关闭实物运动，再调用：

```bash
ros2 service call /visual_servo_6dof/reset_reference std_srvs/srv/Trigger '{}'
```

## 验证记录

已测试瞄准和远近的控制方向、闭环收敛、手眼速度转换、关节顺序、
第六轴偏置，以及禁用运动、错误步长、指令超时和奇异点停机时不发送 SDK 目标。
已读取实际串口反馈，验证默认运动开关关闭。
仿真注入颜色图像后，Servo 状态正常且模拟关节产生运动。
本次未启用真机运动，实物跟随精度和动态响应仍需按上述流程测量。
