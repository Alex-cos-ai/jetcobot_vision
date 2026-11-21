# JetCobot 实时颜色 PID 跟踪

运行两个文件：`tracking_main.py` 负责摄像头、识别、外接圆测距、PID、六轴规划和本地窗口；`sync_plan.py` 独立负责真实关节反馈及可选串口执行。主文件内部加载 launch，自动启动所选求解器，无需另外运行相机、Servo 或看图程序。

**默认预览，不向机械臂发送运动、停止或模式设置指令。** sync 在预览中只读串口并校验规划。跟踪会持续运行，居中、丢失目标都不会结束程序；关闭窗口、按窗口内的 `q` / Esc 或主终端 Ctrl+C 退出。

2026-11-21 已完成蓝色30mm方块实机验证：残缺目标经倾转搜索恢复完整，随后位置PID收敛至估计距离197.2mm，左右/上下误差为0.6mm/-1.9mm。六轴均有实际反馈位移。记录见 [实机验证](validation/hardware_tracking/README.md)。本机执行验证标志已核对，默认启动仍为只读预览，真实运动需显式选择执行模式。Yahboom官方URDF、SRDF和网格保持原样。

## 1. 构建和启动

使用本机系统 Python `/usr/bin/python3`。构建一次：

```bash
cd /home/jetson/jetcobot_colcon_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select jetcobot_driver --symlink-install
```

两个终端都先加载相同 ROS 环境：

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/jetcobot_colcon_ws/install/setup.bash
```

终端一，运行规划主文件：

```bash
python3 /home/jetson/jetcobot_colcon_ws/src/jetcobot_driver/jetcobot_driver/tracking_main.py
```

主文件打开 `/dev/video0`，弹出处理画面。尚未启动 sync 时，先识别并显示 `WAIT_SYNC` / 等待关节反馈，不假设关节全零。

终端二，运行你指定的 sync 文件：

```bash
python3 /home/jetson/jetcobot_colcon_ws/src/jetcobot_driver/jetcobot_driver/sync_plan.py
```

此时默认 `preview`，开始读取 `/dev/ttyUSB0`，主文件持续计算下一步六轴角度，sync 校验并回报接受状态。**不会驱动机械臂。** 只保留这一套流程；串口被已有手动工具、旧 sync 或硬件 launch 占用时，本流程会拒绝重复打开。

本机开始实际跟踪时，将终端二改为：

```bash
python3 /home/jetson/jetcobot_colcon_ws/src/jetcobot_driver/jetcobot_driver/sync_plan.py --mode execute
```

如果 `yahboom_app.service` 占用串口，先运行 `sudo systemctl stop yahboom_app.service`。它是蓝牙/手柄后台服务，启动时会发送初始化动作，不能与sync并行。本次调试只停止服务，没有禁用开机启动；重启设备后需重新检查占用。

也可使用安装入口 `ros2 run jetcobot_driver tracking_main` 和 `ros2 run jetcobot_driver sync_plan`。`--help` 查看参数。配置修改后两个进程都要重启并加载同一文件，避免参数摘要不一致。

## 2. 看图和调试

- 窗口显示所有识别框；选中目标为黄色框和真实轮廓外接圆。
- `r` / `g` / `b` 分别锁定红 / 绿 / 蓝；`a` 自动选色。也可点击识别框选择该颜色，连续实例按位置邻近关联。
- 十字为期望像素 `[320, 240]`。箭头是**相机平移方向**，图中右为相机 x 正、下为相机 y 正；前后方向看 z 速度符号，正表示向目标靠近。
- 完整方块用四边形对角线交点作为对准中心（红十字），外接圆圆心只用于画圆；白十字是画面中心。截边时用可见区域引导相机倾转，保持相机位置，暂停前后距离控制；完整后恢复XYZ PID测距跟踪。
- `Depth estimate` 为测距，`Error camera mm` 为左右/上下/前后尚需消除的距离；`Camera motion mm/s` 为请求速度。截边搜索时显示相机倾转角速度。这些数值不代表执行器已经移动。
- 主文件终端每 **2 秒**打印六轴表：实测、规划、实际发送SDK角度、目标与实测差、角速度。均用度显示，第六轴已扣除项目原有的45°模型偏置。`sent_ns`持续更新且关节反馈变化才代表实际执行；六轴协同不要求每轴每周期都有明显位移。
- `accepted` 为 sync 接受的目标时间标识，`sent` 为实际发送标识。预览的 `sent=0`、`motion enabled=False` 是正常现象。

控制计算 20 Hz，目标发布和串口反馈 10 Hz；日志 0.5 Hz。打印不会阻塞控制。六轴由统一运动学协同求解，不要求每轴始终运动。预览时关节不动，因此终端可能反复给出相近的小步建议，这是预期行为。

短时丢失目标（<0.5 s）衰减速度，随后清空 PID 并保持；重新识别后软启动。HOME 默认关闭，保持感知。可选 `runtime.home_on_lost` 在丢失 5 s 后尝试受约束的小步回到启动实测姿态，途中没有可行步时保持；它不是完整的绕障回家路径规划。

## 3. 外接圆测距及坐标约定

使用已有 HSV 分割，提取轮廓后调用 `cv2.minEnclosingCircle`，不以 bbox 宽度代替半径。先按现有标定去畸变到相同 fx/fy 的图像，圆半径、焦距和像素误差都使用该坐标系。

对于边长 W=30 mm、近似正对相机的正方形，其物理外接圆直径是 `sqrt(2) W = 42.426 mm`：

```text
Z = f * sqrt(2) * W / (2r)
ex = (cx - 320) * Z / f
ey = (cy - 240) * Z / f
ez = Z - 200 mm
```

已有标定主点与画面中心不同，控制器通过归一化参考射线补偿前后移动引入的横纵耦合。倾斜、遮挡、反光、物体尺寸错误会影响测距。超出画面的方块参与倾转搜索，但暂停距离控制；只有完整外接圆恢复后才更新距离。该输入不能观测目标完整的三维姿态。

速度先在相机坐标系生成，再用当前 FK 与手眼旋转转换到基座。Servo 路径还补偿相机到工具原点的平移偏置，避免把相机线速度直接当成工具原点线速度。

## 4. 参数集中在这里

编辑 [config/visual_servo.yaml](config/visual_servo.yaml)，顶层为 `tracking:`。这是应用共用 YAML，通过 Python 显式加载，不能直接作为 ROS `--params-file`。

| 参数位置 | 当前初值 | 调试用途 |
| --- | --- | --- |
| `pid.kp` | `[0.4, 0.4, 0.25]` 1/s | 顺序 x / y / z；优先只调比例 |
| `pid.ki` / `pid.kd` | 全零 | 方向、比例稳定后再增加；分别为 1/s²、无量纲 |
| `pid.deadband_m` | `[.003, .003, .005]` m | 居中抖动时按测量噪声调整 |
| `pid.max_speed_m_s` | `.01`，即 10 mm/s | 当前速度上限；调平滑时先保持此上限 |
| `pid.max_acceleration_m_s2` | `.03` | 末端加速平滑 |
| `pid.input_tau_s` / `depth_tau_s` | `.10` / `.15` s | 像素与半径滤波、深度二次滤波 |
| `pid.derivative_tau_s` | `.10` s | 测量微分滤波 |
| `ik.lambda_max` / `epsilon` | `.1` / `.2` | 仅自研 DLS；平移 Jacobian 先除以 Lref=.1 m，阈值对应归一化后的加权矩阵 |
| `ik.centering_gain` | `.02` | 关节居中项，替代含混的统一 k0 |
| `ik.manipulability_gain` | `.0001` | 对数位置可操作度梯度；`secondary_speed_rad_s=.02` 限制次任务总预算 |
| `model.max_velocity_rad_s` | 六轴均 `.1`，约 5.73°/s | 命令参考递增上限；实际舵机需同时观察反馈 |
| `model.max_acceleration_rad_s2` | 六轴均 `.25` | 关节命令加速度限制；降低可缓和起停 |
| `model.margin_deg` | `5`° | 在 URDF 限位内再留余量 |
| `serial.max_step_rad` / `speed_max` | `.07` / `5` | 目标最多领先反馈约4°，越限时整体停止积累；等级不是rad/s |
| `serial.speed_filter_tau_s` / `speed_change_interval_s` | `.25` / `.25` s | SDK速度等级先滤波，再每次至多升降一级 |
| `pid.settle_exit_ratio` | `1.5` | 居中保持的滞回，防止死区边缘噪声反复启停 |
| `safety.table_z_m` / `table_clearance_m` | `-.01` / `.06` m | 实测桌面及完整运动网格最低点的安全间隙 |
| `safety.max_tracking_excursion_m` | `.15` m | 启动位置周围150mm，仍受绝对工作空间和碰撞约束 |
| `runtime.log_period_s` | `2` s | 只影响打印周期 |

PID 包含实际 dt 积分、条件抗饱和、积分限幅、死区衰减、测量微分低通、速度及加速度限幅。安全层限制运动时暂停积分。

平滑运动还需避免无效启停：DLS从允许时差内的实测关节直接FK，不再等待同一数据经TF发布后回传；timer补调产生的异常dt跳过本次计算，不当成目标丢失。短暂停顿仅重置输出历史，保留仍然新鲜的测量，避免暂停→清测量→再次暂停的循环。超时和碰撞仍停止，不能通过放大timeout掩盖。

死区采用连续输出 `sign(e)*max(|e|-deadband,0)`。三通道进入3/3/5mm死区后保持，只有超出4.5/4.5/7.5mm才重新跟踪；因此小于这些阈值的目标位移可能保持不动。末端加速度当前.03m/s²，关节加速度.25rad/s²，调整后必须重启主文件和sync以加载同一份配置。串口位置舵机仍有死区及离散速度等级，这些改动改善软件造成的启停，不能保证连续速度伺服的硬件表现。

## 5. 后端、运动学和保护

默认使用Weighted DLS位置主任务，截边目标以5维任务保持相机位置并控制两个倾转轴，避免伸直姿态下6维Servo的奇异点急停。完整目标恢复后保持新的相机姿态并运行XYZ PID。显式启动另一MoveIt Servo后端：

```bash
python3 /home/jetson/jetcobot_colcon_ws/src/jetcobot_driver/jetcobot_driver/tracking_main.py --ik-backend moveit_servo
```

`kinematics.py` 从现有 URDF 的关节原点/轴提取 FK 与相机、末端几何 Jacobian；没有复制 DH 数字。DLS 按最小奇异值自适应阻尼，限位权重增大，位置主任务使用严格 SVD 零空间容纳姿态偏好、关节居中、位置可操作度项。阻尼逆的 `I-J⁺J` 不是真零空间，因此没有直接用它宣称严格任务优先。关节触限时冻结并重解剩余关节；6D 主任务可通过 `ik.full_pose` 在 DLS 后端启用，此时没有通用的三维冗余。

共同保护层检查工作空间盒/半径、关节角/速度/加速度、桌面与胶囊体连杆距离、步长和中间采样姿态。边界附近投影掉不可行方向，尽量保留可行动作；约束冲突、初始姿态已违规或最终步校验失败时保持。不能承诺任意目标均可达或任意奇异点均可穿过。

Servo 默认启动自身的主场景监视器，不需要 `move_group` 及其执行管理器。桌面节点经 `/collision_object` 发布几何，再通过 `/get_planning_scene` 回读位置和尺寸确认。没有配置桌面时不会伪造真实障碍物。

专用协议使用最新单点、真实时间戳、会话 ID、配置摘要、生成时间匹配和心跳。主文件退出、过期反馈、过期目标、冲突发布者或串口失败都会阻止发送；已执行的进程发生故障后锁定，排除原因后重新运行 sync。预览从不调用 `send_angles`、`stop` 或 `set_fresh_mode`。

## 6. 真机前核对与调参顺序

本机现已完成SDK限位读取、多姿态FK和低速运动验证，`model.verified`、`safety.verified`、`alignment_verified`均为true，桌面z=-.01m。每次execute还会复检FK、SDK限位和启动姿态。更换硬件/工具/安装位置时，必须重新核对，不能沿用此机器的校准结论。

1. **FK 自检**：先退出占用串口的 sync，再运行下面的一次性只读检查。它按“关节角—SDK坐标—关节角”采样，机械臂移动时拒绝比较。

   ```bash
   python3 /home/jetson/jetcobot_colcon_ws/src/jetcobot_driver/jetcobot_driver/sync_plan.py --fk-check
   ```

   核对 `fk_check` 的 SDK 基座/工具对齐矩阵及机体ZYX约定 `Rz Ry Rx`；本机末端变换已由多姿态数据拟合，保存在应用YAML，未修改官方模型。阈值为10mm/5°。SDK坐标自检不能替代独立实测。
2. **单通道预览**：在 `pid.kp` 中每次只保留一个非零，Ki/Kd 保持零。移动方块，查看箭头、距离正负和六轴增量；只关一个 Kp 并不等于冻结某个关节。
3. **核对保护参数**：填写实测桌面 z（基座系）、工作空间、连杆和相机/夹爪包络；核对允许碰撞对，默认 `[4,6]` 只是腕部安装连接段的待测设置。固定在桌面的基座连接段如需豁免，要按实机明确配置，不能整套关闭桌面保护。核对 SDK 的真实限位、连续小步响应、读写延迟和速度等级。
4. **小增益真机测试**：以上检查完成后再设置模型与几何验证标志，重启主文件；显式运行 `sync_plan.py --mode execute`。它仍须先取得健康规划、确认替换旧指令的 fresh mode，并等待启用之后的新目标。YAML 中的 `serial.mode` 不会使无参数启动自动执行。
5. **逐步增加范围与增益**：先静态小偏差，再缓慢正弦，最后阶跃和较大范围；每次只改一类参数。桌面、限位、奇异和通信保护必须保留。串口角度控制不能提供工业伺服级硬实时同步保证。

默认关闭强光轴瞄准/无滚转复合目标，使用初始姿态保持，避免与平移 PID 同时纠正同一像素误差。它作为后续需额外设计和验证的模式；当前不声称已实现完整的目标姿态跟踪。

## 7. 无设备验证与曲线

不用启动 sync、不打开摄像头或串口，运行本地模拟画面：

```bash
python3 /home/jetson/jetcobot_colcon_ws/src/jetcobot_driver/jetcobot_driver/tracking_main.py --feedback-source simulation --synthetic-camera --ik-backend weighted_dls
```

默认仍有窗口，`--headless` 可关闭窗口。模拟图像按运动学投影世界目标，模拟关节消费规划形成闭环；不要同时启动真实 sync。换 `--ik-backend moveit_servo` 可验证 Servo 路径。

离线仿真与画图（依赖 numpy、OpenCV、PyYAML、matplotlib，无 ROS 运行或串口依赖）：

```bash
ros2 run jetcobot_driver simulate_tracking --trajectory sine --duration 20 --output /tmp/tracking_sine
ros2 run jetcobot_driver simulate_tracking --trajectory step --duration 20 --output /tmp/tracking_step
ros2 run jetcobot_driver simulate_tracking --trajectory boundary --duration 20 --output /tmp/tracking_boundary
ros2 run jetcobot_driver simulate_tracking --trajectory singular --duration 20 --output /tmp/tracking_singular
```

也可用 `PYTHONPATH=/home/jetson/jetcobot_colcon_ws/src/jetcobot_driver python3 -m visual_servo_6dof.simulate_tracking ...` 直接运行离线模块。

输出包括关节角、相机轨迹、归一化 σ_min、剩余限位余量、速度与误差图，以及 3D 轨迹、CSV、NPZ 和 JSON 摘要。四种轨迹均包含短时与长时遮挡/重捕获；singular 使用安全工作空间内的近奇异构型并启用 6D DLS，不以无效的全零初始姿态作为安全测试。

离线验收结果与图见 [validation/README.md](validation/README.md)；本机实测另见 [hardware_tracking](validation/hardware_tracking/README.md)。SDK send_angles对很小的绝对角变化存在明显死区，驱动使用有界命令参考跨过死区；任一轴跟随预算不足时，所有轴的命令增量统一缩放。视觉短暂停顿时保持命令参考，恢复后按真实时间间隔验证增量。

单元/回归测试（先加载上面的 ROS 环境）：

```bash
cd /home/jetson/jetcobot_colcon_ws
PYTHONPATH=src/jetcobot_driver:$PYTHONPATH python3 -m pytest src/jetcobot_driver/test/test_tracking_pipeline.py src/jetcobot_driver/test/test_visual_servo_control.py src/jetcobot_driver/test/test_servo_serial_bridge.py -q
RUN_TRACKING_LAUNCH_TESTS=1 ROS_DOMAIN_ID=91 PYTHONPATH=src/jetcobot_driver:$PYTHONPATH python3 -m pytest src/jetcobot_driver/test/test_tracking_launch.py -q
```

## 8. 问题排查

| 现象 | 原因与调整位置 |
| --- | --- |
| 静止时抖动 | 检查光照、轮廓半径噪声；增大死区或滤波时间常数，降低 Kp；Kd 从零开始 |
| 超调 | 降低 Kp/Ki 和速度上限；确认测距尺寸、SDK响应、命令延迟；必要时少量增加滤波后的 Kd |
| 持续振荡 | 检查轴方向及手眼关系、Ki积累、反馈延迟；降低增益、减轻滤波滞后 |
| 滞后 | 检查图像时间戳和串口吞吐；适当减小滤波时间常数，逐步增加 Kp，不能靠积压指令追赶 |
| 奇异附近减速/保持 | 查看 Servo 状态或 DLS σ_min；保持有限阻尼、次任务限速，退出后通过已验证方式调整实际姿态 |
| 关节卡在限位 | 目标可能不可达；检查真实范围、5°余量、DLS居中权重；允许降级或保持，不能提高限位硬冲 |
| 姿态漂移 | 位置主任务的次任务预算不足或约束冲突；检查相机标定、姿态参考及 `orientation_gain`，必要时另测6D模式 |
| 构型翻转/角度跳变 | 检查SDK/模型转换与第六轴偏置、关节名、过期反馈；本实现不直接求最终点IK，而从实测角逐步积分 |
| 等待反馈/没有轴表 | 运行独立sync、检查串口占用及实际关节值；主文件只识别时不会编造规划 |
| `configuration_mismatch` | 两个入口使用相同YAML、URDF和标定文件并同时重启 |
| 有框但不控制 | 检查截边、测距范围、内参一致性、图像时效及图像时刻TF；框本身不代表通过控制校验 |
| 场景一直未就绪 | 填写真实 `table_z_m`，检查Servo主场景服务与回读；未验证几何时保留预览 |
| sync故障锁定 | 先排除心跳/反馈/目标时效或限位原因，再重启sync；不会自动恢复实机运动 |

旧 `hardware_visual_servo.launch.py` / `servo_serial_bridge.py` 为原有兼容入口，新流程不使用它们。不要与上述两个文件并行启动。`control.py` 保留旧 LQR 数学函数仅供兼容回归，当前跟踪节点使用 `tracking_pid.py`。
