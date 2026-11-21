# JetCobot 实时颜色跟踪改造提示词

以下正文可直接复制给负责修改本项目的编程助手。它基于当前工作空间检查结果编写；它保留最初的实施要求；当前实现与可直接运行的命令见 [README](README.md)。

本版按最新要求采用两个独立入口：先运行 `tracking_main.py` 启动感知与规划，再运行 `sync_plan.py` 接收规划并负责反馈/执行。`tracking_main.py` 和改造后的 `sync_plan.py` 已实现，两个后端均已完成模拟集成验证。默认先验证只读预览，真机执行由 sync 的显式模式选择控制。可行性与已核对的证据见[可行性复核](PID_TRACKING_FEASIBILITY.md)。

---

你是一名熟悉 ROS 2 Humble、MoveIt Servo、视觉伺服、PID 和机械臂微分逆运动学的机器人控制工程师。请在现有项目中实施改造，交付可以构建和运行的代码、配置、测试和中文说明。

工作空间：`/home/jetson/jetcobot_colcon_ws`。
设备：Yahboom JetCobot 六轴机械臂，myCobot 280 类结构，pymycobot 串口接口，末端相机。

## 一、任务优先级与当前阶段

1. 将现有颜色跟踪链路的 LQR 替换为 PID，继续复用已有摄像头、颜色检测、相机标定、手眼变换、ROS 话题和串口桥接。
2. 第一阶段默认只做实时预览：摄像头实时识别颜色方块，在同一窗口显示选中目标、外接圆、运动方向和距离；连续求解六个关节应该如何移动；每 2 秒在启动终端输出一次清晰的关节角表。允许只读机械臂反馈，禁止实际驱动机械臂。
3. 用户先运行主文件 `src/jetcobot_driver/jetcobot_driver/tracking_main.py`，由它内部加载规划侧 launch，启动摄像头、识别、PID、关节解算、本地图像弹窗和日志；再单独运行 `src/jetcobot_driver/jetcobot_driver/sync_plan.py`，由 sync 接收主文件发布的规划信息、读取真实关节并按所选模式执行。主文件不自动启动 sync，不打开串口；sync 不重复启动摄像头、规划或窗口。不要求用户额外手动启动 launch、Servo、图像查看器或调用服务。
4. 程序必须持续跟踪，直到用户按 Ctrl+C、窗口内退出键或主动关闭程序。不能在达到目标、输出一次角度、目标短暂消失、完成一段轨迹或运行固定时长后自动退出。目标居中后保持并持续检测；目标再次移动时自动继续跟踪。
5. 后续单独运行 sync 并显式选择真机执行模式时，由 sync 在设备、反馈和保护就绪后自动开启执行门控，持续接收并发送最新的小步关节目标。主文件不负责替用户开启 sync 的运动权限。控制循环和串口发送不能被“每 2 秒打印一次日志”限速；日志与控制分别计时。默认仍为本阶段要求的预览模式。
6. 分两阶段完成：先让复用 MoveIt Servo 的实时预览链路可验收，再补齐独立的 Weighted DLS 后端和完整安全约束。完整交付须逐项说明两阶段的实现状态，不能把现有 Servo 当成已经实现本文指定的 Weighted DLS、严格零空间和滑墙算法。

阶段验收边界：

| 能力 | 第一阶段 | 第二阶段 |
| --- | --- | --- |
| 两文件启动、相机/弹窗、外接圆、PID、持续运行、每2秒关节表 | 必须完成 | 保持相同接口 |
| 关节解算 | 复用 Servo，三通道平移 + 姿态保持的六维约束 | 独立 Weighted DLS，三维位置主任务 + 有预算的次任务 |
| 运动保护 | Servo 原有保护、真实桌面场景、工作空间/最终目标约束和 sync 独立看门狗；按需提前实现 FK 工具 | 进一步实现胶囊体/连杆约束、活动集重解和可行方向滑墙 |
| 真机执行 | 提供模式与门控；实际执行前必须补齐本机模型/桌面/有效限位并验证，文档审查不代替验证 | 同样需要模型和实机验证；切换后端不能绕过门控 |
| 6D模式、瞄准姿态、HOME | 默认关闭，不作为首轮预览前置条件 | 作为独立可选功能，启用后另行验证 |

六轴“协同”指由统一运动学求解其需要的动作，不要求六个关节每周期都非零。有限机械臂不能对任意大位移或任意目标保证永远可达；遇到无可行运动、安全边界或奇异保护应允许减速/保持，同时保持感知和程序运行。

## 二、先阅读并复用的现有文件

| 文件 | 当前作用 | 主要改造点 |
| --- | --- | --- |
| `src/jetcobot_driver/jetcobot_driver/tracking_main.py`（新增） | 当前没有这个入口 | 规划侧主文件，内部启动规划 launch，管理摄像头、控制、解算与本地窗口，等待 sync 反馈并发布小步目标 |
| `src/jetcobot_driver/jetcobot_driver/sync_plan.py` | 旧 `MycobotReceiver` 订阅 `/joint_states`，施加第六轴偏置后直接 `send_radians(..., 80)` | 改为独立执行入口，复用桥接类读取反馈、订阅专用规划指令、校验并执行；移除旧反馈转指令回路与固定 speed=80 |
| `src/jetcobot_driver/visual_servo_6dof/control.py` | `lqr_gain()`、瞄准误差、相机到工具的速度变换、向量限幅 | 增加 PIDChannel / TrackingPID，替换活动控制链路中的 LQR；保留有用的变换和限幅函数 |
| `src/jetcobot_driver/visual_servo_6dof/visual_servo_6dof.py` | 红绿蓝 HSV 检测、CameraInfo 校验、图像时间戳检查、TF 匹配、Twist 发布、调试图像 | 接入 PID、轮廓最小外接圆测距、选色/锁定、状态机、叠加显示和限频日志 |
| `src/jetcobot_driver/visual_servo_6dof/config/visual_servo.yaml` | 图像/控制参数，当前仍有 `q_aim`、`r_aim` | 建立集中、带单位和中文调参说明的 PID、测距、显示及状态机配置 |
| `src/jetcobot_driver/visual_servo_6dof/config/moveit_servo.yaml` | Servo、奇异点、碰撞和输出话题配置 | 保留保护，检查时序、单位和有效限位，不把 PID 增益塞入 Servo 参数空间 |
| `src/jetcobot_driver/visual_servo_6dof/servo_serial_bridge.py` | `/joint_states` 反馈、单步轨迹校验、唯一串口持有者、默认禁止运动 | 增加明确的预览输出和执行模式；复用串口互斥、反馈时效、状态保护和 fresh mode 逻辑 |
| `src/jetcobot_driver/visual_servo_6dof/trajectory_guard.py` | 关节排序、位置/步长检查、模型与 SDK 的角度转换 | 保留第六轴偏置转换；将固定限位逐步归一到 URDF/集中配置 |
| `src/jetcobot_driver/launch/hardware_visual_servo.launch.py` | 已能组合相机、桥接、TF、MoveIt、Servo 和跟踪器 | 抽取/新增不含串口桥接的规划 launch，供 `tracking_main.py` 加载；旧全套 launch 可保留兼容，但不能混入两文件流程 |
| `src/jetcobot_driver/launch/moveit_servo.launch.py` | 加载 Servo，当前在 Python 中覆盖关节速度/加速度限值 | 将有效覆盖值纳入统一配置，避免不同节点各用一套限位 |
| `src/jetcobot_description/urdf/jetcobot.urdf` | 实际项目的关节链、轴、连杆与碰撞几何 | 优先用其建立 FK/Jacobian，不另抄一套未经核对的 myCobot DH 表 |
| `src/jetcobot_moveit/config/jetcobot.srdf`、`joint_limits.yaml`、`kinematics.yaml` | 运动组、碰撞排除项、关节约束和 KDL 配置 | 核查有效配置和碰撞排除项；不要把 KDL 配置误当作 Servo 算法的保证 |
| `src/jetcobot_driver/setup.py`、`package.xml`、`test/`、相关 README | 安装入口、依赖和现有回归测试 | 同步安装新增配置、脚本、launch 和文档，更新受控制语义变化影响的测试 |

已有重要约定：

- 图像：`/wrist_camera/image_raw`；内参：`/wrist_camera/camera_info`。
- Twist 输入：`/servo_node/delta_twist_cmds`。
- Servo 原始六轴候选输出：`/arm_group_controller/joint_trajectory`，只在规划侧内部消费。
- 主文件到 sync 的专用指令：`/tracking/joint_target`，类型为单点 `trajectory_msgs/JointTrajectory`；经过规划侧最终校验再发布。
- 关节反馈：`/joint_states`；状态：`/servo_node/status` 和桥接状态话题。
- 基座：`base_link`；工具：`6_Link`；标定相机：`camera_optical_frame`。
- 当前实际安装的是 ROS 2 Humble、MoveIt Servo 2.5.9；按本机版本核对支持的参数，不能直接套用新版 Rolling 配置。
- 已有标定文件 `/home/jetson/charuco_handeye_result_filtered.json`，包含 `T_gripper_camera`，方向为 `6_Link <- camera_optical_frame`，平移单位为米。读取并校验，不能任意设为单位矩阵。
- 相机内参文件为 `/home/jetson/.ros/camera_info/wrist_camera.yaml`。当前标定分辨率 640×480，fx≈960.25624、fy≈961.34516，主点约为 (356.02751, 191.13712)。这些数字仅作检查参考，运行时从标定与 CameraInfo 读取。
- 当前 `model_radians()` 对第六轴加 π/4，`hardware_degrees()` 逆向减 45°。保持往返一致，不重复施加偏置；终端明确区分模型角和 SDK 角。
- 现有桥接默认禁止运动，可复用这一机制。读取真实反馈不能同时运行 mock 关节发布器；多个进程不能同时占用串口。

### 两文件入口与通信（强制要求）

`tracking_main.py` 管理规划侧生命周期，使用 `launch.LaunchService` 和 `IncludeLaunchDescription` 加载不含执行器的规划 launch。`sync_plan.py` 作为独立执行进程，直接复用 `ServoSerialBridge` 或抽取后的执行类，不再包含旧 `MycobotReceiver` 的 `/joint_states -> send_radians` 路径。保留现有 `sync_plan = jetcobot_driver.sync_plan:main`，新增主文件安装入口。不要在任一文件里重复实现整套视觉/串口系统。

```text
tracking_main.py
  └─ 规划 launch：相机 → 识别/测距 → PID → Servo/自研IK → 最终校验
       ├─ 本地处理窗口 + 每2秒关节日志
       └─ /tracking/joint_target ───────────────────→ sync_plan.py
          ↑                                                   ├─ 唯一串口持有者
          └─ /joint_states + sync状态 ←───────────────────────┤
                                                              └─ 预览或执行
```

- 主文件先启动时，摄像头和处理画面必须能正常显示；sync 尚未启动则显示 WAIT_FEEDBACK，仅暂停依赖关节的解算与非零目标发布，不能卡住相机初始化。图像显示不依赖 TF/规划就绪。
- sync 启动后先读取并发布有效关节反馈，再等待主文件的规划指令与安全就绪；不能先等规划再发反馈，造成互相等待。只有 sync 打开串口，主文件的设备检查不能要求 `/dev/ttyUSB0` 可用或空闲。
- 规划 launch 不启动 `servo_serial_bridge`、`sync_plan` 或 `tracking_main` 自身；sync 也不包含规划 launch。不能递归启动入口或创建两个串口实例。
- `sync_plan.py --mode preview` 为默认模式：读真实反馈、接收/校验主文件指令，零运动；`sync_plan.py --mode execute` 为显式真机模式，在所有条件就绪后自动启用。模式只由 sync 决定并向主文件报告，主文件不维护可能与 sync 冲突的第二个执行开关。
- 主文件通过 `/tracking/joint_target` 发布六个具名关节的绝对模型角（rad）、期望速度（rad/s）及单步时间；不得发 SDK 角后让 sync 再减一次偏置。sync 在最后下发边界调用 `hardware_degrees()` 转成 SDK 角，日志中标明坐标约定。
- 指令 QoS 使用 volatile、深度 1，校验生成时间、接收时间、六轴完整性、有限值、限位、反馈偏差和最大步长；只保留最新单点，不执行缓存规划路径。`/joint_states` 永远只用作反馈，不承载期望目标。
- Servo 2.5.9 的轨迹输出可能使用零 header 时间戳。规划侧发布到专用话题时必须写入真实生成时间，并检查源反馈和安全状态仍新鲜；不能通过重发旧目标、重新盖时间戳而延长其有效期。诊断状态应带上对应目标/源反馈时间戳，避免把旧 READY 与新目标拼接。
- 主文件发布持续健康状态/心跳；sync 同时检查主文件心跳、指令年龄、当前反馈与所选后端的安全状态。正常零速度保持也要有健康心跳，不能把“主文件死掉”当作目标已经居中。
- 主进程存活心跳必须由 `tracking_main.py` 进程本身产生，例如其后台 ROS executor 线程；与控制节点健康状态分别检查。不能只让 launch 子节点发心跳，否则主文件被强制终止而子节点暂时存活时，sync 仍会误判主文件健康。主文件的 LaunchService 仍在主线程，退出时一并结束心跳线程。
- 规划状态消息最少包含时间戳、会话标识、后端、跟踪状态、就绪/保持/故障原因和目标生成时间；sync 状态包含实际模式、门控、有效反馈时间以及最近接受/发送目标的时间。可优先复用 ROS 现有诊断消息，并将依赖写进 package.xml；不要求为此新建自定义消息包。会话重启、状态对应不上、指令发布者超过一个或后端不匹配时拒绝执行。
- execute 的首次就绪链为：有效反馈 → 主文件/模型/场景就绪 → 后端启动 → 新鲜安全目标 → fresh mode 确认 → 开启门控 → 再接收开启后的新目标。预览、错误后恢复和入口重启都清除旧目标；保护故障后不反复自动解锁。
- 目标尚未出现时只读等待，不为了完成首次使能而发送试探目标。等待感知/反馈是健康等待状态；设备打开失败、模型不合法等启动硬错误则给出明确错误并清理已启动资源，不能把两者混为同一种超时。
- 主文件的 `LaunchService.run()` 留在该进程主线程。窗口放在独立显示节点的主线程，ROS 订阅通过线程安全的最新帧快照与 GUI 配合，避免 LaunchService 与 HighGUI 争夺同一主线程。GUI 周期调用 `waitKey(1)` / `pollKey()`，不在控制回调中 `waitKey(0)` 或阻塞选择 ROI。提供 headless 选项。
- 主文件退出/关闭窗口：尽力发布停止状态或请求 sync 禁用，退出其管理的规划子节点；独立运行的 sync 不被强制杀进程，而应由自己的指令/心跳超时保护停止机械臂，进入禁用的只读保持状态。sync 退出则停止并释放串口，主文件继续显示 WAIT_EXECUTOR/WAIT_FEEDBACK。两个独立进程不能假设共享退出事件。
- 按主文件路径或 ament package share 定位配置，不依赖 cwd。首次构建/环境准备与 IDE 解释器要求写清楚；若支持未 source 直接运行，则实现导入 ROS 前受控加载环境/一次性重启，不能无限递归或自动安装依赖。规划侧日志在主文件终端，执行侧故障日志在 sync 终端，六轴表统一由主文件每 2 秒输出，避免两边刷屏。

## 三、实时检测、选色和外接圆测距

复用现有 HSV 分割与轮廓提取，不重写视觉检测系统。只扩展必要的检测结果结构，携带颜色、轮廓、面积、bbox、圆心、半径和图像时间戳；同步更新调用方和测试。

必须对选中轮廓调用 `cv2.minEnclosingCircle(contour)` 获取 `(cx, cy), r`。当前 `detect_target()` 返回的 `max(bbox_width, bbox_height)` 不是实际外接圆直径，不能直接拿它的一半充当 r。

测距采用：

```text
square_side_mm = 30.0
D_mm = sqrt(2) * square_side_mm       # 正对相机的正方形外接圆实际直径
Z_mm = f_px * D_mm / (2 * r_px)      # 估计沿相机光轴的深度
desired_depth_mm = 200.0
```

如果配置直接提供 `object_circumdiameter_mm`，则使用该物理直径，并检查与边长配置是否冲突。不能一边将 W 定义为方块边长，一边写 `Z = f*W/(2r)`。边长 30 mm 对应 D≈42.426 mm。

先使用内参和畸变参数完成几何一致的去畸变处理，再测量圆心与半径。默认在 640×480、统一焦距 `f_rect=(fx+fy)/2` 的矫正图像中复用检测函数，并用 `K_rect` 计算射线；`K_rect` 的主点显式记录，不把它强行设成期望跟踪点。如果配置使用原图轮廓点去畸变，须重建对应像素坐标后重新拟合外接圆。显示使用同一矫正图像，裁切、缩放时同步转换中心/半径/参考点。不能只矫正中心点而半径仍用原始畸变值，也不能对已矫正的图像再做一次去畸变。

处理 r 过小、轮廓被画面裁切、无目标、NaN、异常深度和严重尺寸跳变。图像中注明“尺寸估计距离”；方块倾斜和遮挡会影响结果，不能宣称单个外接圆恢复了目标三维姿态。

支持 `target_colour=red/green/blue/auto`。窗口中用矩形框圈出目标，并画实际外接圆和颜色名称；允许按键选择颜色，或点击检测框锁定该目标的颜色/实例，给出简洁操作提示。auto 模式优先保持上一目标的颜色和空间连续性，避免在多个物体之间每帧切换；身份或颜色切换时重置控制历史。

## 四、PID：三轴误差到相机平移速度

现有 LQR 的两个横纵通道是瞄准角速度，第三通道是相对距离速度。本次要求为三个平移速度 PID，不能只将 `lqr_gain()` 换个名字并保留原有角速度语义。

对外提供 `controller.update(cx, cy, r, found, t)`，作为更新检测快照的兼容接口，不在图像回调内直接发串口。ROS 控制循环使用 `tick(now)` 或等价定时器，获得测量时间、当前控制时间和最新关节反馈；同一检测快照可被多个控制周期读取，但不能重复更新测量微分。每次控制返回结构化结果，供预览、日志、测试和专用指令发布复用。

目标图像位置默认 (320, 240)，它是期望跟踪位置，不是相机内参主点。由同一投影模型求目标像素和期望像素的归一化射线，计算毫米误差：

```text
ex_mm = Z_mm * (x_normalized_target - x_normalized_reference)
ey_mm = Z_mm * (y_normalized_target - y_normalized_reference)
ez_mm = Z_mm - desired_depth_mm
```

在等焦距、已矫正图像中，前两式等价于 `(cx-u0)*Z/f`、`(cy-v0)*Z/f`。支持 fx、fy 分别处理，不混淆画面中心与标定主点。相机光学轴默认 x 向右、y 向下、z 向前；通过合成投影测试确认符号。目标偏右时应使相机产生能让目标回到画面中心的运动，不能只看日志正负猜方向。

这里的“归一化射线”明确指 `[x_n, y_n, 1]`，不是范数为 1 的单位射线。现有 `pixel_ray()` 返回单位向量，若复用必须先除以该向量的 z 分量，再取 x/y。

由于期望图像中心不等于标定主点，前后移动也会影响横纵误差。默认姿态保持的平移模式加入几何前馈：

```text
uz = PID_z(ez)
v_cam_z = uz
v_cam_x = PID_x(ex) + x_normalized_reference * uz
v_cam_y = PID_y(ey) + y_normalized_reference * uz
```

对静止目标、相机仅平移，有 `d(ex)/dt=-vx+x_ref*vz`，上述前馈抵消该耦合。之后统一做速度/加速度限制。若启用明显的瞄准角速度，必须使用一致的图像交互模型处理旋转耦合，不能继续假定三个图像误差与平移速度完全解耦。

实现 `PIDChannel` 和 `TrackingPID`：

- 比例项、按实际 dt 的积分、积分限幅、条件积分抗饱和、死区内积分衰减。
- 对测量量求微分并加一阶低通，明确误差与执行方向下的微分符号，避免设定值阶跃引起微分冲击。
- 本文采用 `e=measurement-reference`、相机向目标运动的正输出使 e 减小；因此相应通道可写成 `u=Kp*e+Ki*I+Kd*filtered_dmeasurement`。不能照搬 `e=setpoint-measurement` 控制器的负测量微分符号。以 z 为例，Z 正在下降时微分项应降低继续前进的速度；测试必须验证该阻尼方向。
- x/y 死区默认 3 mm，z 死区默认 5 mm；阈值可配。
- 速度和加速度限幅；向量速度限制尽量保持方向。
- cx、cy、r 一阶低通，Z 再独立低通；优先用 `alpha=dt/(tau+dt)` 适配变化周期。新图像才更新测量滤波/测量微分，不把重复旧帧当作新测量。
- `reset()` 清空积分、微分、滤波历史和上一次输出。重获目标后软启动，避免突跳。
- 修改跟踪参考像素、期望深度或目标身份时重置相关历史；测量微分使用测量时间差，控制积分/限加速度使用实际控制 dt，不混用。
- 把运动保护/下游饱和反馈给 PID，避免目标不可达时继续积分。

建议首轮预览参数，须写进配置并明确未经过本机真机调参：

| 通道 | Kp (1/s) | Ki (1/s²) | Kd（无量纲） | 死区 |
| --- | --- | --- | --- | --- |
| x | 0.4 | 0.0 | 0.0 | 3 mm |
| y | 0.4 | 0.0 | 0.0 | 3 mm |
| z | 0.25 | 0.0 | 0.0 | 5 mm |

先验证纯 P，再逐步加入必要的 I/D。积分限幅要说明限制的是积分状态还是积分输出，给出对应单位。滤波时间常数初值可用像素/半径 0.10 s、距离 0.15 s、微分 0.10 s，并说明滤波叠加会增加延迟。

原要求 50 mm/s、200 mm/s²作为可选上限档；首轮沿用项目更低的 10 mm/s、30 mm/s²。参数不能散落在代码中。内部推荐统一 m、rad、s，界面和日志转换为 mm、deg；PID 的毫米误差到 ROS 速度只能在明确边界换算一次。

## 五、坐标、末端参考点和姿态

相机平移速度转到基座坐标：

```text
v_cam_base = R_base_ee @ R_ee_cam @ v_cam
```

这只改变坐标表达。若还有角速度，必须处理相机原点与工具原点的偏移，复用 `camera_to_tool_twist()` 的完整刚体速度变换；只乘旋转矩阵不足以完成不同参考点的 Twist 变换。

自研 IK 若以相机中心位置为主任务，应计算相机中心的 Jacobian：

```text
r_base = R_base_ee @ p_ee_cam
Jv_cam = Jv_ee - skew(r_base) @ Jw_ee
```

不能将相机中心速度直接与工具中心 Jacobian 混用。

第一阶段固定为“相机三通道平移 PID + 进入跟踪时的姿态保持”，不同时开启强瞄准姿态控制。姿态参考使用已测初始姿态，必要时以小角速度校正偏差，主轴/滚转保持由同一 Twist 任务表达。现有 Servo 默认六维均控制；将角速度填零仍是六维约束，不会自动得到三个冗余自由度。因此第一阶段可能在全姿态奇异点或可达边界减速/保持，不能承诺大范围始终运动。

第二阶段位置主任务才采用所选控制点的三维 Jacobian；姿态/滚转作为有预算的次任务。这里默认保持参考姿态；若增加大偏差瞄准模式，必须明确视线与图像目标点、角速度对像素误差的耦合，单独验收。若期望点为画面中心 (320,240)，它在当前内参下不沿光学 z 轴，“光轴精确穿过目标”和“目标精确位于画面中心”不是同一要求。

只有中心/半径观测时，不能恢复目标完整 6D 姿态。可选全位姿模式的姿态参考来自显式设定、初始姿态或其他可靠观测。6 关节在满秩 6D 主任务下没有额外零空间，次任务此时应禁用或仅在约束优化中作软偏好。

## 六、多轴解算与两种后端

### 第一阶段：复用 MoveIt Servo

保持链路：

```text
现有摄像头 → 颜色轮廓/外接圆 → TrackingPID
→ 相机/工具 Twist 变换 → MoveIt Servo
→ 原始六关节 JointTrajectory → 规划侧最终校验 → /tracking/joint_target
→ 独立运行的 sync_plan.py → 只读预览或串口执行
```

从真实关节反馈计算目标，不用 all-zero 假姿态冒充实测。订阅 Servo 实际输出的六关节目标用于显示，不能用“像素偏右就转 J1”之类经验映射代替 IK。

规划侧最终校验与 sync 接收校验复用同一关节排序/角度约定和配置。显示区分“Servo 原始候选”“校验后目标”“sync 实际接受/发送”三者；sync 在预览时也要运行校验并反馈接受/拒绝结果，不能因运动开关关闭就直接跳过校验。

Servo 的 `speed_units` 输入为 m/s 和 rad/s，不把 `scale.linear` 误认为该模式下的实际速度上限；限速必须在控制/约束中生效。保留碰撞、奇异点和关节保护，不能为让模型动起来关闭保护或无限提高奇异点阈值。

本机 2.5.9 具有 drift/control dimensions 接口，但仅把控制维度设为 false 不等于移除 Jacobian 中的任务约束；KDL 插件路径与逆 Jacobian 路径还需分别核实。第一阶段不依赖这些接口实现零空间，不改变全姿态求解来假装满足第二阶段要求。

### 第二阶段：独立 Weighted DLS 后端

提供清晰的 `ik_backend` 选择；两个后端互斥，禁止把自研 IK 的关节目标再送入另一个笛卡尔 IK 重复解算。安全检查与执行入口必须明确，自研后端不能假借一个未参与其求解的 Servo 状态证明轨迹安全。选择 weighted_dls 时，sync 不再硬等待 `/servo_node/status`，而是检查该后端自己的实时安全状态、心跳和最终目标；这部分需要显式改造现有桥接，不能只改规划器名称。

实现 `Kinematics`、`DifferentialIK`、`SafetyGuard`、`ArmController` 和 `TrackingStateMachine`，按职责组织文件，不为凑类名大规模重构现有检测。

运动学：

- 优先从现有 URDF 关节 origin/axis/固定变换构建 FK 和几何 Jacobian。URDF 不直接保存 DH 表，不要求“从 URDF 直接读出 DH”。若另支持 DH YAML，注明 standard/modified DH、单位、符号、零偏和待实机核对项。
- FK(q) 返回 4×4 变换，同时返回各连杆/关节/工具/相机位置供保护使用。每周期重算 J(q)。
- 提供 `get_angles()` / `get_coords()` 只读 FK 自检，统一 SDK 基座、工具参考点、角度偏置和姿态约定后再比较；报告位置和旋转角误差及阈值。SDK 读取全部由 sync 调度，主文件不能为自检再开串口；较慢的 get_coords 放入只读诊断模式或低频任务，并检查采样时间差。SDK 坐标通常来自其内部模型，这项检查不能替代外部实测标定。

位置主任务取所选控制点的 3×6 `J_active=Jv`，使用明确的任务归一化：

```text
J_task = task_scale @ J_active
v_task = task_scale @ v_active
sigma_min = smallest_singular_value(J_task @ W^(-1/2))
J_W_plus = W_inv @ J_task.T @ solve(J_task @ W_inv @ J_task.T + lambda²*I, I)
qdot = J_W_plus @ v_task + N @ qdot0
qdot0 = k0 * grad(H)
lambda² = 0                                       if sigma_min > epsilon
lambda² = (1-(sigma_min/epsilon)²) * lambda_max²   otherwise
```

- W 为正定对角权重，关节越接近安全限位，其权重越大并有数值上限。
- 使用线性求解/SVD，不显式计算矩阵逆。零阻尼时也要处理秩亏与数值阈值。
- 默认使用秩阈值 SVD 构造真正的零空间，验证 `Jv@N` 残差。阻尼伪逆生成的 `I-J_W_plus@Jv` 通常不是严格零空间投影，只能作为明确标注的近似选项。数值秩变化时使用阈值滞回和次任务权重平滑衰减，避免投影维数切换引入速度跳变。
- 次任务包含姿态/滚转偏好、关节居中和奇异性回避，且均有速度预算。仅当位置任务秩为 3 时，六轴位置任务局部零空间维数为 3；三项偏好不等于各自拥有一个始终可用的自由度。
- 若 H 使用梯度上升，关节项必须是“越远离限位越高”的奖励，或对限位惩罚取负号，不能对正惩罚做梯度上升。
- 位置任务的 sigma_min 来自活动 `Jv`；可操作度优先用对应任务 `sqrt(det(Jv@Jv.T))`，用奇异值乘积/对数形式保证稳定性。完整 6×6 Jacobian 指标可另行记录，注明不同含义。
- 6D 模式必须对平移/转动做任务尺度归一化，例如配置特征长度，再计算奇异值和阻尼阈值；不混用 m/rad 不同量纲的阈值。
- 给定 W 后求解的有效矩阵是 `A=task_scale @ J_active @ W^(-1/2)`；活动集和权重变化时，记录其最小奇异值并用一致归一化计算阻尼。原始 Jv 的可操作度另记作几何指标。不要只监测原始矩阵而忽略加权/固定关节后的病态程度。
- 给出与选定单位和归一化方式匹配的 epsilon、lambda_max、k0 初值及依据。不得把未经归一化的固定数值当成通用推荐。
- 可从 `L_ref=0.1 m`、位置任务 `task_scale=I/L_ref`、W 对角元素不小于 1、`epsilon=0.2`、`lambda_max=0.1` 开始离线调试；此时两个阻尼参数对应归一化矩阵。将 H 定义为归一化关节居中/可操作度奖励并标明求导变量后，k0 可从 0.02/s 起步，再限制次任务实际速度；若改变 H 的尺度须重调 k0。这些是仿真起点，不是已验证的硬件推荐值。
- 支持每周期 1–3 次小步迭代精修，但总步长、总执行时间和所有安全约束仍须限制。

## 七、关节、工作空间和碰撞保护

关节约束统一来自模型与集中配置：角度范围、安全余量默认 5°、每轴最大速度/加速度、最大单步、反馈到目标最大偏差。必须检查模型范围与 SDK/实机范围的交集，并正确应用关节偏置。

安全余量只在同一套有效区间上扣除一次；不能把已经减掉 5° 的限位再交给另一个节点重复扣除。初始实测值已在安全区间之外时先报告，不把它静默裁成区间内的一个新目标并发送。

当前缺项需要修复：URDF 中各关节 `velocity=0`；MoveIt `joint_limits.yaml` 关闭速度和加速度限制；`moveit_servo.launch.py` 临时对 Servo 覆盖为 0.3 rad/s、0.5 rad/s²；桥接另有 0.035 rad 单步上限。这些值要形成一致且可检查的有效配置，不能只改 YAML 中没有被使用的字段。首轮可以从每轴 0.1 rad/s、0.3 rad/s²开始，SDK speed 保持现有低速范围，并说明均待实机验证。

速度保护：

- 任一轴超速时同比缩放全部 qdot：`s=min(1, min(qdot_max_i/abs(qdot_i)))`，零分量正确处理。
- 这种缩放保持当前解的任务速度方向；后续若又逐轴截断/修改，不得继续声称方向严格保持。
- 预测下一步越限时，用活动集固定受限关节，再求解可用关节；可释放允许远离限位的关节。自由度不足时降低任务优先级或速度，求解无可行结果时保持并解释原因。
- 把关节速度、加速度、位置、单步限制和已执行的前一步指令共同纳入计算；最后再验证最终 q_cmd。安全所需急停可以优先于普通平滑，但必须记录原因。

工作空间和碰撞：

- 配置基座坐标下的 x/y/z 边界、参考原点明确的最大伸展半径，注明具体约束的是工具还是相机中心。
- 配置 `table_z_m`、安全距离默认 0.05 m、连杆/工具/相机的碰撞尺寸。活动连杆用胶囊体，检查非相邻连杆，并覆盖夹爪、相机支架等附件。
- 不要简单要求“所有关节中心包括基座都高于 50 mm”；固定底座可能就在桌面上。桌面例外对象和安装几何必须显式配置，活动连杆检查应包含半径/厚度。
- MoveIt 后端将真实桌面加入 PlanningScene，并等待场景就绪；现有 `check_collisions=true` 本身不会自动创建桌面。
- 核查 SRDF 中大量 `Never` 排除项是否符合安装后的相机/夹爪几何，不直接沿用为安全结论，也不无依据地全部删掉。
- 对可求约束法向的工作空间/桌面边界，在任务速度层去除朝违规方向的分量，保留切向运动。多边界同时生效时采用一致约束投影/小型约束求解，避免简单顺序投影后又违反先前边界。
- 中间连杆和自碰撞约束使用相应连杆 Jacobian/距离梯度，必要时在关节空间约束；不能只投影末端速度就宣称保护了全部连杆。
- 对最终步长及必要的中间采样验证碰撞和限位。可行切向解可以继续运动；无法得到已验证安全解时拒绝该步，输出零并显示原因。滑墙不能覆盖碰撞硬停。
- 安全参数缺失时列出待实测项并限制执行，不能编造臂展、桌高、碰撞半径或已通过的真机验证。

## 八、反馈、串口和预览语义

自研 IK 每次采用 `q_cmd = q_estimated_current + qdot*command_horizon` 的小步，以最新有效实测角为依据。20 Hz PID 的 dt、10 Hz 执行的 0.1 s 目标时间跨度、反馈间隔是不同的时间量，不能一律用 0.05 s。Servo 后端使用它实际产生的目标位置和 `time_from_start`，不重复积分。日志的速度使用消息速度或同一目标跨度换算。

两次反馈之间可做有界预测，不能在同一旧实测角上反复计算相同的 0.05 s 小目标却宣称实现 0.1 s 的执行速度，也不能无界累积目标导致追赶。实机不动的 preview 不推进虚构实测状态。

将规划反馈源与执行模式分开，只允许以下组合：

| 主文件 feedback_source | 单独运行的 sync | 含义 |
| --- | --- | --- |
| `hardware`（默认） | 尚未启动 | 摄像头/窗口继续，等待真实关节反馈，不发布非零关节目标 |
| `hardware` | `--mode preview`（默认） | sync 真实反馈+目标校验，零运动；显示本周期建议动作 |
| `hardware` | `--mode execute` | sync 真实反馈+执行门控+实际发送；本次开发验证不运行此模式 |
| `simulation` | 不运行硬件 sync | 主文件启动带延迟/限速的 mock 反馈与目标消费，清楚显示 SIMULATED |

simulation 与硬件 sync 同时出现时拒绝执行，防止两个 `/joint_states` 来源混用。不能把预览目标回填为实测反馈，也不能把纯摄像头像素演示声称为物理机械臂闭环验证。

实物摄像头 + mock 关节只能验证识别到规划的接口，不能代表机械臂运动后相机画面也随之改变。真正的闭环仿真须由模拟关节/FK 更新相机位姿，再投影世界目标生成合成观测；在结果中区分这两种演示。

桥接只保留最新目标，禁止指令堆积。保持单串口持有者、反馈来源检查、目标时效校验、Servo/自研安全状态、单步验证及退出清理。初次反馈读取、故障恢复、重获目标和启用动作时要清掉过期目标。

SDK `send_angles(q_sdk_degrees, speed)` 的 speed 是 SDK 速度等级，不是 rad/s，也不能保证六轴严格按指定 qdot 同步。根据已求解速度给出有界映射，默认先沿用低速设置；实际平滑度需通过延迟、跟踪误差和串口吞吐调试，不能承诺硬实时。

允许短暂使用上次有效反馈作显示或有限预测，同时记录失败次数和反馈年龄；无效读取不刷新有效反馈时间，超时必须停止执行。不能在串口持续失败时仍无限下发命令。实际反馈率沿用当前 10 Hz 起步，控制可配 10–20 Hz；串口耗时计入预算，不强行宣称每次 20 Hz 都有新实测角。为达到图像时刻的 TF 插值，控制要允许短暂等待下一帧反馈；若等待预算超限就保持，不用未来 TF 或无界外推冒充准确配对。

预览模式下不得调用 `send_angles()`、`send_radians()`、`send_coords()`、自动回零或调整姿态；启动、退出和目标丢失均不能绕过此限制。没有关节反馈时继续显示识别，但关节栏标为“反馈不可用”，不得编造有效角度。若用户显式选 simulation，则显示模拟角度。

## 九、时序、丢失与持续运行

- 控制定时器使用实际 dt，建议初值 0.05 s；dt 非有限、非正或超过可配异常阈值时跳过计算、清理相关历史并进入保持，不把一个大 dt 静默截成正常值后继续运动。
- 默认相机 30 Hz、PID 20 Hz、Servo 发布/自研目标输出 10 Hz、sync 读写 10 Hz，关节日志每 2 s。`incoming_command_timeout` 初值可取 0.2 s，sync 指令/心跳/有效反馈超时先沿用 0.35 s；实测调度与串口耗时后校核。不能让 Servo 的 0.1 s 超时恰好等于 10 Hz 输入周期。停止延迟还包含回调调度和串口阻塞，不能承诺超时一到硬件就瞬间静止。
- 图像时间戳与 TF 使用 ROS 时间，循环 dt/超时使用单调时钟，禁止混用两个时钟作减法。
- 控制器读取最新有效检测快照；现有短 TF 匹配队列可保留，但必须有界，优先新鲜数据，不能不断追赶陈旧图像。
- 状态机包含 IDLE、TRACKING、LOST、HOLD，以及可选 HOME / FAULT。程序不因这些状态转换而自动退出。
- 丢失小于 0.5 s：在输入/反馈和安全条件仍允许的前提下，让上一输出指数衰减至零，不继续放大旧误差。传感器过期、无有效 TF 或碰撞故障应按安全条件立即保持。
- 区分“新图像里 found=false”和“图像流停止/过期”。前者可做短时衰减；后者优先按图像时效阈值保持。0.5 s 是目标丢失状态阈值，不会覆盖更早触发的 0.35 s 反馈/通信故障停止。
- 丢失达到 0.5 s：PID reset，保持位姿，持续检测。重获有效目标自动软启动恢复 TRACKING。
- 原需求“丢失 5 s 后回初始位姿”做成 `home_on_lost` 可选项，默认 false。启用后也只允许执行经过规划/碰撞验证的慢速回程，HOME 期间继续感知并定义重获目标时的切换；预览绝不向真机回零，串口故障时也不得回零。
- HOME 由规划侧产生经过验证的小步流并继续使用同一专用指令与 sync 门控，不能另开 SDK 通道发送一条未经保护的回零命令；现有单步桥接不直接接收整段规划轨迹。
- 不使用固定轮数 for 循环、只运行一次的求解或阻塞式等待到位。控制、绘图和打印互不阻塞。
- Ctrl+C / q / Esc 退出主文件及其规划 launch，并通知 sync 禁用；sync 的独立看门狗负责在通知丢失时仍能停止。单独退出 sync 时停止并释放串口。普通目标丢失后可以自动恢复跟踪，执行保护锁存故障后不能无提示自动重新使能。

## 十、画面与终端输出

运行 `tracking_main.py` 后，其内部规划 launch 自动打开实时窗口，也继续发布已有 `debug_image`。sync 不创建第二个图像窗口。保留 headless 配置，窗口退出事件通知主文件关闭规划侧并使 sync 停止。无需用户另开 rqt 才能看到识别效果。

画面至少显示：

- 目标颜色、矩形检测框、最小外接圆、圆心、半径、期望图像中心。
- Z 估计值、期望 Z、x/y/z 毫米误差。
- 需要移动的相机方向和距离、PID 期望速度、保护后实际采用速度。箭头要说明表示相机运动方向还是像素误差，避免符号相反造成误解。
- 状态机、PREVIEW/SIMULATED/EXECUTE、图像/关节反馈年龄、Servo/安全状态、动作被缩小或拒绝的原因。
- 六轴本周期建议增量，或简明关节摘要。

普通 OpenCV `putText` 不支持直接正确绘制中文；图像叠字可用简短英文/轴标签，并在中文 README 解释；若要求窗口内中文，显式加载现有字体和适当绘制方法，不静默引入缺失字体依赖。

终端每 2.0 s 用单独计时器输出一次六轴表格，禁止用 `sleep(2)` 阻塞控制回调。首次有效结果可以立即打印，之后按周期打印；重要状态变化/故障可及时打印但应去重限频。

输出示意，数字仅为格式示例：

```text
[PREVIEW | TRACKING blue] control=20Hz feedback=10Hz log_period=2.0s
depth_est=245mm target=200mm error_cam=(+12,-8,+45)mm
v_cam_requested=(+4.8,-3.2,+10.0)mm/s safety=OK
joint  measured_sdk(deg)  next_sdk(deg)  delta_deg_per_step  qdot(deg/s)
J1            ...             ...               ...              ...
J2            ...             ...               ...              ...
J3            ...             ...               ...              ...
J4            ...             ...               ...              ...
J5            ...             ...               ...              ...
J6            ...             ...               ...              ...
source=hardware  mode=preview  motion_sent=false
```

明确 `delta` 是一条目标消息时间跨度内的增量，`next_sdk` 是下发格式的绝对角，不是“2 秒后累计转角”。偏置转换后再打印 SDK 目标。保护停机时打印 HOLD 与原因，不能把保护前原始候选显示成 sync 已接受/执行的动作。界面标签随 sync 报告的实际模式变化。

## 十一、配置集中与启动验收

在 `visual_servo.yaml` 里明确设置“PID 调参区”，包含 x/y/z 的 Kp/Ki/Kd、积分限值、死区、输入/距离/微分滤波时间常数、速度/加速度、软启动、dt 范围和测距参数。补齐下列配置，并让 launch 真正传递给相应节点：

```text
feedback_source: hardware                 # 主文件消费，另可选 simulation
sync.execution_mode: preview              # 仅sync消费；CLI --mode execute可显式覆盖
ik_backend: moveit_servo                  # 第二阶段支持 weighted_dls
target_colour: auto
control_frequency: 20.0
command_frequency: 10.0                   # Servo发布或自研目标输出频率
sync.update_rate: 10.0                    # 真实反馈/串口执行频率
sync.command_timeout_s: 0.35
sync.feedback_timeout_s: 0.35
sync.planner_heartbeat_timeout_s: 0.35
joint_log_period_s: 2.0
show_window: true
square_side_mm: 30.0
desired_depth_mm: 200.0
home_on_lost: false
```

上述是功能配置意图，`sync.` 前缀表示参数归属，不要求照抄为现有桥接参数名。请按真实节点名与 `ros__parameters` 生成合法 YAML，主文件和 sync 从同一份配置读取各自部分。命令行高于 YAML，YAML 高于代码默认值；启动时回显有效值和消费者，禁止两个入口对同一开关各作决定。串口、工作空间、桌面/胶囊体和 IK 参数可放独立 YAML，通过明确路径统一加载。

移除活动路径中的 `q_aim`、`r_aim` 和 LQR 依赖；若为兼容暂时保留，注明已弃用及是否仍有调用者。参数热更新若实现，需要校验并按需 reset；若未实现，明确修改 YAML 后重新启动，不能写成支持 `ros2 param set` 却没有更新内部缓存。

交付两文件运行方式。首次构建和环境准备完成后，先在一个终端运行规划主文件：

```bash
python3 /home/jetson/jetcobot_colcon_ws/src/jetcobot_driver/jetcobot_driver/tracking_main.py --target-colour blue
```

此时应自动打开摄像头与处理画面弹窗，显示 WAIT_FEEDBACK。再在另一个终端运行 sync，完成本阶段只读预览：

```bash
python3 /home/jetson/jetcobot_colcon_ws/src/jetcobot_driver/jetcobot_driver/sync_plan.py --mode preview
```

后续需要真机执行时，主文件运行方式相同；先退出只读 sync，释放串口，再运行同一个 sync 文件的执行模式：

```bash
python3 /home/jetson/jetcobot_colcon_ws/src/jetcobot_driver/jetcobot_driver/sync_plan.py --mode execute
```

execute 模式由 sync 自己等待全部执行条件并启用门控，随后持续发送主文件规划且通过校验的小步目标，不要求第三个终端或额外手工服务命令。本阶段不实际运行 execute 命令。故障停机后不得无提示反复自动解锁恢复运动，须明确原因和恢复方式。

这些新增入口和选项必须实现并测试，不得只写进 README。主文件只把规划/选色参数传给内部 launch；sync 的 mode 独立控制执行。启动 Servo 仅用于解算，不等于启用实物运动。摄像头可复用已存在的唯一发布者，不得重复打开设备。无串口仿真由主文件 `--feedback-source simulation` 启动 mock 反馈/执行消费，不运行硬件 sync，也不要求 `/dev/ttyUSB0`。

明确列出首次构建、环境准备、两个文件的启动顺序、模式/颜色切换、分别停止时的行为及 PID 调参位置。`ros2 run jetcobot_driver tracking_main` 和 `ros2 run jetcobot_driver sync_plan` 可作为等价安装入口，但不能要求额外手工启动 launch。

## 十二、测试、交付与完成标准

交付完整 Python 代码与合法 YAML。数值控制核心尽量只依赖 numpy；ROS/OpenCV/pymycobot 复用现有环境；绘图脚本可以用 matplotlib 并声明依赖。roboticstoolbox-python 若用于交叉验证，应明确为可选依赖，不强制引入大型新框架。

测试要覆盖真实故障和闭环行为：

1. PID：变 dt、饱和抗积分、死区衰减、测量微分滤波、reset、软启动；无效输入不传播 NaN。
2. 外接圆：用已知边长的合成投影验证 `sqrt(2)` 几何、单位换算、去畸变后半径、深度估计和裁切处理；倾斜方块场景量化估计偏差，不把正对相机测试结果推广到任意姿态。
3. 视觉反馈符号：合成相机投影下，目标左/右/上/下/远/近均沿正确方向收敛；原有“偏右仅转相机”的 LQR 测试按新平移任务语义更新，保留手眼参考点变换回归。
4. 六轴 FK/Jacobian：有限差分与解析 Jacobian 对比；SDK/模型角度转换往返；近奇异点数值有限，严格零空间模式验证任务泄漏。
5. 约束：近限位、不可达目标、桌面/非相邻连杆碰撞、多个边界同时生效、速度/加速度和最终单步约束；检查实际采用的 q_cmd。
6. 默认零运动：mock SDK 断言 sync 预览全程所有运动方法调用次数为零，包括启动、跟踪、丢失、恢复和退出。确认日志 2 s 周期不降低控制/发布频率。
7. 持续运行：目标进入死区、短暂丢失、长时间丢失后两个程序仍健康；重获目标自动恢复。主文件中止后它管理的规划资源退出，独立 sync 按看门狗禁用；sync 中止后主文件继续识别并显示等待反馈。无反馈/陈旧图像不生成伪有效动作。
8. 仿真脚本不连真机、不打开串口，使用世界坐标目标和模拟相机投影形成真正闭环，再生成大范围正弦、阶跃、遮挡/丢失、边界与奇异点场景。不能仅给定像素轨迹后独立积分关节就宣称完成闭环验证。
9. 保存关节角、速度/加速度、末端/相机轨迹、sigma_min、限位余量、状态/约束记录和图表。曲线附数值阈值/断言，明确测试轨迹覆盖范围，不能据有限仿真承诺真机“绝不碰撞/飞车”。
10. 两入口验收：mock launch/SDK 验证先启动主文件能显示画面并等待、sync 默认预览并先发反馈、参数归属和 execute 就绪门控、不递归启动入口且只有一个串口实例。覆盖任一入口退出/重启、心跳中断、状态/目标乱序或陈旧、两个发布者冲突、sim 与实机反馈冲突、启动部分失败清理。实际集成至少验证从不同 cwd 按顺序运行两个文件、获得本地画面与连续有效解算；没有桌面或设备时如实标注未验证部分。

第一阶段按其验收边界运行适用测试，第二阶段补充 DLS/零空间/完整连杆约束测试；不能因第一阶段通过而声称第二阶段完成。

交付中文调试步骤：只读 FK 自检 → 单个 PID 通道方向测试 → 全程预览 → 小速度/纯 P 真机联调准备 → 逐步扩大范围与按需增加 I/D。未经用户另行指示，不执行任何真机运动测试。

给出抖动、超调、振荡、滞后、奇异点停机、关节卡限位、姿态漂移、构型跳变的排查表，覆盖测距噪声、滤波延迟、dt/时间戳、手眼参考点、偏置、SDK 队列/反馈滞后和可行域。

最终回复说明：修改文件、可直接运行的命令、PID 调参位置、预览与真机执行的区别、实际跑过的检查和结果、待实机核对的模型/场景参数。明确哪些功能复用 Servo，哪些由自研后端实现，哪些尚未验证；不能用建议替代已授权的代码交付，也不能把代码静态检查写成完成真机验证。

## 参考资料

- MoveIt Servo Humble 文档：<https://moveit.picknik.ai/humble/doc/examples/realtime_servo/realtime_servo_tutorial.html>
- MoveIt Servo 2.5.9 实现（版本相关行为以此及本机源码/头文件核对）：<https://github.com/moveit/moveit2/blob/2.5.9/moveit_ros/moveit_servo/src/servo_calcs.cpp>
- OpenCV 外接圆：<https://docs.opencv.org/4.x/d3/dc0/group__imgproc__shape.html>
- OpenCV HighGUI 事件处理：<https://docs.opencv.org/4.x/d7/dfc/group__highgui.html>
- pymycobot 官方 myCobot 280 接口说明：<https://github.com/elephantrobotics/pymycobot/blob/main/docs/MyCobot_280_en.md>

---

本文件只描述后续实现任务；生成此提示词本身没有更改运行配置，也没有启动摄像头、串口或机械臂。
