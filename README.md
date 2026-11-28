# JetCobot Vision 工作区

这是一个 ROS 2 工作区，包含机械臂模型、MoveIt 配置以及相机视觉伺服/跟踪驱动。主要代码位于 `src/`。

## 工作区结构

```text
src/
├── jetcobot_description/  # 机械臂 URDF、网格模型和模型显示启动文件
├── jetcobot_moveit/       # MoveIt 配置、规划启动文件、标定和调试工具
└── jetcobot_driver/       # 视觉伺服、PID 跟踪、关节反馈/执行与相机驱动
```

### `jetcobot_description`

- `urdf/jetcobot.urdf`：机械臂本体、连杆和关节模型，是运动学与显示的基础模型。
- `meshes/`：机械臂、相机和夹爪的视觉/碰撞网格。
- `launch/display.launch.py`：用于显示机器人模型。

### `jetcobot_moveit`

- `config/`：MoveIt 规划组、运动学、关节限制、控制器、初始关节位置和 RViz 配置。
- `launch/`：MoveIt、机器人状态发布、控制器和 RViz 等启动文件。
- `script/`：标定、目标调试、预抓取和局部关节控制工具。
- `src/`：C++ MoveIt 示例与规划程序，例如设置目标关节、目标位置和避障相关程序。
- `scene/`：MoveIt 场景示例。
- `docs/`：手眼标定和预抓取相关说明。

`config/ros2_controllers.yaml` 中的 `arm_group_controller` 是位置接口的关节轨迹控制器配置，不是本项目视觉位置环 PID 的 Kp/Ki/Kd 调参文件。

### `jetcobot_driver`

- `jetcobot_driver/`：ROS 2 Python 入口程序，包括 `tracking_main.py`（规划/跟踪主入口）和 `sync_plan.py`（关节反馈同步与可选串口执行）。
- `visual_servo_6dof/`：视觉伺服核心实现，详见下表。
- `launch/`：相机、MoveIt Servo、视觉伺服、跟踪规划等启动文件。
- `test/`：跟踪流程、PID 控制、安全门控和启动集成测试。

| 文件 | 作用 |
| --- | --- |
| `visual_servo_6dof/visual_servo_6dof.py` | 视觉伺服 ROS 节点：处理目标观测、控制状态、运动学规划和目标发布 |
| `visual_servo_6dof/tracking_pid.py` | **三轴视觉位置 PID 实现**，按相机坐标系 x/y/z 误差计算相机平移速度 |
| `visual_servo_6dof/tracking_config.py` | 读取并校验共享 YAML 配置 |
| `visual_servo_6dof/tracking_display.py` | 显示相机图像、检测结果、误差和速度等调试信息 |
| `visual_servo_6dof/tracking_mock.py` | 仿真反馈/合成图像相关节点，用于无真机验证 |
| `visual_servo_6dof/kinematics.py` | 机械臂运动学和微分逆运动学计算 |
| `visual_servo_6dof/trajectory_guard.py` | 轨迹约束与速度/加速度限制 |
| `visual_servo_6dof/safety.py`、`mesh_guard.py`、`fcl_guard.cpp` | 工作空间、碰撞和网格安全检查 |
| `visual_servo_6dof/serial_sdk.py`、`joint_command.py` | 串口 SDK 通信和关节命令处理 |
| `jetcobot_driver/sync_plan.py` | 同步关节反馈、校验规划结果，并在明确选择执行模式后与串口执行链路配合 |
| `visual_servo_6dof/wrist_camera.py` | 腕部相机 ROS 驱动/图像发布 |
| `visual_servo_6dof/tracking_scene.py` | 跟踪场景信息发布 |
| `visual_servo_6dof/simulate_tracking.py` | 离线跟踪与轨迹仿真、结果输出 |
| `visual_servo_6dof/control.py` | 旧控制数学函数兼容代码；当前实时跟踪使用 `tracking_pid.py` |

`visual_servo_6dof/validation/` 保存离线仿真和实机验证结果；`PID_TRACKING_PROMPT.md` 与 `PID_TRACKING_FEASIBILITY.md` 是 PID 跟踪设计资料。

## PID 在哪里

这里的 PID 指相机视觉伺服的位置跟踪 PID：

- **代码实现**：[`tracking_pid.py`](src/jetcobot_driver/visual_servo_6dof/tracking_pid.py)，核心类为 `PIDChannel` 和 `TrackingPID`。
- **控制器创建和调用**：[`visual_servo_6dof.py`](src/jetcobot_driver/visual_servo_6dof/visual_servo_6dof.py) 创建 `TrackingPID` 并将相机检测误差送入控制流程。
- **PID 参数**：[`visual_servo.yaml`](src/jetcobot_driver/visual_servo_6dof/config/visual_servo.yaml) 的 `tracking.pid` 区域。
- **配置加载和校验**：[`tracking_config.py`](src/jetcobot_driver/visual_servo_6dof/tracking_config.py)。

`kp`、`ki`、`kd` 各有三个值，顺序是相机坐标系 **x、y、z**（图像左右、上下和前后/深度方向）。参数内部以米和秒为单位。常用参数：

| 参数 | 用途 |
| --- | --- |
| `kp` | 比例增益，决定误差产生速度的强弱 |
| `ki` | 积分增益，用于消除长期偏差；积分受 `integral_limit_m_s` 限制 |
| `kd` | 基于测量微分的微分增益；实现中有 `derivative_tau_s` 滤波 |
| `deadband_m` | 各轴误差死区，落入死区后该轴不继续追踪 |
| `integral_limit_m_s` | 各轴积分状态限幅 |
| `input_tau_s`、`depth_tau_s` | 图像观测和深度测量滤波时间常数 |
| `derivative_tau_s` | 微分滤波时间常数 |
| `max_speed_m_s` | 末端/相机平移速度上限 |
| `max_acceleration_m_s2` | 平移速度变化的加速度限制 |
| `settle_exit_ratio` | 已收敛后再次启动跟踪的误差滞回系数 |

当前配置默认 `ki`、`kd` 为零。建议先确认坐标轴方向，再逐轴小幅调整 `kp`；出现稳定偏差且确认标定、测距、坐标变换无误后，再少量增加 `ki`；只有在有明显超调且观测噪声受控时才尝试 `kd`。一次只改一类参数，并保留速度、加速度、关节限位和碰撞保护。

## 调试 PID

主入口默认用配置文件里的 `tracking.pid`。在工作区根目录构建并加载环境：

```bash
cd ~/jetcobot_vision
source install/setup.bash
colcon build --packages-select jetcobot_driver --symlink-install
```

启动视觉规划/调试主程序：

```bash
python3 src/jetcobot_driver/jetcobot_driver/tracking_main.py \
  --config src/jetcobot_driver/visual_servo_6dof/config/visual_servo.yaml
```

如需查看本地无设备仿真入口，可使用：

```bash
python3 src/jetcobot_driver/jetcobot_driver/tracking_main.py \
  --feedback-source simulation --synthetic-camera --ik-backend weighted_dls \
  --config src/jetcobot_driver/visual_servo_6dof/config/visual_servo.yaml
```

调参时观察显示窗口和主程序终端中的目标检测、`Error camera mm`（相机坐标误差）及 `Camera motion mm/s`（PID 请求速度）。配置文件修改后需要重启相关进程；规划端和同步端应使用同一份 YAML。实机调试先保持默认预览模式，确认图像、误差方向、速度输出和安全限位都正确后，才按设备操作流程显式进入执行模式。不要为调参绕过安全检查。

当前主跟踪程序把视觉误差映射为相机/末端期望速度，再由逆运动学和轨迹限制分配到六个关节；它不是六个相互独立的关节 Kp/Ki/Kd 环。因此改 `tracking.pid.kp/ki/kd` 调的是视觉位置环，而不是 `ros2_control` 关节控制器的伺服增益。

## 相关说明

- [视觉 PID 跟踪详细说明](src/jetcobot_driver/visual_servo_6dof/README.md)
- [PID 跟踪设计提示](src/jetcobot_driver/visual_servo_6dof/PID_TRACKING_PROMPT.md)
- [PID 跟踪可行性分析](src/jetcobot_driver/visual_servo_6dof/PID_TRACKING_FEASIBILITY.md)
- [离线验证结果索引](src/jetcobot_driver/visual_servo_6dof/validation/README.md)
