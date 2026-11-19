# ChArUco 多 tag 手眼标定

适用于腕部相机（eye-in-hand）、标定板固定在桌面、机械臂改变姿态的场景。
程序读取相机和真实关节反馈对应的 TF，不发送机械臂或夹爪运动命令。

## 标定板与坐标

`标定板_ChArUco_5x7_28mm_A4.pdf` 对应 5 列 × 7 行**方格**，不是 5×7 内角点：
格长 28 mm，tag 外边长 21 mm，`DICT_5X5_100`，tag ID 0～16，共 24 个内角点。
使用 OpenCV contrib >= 4.8；本机 OpenCV 4.10 已通过检测检查。

打印时选 100% / 实际大小，量一下 PDF 下方 100 mm 标尺和多个方格的总长度。
PDF 上的栅格化板区域标注约 140.1×196.2 mm；程序使用设计尺寸 140×196 mm。
如实际测量有缩放，用测得的平均格长设置 `square_length_m`，并同比例修改
`marker_length_m`。尺寸错误会直接按比例影响测距，程序不能自动修正打印误差。
把纸平整贴在硬板上，整个采集和独立验证过程中保持板、机器人底座、相机安装不动。
相机的分辨率、对焦、内参也应保持一致，`camera_info` 必须确实对应当前图像。

棋盘坐标原点为正看 PDF 时棋盘**外边界左上角**，x 向右、y 向下、z 指向板内。
这是按当前 OpenCV 对象点实际定义使用的约定；不沿用旧单 tag 的中心原点。
检测采用带内参的 ChArUco 亚像素角点插值、RANSAC 内点筛选和整板平面 PnP。
[OpenCV ChArUco 检测说明](https://docs.opencv.org/4.10.0/df/d4a/tutorial_charuco_detection.html)
介绍了这条处理流程和不同版本的坐标变化。

## 构建与采集

在工作区运行：

```bash
cd /home/jetson/jetcobot_colcon_ws
source install/setup.bash
colcon build --packages-select jetcobot_moveit --symlink-install
source install/setup.bash
ros2 run jetcobot_moveit charuco_handeye_collect --ros-args \
  --params-file src/jetcobot_moveit/config/charuco_handeye.yaml
```

相机、机器人真实关节状态及 robot_state_publisher 应已启动。
默认读取 `/wrist_camera/image_raw`、`/wrist_camera/camera_info`，查询
`dummy <- 6_Link` 在**图像时间戳**的 TF。CameraInfo 和图像必须都标记为
`camera_optical_frame`；如实际名称不同，修改 YAML 的 `camera_frame`。
使用原始图像配 K/D 内参；不接受鱼眼模型或直接把校正图像当原始图像。
收到 TF 或时间错误时检查反馈时间戳和时钟，程序不会退回最新 TF 来凑样本。

调试图像为 `/charuco_handeye/debug_image`，可在 RViz Image 或 rqt_image_view 中查看。
每到一个新姿态，停稳约 1 秒，另一个已 source 的终端执行：

```bash
ros2 service call /charuco_handeye/save_sample std_srvs/srv/Trigger '{}'
```

建议保存 20～30 个姿态，分布在不同位置、距离及左右/上下倾角，绕至少两个不同轴旋转，
避免只平移或只转一个轴。每次尽量让大部分棋盘可见，板在图像里足够大且清晰，避免反光。
默认要求至少 4 个 tag、8 个内角点，内点凸包覆盖板面积 >=20%、画面面积 >=1%，
重投影 RMS <=0.8 px。最近 0.8 秒中至少 6 帧、跨越 0.35 秒的窗口必须稳定；
末端变化不超过 1 mm / 0.5°，棋盘相机位姿变化不超过 4 mm / 1.5°。
从稳定窗口中选择重投影 RMS 最小的一对同步观测保存，保持图像与末端 TF 一一配对。
所有已有样本都参与重复姿态检查，不会连续保存相同姿态。

数据默认保存在 `/home/jetson/charuco_handeye_samples.json`，每次保存原子写入，
支持同一配置下继续追加。换板参数、相机内参或帧名时请使用新的输出文件。
保存错了可撤销最后一组：

```bash
ros2 service call /charuco_handeye/undo_sample std_srvs/srv/Trigger '{}'
```

## 离线解算

采集结束后停止采集器，运行：

```bash
ros2 run jetcobot_moveit charuco_handeye_solve
```

默认使用 PARK，另外计算 TSAI/HORAUD/ANDREFF/DANIILIDIS 作对照，显示全样本离散与
每五组留出一组的验证误差。结果写到 `/home/jetson/charuco_handeye_result.json`。
没有自动剔除手眼样本；如果留出误差很大，应检查图像、板尺寸、内参、同步、关节反馈和运动多样性。
低重投影误差本身不能证明手眼结果准确。

变换含义：

```text
T_base_board = T_base_gripper @ T_gripper_camera @ T_camera_board
```

`T_gripper_camera` 是 `6_Link <- camera_optical_frame`；结果 JSON 的
`T_base_board`（也见 `methods.PARK.all_sample_scatter.reference_T_base_target`）
是该批棋盘基座位姿均值。
此值原点在新棋盘左上角，不能直接与旧单 tag 中心的坐标比较。
这些程序使用独立的样本/结果文件。`pregrasp_move.py` 已接入筛选后新手眼文件，
继续按单 AprilTag 观测；参见 [抓取接入说明](pregrasp_new_handeye.md)。

## 独立验证，并与旧手眼变换比较

保持棋盘和底座不动，重启采集器，换一个输出文件，加载新标定结果以观察实时 TF：

```bash
ros2 run jetcobot_moveit charuco_handeye_collect --ros-args \
  --params-file src/jetcobot_moveit/config/charuco_handeye.yaml \
  -p output_file:=/home/jetson/charuco_validation_samples.json \
  -p calibration_file:=/home/jetson/charuco_handeye_result.json
```

加载标定结果后，会按每帧观测发布 `dummy -> base_chessboard`。
确保其它节点没有同时发布同名 child frame；需要并排看不同结果时改 `board_frame`。
采集 8～12 个未用于解算的新姿态，然后运行：

```bash
ros2 run jetcobot_moveit charuco_handeye_solve \
  --input /home/jetson/charuco_validation_samples.json \
  --calibration /home/jetson/charuco_handeye_result.json \
  --compare-calibration /home/jetson/handeye_result.json
```

验证不重新拟合，拒绝使用已参与拟合的样本和时间戳，并检查板参数、帧名和内参一致性。
结果默认写到 `/home/jetson/charuco_handeye_validation.json`，重点看：

| 字段 | 含义 |
| --- | --- |
| `metrics.translation_rms_mm` / `translation_max_mm` | 相对原标定棋盘基座位姿的误差，含均值偏移 |
| `batch_scatter.translation_rms_mm` / `rotation_rms_deg` | 新手眼变换下，同批新观测的棋盘位姿离散 |
| `comparison.batch_scatter` | 旧手眼变换下，同一批 ChArUco 观测的棋盘位姿离散 |
| `reference_shift` | 新观测的均值与原标定棋盘位姿的偏移 |

新结果的 `batch_scatter` 明显小于 `comparison.batch_scatter`，才说明在这批数据上
新手眼变换使基座棋盘位姿更稳定。这个比较让两个手眼变换使用相同整板观测；
它不单独测量单 tag 与整板检测器的噪声差别。批内离散小仍不能排除统一偏移，
绝对准确性还需要实测棋盘角点在基座坐标中的位置。若底座/棋盘在两批之间移动，
原参考误差会包含这次移动，应重新固定并采集；需要换参考时支持共用求解器的 `--reference-report`。

多角点能降低视觉观测的不稳定性，但无法修正错误内参、纸张弯曲、关节零偏、
机械回差或错误的相机/机械臂时间同步。是否改善要以新姿态验证数据为准。
