# h1_bringup

ROS 2 H1 机器人状态可视化包。把 Unitree `/lowstate` 转为标准 `/joint_states`，通过 robot_state_publisher 和 RViz 展示机器人关节姿态。本包不发送运动控制指令，也不包含发布 `/lowstate` 的机器人通信驱动。

## 数据流

```text
外部机器人通信程序 → /lowstate (unitree_go/msg/LowState)
                         ↓ lowstate_to_joint_states.py
                    /joint_states (sensor_msgs/msg/JointState)
                         ↓ robot_state_publisher + URDF
                       /tf → RViz
```

## 文件结构

- `launch/h1_real_rviz.launch.py`：启动状态转换、robot_state_publisher、RViz 和相机静态 TF。
- `scripts/lowstate_to_joint_states.py`：提取电机位置、速度并应用关节角度偏移。
- `config/h1_joint_map.yaml`：19 个关节名称与电机数组下标的映射。
- `model/`：完整复制自本机 `/home/xuanhaosong/Desktop/h1_model`，包含 `h1/`（URDF、meshes、MuJoCo XML）、`assets/` 和标定脚本及配置。启动使用 `model/h1/urdf/h1_rviz.urdf`，其中网格路径已改成 `package://h1_bringup/model/h1/meshes/...`。MuJoCo 使用相对路径，已修正旧模型中的网格目录及脚踝文件名大小写；标定配置按脚本所在目录加载。标定脚本不在本包启动流程中。
- `third_party/unitree_go.tar.gz`：从相邻 `humanoid_bridge/thirdparty/unitree/unitree/unitree_go` 保存的消息包源码副本。使用压缩包避免工作空间扫描出重复的 ROS 包。

## 环境与依赖

本机安装的是 ROS 2 Humble。运行需要 `rclpy`、`sensor_msgs`、`launch`、`launch_ros`、`robot_state_publisher`、`rviz2`、`tf2_ros`、PyYAML 和 `unitree_go`。通用运行依赖已写入 `package.xml`；ROS 2 本身和系统库不复制进项目。

### 准备 Unitree 消息包

**本项目已保存 `unitree_go` 的源码副本，不需要再从外部 `humanoid_bridge` 获取源码。但压缩包不会自动参与编译，运行时仍需要已编译并加载的 `unitree_go` 消息包。**

根据工作空间情况选择：

- 当前工作空间：相邻的 `humanoid_bridge` 已包含 `unitree_go`，直接编译现有包，不要重复解压。同一个工作空间不能同时存在两个名为 `unitree_go` 的源码包。
- 新机器或独立工作空间：使用本项目随附压缩包，按下面步骤解压、安装依赖并编译，无需复制整个 `humanoid_bridge`。
- 已在其他工作空间安装 `unitree_go`：可先 source 那个工作空间的 `install/setup.bash`，然后编译本包；这种方式仍依赖那个外部安装环境。

在新工作空间中，可把随附副本解压到 `src/`（以下假设本项目位于 `src/h1_bringup`）：

```bash
cd <工作空间>
tar -xzf src/h1_bringup/third_party/unitree_go.tar.gz -C src
```

副本保留上游消息定义、CMake 配置和依赖要求；其 CMake 还使用 `rosidl_generator_dds_idl`，需要在环境中可用。它声明的许可证为 BSD 3-Clause，模型文件保留源文件中的声明。

### 安装依赖、编译

```bash
source /opt/ros/humble/setup.bash
cd <工作空间>
rosdep install --from-paths src/h1_bringup --ignore-src -r -y
rosdep install --from-paths src/unitree_go --ignore-src -r -y
colcon build --packages-up-to h1_bringup --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

上面第二条 `rosdep` 命令适用于刚解压到 `src/unitree_go` 的情况；如果使用当前 `humanoid_bridge` 中的源码，把路径改为 `src/humanoid_bridge/thirdparty/unitree/unitree/unitree_go`。若使用其他工作空间中已安装的消息包，则跳过该条命令，并先 source 对应环境。`rosdep` 不能替代 Unitree 自定义消息包源码。

编译并 source 后，检查消息包是否可用：

```bash
ros2 pkg prefix unitree_go
python3 -c 'from unitree_go.msg import LowState; print("unitree_go 可用")'
```

如果提示 `No module named 'unitree_go'`，检查是否已经解压、编译，以及当前终端是否执行了 `source install/setup.bash`。每次打开新终端都需要重新加载 ROS 2 和工作空间环境。

模型和 Unitree 消息源码已随项目保存；ROS 2、系统依赖以及发布 `/lowstate` 的外部通信程序仍需准备。消息包只提供消息类型，不会自行连接机器人或发布状态。

## 启动

先通过外部驱动或 DDS 程序发布 `/lowstate`，然后执行：

```bash
source /opt/ros/humble/setup.bash
source <工作空间>/install/setup.bash
ros2 launch h1_bringup h1_real_rviz.launch.py
```

无需再依赖 Desktop 下的模型目录。RViz 暂无预设配置：把 Fixed Frame 设为 `pelvis`，添加 RobotModel，Description Source 选择 Topic，Description Topic 选择 `/robot_description`。

启动还会发布零平移、零旋转的 `camera_frame → zed_camera_link` 静态 TF；该变换沿用原配置，不代表实际相机外参标定结果。本包只显示关节姿态，没有融合 IMU 或里程计来恢复机身在世界中的运动。

## 关节映射与偏移

修改 `config/h1_joint_map.yaml` 可调整电机下标，`name` 必须与 URDF 关节名称一致。索引越界时当前脚本输出零位置、零速度。

关节角度按 `电机位置 + offset` 发布，单位为弧度；偏移初始为零。可在运行时更新，例如：

```bash
ros2 topic pub --once /joint_offsets sensor_msgs/msg/JointState \
  "{name: ['left_shoulder_pitch_joint'], position: [0.1]}"
```

偏移只影响显示，节点重启后不保留。转换脚本还支持 `lowstate_topic`、`joint_states_topic`、`joint_map_yaml` 三个 ROS 参数；当前 launch 固定传入默认话题和包内映射配置。

### 双臂关节偏移配置

启动本包后，在另一个已加载 ROS 2 和工作空间环境的终端中执行以下命令，一次设置双臂的 8 个关节偏移。`position` 中的值按 `name` 顺序对应，单位为弧度：

```bash
ros2 topic pub --once /joint_offsets sensor_msgs/msg/JointState "{name: ['left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint', 'left_elbow_joint', 'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint', 'right_elbow_joint'], position: [2.9, 3.858, 0.025, -0.075, 2.8719125, -3.87465, -0.010, -0.05]}"
```

这组数值由使用者提供；转换节点重启后需要重新执行。它会覆盖上面示例中设置的左肩俯仰偏移。

## 排查

```bash
ros2 topic info /lowstate --verbose
ros2 topic echo /joint_states --once
ros2 run tf2_ros tf2_echo pelvis left_elbow_link
```

- 无关节数据：检查外部发布程序、消息类型、ROS_DOMAIN_ID 和 DDS 网络配置。
- QoS 不兼容：当前 `/lowstate` 订阅为 RELIABLE、depth=1，`/joint_states` 发布为 BEST_EFFORT、depth=5，需要上下游兼容。
- 模型缺失：重新编译并 source 安装环境，确认包内 `model/` 已安装。
- 姿态不符：检查电机索引、URDF 关节轴方向和角度偏移。

目前没有自动化功能测试；启动与实机通信需要在 ROS 环境中验证。

### 附带旧模型的限制

`model/assets/mjcf/new_h1.xml` 原始文件引用的 `model/assets/common/scene.xml` 和 `mimic_sites.xml` 在源目录中就不存在，因此该旧模型仍不能独立加载。未用其他文件替代，以免改变模型含义；本包 RViz 启动使用的 URDF 不依赖这两个文件。
