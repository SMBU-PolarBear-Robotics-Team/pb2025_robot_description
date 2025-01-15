# pb2025_robot_description

![PolarBear Logo](https://raw.githubusercontent.com/SMBU-PolarBear-Robotics-Team/.github/main/.docs/image/polarbear_logo_text.png)

SMBU PolarBear Team robot description package for RoboMaster 2025.

深圳北理莫斯科大学北极熊战队 - RoboMaster 2025 赛季通用机器人关节描述包。

## 1. 概述

本功能包含深圳北理莫斯科大学北极熊战队用于 RoboMaster 赛事的机器人关节描述文件。它将读取机器人描述文件并转换为 TF 和 joint_states ，以便与其他 ROS2 节点配合使用。

## 2. 功能包结构

### 2.1 机器人描述文件

本项目使用 [xmacro](https://github.com/gezp/xmacro) 格式描述机器人关节信息，可以更灵活的组合已有模型。

当前机器人描述文件基于 [rmua19_standard_robot](https://github.com/robomaster-oss/rmoss_gz_resources/tree/humble/resource/models/rmua19_standard_robot) 进行二次编辑，加入了工业相机和激光雷达等传感器。

- [pb2025_sentry_robot](./resource/xmacro/pb2025_sentry_robot.sdf.xmacro)

    搭载云台相机 industrial_camera 和激光雷达 rplidar_a2 和 Livox mid360，其中相机与 gimbal_pitch 轴固连，mid360 倾斜侧放与 chassis 固连。

    ![sentry](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/sentry_description.1sf3yc69kr.webp)

    ![frames](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/frames.5xaq4wriyy.webp)

## 3. 订阅话题

None.

## 4. 发布话题

- `robot_description (std_msgs/msg/String)`

    机器人描述文件（字符串形式）。

- `joint_states (sensor_msgs/msg/JointState)`

    如果命令行中未给出 URDF，则此节点将侦听 `robot_description` 话题以获取要发布的 URDF。一旦收到一次，该节点将开始将关节状态发布到 `joint_states` 话题。

- `any_topic (sensor_msgs/msg/JointState)`

    如果 `sources_list` 参数不为空（请参阅下面的参数），则将订阅此参数中的每个命名话题以进行联合状态更新。不要将默认的 `joint_states` 话题添加到此列表中，因为它最终会陷入无限循环。

- `tf, tf_static (tf2_msgs/msg/TFMessage)`

    机器人关节坐标系信息。

## 5. Launch file 对外参数

- `use_sim_time (bool, default: False)`

    是否使用仿真时间。

- `robot_name (str, default: "pb2025_sentry_robot")`

    机器人 XMacro 描述文件的**名字（无需后缀）**。描述文件应位于 `package://pb2025_robot_description/resource/xmacro` 目录下。

- `robot_xmacro_file (str, default: "package://pb2025_robot_description/resource/xmacro/pb2025_sentry_robot.sdf.xmacro")`

    机器人 XMacro 描述文件的**绝对路径**。本参数的优先级高于 `robot_name`，即若设置了 `robot_xmacro_file`，则 `robot_name` 参数无效。若未设置 `robot_xmacro_file`，则使用 `robot_name` 参数并自动补全路径作为 `robot_xmacro_file` 的值。

- `source_list (array of strings, default: "['serial/gimbal_joint_state']")`

    该数组中的每个字符串代表一个话题名称。对于每个字符串，创建对 `sensor_msgs/msg/JointStates` 类型的命名话题的订阅。发布到该话题将更新 `joint_states` 中指定的关节状态。

- `rviz_config_file (str, default: "package://pb2025_robot_description/rviz/visualize_robot.rviz")`

    RViz 配置文件路径。

- `use_rviz (bool, default: True)`

    是否启动 RViz 可视化界面。

- `use_respawn (bool, default: False)`

    是否在节点退出时尝试重启节点。

- `log_level (str, default: "info")`

    日志级别。

## 6. 使用说明

## 6.1 安装

- Ubuntu 22.04
- ROS: Humble

1. 安装依赖

    注：若同一工作空间中已克隆过 rmoss_gz_resources 和 sdformat_tools，请跳过此克隆步骤。

    ```bash
    git clone https://github.com/SMBU-PolarBear-Robotics-Team/rmoss_gz_resources.git --depth=1
    git clone https://github.com/gezp/sdformat_tools.git
    ```

    ```bash
    pip install xmacro
    ```

    ```bash
    rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
    ```

2. 克隆本项目

    ```bash
    git clone https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_robot_description.git
    ```

3. 编译

    ```bash
    colcon build --symlink-install
    ```

### 6.2. 使用说明

- 在 RViz 中可视化机器人，请将此存储库安装到您的工作区并执行以下命令：

    ```bash
    ros2 launch pb2025_robot_description robot_description_launch.py
    ```

    启动文件 `robot_description_launch.py` ，其中的参数已在上文中介绍。

- 通过 Python API，在 launch file 中解析 XMacro 文件，生成 URDF 和 SDF 文件（推荐）

    > Tips:
    >
    > [robot_state_publisher](https://github.com/ros/robot_state_publisher) 需要传入 urdf 格式的机器人描述文件
    >
    > Gazebo 仿真器 spawn robot 时，需要传入 sdf / urdf 格式的机器人描述文件

    感谢前辈的开源工具 [xmacro](https://github.com/gezp/xmacro) 和 [sdformat_tools](https://github.com/gezp/sdformat_tools) ，这里简述 XMacro 转 URDF 和 SDF 的示例，用于在 launch file 中生成 URDF 和 SDF 文件。

    ```python
    from xmacro.xmacro4sdf import XMLMacro4sdf
    from sdformat_tools.urdf_generator import UrdfGenerator

    xmacro = XMLMacro4sdf()
    xmacro.set_xml_file(robot_xmacro_path)

    # Generate SDF from xmacro
    xmacro.generate()
    robot_xml = xmacro.to_string()

    # Generate URDF from SDF
    urdf_generator = UrdfGenerator()
    urdf_generator.parse_from_sdf_string(robot_xml)
    robot_urdf_xml = urdf_generator.to_string()
    ```

- 通过命令行直接转换输出 SDF 文件（不推荐）

    ```bash
    source install/setup.bash

    xmacro4sdf src/pb2025_robot_description/resource/xmacro/pb2025_sentry_robot.sdf.xmacro > src/pb2025_robot_description/resource/xmacro/pb2025_sentry_robot.sdf
    ```
