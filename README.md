# pb2025_robot_description

SMBU PolarBear Team robot description package for RoboMaster 2025.

深圳北理莫斯科大学北极熊战队 - RoboMaster 2025 赛季通用机器人关节描述包。

## 1. 项目说明

本项目使用 [xmacro](https://github.com/gezp/xmacro) 格式描述机器人关节信息，可以更灵活的组合已有模型。

当前机器人描述文件基于 [rmua19_standard_robot](https://github.com/robomaster-oss/rmoss_gz_resources/tree/humble/resource/models/rmua19_standard_robot) 进行二次编辑，加入了工业相机和激光雷达等传感器。

- [pb2025_sentry_robot](src/pb2025_robot_description/resource/xmacro/pb2025_sentry_robot.sdf.xmacro)

    搭载云台相机 industrial_camera 和激光雷达 rplidar_a2 和 Livox mid360，其中相机与 gimbal_pitch 轴固连，mid360 倾斜侧放与 chassis 固连。

    ![sentry](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/sentry_description.1sf3yc69kr.webp)

## 2. 环境配置

- Ubuntu 22.04
- ROS: Humble

1. 安装依赖

    注：若同一工作空间中已克隆过 rmoss_gz_resources 和 sdformat_tools，请跳过此克隆步骤。

    ```bash
    git clone https://github.com/LihanChen2004/rmoss_gz_resources.git --depth=1
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
    git clone https://github.com/SMBU-PolarBear-Robot-Team/pb2025_robot_description.git
    ```

3. 编译

    ```bash
    colcon build --symlink-install
    ```

## 3. 使用说明

- 编写了一个用于测试的 launch 文件，可以在 RViz 中查看机器人模型

    ```bash
    ros2 launch pb2025_robot_description test_robot_description_launch.py
    ```

- 通过 Python API，在 launch file 中解析 xmacro 文件，生成 URDF 和 SDF 文件（推荐）

    > Tips:
    >
    > [robot_state_publisher](https://github.com/ros/robot_state_publisher) 需要传入 urdf 格式的机器人描述文件
    >
    > Gazebo 仿真器 spawn robot 时，需要传入 sdf / urdf 格式的机器人描述文件

    感谢前辈的开源工具 [xmacro](https://github.com/gezp/xmacro) 和 [sdformat_tools](https://github.com/gezp/sdformat_tools) ，这里简述 xmacro 转 urdf 和 sdf 的示例，用于在 launch file 中生成 URDF 和 SDF 文件。

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

- 通过命令行直接转换输出 sdf 文件（不推荐）

    ```bash
    source install/setup.bash

    xmacro4sdf src/pb2025_robot_description/resource/xmacro/pb2025_sentry_robot.sdf.xmacro > src/pb2025_robot_description/resource/xmacro/pb2025_sentry_robot.sdf
    ```
