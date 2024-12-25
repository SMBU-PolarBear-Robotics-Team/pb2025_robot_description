import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from sdformat_tools.urdf_generator import UrdfGenerator
from xmacro.xmacro4sdf import XMLMacro4sdf


def generate_launch_description():
    # Get the launch directory
    pkg_pb2025_robot_description_dir = get_package_share_directory(
        "pb2025_robot_description"
    )

    xmacro_description = os.path.join(
        pkg_pb2025_robot_description_dir,
        "resource",
        "xmacro",
        "pb2025_sentry_robot.sdf.xmacro",
    )
    rviz_config_file = os.path.join(
        pkg_pb2025_robot_description_dir,
        "rviz",
        "visualize_robot.rviz",
    )

    use_sim_time = LaunchConfiguration("use_sim_time", default="False")

    xmacro = XMLMacro4sdf()
    xmacro.set_xml_file(xmacro_description)

    # Generate SDF from xmacro
    xmacro.generate()
    robot_xml = xmacro.to_string()

    # Generate URDF from SDF
    urdf_generator = UrdfGenerator()
    urdf_generator.parse_from_sdf_string(robot_xml)
    robot_urdf_xml = urdf_generator.to_string()

    start_joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
            }
        ],
    )

    start_robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": robot_urdf_xml,
            }
        ],
    )

    start_rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the actions to launch all nodes
    ld.add_action(start_joint_state_publisher_node)
    ld.add_action(start_robot_state_publisher_node)
    ld.add_action(start_rviz_node)

    return ld
