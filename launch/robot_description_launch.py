import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from sdformat_tools.urdf_generator import UrdfGenerator
from xmacro.xmacro4sdf import XMLMacro4sdf


def launch_setup(context: LaunchContext) -> list:
    """
    NOTE: Using OpaqueFunction in order to get the context in string format...
    But it is too hacky and not recommended.
    """

    use_sim_time = LaunchConfiguration("use_sim_time")
    source_list = LaunchConfiguration("source_list")
    offset_timestamp = LaunchConfiguration("offset_timestamp")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_rviz = LaunchConfiguration("use_rviz")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # Load the robot xmacro file from the launch configuration
    xmacro = XMLMacro4sdf()
    xmacro.set_xml_file(context.launch_configurations["robot_xmacro_file"])

    # Generate SDF from xmacro
    xmacro.generate()
    robot_xml = xmacro.to_string()

    # Generate URDF from SDF
    urdf_generator = UrdfGenerator()
    urdf_generator.parse_from_sdf_string(robot_xml)
    robot_urdf_xml = urdf_generator.to_string()

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    start_joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "rate": 200,
                "source_list": source_list,
                "offset_timestamp": offset_timestamp,
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=remappings,
    )

    start_robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "publish_frequency": 200.0,
                "robot_description": robot_urdf_xml,
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=remappings,
    )

    start_rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
        remappings=remappings,
    )

    return [
        stdout_linebuf_envvar,
        colorized_output_envvar,
        start_joint_state_publisher_node,
        start_robot_state_publisher_node,
        start_rviz_node,
    ]


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("pb2025_robot_description")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        "robot_name",
        default_value="pb2025_sentry_robot",
        description="The file name of the robot xmacro to be used",
    )

    declare_robot_xmacro_file_cmd = DeclareLaunchArgument(
        "robot_xmacro_file",
        default_value=[
            # Use TextSubstitution to concatenate strings
            TextSubstitution(text=os.path.join(bringup_dir, "resource", "xmacro", "")),
            LaunchConfiguration("robot_name"),
            TextSubstitution(text=".sdf.xmacro"),
        ],
        description="The file path of the robot xmacro to be used",
    )

    declare_source_list_cmd = DeclareLaunchArgument(
        "source_list",
        default_value="['serial/gimbal_joint_state']",
        description="Array of topic names for subscriptions to sensor_msgs/msg/JointStates. Defaults to ['serial/gimbal_joint_state']",
    )

    declare_offset_timestamp_cmd = DeclareLaunchArgument(
        "offset_timestamp",
        default_value="0.0",
        description="Offset timestamp for joint state publisher",
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(bringup_dir, "rviz", "visualize_robot.rviz"),
        description="Full path to the RViz config file to use",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Whether to start RViz"
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_robot_xmacro_file_cmd)
    ld.add_action(declare_source_list_cmd)
    ld.add_action(declare_offset_timestamp_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add the actions to launch all of the nodes
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
