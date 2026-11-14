"""
bringup.launch.py -- start the controller manager with the payload hardware and its controllers.

  ros2 launch payload_bringup bringup.launch.py                              # mock hardware, URC
  ros2 launch payload_bringup bringup.launch.py hardware:=can can_interface:=vcan0 competition:=urc

Arguments
  hardware      mock  = ros2_control's built-in fake (no bus needed)
                can   = the real PayloadCanSystem plugin (needs a SocketCAN interface, e.g. vcan0)
  competition   urc | erc: which telemetry is real (the other competition's reads NaN)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hardware = LaunchConfiguration("hardware")
    can_interface = LaunchConfiguration("can_interface")
    competition = LaunchConfiguration("competition")

    # The mechanical URDF comes from its own package so a new CAD export can be dropped in.
    description_file = LaunchConfiguration("description_file")

    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ",
                PathJoinSubstitution([FindPackageShare("payload_bringup"), "urdf", "payload.urdf.xacro"]),
                " description_file:=", description_file,
                " hardware:=", hardware,
                " can_interface:=", can_interface,
                " competition:=", competition]),
        value_type=str)

    controllers = PathJoinSubstitution([FindPackageShare("payload_bringup"), "config", "controllers.yaml"])

    control_node = Node(package="controller_manager", executable="ros2_control_node",
                        parameters=[{"robot_description": robot_description}, controllers], output="screen")
    # robot_state_publisher turns joint states into TF; the URDF <mimic> tags make the two
    # prismatic joints follow their screws automatically.
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher", 
            parameters=[{"robot_description": robot_description}])

    def spawn(name):
        return Node(package="controller_manager", executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager"])

    return LaunchDescription([
        DeclareLaunchArgument("description_file", default_value=PathJoinSubstitution(
            [FindPackageShare("payload_bringup"), "urdf", "payload_description.urdf"])),
        DeclareLaunchArgument("hardware", default_value="mock"),
        DeclareLaunchArgument("can_interface", default_value="vcan0"),
        DeclareLaunchArgument("competition", default_value="urc"),
        control_node, rsp,
        spawn("joint_state_broadcaster"),
        spawn("main_position_controller"),
        spawn("drill_velocity_controller"),
        spawn("payload_io_controller"),
    ])
