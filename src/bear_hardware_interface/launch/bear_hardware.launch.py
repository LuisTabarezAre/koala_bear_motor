from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("bear_hardware_interface"), "description", "koala.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("bear_hardware_interface"), "config", "bear_controllers.yaml"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "--inactive", "--controller-manager", "/controller_manager"],
    )

    effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["effort_controller", "--inactive", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        velocity_controller_spawner,
        effort_controller_spawner
    ])