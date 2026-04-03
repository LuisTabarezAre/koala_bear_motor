from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Obtener el URDF a través de xacro
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

    # 2. Ruta al archivo de configuración de los controladores
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("bear_hardware_interface"), "config", "bear_controllers.yaml"]
    )

    # 3. Nodo del Controller Manager
    # Importante: Aquí pasamos tanto el URDF como el YAML de los controladores
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    # 4. Publicador del estado del robot (robot_state_publisher)
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # 5. Spawners (Cargan y activan los controladores automáticamente)
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

    # Retornamos la lista de nodos a ejecutar
    return LaunchDescription([
        control_node,
        robot_state_pub_node, # Nombre corregido aquí
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
    ])