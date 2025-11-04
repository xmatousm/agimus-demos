from controller_manager.launch_utils import (
    generate_load_controller_launch_description,  # noqa: I001
)
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    OpaqueFunction,
    Shutdown,
)
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(
        context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    kuka_controllers_params = LaunchConfiguration("kuka_controllers_params")

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            kuka_controllers_params,
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        on_exit=Shutdown(),
    )

    spawn_default_controller = generate_load_controller_launch_description(
        "joint_state_broadcaster"
    )

    return [
        controller_manager_node,
        spawn_default_controller,
    ]


def generate_launch_description():
    return LaunchDescription(
        [OpaqueFunction(function=launch_setup)]
    )
