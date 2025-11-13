from launch import LaunchContext, LaunchDescription
from launch.actions import (
    OpaqueFunction,
)
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import (
    LaunchConfiguration,
)
from launch_ros.actions import Node

from agimus_demos_common.launch_utils_kuka import (
    generate_default_kuka_args,
    get_use_sim_time,
)

def launch_setup(
        context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    robot_name_str = LaunchConfiguration("robot_name").perform(context)

    rviz_config_path = LaunchConfiguration("rviz_config_path")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        parameters=[get_use_sim_time()],
        arguments=["--display-config", rviz_config_path],
        namespace=robot_name_str,
    )

    print(rviz_config_path.perform(context))
    return [
        rviz_node,
    ]

def generate_launch_description():
    return LaunchDescription(
        generate_default_kuka_args()
        + [OpaqueFunction(function=launch_setup)]
    )
