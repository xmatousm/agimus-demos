from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from controller_manager.launch_utils import (
    generate_controllers_spawner_launch_description,
)

from agimus_demos_common.launch_utils_kuka import (
    path_join,
    include_path_join,
)

def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:

    gz_verbose_bool = LaunchConfiguration("gz_verbose").perform(context).lower() == "true"
    gz_headless_bool = LaunchConfiguration("gz_headless").perform(context).lower() == "true"
    gz_gui_config_path_str = path_join("config", "gz_gui.config").perform(context)
    robot_name_str = LaunchConfiguration("robot_name").perform(context)

    world = path_join("config", "kuka", "gazebo_empty_world.sdf").perform(context)

    gazebo_empty_world = include_path_join(
        "launch", "gz_sim.launch.py", pkg="ros_gz_sim",
        launch_arguments={
            "gz_args": world + " -r"
            + f" {'-s' if gz_headless_bool else ''}"
            + f" {'-v 3' if gz_verbose_bool else ''}"
            + f" --gui-config {gz_gui_config_path_str}"
        }
    )

    ros_gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "expand_gz_topic_names": True,
                "use_sim_time": True,
                "config_file": path_join("config", "gz_bridge.yaml")
            }
        ],
        output="screen",
    )

    robot_spawner_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", f"/{robot_name_str}/robot_description"], # TODO
        parameters=[{"use_sim_time": True}],
        output="screen",
        remappings=[
            ('/robot_state_publisher', f'/{robot_name_str}/robot_state_publisher'),
            ],
        namespace=robot_name_str,
       )

    spawn_default_controllers = generate_controllers_spawner_launch_description(
        [
            "joint_state_broadcaster",
        ],
        extra_spawner_args=["--controller-manager",
                            f"/{robot_name_str}/controller_manager"],
    )

    return [
        gazebo_empty_world,
        ros_gz_bridge_node,
        robot_spawner_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=robot_spawner_node,
                on_exit=[spawn_default_controllers],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [OpaqueFunction(function=launch_setup)]
    )
