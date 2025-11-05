from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler, \
    IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.launch_description_entity import LaunchDescriptionEntity
from launch_ros.actions import Node
from launch.substitutions import (
    LaunchConfiguration,
)

from agimus_demos_common.launch_utils_kuka import (
    generate_default_kuka_args,
    get_use_sim_time,
    path_join,
)

PKG = "agimus_demo_02_simple_pd_plus_kuka"


def launch_setup(
        context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    kuka_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path_join("launch", "kuka", "kuka_common_lfc.launch.py")
        )
    )

    # do not launch the rest if we are on the aux launch
    on_aux_bool = context.perform_substitution(
        LaunchConfiguration("on_aux")).lower() == "true"

    if on_aux_bool:
        return [kuka_robot_launch]

    pd_plus_controller_params = path_join(
        "config", "pd_plus_controller_params.yaml", pkg=PKG)

    wait_for_non_zero_joints_node = Node(
        package="agimus_demos_common",
        executable="wait_for_non_zero_joints_node",
        parameters=[get_use_sim_time(), {'timeout': 10.0}],
        output="screen",
    )

    pd_plus_controller_node = Node(
        package="linear_feedback_controller",
        executable="pd_plus_controller",
        parameters=[get_use_sim_time(), pd_plus_controller_params],
        output="screen",
    )

    return [
        kuka_robot_launch,
        wait_for_non_zero_joints_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=wait_for_non_zero_joints_node,
                on_exit=[pd_plus_controller_node],
            )
        ),
    ]

def generate_launch_description():
    return LaunchDescription(
        generate_default_kuka_args() + [OpaqueFunction(function=launch_setup)]
    )
