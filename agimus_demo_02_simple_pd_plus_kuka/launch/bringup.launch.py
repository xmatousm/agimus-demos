from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_entity import LaunchDescriptionEntity
from launch_ros.actions import Node

from agimus_demos_common.launch_utils_kuka import (
    generate_default_kuka_args,
    get_use_sim_time,
    path_join,
    include_path_join,
    SetupContext,
    remap_to_ns
)

PKG = "agimus_demo_02_simple_pd_plus_kuka"


def launch_setup(
        context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    ctx = SetupContext(context, PKG)

    kuka_robot_launch = include_path_join(
        "launch", "kuka", "kuka_common_lfc.launch.py")

    # do not launch the rest if we are on the aux launch
    if ctx.config_bool("on_aux"):
        return [kuka_robot_launch]

    robot_name_str = ctx.config("robot_name")

    pd_plus_controller_params = path_join(
        "config", robot_name_str + "_pd_plus_controller_params.yaml", pkg=PKG)

    pd_plus_controller_node = Node(
        package="linear_feedback_controller",
        executable="pd_plus_controller",
        parameters=[get_use_sim_time(), pd_plus_controller_params],
        output="screen",
        namespace=robot_name_str,
        remappings=remap_to_ns(robot_name_str, 'robot_description', 'control')
    )

    return [
        kuka_robot_launch,
        pd_plus_controller_node,
    ]


def generate_launch_description():
    return LaunchDescription(
        generate_default_kuka_args() + [OpaqueFunction(function=launch_setup)]
    )
