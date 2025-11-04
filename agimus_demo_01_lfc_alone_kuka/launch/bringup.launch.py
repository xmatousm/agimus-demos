from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.launch_description_sources import PythonLaunchDescriptionSource

from agimus_demos_common.launch_utils_kuka import (
    generate_default_kuka_args,
    path_join,
)

def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path_join("launch", "kuka", "kuka_common_lfc.launch.py")
        )
    )]

def generate_launch_description():
    return LaunchDescription(
        generate_default_kuka_args() + [OpaqueFunction(function=launch_setup)]
    )
