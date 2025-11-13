from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import LaunchConfiguration

from agimus_demos_common.launch_utils_kuka import path_join, include_path_join

PKG = "agimus_demo_00_kuka_controller"


def launch_setup(
        context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    preset = LaunchConfiguration("preset").perform(context)

    if preset == 'default':
        controller_names = ["effort_example_controller"]
    else:
        controller_names = ["effort_example_controller_" + preset]

    controller_params = path_join(
        "config", "effort_example_controller.yaml", pkg=PKG)

    return [include_path_join(
        "launch", "kuka", "kuka_common.launch.py",
        launch_arguments=(
            ("external_controllers_names", str(controller_names)),
            ("external_controllers_params", controller_params),
        )
    )]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "preset",
                default_value="default",
                description="Effort example controller parameters preset.",
                choices=["default", "home0", "home1", "home2", "home3"],
            )]
        + [OpaqueFunction(function=launch_setup)]
    )
