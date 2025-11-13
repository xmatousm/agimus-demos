from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
)
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from agimus_demos_common.launch_utils_kuka import (
    path_join,
    generate_default_kuka_args,
)


def launch_setup(
        context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    controllers_names = [
        "linear_feedback_controller",
        "joint_state_estimator",
    ]

    controller_params = LaunchConfiguration("linear_feedback_controller_params")

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path_join("launch", "kuka", "kuka_common.launch.py")
        ),
        launch_arguments=(
            ("external_controllers_names", str(controllers_names)),
            ("external_controllers_params", controller_params),
        )
    )]


def generate_args():
    return [
        DeclareLaunchArgument(
            "linear_feedback_controller_params",
            default_value=path_join(
                "config", "kuka",
                PythonExpression(
                    ['"', LaunchConfiguration("robot_name"),
                     '_linear_feedback_controller_params.yaml"'])),
            description="Path to the yaml file use to define "
                        + "Linear Feedback Controller's and Joint State Estimator's params.",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        generate_default_kuka_args()
        + generate_args()
        + [OpaqueFunction(function=launch_setup)]
    )
