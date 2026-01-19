from launch import LaunchContext, LaunchDescription
from launch.actions import (
    OpaqueFunction,
    DeclareLaunchArgument,
)

from launch.launch_description_entity import LaunchDescriptionEntity

from agimus_demos_common.launch_utils_kuka import (
    generate_default_kuka_args,
    path_join,
    required_node,
    SetupContext,
)

import agimus_demos_common.launch_nodes as nodes

PKG = "agimus_demo_14_cardboard_detect"


def launch_setup(
        context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    ctx = SetupContext(context, PKG)

    camera_embedded = ctx.config_bool("camera_embedded")
    cardboard_yaml = path_join("config", "cardboard.yaml", pkg=PKG)
    calib_file = path_join("config", "calib_cam.yaml", pkg=PKG)
    template_file = path_join("config", "template.png", pkg=PKG)
    if ctx.config_bool("simulate"):
        sample_file = path_join("config", "image.png", pkg=PKG)
    else:
        sample_file = None

    launch = []

    if not camera_embedded:
        camera_node = nodes.camera(
            cardboard_yaml, calib_file, sample_file, debug=True)

        launch += [*required_node(camera_node)]

        calib_file = None
        sample_file = None

    detector_node = nodes.detector(
        cardboard_yaml, template_file,
        calib_file=calib_file, simulate_file=sample_file, debug=True)

    launch += [*required_node(detector_node)]

    return launch


def generate_args():
    return [
        DeclareLaunchArgument(
            "camera_embedded",
            default_value="false",
            description="Camera is embedded with detector node or not.",
        ),

        DeclareLaunchArgument(
            "simulate",
            default_value="false",
            description="Whether to simulate a camera image.",
        ),

    ]

def generate_launch_description():
    return LaunchDescription(
        generate_args()
        + generate_default_kuka_args()
        + [OpaqueFunction(function=launch_setup)]
    )
