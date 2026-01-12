from launch import LaunchContext, LaunchDescription
from launch.actions import (
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
    DeclareLaunchArgument,
)

from launch.event_handlers import OnProcessStart
from launch.launch_description_entity import LaunchDescriptionEntity
from launch_ros.actions import Node

from agimus_demos_common.launch_utils_kuka import (
    generate_default_kuka_args,
    generate_mpc_args,
    get_use_sim_time,
    parameter_value_xacro,
    include_path_join,
    wait_for_non_zero_joints_run,
    remap_to_ns,
    required_node,
    SetupContext,
)

import agimus_demos_common.launch_nodes as nodes

from agimus_demos_common.static_transform_publisher_node import (
    static_transform_publisher_node,
)

PKG = "agimus_demo_03_mpc_dummy_traj_kuka"


def launch_setup(
        context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    ctx = SetupContext(context, PKG)

    kuka_robot_launch = include_path_join(
        "launch", "kuka", "kuka_common_lfc.launch.py")

    # do not launch the rest if we are on the aux launch
    if ctx.config_bool("on_aux"):
        return [kuka_robot_launch]

    robot_name = ctx.config("robot_name")
    use_mpc_debugger = ctx.config("use_mpc_debugger")
    use_mod_publisher = ctx.config_bool("mod_publisher")
    agimus_controller_yaml = ctx.config_path("config", "controller_config_file")
    ocp_definition_file = ctx.config_path("config", "ocp_definition_file")

    agimus_controller_node = nodes.agimus_controller(
        robot_name, agimus_controller_yaml, ocp_definition_file)

    trajectory_weights_yaml = ctx.config_path(
        "config", "trajectory_config_file")

    if use_mod_publisher:
        simple_trajectory_publisher_node = Node(
            package="agimus_demos_common",
            executable="simple_trajectory_publisher_mod",
            parameters=[get_use_sim_time(), trajectory_weights_yaml],
            output="screen",
            namespace=robot_name,
            remappings=remap_to_ns(robot_name,
                                   "robot_description",
                                   'linear_feedback_controller/get_parameters',
                                   'agimus_controller_node/get_parameters',
                                   ),
            ros_arguments=["--log-level",
                           "lbr.simple_trajectory_publisher:=debug"],
        )
    else:
        simple_trajectory_publisher_node = Node(
            package="agimus_controller_ros",
            executable="simple_trajectory_publisher",
            parameters=[get_use_sim_time(), trajectory_weights_yaml],
            output="screen",
            namespace=robot_name,
            remappings=remap_to_ns(robot_name,
                                   "robot_description",
                                   'linear_feedback_controller/get_parameters',
                                   'agimus_controller_node/get_parameters',
                                   ),
        )

    obstacles_config_path = ctx.config_path("urdf", "obstacles_config_file")

    environment_description = parameter_value_xacro(obstacles_config_path)

    environment_publisher_node = nodes.environment_publisher(
        robot_name, environment_description)

    tf_node = static_transform_publisher_node(
        frame_id=f"{robot_name}_link_0",
        child_frame_id="obstacle1",
    )

    mpc_debugger = nodes.mpc_debugger(robot_name, use_mpc_debugger)

    return [
        kuka_robot_launch,
        *wait_for_non_zero_joints_run(robot_name,
                                      [agimus_controller_node,
                                       environment_publisher_node]
                                      ),
        tf_node,
        *required_node(mpc_debugger),
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=agimus_controller_node,
                on_start=TimerAction(
                    period=2.0,
                    actions=[simple_trajectory_publisher_node],
                ),
            )
        ),
        required_node(simple_trajectory_publisher_node)[1]
    ]


def generate_args():
    return [
        DeclareLaunchArgument(
            "trajectory_config_file",
            default_value="trajectory_weigths_params.yaml",
            description="Trajectory configuration YAML file.",
        ),
        DeclareLaunchArgument(
            "mod_publisher",
            default_value="false",
            description="Whether to use modified trajectory publisher.",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        generate_args()
        + generate_mpc_args()
        + generate_default_kuka_args()
        + [OpaqueFunction(function=launch_setup)]
    )
