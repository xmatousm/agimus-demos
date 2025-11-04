from launch import LaunchContext, LaunchDescription
from launch.actions import (
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
)

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.events import (Shutdown)

from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from agimus_demos_common.launch_utils_kuka import (
    generate_default_kuka_args,
    get_use_sim_time,
    parameter_value_xacro,
    path_join,
    LogError,
)
from agimus_demos_common.static_transform_publisher_node import (
    static_transform_publisher_node,
)

from agimus_demos_common.mpc_debugger_node import mpc_debugger_node

PKG = "agimus_demo_03_mpc_dummy_traj_kuka"


def launch_setup(
        context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    kuka_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path_join("launch", "kuka", "kuka_common_lfc.launch.py")
        )
    )

    ocp_choice_arg = LaunchConfiguration("ocp")
    use_mpc_debugger = LaunchConfiguration("use_mpc_debugger")
    use_mpc_debugger_str = use_mpc_debugger.perform(context)

    use_collision_detection = (
            context.perform_substitution(ocp_choice_arg).lower()
            == "custom_with_collision_avoidance"
    )

    agimus_controller_yaml = path_join(
        "config", "agimus_controller_params.yaml", pkg=PKG)

    if use_collision_detection:
        ocp_definition_file = path_join(
            "config",
            LaunchConfiguration("ocp_definition_file").perform(context),
            pkg=PKG)

        extra_params = {
            "ocp": {
                "definition_yaml_file": ocp_definition_file.perform(context)}
        }
    else:
        extra_params = {}

    wait_for_non_zero_joints_node = Node(
        package="agimus_demos_common",
        executable="wait_for_non_zero_joints_node",
        parameters=[get_use_sim_time(), {'timeout': 10.0}],
        output="screen",
    )

    agimus_controller_node = Node(
        package="agimus_controller_ros",
        executable="agimus_controller_node",
        parameters=[
            get_use_sim_time(),
            agimus_controller_yaml,
            extra_params,
        ],
        output="screen",
        remappings=[("robot_description", "robot_description_with_collision")],
    )

    trajectory_weights_yaml = (
        path_join("config",
                  LaunchConfiguration("trajectory_config_file").perform(
                      context),
                  pkg=PKG))

    simple_trajectory_publisher_node = Node(
        package="agimus_controller_ros",
        executable="simple_trajectory_publisher",
        parameters=[get_use_sim_time(), trajectory_weights_yaml],
        output="screen",
    )

    obstacles_config_path = path_join(
        "urdf",
        LaunchConfiguration("obstacles_config_file").perform(context),
        pkg=PKG)

    environment_description = parameter_value_xacro(obstacles_config_path)

    environment_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="environment_publisher",
        output="screen",
        remappings=[("robot_description", "environment_description")],
        parameters=[{"robot_description": environment_description}],
    )
    tf_node = static_transform_publisher_node(
        frame_id="lbr_link_0",
        child_frame_id="obstacle1",
    )

    mpc_debugger = mpc_debugger_node(
        "lbr_link_ee",
        parent_frame="lbr_link_0",
        cost_plot=use_mpc_debugger_str == 'full',
        node_kwargs=dict(
            remappings=[
                ("robot_description", "robot_description_with_collision")],
            condition=IfCondition(
                PythonExpression(["'", use_mpc_debugger_str, "' != 'false'"])),
        ),
    )

    mpc_debugger_required = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=mpc_debugger,
            on_exit=[
                LogError(msg="MPC debugger exitted."),
                EmitEvent(event=Shutdown())]))

    return [
        kuka_robot_launch,
        wait_for_non_zero_joints_node,
        tf_node,
        mpc_debugger,
        mpc_debugger_required,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=wait_for_non_zero_joints_node,
                on_exit=[
                    agimus_controller_node,
                    environment_publisher_node,
                ],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=agimus_controller_node,
                on_start=TimerAction(
                    period=2.0,
                    actions=[simple_trajectory_publisher_node],
                ),
            )
        ),
    ]


def generate_args():
    return [
        DeclareLaunchArgument(
            "use_mpc_debugger",
            default_value="false",
            description="Launches the mpc_debugger_node along.",
            choices=["false", "markers", "full"],
        ),
        DeclareLaunchArgument(
            "ocp",
            default_value="custom_with_collision_avoidance",
            description="Select the ocp to use. Either the default one or the one from this package that does collision avoidance.",
            choices=["default_ocp", "custom_with_collision_avoidance"]
        ),
        DeclareLaunchArgument(
            "ocp_definition_file",
            default_value="ocp_definition_file.yaml",
            description="OCP configuration YAML file.",
        ),
        DeclareLaunchArgument(
            "trajectory_config_file",
            default_value="trajectory_weigths_params.yaml",
            description="Trajectory configuration YAML file.",
        ),
        DeclareLaunchArgument(
            "obstacles_config_file",
            default_value="obstacles.xacro",
            description="Obstacles definition XACRO file.",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        generate_args()
        + [OpaqueFunction(function=launch_setup)]
    )
