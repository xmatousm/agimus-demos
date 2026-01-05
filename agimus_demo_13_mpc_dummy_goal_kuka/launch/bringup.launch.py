from launch import LaunchContext, LaunchDescription
from launch.actions import (
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
    DeclareLaunchArgument,
)

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
    include_path_join,
    wait_for_non_zero_joints_run,
    remap_to_ns,
    required_node,
)
from agimus_demos_common.static_transform_publisher_node import (
    static_transform_publisher_node,
)

from agimus_demos_common.mpc_debugger_node import mpc_debugger_node

PKG = "agimus_demo_13_mpc_dummy_goal_kuka"


def launch_setup(
        context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    kuka_robot_launch = include_path_join(
        "launch", "kuka", "kuka_common_lfc.launch.py")

    # do not launch the rest if we are on the aux launch
    if LaunchConfiguration("on_aux").perform(context).lower() == "true":
        return [kuka_robot_launch]

    robot_name_str = LaunchConfiguration("robot_name").perform(context)

    ocp_choice_arg = LaunchConfiguration("ocp")
    use_mpc_debugger = LaunchConfiguration("use_mpc_debugger")
    use_mpc_debugger_str = use_mpc_debugger.perform(context)

    use_collision_detection = (
            context.perform_substitution(ocp_choice_arg).lower()
            == "custom_with_collision_avoidance"
    )

    agimus_controller_yaml = path_join(
        "config",
        LaunchConfiguration("controller_config_file").perform(context),
        pkg=PKG)

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

    agimus_controller_node = Node(
        package="agimus_controller_ros",
        executable="agimus_controller_node",
        parameters=[
            get_use_sim_time(),
            agimus_controller_yaml,
            extra_params,
        ],
        output="screen",
        remappings=remap_to_ns(robot_name_str,
                               "robot_description",
                               "environment_description",
                               'linear_feedback_controller/get_parameters'
                               ),
        namespace=robot_name_str,
    )

    trajectory_weights_yaml = (
        path_join("config",
                  LaunchConfiguration("trajectory_config_file").perform(
                      context),
                  pkg=PKG))

    simple_trajectory_goal_publisher_node = Node(
        package="agimus_demos_common",
        executable="simple_trajectory_goal_publisher",
        parameters=[get_use_sim_time(), trajectory_weights_yaml],
        output="screen",
        namespace=robot_name_str,
    )

    trajectory_goal_server_node = Node(
        package="agimus_demos_common",
        executable="trajectory_goal_server",
        parameters=[get_use_sim_time()],
        output="screen",
        namespace=robot_name_str,
        remappings=remap_to_ns(robot_name_str,
                               "robot_description",
                               'linear_feedback_controller/get_parameters',
                               'agimus_controller_node/get_parameters',
                               ),
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
        namespace=robot_name_str
    )
    tf_node = static_transform_publisher_node(
        frame_id=f"{robot_name_str}_link_0",
        child_frame_id="obstacle1",
    )

    mpc_debugger = mpc_debugger_node(
        f"{robot_name_str}_link_tool",
        parent_frame=f"{robot_name_str}_link_0",
        cost_plot=use_mpc_debugger_str == 'full',
        node_kwargs=dict(
            remappings=[
                ("/robot_description",
                 f"/{robot_name_str}/robot_description_with_collision"),
                *remap_to_ns(robot_name_str,
                             "environment_description",
                             'linear_feedback_controller/get_parameters',
                             'agimus_controller_node/get_parameters',
                             ),
            ],
            condition=IfCondition(
                PythonExpression(["'", use_mpc_debugger_str, "' != 'false'"])),
            namespace=robot_name_str
        ),
    )

    return [
        kuka_robot_launch,
        *wait_for_non_zero_joints_run(robot_name_str,
                                      [agimus_controller_node,
                                       environment_publisher_node]
                                      ),
        tf_node,
        *required_node(mpc_debugger),
        *required_node(simple_trajectory_goal_publisher_node),
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=agimus_controller_node,
                on_start=TimerAction(
                    period=2.0,
                    actions=[trajectory_goal_server_node],
                ),
            )
        ),
        required_node(trajectory_goal_server_node)[1],

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
            description="The ocp to use. Either the default one or the one from this package that does collision avoidance.",
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
            "controller_config_file",
            default_value="agimus_controller_params.yaml",
            description="Agimus controller configuration YAML file.",
        ),
        DeclareLaunchArgument(
            "obstacles_config_file",
            default_value="obstacles_none.xacro",
            description="Obstacles definition XACRO file.",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        generate_args()
        + generate_default_kuka_args()
        + [OpaqueFunction(function=launch_setup)]
    )
