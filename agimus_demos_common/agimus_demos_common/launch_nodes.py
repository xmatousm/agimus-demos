from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.substitution import Substitution
from launch_ros.parameter_descriptions import ParameterValue

from agimus_demos_common.launch_utils_kuka import (
    generate_default_kuka_args,
    get_use_sim_time,
    parameter_value_xacro,
    include_path_join,
    wait_for_non_zero_joints_run,
    remap_to_ns,
    required_node,
    SetupContext,
)

from agimus_demos_common.mpc_debugger_node import mpc_debugger_node


def environment_publisher(robot_name: str,
                          environment_description: ParameterValue) -> Node:
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="environment_publisher",
        output="screen",
        remappings=[("robot_description", "environment_description")],
        parameters=[{"robot_description": environment_description}],
        namespace=robot_name,
    )


def trajectory_goal_server(robot_name: str) -> Node:
    return Node(
        package="agimus_demos_common",
        executable="trajectory_goal_server",
        parameters=[get_use_sim_time()],
        output="screen",
        namespace=robot_name,
        remappings=remap_to_ns(robot_name,
                               "robot_description",
                               'linear_feedback_controller/get_parameters',
                               'agimus_controller_node/get_parameters',
                               ),
    )


def mpc_debugger(robot_name: str, use_mpc_debugger: str) -> Node:
    return mpc_debugger_node(
        f"{robot_name}_link_tool",
        parent_frame=f"{robot_name}_link_0",
        cost_plot=use_mpc_debugger == 'full',
        node_kwargs=dict(
            remappings=[
                ("/robot_description",
                 f"/{robot_name}/robot_description_with_collision"),
                *remap_to_ns(robot_name,
                             "environment_description",
                             'linear_feedback_controller/get_parameters',
                             'agimus_controller_node/get_parameters',
                             ),
            ],
            condition=IfCondition(
                PythonExpression(["'", use_mpc_debugger, "' != 'false'"])),
            namespace=robot_name
        ),
    )


def agimus_controller(robot_name: str, agimus_controller_yaml: Substitution,
                      extra_params) -> Node:
    return Node(
        package="agimus_controller_ros",
        executable="agimus_controller_node",
        parameters=[
            get_use_sim_time(),
            agimus_controller_yaml,
            extra_params,
        ],
        output="screen",
        remappings=remap_to_ns(robot_name,
                               "robot_description",
                               "environment_description",
                               'linear_feedback_controller/get_parameters'
                               ),
        namespace=robot_name,
    )
