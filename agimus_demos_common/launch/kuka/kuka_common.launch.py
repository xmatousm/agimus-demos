import ast
from copy import deepcopy

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node

from controller_manager.launch_utils import (
    generate_controllers_spawner_launch_description,  # noqa: I001
)

from agimus_demos_common.launch_utils_kuka import (
    generate_default_kuka_args,
    get_use_sim_time,
    parameter_value_xacro,
    path_join,
)

import launch.logging


def launch_setup(
        context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    arm_id = LaunchConfiguration("arm_id")
    robot_name = LaunchConfiguration("robot_name")
    use_gazebo = LaunchConfiguration("use_gazebo")
    external_controllers_params = LaunchConfiguration("external_controllers_params")
    external_controllers_names = LaunchConfiguration("external_controllers_names")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config_path = LaunchConfiguration("rviz_config_path")
    joint_limits_config_path = LaunchConfiguration("joint_limits_config_path")
    system_config_path = LaunchConfiguration("system_config_path")

    use_gazebo_bool = context.perform_substitution(use_gazebo).lower() == "true"
    use_rviz_bool = context.perform_substitution(use_rviz).lower() == "true"
    external_controllers_params_str = context.perform_substitution(
        external_controllers_params
    )

    external_controllers_names_list = ast.literal_eval(
        context.perform_substitution(external_controllers_names)
    )

    logger = launch.logging.get_logger(__name__)
    logger.info(f'GAZEBO: {use_gazebo_bool}')
    logger.info(f'ROBOT NAME: {context.perform_substitution(robot_name)}')

    wait_for_non_zero_joints_node = Node(
        package="agimus_demos_common",
        executable="wait_for_non_zero_joints_node",
        name="wait_for_non_zero_joints_node",
        parameters=[get_use_sim_time(), {'timeout': 10.0}],
        output="screen",
    )

    spawn_external_controllers = generate_controllers_spawner_launch_description(
        deepcopy(external_controllers_names_list),
        controller_params_files=(
            [external_controllers_params_str]
            if external_controllers_params_str != ""
            else None
        ),
        extra_spawner_args=[
            "--inactive",
            "--controller-manager-timeout",
            "10000000",
        ],
    )

    activate_external_controllers = ExecuteProcess(
        cmd=[
                "ros2",
                "control",
                "switch_controllers",
                "--activate",
            ]
            + deepcopy(external_controllers_names_list),
        output="screen",
    )

    spawn_external_controllers_on_exit_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=wait_for_non_zero_joints_node,
            on_exit=[spawn_external_controllers],
        ),
        condition=IfCondition(
            PythonExpression(
                [
                    external_controllers_names,
                    " != ['']",
                ]
            )
        ),
    )

    activate_external_controllers_on_exit_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_external_controllers.entities[2],
            on_exit=[activate_external_controllers],
        ),
        condition=IfCondition(
            PythonExpression(
                [
                    external_controllers_names,
                    " != ['']",
                ]
            )
        ),
    )

    kuka_hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [path_join("launch", "kuka", "kuka_hardware.launch.py")]),
        #condition=UnlessCondition(
        #    OrSubstitution(
        #        use_gazebo, PythonExpression(["'", aux_computer_ip, "' != ''"])
        #    )
        #),
    )
    # Auxiliary computer's docker does not have all the dependencies like kuka_description
    # It is better to return with only minimal number of code evaluated to avoid errors
    # evaluating paths to packages that do not exist in the system. From this point on
    # checking if we are running on auxiliary computer is not required.
    if False:
        return [
            kuka_hardware_launch,
            wait_for_non_zero_joints_node,
            spawn_external_controllers_on_exit_event,
            activate_external_controllers_on_exit_event,
        ]

    #kuka_remote_hardware_launch = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(
    #        [
    #            PathJoinSubstitution(
    #                [
    #                    FindPackageShare("agimus_demos_common"),
    #                    "launch",
    #                    "kuka",
    #                    "kuka_remote_hardware.launch.py",
    #                ]
    #            )
    #        ]
    #    ),
    #    launch_arguments={
    #        "robot_ip": robot_ip,
    #        "aux_computer_ip": aux_computer_ip,
    #        "aux_computer_user": aux_computer_user,
    #        "arm_id": arm_id,
    #        "kuka_controllers_params": kuka_controllers_params,
    #    }.items(),
    #    condition=UnlessCondition(
    #        OrSubstitution(
    #            use_gazebo, PythonExpression(["'", aux_computer_ip, "' == ''"])
    #        )
    #    ),
    # )

    kuka_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            path_join("launch", "kuka", "kuka_simulation.launch.py")]),
        condition=IfCondition(use_gazebo),
    )

    system_config_file = system_config_path.perform(context)
    joint_limits_file = joint_limits_config_path.perform(context)

    arm_id_str = context.perform_substitution(arm_id)
    xacro_args = {
        "robot_name": robot_name,
        "mode": "gazebo" if use_gazebo_bool else "hardware",
        "system_config_path": system_config_file,
        "joint_limits_path": joint_limits_file,
    }
    robot_description_file_substitution = path_join(
            "urdf", arm_id_str, f"{arm_id_str}.xacro", pkg="lbr_description")

    robot_description = parameter_value_xacro(
        robot_description_file_substitution, xacro_args)

    xacro_collision_args = xacro_args.copy()
    xacro_collision_args["gazebo"] = "false"
    xacro_collision_args["with_sc"] = "true"

    robot_description_with_collision = parameter_value_xacro(
        robot_description_file_substitution, xacro_collision_args)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[get_use_sim_time(), {"robot_description": robot_description}],
        output="screen",
        # namespace='lbr', # TODO
    )
    robot_collision_publisher_node = Node(
        package="agimus_demos_common",
        executable="string_publisher",
        name="robot_description_with_collision_publisher",
        output="screen",
        parameters=[
            get_use_sim_time(),
            {
                "topic_name": "robot_description_with_collision",
                "string_value": robot_description_with_collision,
            },
        ],
    )

    srdf_file_substitution = path_join(
        "config", f"{arm_id_str}.srdf", pkg=f"{arm_id_str}_moveit_config")
    srdf_file = srdf_file_substitution.perform(context)
    with open(srdf_file, "r") as f:
        robot_srdf_description = f.read()

    robot_srdf_publisher_node = Node(
        package="agimus_demos_common",
        executable="string_publisher",
        name="robot_srdf_description_publisher",
        output="screen",
        parameters=[
            {
                "topic_name": "robot_srdf_description",
                "string_value": robot_srdf_description,
            }
        ],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[
            get_use_sim_time(),
            {
                "source_list": [
                    "joint_states",
                    # f"{arm_id_str}_gripper/joint_states",
                ],
                "rate": 30,
            },
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        parameters=[get_use_sim_time()],
        arguments=["--display-config", rviz_config_path],
        condition=IfCondition(use_rviz),
    )

    return [
        kuka_hardware_launch,
        #kuka_remote_hardware_launch,
        kuka_simulation_launch,
        wait_for_non_zero_joints_node,
        spawn_external_controllers_on_exit_event,
        activate_external_controllers_on_exit_event,
        robot_state_publisher_node,
        robot_collision_publisher_node,
        robot_srdf_publisher_node,
        joint_state_publisher_node,
        rviz_node,
    ]

def generate_launch_description():
    return LaunchDescription(
        generate_default_kuka_args()
        + [OpaqueFunction(function=launch_setup)]
    )
