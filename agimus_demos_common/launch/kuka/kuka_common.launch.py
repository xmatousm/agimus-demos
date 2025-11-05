import ast
from copy import deepcopy

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
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
    external_controllers_params = LaunchConfiguration(
        "external_controllers_params")
    external_controllers_names = LaunchConfiguration(
        "external_controllers_names")
    rviz_config_path = LaunchConfiguration("rviz_config_path")
    joint_limits_config_path = LaunchConfiguration("joint_limits_config_path")
    system_config_path = LaunchConfiguration("system_config_path")

    use_rviz_bool = context.perform_substitution(
        LaunchConfiguration("use_rviz")).lower() == "true"
    use_gazebo_bool = context.perform_substitution(
        LaunchConfiguration("use_gazebo")).lower() == "true"
    use_aux_bool = context.perform_substitution(
        LaunchConfiguration("use_aux")).lower() == "true"
    on_aux_bool = context.perform_substitution(
        LaunchConfiguration("on_aux")).lower() == "true"

    external_controllers_params_str = context.perform_substitution(
        external_controllers_params
    )

    if on_aux_bool and use_aux_bool:
        raise RuntimeError(
            "Cannot use both use_aux and on_aux at the same time.")

    external_controllers_names_str = context.perform_substitution(
        external_controllers_names)
    external_controllers_names_list = ast.literal_eval(
        context.perform_substitution(external_controllers_names)
    )

    logger = launch.logging.get_logger(__name__)

    if on_aux_bool:
        logger.info(f'LAUNCH: AUX')
    elif use_aux_bool:
        logger.info(f'LAUNCH: USE AUX')
    else:
        logger.info(f'LAUNCH: FULL')

    logger.info(f'GAZEBO: {use_gazebo_bool}')
    logger.info(f'ROBOT NAME: {context.perform_substitution(robot_name)}')
    logger.info(f'EXTERNAL CONTROLLERS: {external_controllers_names_list}')

    wait_for_non_zero_joints_node = Node(
        package="agimus_demos_common",
        executable="wait_for_non_zero_joints_node",
        name="wait_for_non_zero_joints_node",
        parameters=[get_use_sim_time(), {'timeout': 10.0}],
        output="screen",
    )

    launch_config: list[LaunchDescriptionEntity] = [
        wait_for_non_zero_joints_node]

    if not use_aux_bool:  # full launch or aux launch
        # gazebo or hardware launch
        if use_gazebo_bool:
            launch_config.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [path_join("launch", "kuka",
                                   "kuka_simulation.launch.py")])))

        else:
            launch_config.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [path_join("launch", "kuka",
                                   "kuka_hardware.launch.py")])))

        # switch to external controllers if any
        if external_controllers_names_str != "":
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
                )
            )

            activate_external_controllers_on_exit_event = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_external_controllers.entities[2],
                    on_exit=[activate_external_controllers],
                )
            )

            launch_config.append(spawn_external_controllers_on_exit_event)
            launch_config.append(activate_external_controllers_on_exit_event)

    if not on_aux_bool:  # full launch or using aux launch
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
            "urdf", f"{arm_id_str}.xacro", pkg="agimus_description")

        robot_description = parameter_value_xacro(
            robot_description_file_substitution, xacro_args)

        xacro_collision_args = xacro_args.copy()
        xacro_collision_args["gazebo"] = "false"
        xacro_collision_args["with_sc"] = "true"

        robot_description_with_collision = parameter_value_xacro(
            robot_description_file_substitution, xacro_collision_args)

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

        robot_state_publisher_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[get_use_sim_time(),
                        {"robot_description": robot_description}],
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

        launch_config += [
            joint_state_publisher_node,
            robot_state_publisher_node,
            robot_collision_publisher_node,
            robot_srdf_publisher_node,
        ]

        if use_rviz_bool and not on_aux_bool:
            # aux launch does not run rviz
            launch_config.append(
                Node(
                    package="rviz2",
                    executable="rviz2",
                    parameters=[get_use_sim_time()],
                    arguments=["--display-config", rviz_config_path]))

    return launch_config


def generate_launch_description():
    return LaunchDescription(
        generate_default_kuka_args()
        + [OpaqueFunction(function=launch_setup)]
    )
