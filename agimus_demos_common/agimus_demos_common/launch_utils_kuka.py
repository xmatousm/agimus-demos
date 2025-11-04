from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, \
    Command, FindExecutable
from launch.substitution import Substitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.frontend import expose_action
from launch.launch_context import LaunchContext
from launch.actions import LogInfo


def generate_default_kuka_args() -> list[DeclareLaunchArgument]:
    """Generates a list of default arguments for launch files used by
     Agimus Demos for Kuka robots.

    Returns:
        list[DeclareLaunchArgument]: List of DeclareLaunchArgument objects with
            arguments expected by `kuka_common.launch.py` and `kuka_common_lfc.launch.py`
    """

    return [
        DeclareLaunchArgument(
            "arm_id",
            default_value="iiwa7",
            description="ID of the type of arm used. Supported values: iiwa7",
            choices=["iiwa7"],
        ),
        DeclareLaunchArgument(
            "robot_name",
            default_value="lbr",
            description="Name of the robot to distinguish multiple arms",
        ),
        DeclareLaunchArgument(
            "use_gazebo",
            default_value="false",
            description="Configures launch file for Gazebo simulation. ",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Visualize the robot in RViz",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "rviz_config_path",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("agimus_demos_common"),
                    "rviz",
                    "kuka",
                    "preview.rviz",
                ]
            ),
            description="Path to RViz configuration file",
        ),
        DeclareLaunchArgument(
            "joint_limits_config_path",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("agimus_demos_common"),
                    "config",
                    "kuka",
                    "joint_limits.yaml",
                ]
            ),
            description="Path to joint limits YAML file",
        ),
        DeclareLaunchArgument(
            "system_config_path",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("agimus_demos_common"),
                    "config",
                    "kuka",
                    "lbr_system_config.yaml",
                ]
            ),
            description="Path to LBR system config YAML file",
        ),
        DeclareLaunchArgument(
            "gz_verbose",
            default_value="false",
            description="Whether to set verbosity level of Gazebo to 3.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "gz_headless",
            default_value="false",
            description="Whether to launch Gazebo in headless mode "
                        + "(no GUI is launched, only physics server).",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "kuka_controllers_params",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("agimus_demos_common"),
                    "config",
                    "kuka",
                    "controllers.yaml",
                ]
            ),
            description="Path to the yaml file use to define controller parameters.",
        ),
    ]


def get_use_sim_time() -> dict[str, LaunchConfiguration]:
    """Helper function creating action setting param `use_sim_time`.

    Returns:
        dict[str, LaunchConfiguration]: Dict mapping value of
        `use_gazebo` launch argument to `use_sim_time` param.
    """
    return {"use_sim_time": LaunchConfiguration("use_gazebo")}


def parameter_value_xacro(
        xacro_file_substitution: Substitution,
        xacro_args: dict | None = None) -> ParameterValue:
    """Apply xacro command to a xacro file with arguments."""

    if xacro_args is not None:
        params = [arg
                  for key, val in xacro_args.items()
                  for arg in (f" {key}:=", val)]
    else:
        params = []

    return ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                xacro_file_substitution,
                # Convert dict to list of parameters
                *params,
            ]
        ),
        value_type=str,
    )


def path_join(*items: str, pkg: str = "agimus_demos_common") -> Substitution:
    return PathJoinSubstitution(
        [FindPackageShare(pkg), *items])


@expose_action('log_error')
class LogError(LogInfo):
    """Action that logs an error message when executed."""

    def execute(self, context: LaunchContext) -> None:
        """Execute the action."""
        self.__logger.error(
            ''.join([context.perform_substitution(sub) for sub in self.msg])
        )
        return None
