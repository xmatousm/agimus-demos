from typing import List
import numpy as np
import rclpy
from rclpy.task import Future
from agimus_controller_ros.ros_utils import (
    weighted_traj_point_to_mpc_msg,
    get_param_from_node,
)

from rclpy.qos import QoSProfile, ReliabilityPolicy

from agimus_msgs.msg import MpcDebug

from agimus_controller.trajectory import TrajectoryPointWeights

from agimus_demos_common.trajectory_weights_parameters import (
    trajectory_weights_params,
)

from agimus_demos_common.trajectories.line_cartesian_space import \
    LineCartesianSpace

from agimus_controller_ros.simple_trajectory_publisher import (
    TrajectoryPublisherBase,
)

from agimus_demos_common.trajectories.trajectory_base_mod import \
    TrajectoryBaseMod


class SimpleTrajectoryPublisherMod(TrajectoryPublisherBase):
    """This is a modified simple trajectory publisher."""

    def __init__(self):
        self.initialized = False
        super().__init__("simple_trajectory_publisher")

        self.param_listener = trajectory_weights_params.ParamListener(self)

        self.params = self.param_listener.get_params()
        self.ee_frame_name = self.params.ee_frame_name
        self._id: int = 0
        self.t: np.float64 = np.float64(0.0)
        self.dt = get_param_from_node(
            self, "agimus_controller_node", "ocp.dt"
        ).double_value

        # compute horizon size

        n_steps = get_param_from_node(
            self, "agimus_controller_node", "ocp.dt_factor_n_seq.n_steps"
        ).integer_array_value

        factors = get_param_from_node(
            self, "agimus_controller_node", "ocp.dt_factor_n_seq.factors"
        ).integer_array_value

        horizon_steps = 0
        for factor, n_step in zip(factors, n_steps):
            horizon_steps += factor * n_step

        self.get_logger().info(f"Detected horizon: {horizon_steps}")
        self.point_delta = int(horizon_steps * 1.3)  # TODO
        self.get_logger().info(f"Used point delta: {self.point_delta}")
        self.delay = self.point_delta * self.dt

        self.last_mpc_point_id = None

        self.croco_nq = 7
        self.future_init_done = Future()
        self.future_trajectory_done = Future()
        self.use_q = False  # send current q to trajectory

        self.trajectory = self.get_trajectory(self.params.trajectory_name)

        self._mpc_debug_sub = self.create_subscription(
            MpcDebug,
            "mpc_debug",
            self.mpc_debug_callback,
            qos_profile=QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            ),
        )

        self.first_run = True
        self.initialized = True
        self.get_logger().info("Initialized.")

    def ready_callback(self):
        if not self.initialized:
            # Base can run this via the timer before our init finishes
            self.get_logger().warn("Not ready.")
            self.destroy_timer(self.timer)
            self.timer = self.create_timer(0.1, self.ready_callback)
            return

        self.destroy_timer(self.timer)

        self.get_logger().info("Ready.")
        self.timer = self.create_timer(self.dt, self.publish_mpc_input)

    def mpc_debug_callback(self, msg: MpcDebug):
        self.last_mpc_point_id = msg.trajectory_point_id

    def get_trajectory(self, trajectory_name: str) -> TrajectoryBaseMod:
        """Build the chosen trajectory."""
        if trajectory_name in ("line_cartesian_space",
                               ):
            x = self.params.line_endpoints.x
            time = self.params.line_endpoints.time
            w_mul = self.params.line_endpoints.w_mul
            rpy = self.params.line_endpoints.rotation
            tol = self.params.line_endpoints.goal_tolerance
            tol_boost = self.params.line_endpoints.goal_tolerance_boost
            w_boost = self.params.line_endpoints.goal_weight_boost

            assert len(rpy) == 3, "rotation length must be 3"

            assert len(x) > 0 and len(
                x) % 3 == 0, "x length must be multiple of 3"
            npts = len(x) // 3
            assert len(
                time) == npts + 1, "time length must be number of points + 1"

            if len(w_mul) <= 1:
                w_mul = None
            else:
                assert len(
                    w_mul) == npts, "w_mul length must be number of points"

            if len(tol) <= 1:
                tol = None
            else:
                assert len(
                    tol) == npts, "goal_tolerance length must be number of points"

            weights = TrajectoryPointWeights(
                w_robot_configuration=self.get_weights(
                    self.params.w_q, self.croco_nq),
                w_robot_velocity=self.get_weights(
                    self.params.w_qdot, self.croco_nq),
                w_robot_acceleration=self.get_weights(
                    self.params.w_qddot, self.croco_nq),
                w_robot_effort=self.get_weights(
                    self.params.w_robot_effort, self.croco_nq
                ),
                w_end_effector_poses={
                    self.ee_frame_name: self.get_weights(self.params.w_pose, 6)
                }

            )

            if trajectory_name == "line_cartesian_space":
                return LineCartesianSpace(
                    x=x, transition_time=time, w_mul=w_mul,
                    ee_frame_name=self.ee_frame_name,
                    rotation_rpy=rpy,
                    weights=weights,
                    goal_tolerance=tol,
                    goal_tolerance_boost=tol_boost,
                    goal_weight_boost=w_boost,
                    logger=self.get_logger(),
                )

        else:
            raise ValueError("Unknown Trajectory " + trajectory_name)

    def get_weights(self, weights: List[np.float64], size: int) -> np.ndarray:
        """
        Return weights with right size if user sent only one value, otherwise
        directly returns weights.
        """
        if len(weights) == 1:
            return np.array(weights * size)
        else:
            return np.array(weights)

    def publish_mpc_input(self):
        if self.first_run:
            self.get_logger().info("Running.")
            self.first_run = False
            self.trajectory.initialize(self.robot_models.robot_model, self.q0)
            self.future_init_done.set_result(True)

        delay = None
        if self.last_mpc_point_id is not None:
            delay = self._id - self.last_mpc_point_id
            if delay > self.point_delta:
                self.get_logger().error(
                    f"{self._id}: Input to MPC delay: {delay}; skipping one cycle.")
                return

        w_traj_point = self.trajectory.get_traj_point_at_tq(self.t,
                                                            self.current_q)
        w_traj_point.point.id = self._id
        msg = weighted_traj_point_to_mpc_msg(w_traj_point)
        self._id += 1

        self.publisher_.publish(msg)
        if self.trajectory.trajectory_is_done:
            self.future_trajectory_done.set_result(True)
        self.t += self.dt


def main(args=None):
    rclpy.init(args=args)
    node = SimpleTrajectoryPublisherMod()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
