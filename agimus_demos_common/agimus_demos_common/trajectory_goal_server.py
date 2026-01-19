import numpy as np
import rclpy
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile, ReliabilityPolicy
import pinocchio as pin

from agimus_demos_msgs.action import TrajectoryGoal
from agimus_msgs.msg import MpcDebug

import time


from agimus_controller_ros.simple_trajectory_publisher import (
    TrajectoryPublisherBase,
)

from rclpy.task import Future

from agimus_controller_ros.ros_utils import (
    weighted_traj_point_to_mpc_msg,
    get_param_from_node,
)

from agimus_demos_common.trajectories.line_segment_cartesian_space import \
    LineSegmentCartesianSpace

from agimus_demos_common.trajectories.saw_line_segment_cartesian_space import \
    SawLineSegmentCartesianSpace

from agimus_controller.trajectory import TrajectoryPointWeights

class TrajectoryGoalServer(TrajectoryPublisherBase):

    def __init__(self):
        self.future_base_init_done = Future()
        super().__init__('trajectory_goal_server')

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

        self.last_mpc_point_id = None

        self._mpc_debug_sub = self.create_subscription(
            MpcDebug,
            "mpc_debug",
            self.mpc_debug_callback,
            qos_profile=QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            ),
        )

        self.get_logger().info(f'Waiting for base init')

        rclpy.spin_until_future_complete(self, self.future_base_init_done)
        self.get_logger().info(f'Starting action server')

        self.goal_done = Future()
        self.last_w_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.last_x_from = None
        self.last_r_from = None
        self.tr0 = self.get_clock().now().nanoseconds / 1e9
        self.tt0 = time.time_ns() / 1e9
        self.t: np.float64 = 0.0
        self.trajectory = None
        self.w_traj_point = None
        self._point_id = 0
        self.timer = self.create_timer(self.dt, self.publish_mpc_input)
        self.current_pose = None

        self._action_server = ActionServer(
            self,
            TrajectoryGoal,
            'trajectory_goal',
            self.execute_callback)

    def mpc_debug_callback(self, msg: MpcDebug):
        self.last_mpc_point_id = msg.trajectory_point_id

    def ready_callback(self):
        self.get_logger().error(f'ready')
        self.future_base_init_done.set_result(True)

    def publish_mpc_input(self):
        if self.trajectory is None and self.w_traj_point is None:
            return

        delay = None
        if self.last_mpc_point_id is not None:
            delay = self._point_id - self.last_mpc_point_id
            if delay > self.point_delta:
                self.get_logger().error(
                    f"{self._point_id}: Input to MPC delay: {delay}; skipping one cycle.")
                return

        if self.trajectory is not None:
            self.w_traj_point = self.trajectory.get_traj_point_at_tq(
                self.t, self.current_q)
            self.t += self.dt
            if not self.trajectory.running:
                self.goal_done.set_result(True)
                self.current_pose = self.trajectory.get_end_effector_pose_from_q_as_se3(
                    self.current_q)
                self.trajectory = None

        self.w_traj_point.point.id = self._point_id
        self._point_id += 1
        msg = weighted_traj_point_to_mpc_msg(self.w_traj_point)
        self.publisher_.publish(msg)

    def execute_callback(self, goal_handle):
        goal: TrajectoryGoal.Goal = goal_handle.request

        g = goal.goal

        self.get_logger().info(f'Executing goal {g.trajectory_type} {g.id} ({g.duration}s)')

        weights = TrajectoryPointWeights(
            w_robot_configuration=np.array(g.w_q),
            w_robot_velocity=np.array(g.w_qdot),
            w_robot_acceleration=np.array(g.w_qddot),
            w_robot_effort=np.array(g.w_robot_effort),
            w_end_effector_poses={
                g.frame_name: np.array(g.w_pose),
            })

        if g.trajectory_type == 'saw_line_cartesian_space':
            trajectory = SawLineSegmentCartesianSpace(g.frame_name)
            assert g.s1 > 0.0
            assert len(g.v1) == 3
            trajectory.tooth_length = g.s1
            trajectory.tooth_tip = np.array(g.v1)

        else:
            trajectory = LineSegmentCartesianSpace(g.frame_name)
            trajectory.goal_weight_boost = g.goal_weight_boost
            trajectory.goal_tolerance_boost = g.goal_tolerance_boost


        trajectory.initialize(self.robot_models.robot_model, self.q0)
        trajectory.initialize_w(weights)
        rotation = pin.rpy.rpyToMatrix(
                g.rot_rpy[0], g.rot_rpy[1], g.rot_rpy[2])


        if self.last_x_from is None:
            self.last_x_from = trajectory.ee_init_pos.translation
            self.last_r_from = trajectory.ee_init_pos.rotation

        trajectory.set_segment(
            t=self.t,
            x_from=self.last_x_from,
            x_to=np.array(g.pose),
            r_from=self.last_r_from,
            r_to=rotation,
            duration=g.duration if g.duration > 0.0 else None,
            velocity=g.speed if g.speed > 0.0 else None,
            w_pose_from=self.last_w_pose, w_pose_to=np.array(g.w_pose),
        )

        trajectory.goal_tolerance = g.min_distance if g.min_distance > 0.0 else None
        trajectory.logger = self.get_logger()

        self.last_w_pose = np.array(g.w_pose)
        self.last_x_from = np.array(g.pose)
        self.last_r_from = np.array(rotation)

        self.goal_done = Future()
        self.trajectory = trajectory
        rclpy.spin_until_future_complete(self, self.goal_done)
        err = np.sqrt(np.sum((self.current_pose.translation - g.pose)**2))

        self.get_logger().info(f'Goal finished {g.id} ({err})')

        result = TrajectoryGoal.Result()

        goal_handle.succeed()

        result.distance = err
        result.id = g.id
        return result


def main(args=None):
    rclpy.init(args=args)

    action_server = TrajectoryGoalServer()

    rclpy.spin(action_server)


if __name__ == '__main__':
    main()
