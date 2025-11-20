from copy import deepcopy
import numpy as np
import pinocchio as pin
import time

from agimus_controller.trajectories.trajectory_base import TrajectoryBase
from agimus_controller.trajectory import (
    TrajectoryPoint,
    TrajectoryPointWeights,
    WeightedTrajectoryPoint,
    interpolate_weights
)


class LineSegmentCartesianSpace(TrajectoryBase):
    """ Define the trajectory of a line segment defined by end-points in cartesian space."""

    def __init__(
            self,
            ee_frame_name,
    ):

        super().__init__(ee_frame_name)
        self.w_q = None
        self.w_qdot = None
        self.w_qddot = None
        self.w_robot_effort = None
        self.w_pose = None
        self.ee_init_pos = None
        self.t_from = None
        self.t_to = None
        self.x_from = None
        self.x_to = None
        self.w_pose_from = None
        self.w_pose_to = None
        self.t_duration = None
        self.ee_init_pos = None
        self.running = False

    def initialize(self, pin_model: pin.Model, q0: np.ndarray) -> None:
        """Initialize the trajectory generator."""
        super().initialize(pin_model, q0)
        self.ee_init_pos = self.get_end_effector_pose_from_q_as_se3(self.q0)

    def initialize_w(self, w_pose_from, w_pose_to,
                    w_q, w_qdot, w_qddot, w_robot_effort):

        self.w_pose_from = w_pose_from,
        self.w_pose_to = w_pose_to,
        self.w_q = np.array(w_q)
        self.w_qdot = np.array(w_qdot)
        self.w_qddot = np.array(w_qddot)
        self.w_robot_effort = np.array(w_robot_effort)

    def set_segment(self, t, x_from, x_to, t_duration,
                    w_pose_from=None, w_pose_to=None,
                    w_q=None, w_qdot=None, w_qddot=None, w_robot_effort=None):
        self.running = True
        self.x_from = x_from
        self.x_to = x_to
        self.t_duration = t_duration

        self.t_from = t
        self.t_to = t + t_duration

        if w_pose_from is not None:
            self.w_pose_from = w_pose_from
            self.w_pose_to = w_pose_to

        if w_q is not None:
            self.w_q = np.array(w_q)

        if w_qdot is not None:
            self.w_qdot = np.array(w_qdot)

        if w_qddot is not None:
            self.w_qddot = np.array(w_qddot)

        if w_robot_effort is not None:
            self.w_robot_effort = np.array(w_robot_effort)

        assert self.w_pose_from is not None
        assert self.w_pose_to is not None

        assert self.w_q is not None
        assert self.w_qdot is not None
        assert self.w_qddot is not None
        assert self.w_robot_effort is not None

    def interpolate_weighted_point(self, alpha, alpha_w
                                   ) -> WeightedTrajectoryPoint:
        """Interpolate cartesian line."""

        beta = 1 - alpha

        ee_des_pos = self.ee_init_pos.copy()
        ee_des_pos.translation[0] = self.x_from[0] * beta + self.x_to[0] * alpha
        ee_des_pos.translation[1] = self.x_from[1] * beta + self.x_to[1] * alpha
        ee_des_pos.translation[2] = self.x_from[2] * beta + self.x_to[2] * alpha

        q = self.q0.copy()
        dq = np.zeros(self.pin_model.nv)
        ddq = np.zeros(self.pin_model.nv)
        u = pin.rnea(self.pin_model, self.pin_data, q, dq, ddq)

        beta_w = 1 - alpha_w

        w_pose = self.w_pose_from * beta_w + self.w_pose_to * alpha_w

        traj_point = TrajectoryPoint(
            robot_configuration=q,
            robot_velocity=dq,
            robot_acceleration=ddq,
            robot_effort=u,
            end_effector_poses={
                self.ee_frame_name: pin.SE3ToXYZQUAT(ee_des_pos)},
        )

        traj_weights = TrajectoryPointWeights(
            w_robot_configuration=self.w_q,
            w_robot_velocity=self.w_qdot,
            w_robot_acceleration=self.w_qddot,
            w_robot_effort=self.w_robot_effort,
            w_end_effector_poses={
                self.ee_frame_name: w_pose,
            }
        )

        return WeightedTrajectoryPoint(
            point=deepcopy(traj_point), weights=deepcopy(traj_weights)
        )

    def get_traj_point_at_t(self, t: np.float64) -> WeightedTrajectoryPoint:
        assert t >= self.t_from, "t not monotonous"

        if t >= self.t_to:
            self.running = False

        alpha = min((t - self.t_from) / self.t_duration, 1.0)

        return self.interpolate_weighted_point(alpha, alpha)
