from copy import deepcopy
import numpy as np
import pinocchio as pin
from scipy.linalg import expm
from typing import Optional
from agimus_controller.trajectory import (
    TrajectoryPoint,
    WeightedTrajectoryPoint,
)

from agimus_demos_common.trajectories.trajectory_segment import \
    TrajectorySegment


class SawLineSegmentCartesianSpace(TrajectorySegment):
    """Saw-line segment defined by end-points in the cartesian space."""

    def __init__(self, ee_frame_name: str):
        super().__init__(ee_frame_name)
        self.ik_q = None
        self.logger = None
        self.tooth_length = None
        self.tooth_length_rel = None
        self.tooth_tip = None

    def initialize(self, pin_model: pin.Model, q0: np.ndarray) -> None:
        """Initialize the trajectory generator."""
        super().initialize(pin_model, q0)
        self.ik_q = q0.copy()

    def init_segment(self) -> None:
        d = np.linalg.norm(self.x_to - self.x_from)
        self.tooth_length_rel = self.tooth_length / d

    def interpolate_weighted_point(self, alpha, alpha_w
                                   ) -> WeightedTrajectoryPoint:
        """Interpolate cartesian line segment."""

        translation_line = self.x_from + alpha * self.x_delta
        rotation = expm(self.r_delta_log * alpha) @ self.r_from

        # distance along
        saw_n = alpha // self.tooth_length_rel
        saw_t = (alpha % self.tooth_length_rel) / self.tooth_length_rel

        alpha_tooth = saw_n * self.tooth_length_rel
        t0 = self.x_from + alpha_tooth * self.x_delta
        t1 = t0 + self.tooth_tip
        t2 = self.x_from + (alpha_tooth + self.tooth_length_rel) * self.x_delta

        if saw_t < 0.5:
            translation = t0 + (t1 - t0) * saw_t * 2
        else:
            translation = t1 + (t2 - t1) * (saw_t - 0.5) * 2

        ee_des_pos = pin.SE3(rotation, translation)

        # required velocity computed from the current, and the last point
        dt = self.current_t - self.last_t

        if dt == 0.0:
            q = self.q0
            dq = np.zeros(self.pin_model.nv)
        else:
            ee_des_vel = (translation - self.last_x) / dt
            q, dq = self.inverse_kinematics(ee_des_pos, ee_des_vel, self.ik_q)

        self.ik_q = q
        self.last_x = translation

        ddq = np.zeros(self.pin_model.nv)
        u = pin.rnea(self.pin_model, self.pin_data, q, dq, ddq)

        traj_point = TrajectoryPoint(
            robot_configuration=q,
            robot_velocity=dq,
            robot_acceleration=ddq,
            robot_effort=u,
            end_effector_poses={
                self.ee_frame_name: pin.SE3ToXYZQUAT(ee_des_pos)},
        )

        # optionally interpolate pose weights
        traj_weights = deepcopy(self.weights)
        if self.w_pose_from is not None:
            w_pose = self.w_pose_from * (1 - alpha_w) + self.w_pose_to * alpha_w
            traj_weights.w_end_effector_poses[self.ee_frame_name] = w_pose

        return WeightedTrajectoryPoint(
            point=deepcopy(traj_point), weights=traj_weights)

    def get_traj_point_at_tq(self, t: np.float64, q: np.ndarray
                             ) -> WeightedTrajectoryPoint:
        assert t >= self.t_from
        self.current_t = t

        # finishing criterion - time
        if t >= self.t_to:
            self.running = False

        alpha = min((t - self.t_from) / self.duration, 1.0)

        point = self.interpolate_weighted_point(alpha, alpha)
        self.last_t = t
        return point
