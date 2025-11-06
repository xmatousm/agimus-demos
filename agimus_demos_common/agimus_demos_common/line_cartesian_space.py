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


class LineCartesianSpace(TrajectoryBase):
    """ Define the trajectory of a line defined by two end-points in cartesian space."""

    def __init__(
            self,
            x,
            transition_time,
            ee_frame_name,
            w_q,
            w_qdot,
            w_qddot,
            w_robot_effort,
            w_pose,
    ):
        """Initialize parameters needed for the line in cartesian space trajectory.

        Args:
            x (x,y,z, ...): cartesian coords of line end-points (num_pts * 3)
            transition_time (t0, t1, ...): transition times (num_pts + 1)
            init_time (float)
            period (float): period in seconds.
            ee_frame_name (_type_): Name of the end effector frame.
            w_q (_type_): weight for the robot configuration.
            w_qdot (_type_): weight for the robot velocity.
            w_qddot (_type_): weight for the robot acceleration.
            w_robot_effort (_type_): weight for the robot effort.
            w_pose (_type_): weight for the end effector pose.

        """

        super().__init__(ee_frame_name)
        self.x = np.array(x).reshape((-1, 3))
        self.n_segments = len(self.x)

        self.x = np.vstack([self.x, self.x[0]])
        self.trasition_time = transition_time
        self.w_q = w_q
        self.w_qdot = w_qdot
        self.w_qddot = w_qddot
        self.w_robot_effort = w_robot_effort
        self.w_pose = w_pose
        self.ee_init_pos = None
        self.t_from = None
        self.t_to = 0.0
        self.x_from = None
        self.x_to = None
        self.t_duration = None
        self.segment = -1
        self.weights_from: TrajectoryPointWeights = None
        self.weights_to: TrajectoryPointWeights = None

    def initialize(self, pin_model: pin.Model, q0: np.ndarray) -> None:
        """Initialize the trajectory generator."""
        super().initialize(pin_model, q0)
        self.ee_init_pos = self.get_end_effector_pose_from_q_as_se3(self.q0)
        self.t_to = 0.0
        self.t_from = 0.0
        self.segment = -1
        self.rt0 = time.time()

    def get_traj_point_at_t(self, t: np.float64) -> WeightedTrajectoryPoint:
        assert t >= self.t_from, "t not monotonous"
        rt = time.time() - self.rt0

        if t >= self.t_to:  # switch the segment
            if self.segment < 0:
                self.x_from = self.ee_init_pos.translation
                self.x_to = self.x[0]
                self.t_duration = self.trasition_time[0]

                self.segment = 0
            else:
                self.x_from = self.x[self.segment]
                self.x_to = self.x[self.segment + 1]
                self.t_duration = self.trasition_time[self.segment + 1]

                self.segment = (self.segment + 1) % self.n_segments

            self.weights_from = TrajectoryPointWeights(
                w_robot_configuration=self.w_q,
                w_robot_velocity=self.w_qdot,
                w_robot_acceleration=self.w_qddot,
                w_robot_effort=self.w_robot_effort,
                w_end_effector_poses={self.ee_frame_name: self.w_pose},
            )

            self.weights_to = TrajectoryPointWeights(
                w_robot_configuration=self.w_q,
                w_robot_velocity=self.w_qdot,
                w_robot_acceleration=self.w_qddot,
                w_robot_effort=self.w_robot_effort,
                w_end_effector_poses={self.ee_frame_name: self.w_pose},
            )

            self.t_from = t
            self.t_to = t + self.t_duration

        # interpolate cartesian line
        alpha = (t - self.t_from) / self.t_duration
        beta = 1 - alpha

        ee_des_pos = self.ee_init_pos.copy()
        ee_des_pos.translation[0] = self.x_from[0] * beta + self.x_to[0] * alpha
        ee_des_pos.translation[1] = self.x_from[1] * beta + self.x_to[1] * alpha
        ee_des_pos.translation[2] = self.x_from[2] * beta + self.x_to[2] * alpha

        q = self.q0.copy()
        dq = np.zeros(self.pin_model.nv)
        ddq = np.zeros(self.pin_model.nv)
        u = pin.rnea(self.pin_model, self.pin_data, q, dq, ddq)

        traj_point = TrajectoryPoint(
            time_ns=t,
            robot_configuration=q,
            robot_velocity=dq,
            robot_acceleration=ddq,
            robot_effort=u,
            end_effector_poses={
                self.ee_frame_name: pin.SE3ToXYZQUAT(ee_des_pos)},
        )

        #print(self.weights_from, self.weights_to, alpha)
        traj_weights = self.weights_from
        #interpolate_weights(self.weights_from, self.weights_to, alpha))

        return WeightedTrajectoryPoint(
            point=deepcopy(traj_point), weights=deepcopy(traj_weights)
        )
