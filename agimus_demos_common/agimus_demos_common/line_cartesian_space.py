import numpy as np
import pinocchio as pin

from agimus_controller.trajectories.trajectory_base import TrajectoryBase
from agimus_controller.trajectory import WeightedTrajectoryPoint
from agimus_demos_common.line_segment_cartesian_space import \
    LineSegmentCartesianSpace


class LineCartesianSpace(TrajectoryBase):
    """ Define the trajectory of a poly-line defined by end-points in cartesian space."""

    def __init__(
            self,
            x,
            transition_time,
            w_pose_mul,
            ee_frame_name,
            w_q,
            w_qdot,
            w_qddot,
            w_robot_effort,
            w_pose,

    ):
        """Initialize the poly-line trajectory in cartesian space trajectory.

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
        self.n_points = len(self.x)
        self.trasition_time = transition_time
        self.w_pose_mul = w_pose_mul if w_pose_mul else [1.0] * self.n_points
        self.w_q = np.array(w_q)
        self.w_qdot = np.array(w_qdot)
        self.w_qddot = np.array(w_qddot)
        self.w_robot_effort = np.array(w_robot_effort)
        self.w_pose = np.array(w_pose)
        self.ee_init_pos = None
        self.point = -1  # the current point we are moving to

        self.segment = LineSegmentCartesianSpace(ee_frame_name)

    def initialize(self, pin_model: pin.Model, q0: np.ndarray) -> None:
        """Initialize the trajectory generator."""
        super().initialize(pin_model, q0)
        self.ee_init_pos = self.get_end_effector_pose_from_q_as_se3(self.q0)

        self.segment.initialize(pin_model, q0)
        self.segment.initialize_w(self.w_pose, self.w_pose,
                                  self.w_q, self.w_qdot, self.w_qddot,
                                  self.w_robot_effort)
        self.point = -1

    def get_traj_point_at_t(self, t: np.float64) -> WeightedTrajectoryPoint:
        if not self.segment.running:  # switch the segment
            if self.point < 0:
                self.segment.set_segment(
                    t=t,
                    x_from=self.ee_init_pos.translation,
                    x_to=self.x[0],
                    t_duration=self.trasition_time[0],
                    w_pose_from=self.w_pose * self.w_pose_mul[0],
                    w_pose_to=self.w_pose * self.w_pose_mul[0])
                self.point = 0
            else:
                point_from = self.point
                self.point = (self.point + 1) % self.n_points

                self.segment.set_segment(
                    t=t,
                    x_from=self.x[point_from],
                    x_to=self.x[self.point],
                    t_duration=self.trasition_time[point_from + 1],
                    w_pose_from=self.w_pose * self.w_pose_mul[point_from],
                    w_pose_to=self.w_pose * self.w_pose_mul[self.point])

        # interpolate cartesian line
        return self.segment.get_traj_point_at_t(t)
