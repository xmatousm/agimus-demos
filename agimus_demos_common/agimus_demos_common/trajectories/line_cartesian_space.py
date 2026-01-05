import numpy as np
import pinocchio as pin
from agimus_controller.trajectory import WeightedTrajectoryPoint

from agimus_demos_common.trajectories.line_segment_cartesian_space import \
    LineSegmentCartesianSpace

from agimus_demos_common.trajectories.trajectory_base_mod import \
    TrajectoryBaseMod
from agimus_controller.trajectory import TrajectoryPointWeights


class LineCartesianSpace(TrajectoryBaseMod):
    """Trajectory of poly-line defined by end-points in cartesian space."""

    def __init__(
            self,
            x,
            transition_time,
            w_mul,
            ee_frame_name: str,
            rotation_rpy,
            weights: TrajectoryPointWeights,
            goal_tolerance=None,
            goal_tolerance_boost=1.0,
            goal_weight_boost=1.0,
            logger=None,
    ):

        super().__init__(ee_frame_name)
        self.x = np.array(x).reshape((-1, 3))
        self.n_points = len(self.x)
        self.transition_time = transition_time
        self.w_mul = w_mul if w_mul else [1.0] * self.n_points
        self.weights = weights
        self.w_pose = weights.w_end_effector_poses[ee_frame_name]
        self.ee_init_pos = None
        self.point = -1  # the current point we are moving to

        self.segment = LineSegmentCartesianSpace(ee_frame_name)
        self.segment.logger = logger
        self.rotation = pin.rpy.rpyToMatrix(
            rotation_rpy[0], rotation_rpy[1], rotation_rpy[2])
        self.goal_tolerance = goal_tolerance
        if self.goal_tolerance is None:
            self.goal_tolerance = [None] * self.n_points
        self.goal_tolerance_boost = goal_tolerance_boost
        self.goal_weight_boost = goal_weight_boost
        self.logger = logger

    def initialize(self, pin_model: pin.Model, q0: np.ndarray) -> None:
        """Initialize the trajectory generator."""
        super().initialize(pin_model, q0)
        self.ee_init_pos = self.get_end_effector_pose_from_q_as_se3(self.q0)

        self.segment.initialize(pin_model, q0)
        self.segment.initialize_w(self.weights)
        self.segment.goal_weight_boost = self.goal_weight_boost
        self.segment.goal_tolerance_boost = self.goal_tolerance_boost
        self.point = -1

    def get_traj_point_at_tq(self, t: np.float64, q: np.ndarray
                             ) -> WeightedTrajectoryPoint:
        if not self.segment.running:  # switch the segment
            if self.point < 0:
                self.segment.set_segment(
                    t=t,
                    x_from=self.ee_init_pos.translation,
                    x_to=self.x[0],
                    r_from=self.ee_init_pos.rotation,
                    r_to=self.rotation,
                    duration=self.transition_time[0],
                    w_pose_from=self.w_pose * self.w_mul[0],
                    w_pose_to=self.w_pose * self.w_mul[0])
                self.segment.goal_tolerance = self.goal_tolerance[0]
                self.point = 0
            else:
                point_from = self.point
                self.point = (self.point + 1) % self.n_points

                self.segment.set_segment(
                    t=t,
                    x_from=self.x[point_from],
                    x_to=self.x[self.point],
                    r_from=self.rotation,
                    r_to=self.rotation,
                    duration=self.transition_time[point_from + 1],
                    w_pose_from=self.w_pose * self.w_mul[point_from],
                    w_pose_to=self.w_pose * self.w_mul[self.point])
                self.segment.goal_tolerance = self.goal_tolerance[self.point]

            if self.logger is not None:
                self.logger.info(f"Point set: {self.point}, " +
                                 f"t={self.segment.duration}, " +
                                 f"x_to={self.segment.x_to}")

        # interpolate cartesian line
        return self.segment.get_traj_point_at_tq(t, q)
