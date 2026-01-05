from abc import ABC, abstractmethod
import numpy as np
import pinocchio as pin

from agimus_controller.trajectories.trajectory_base import TrajectoryBase
from agimus_controller.trajectory import WeightedTrajectoryPoint


class TrajectoryBaseMod(TrajectoryBase, ABC):
    """Base class for a Trajectory generator."""

    def __init__(self, ee_frame_name) -> None:
        super().__init__(ee_frame_name)

    @abstractmethod
    def get_traj_point_at_tq(
            self, t: np.float64, q: np.ndarray) -> WeightedTrajectoryPoint:
        """Weighted trajectory point at time t and current configuration q."""

    def get_traj_point_at_t(self, t: np.float64) -> WeightedTrajectoryPoint:
        """Return Weighted Trajectory point of the trajectory at time t."""
        raise NotImplementedError()

    def inverse_kinematics(
            self,
            ee_des_pos: pin.SE3,
            ee_des_vel: np.ndarray,
            ref_q: np.ndarray,
            precision=1e-5,
            it_max=10000,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Inverse kinematics to reach the desired end effector pose."""
        i = 0
        success = False
        ik_q = ref_q.copy()
        while True:
            ik_ee_pose = self.get_end_effector_pose_from_q_as_se3(ik_q)
            dMi = ee_des_pos.actInv(ik_ee_pose)
            error = pin.log(dMi).vector[:3]
            if np.linalg.norm(error) < precision:
                success = True
                break
            if i > it_max:
                break

            pin.computeJointJacobians(self.pin_model, self.pin_data, ik_q)
            jaco_ee = pin.getFrameJacobian(
                self.pin_model,
                self.pin_data,
                self.ee_frame_id,
                pin.ReferenceFrame.LOCAL,
            )[:3, :]
            dq = -jaco_ee.T @ np.linalg.solve(jaco_ee @ jaco_ee.T, error)
            ik_q[:] = pin.integrate(self.pin_model, ik_q, dq)
            i += 1

        if not success:
            error_msgs = (
                f"Inverse kinematics 6D failed to converge with error: "
                f"{error}. Number of iteration: {i}"
            )
            raise RuntimeError(error_msgs)

        pin.forwardKinematics(self.pin_model, self.pin_data, ik_q)
        pin.updateFramePlacement(self.pin_model, self.pin_data,
                                 self.ee_frame_id)
        pin.computeJointJacobians(self.pin_model, self.pin_data, ik_q)
        jaco_ee = pin.getFrameJacobian(
            self.pin_model,
            self.pin_data,
            self.ee_frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )[:3, :]
        dq = jaco_ee.T @ np.linalg.solve(jaco_ee @ jaco_ee.T, ee_des_vel[:3])

        return ik_q, dq.copy()
