from typing import List
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future

from agimus_demos_common.trajectory_weights_parameters import (
    trajectory_weights_params,
)

from agimus_demos_msgs.action import TrajectoryGoal


class SimpleTrajectoryGoalPublisher(Node):

    def __init__(self):
        self.initialized = False
        super().__init__("simple_trajectory_goal_publisher")
        self._action_client = ActionClient(self, TrajectoryGoal,
                                           'trajectory_goal')

        self.param_listener = trajectory_weights_params.ParamListener(self)

        self.params = self.param_listener.get_params()
        self._id: int = -1
        self.croco_nq = 7
        self.point = -1

        # general parameters
        self.ee_frame_name = self.params.ee_frame_name

        self.w_q = self.get_weights(self.params.w_q, self.croco_nq)
        self.w_qdot = self.get_weights(self.params.w_qdot, self.croco_nq)
        self.w_qddot = self.get_weights(self.params.w_qddot, self.croco_nq)
        self.w_robot_effort = self.get_weights(
            self.params.w_robot_effort, self.croco_nq
        )
        self.w_pose = self.get_weights(self.params.w_pose, 6)

        self.trajectory_name = self.params.trajectory_name
        # trajectory parameters
        if self.trajectory_name == 'line_cartesian_space':
            self._init_line_params()
        elif self.trajectory_name == 'saw_line_cartesian_space':
            self._init_line_params()
            tooth_length = self.params.saw.tooth_length
            tooth_tip = self.params.saw.tooth_tip

            assert len(tooth_tip) == 3, "tooth tip length must be 3"
            assert tooth_length > 0.0, "tooth length must be positive"

            self.tooth_length = tooth_length
            self.tooth_tip = tooth_tip

        else:
            raise ValueError(
                f"Trajectory {self.params.trajectory_name} not supported")

    def _init_line_params(self):
        x = self.params.line_endpoints.x
        time = self.params.line_endpoints.time
        w_mul = self.params.line_endpoints.w_mul
        rpy = self.params.line_endpoints.rotation
        tol = self.params.line_endpoints.goal_tolerance

        assert len(rpy) == 3, "rotation length must be 3"

        assert len(x) > 0 and len(x) % 3 == 0, "x length must be multiple of 3"
        self.npts = len(x) // 3
        assert len(
            time) == self.npts + 1, "time length must be number of points + 1"

        if len(w_mul) <= 1:
            w_mul = [1.0] * self.npts
        else:
            assert len(
                w_mul) == self.npts, "w_mul length must be number of points"

        if len(tol) <= 1:
            tol = None
        else:
            assert len(
                tol) == self.npts, "goal_tolerance length must be number of points"

        self.x = x
        self.transition_time = time
        self.w_pose_mul = w_mul
        self.rotation = rpy
        self.tol = tol
        self.tol_boost = self.params.line_endpoints.goal_tolerance_boost
        self.w_boost = self.params.line_endpoints.goal_weight_boost

    def get_weights(
            self, weights: List[float], size: int
    ) -> List[float]:
        """
        Return weights with right size if user sent only one value, otherwise
        directly returns weights.
        """
        if len(weights) == 1:
            return weights * size
        else:
            return weights

    def send_goal(self):
        self._id += 1
        goal = TrajectoryGoal.Goal()
        g = goal.goal
        g.id = self._id
        g.frame_name = self.ee_frame_name
        g.trajectory_type = self.trajectory_name
        if self.trajectory_name == 'saw_line_cartesian_space':
            g.s1 = self.tooth_length
            g.v1 = self.tooth_tip

        g.w_q = self.w_q
        g.w_qdot = self.w_qdot
        g.w_qddot = self.w_qddot
        g.w_robot_effort = self.w_robot_effort
        g.id = self._id
        g.speed = -1.0
        g.goal_tolerance_boost = self.tol_boost
        g.goal_weight_boost = self.w_boost
        if self.point < 0:
            g.duration = self.transition_time[0]
            g.pose = self.x[0:3]
            g.rot_rpy = self.rotation
            goal.goal.w_pose = [w * self.w_pose_mul[0] for w in self.w_pose]
            self.point = 0
            g.min_distance = 0.0

        else:
            point_from = self.point
            self.point = (self.point + 1) % self.npts
            g.rot_rpy = self.rotation
            g.duration = self.transition_time[point_from + 1]
            g.pose = self.x[self.point * 3:self.point * 3 + 3]
            goal.goal.w_pose = [w * self.w_pose_mul[self.point]
                                for w in self.w_pose]
            g.min_distance = self.tol[self.point] if self.tol else 0.0

        self.get_logger().info("Wait for server")

        self._action_client.wait_for_server()

        self.get_logger().info(f"Sending goal {self._id}, point: {self.point}")

        # send goal
        goal_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future)
        goal_handle = goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        # wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        self.get_logger().info(f'Result: {result}')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleTrajectoryGoalPublisher()

    try:
        while True:
            node.send_goal()

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
