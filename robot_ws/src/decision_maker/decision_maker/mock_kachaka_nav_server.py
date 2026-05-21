import math
import time

import rclpy
from kachaka_interfaces.action import Navigate
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class MockKachakaNavServer(Node):
    """Mock server for kachaka_interfaces/Navigate on /Navigate_to_pose."""

    def __init__(self):
        super().__init__("mock_kachaka_nav_server")
        self.declare_parameter("duration_sec", 1.0)
        self.duration_sec = (
            self.get_parameter("duration_sec").get_parameter_value().double_value
        )
        self.server = ActionServer(
            self,
            Navigate,
            "/Navigate_to_pose",
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
        )
        self.get_logger().info("Mock Kachaka Navigate server ready at /Navigate_to_pose")

    def goal_cb(self, goal_request):
        self.get_logger().info(
            f"NAV goal received: ({goal_request.target_x:.2f}, {goal_request.target_y:.2f})"
        )
        return GoalResponse.ACCEPT

    def cancel_cb(self, _goal_handle):
        self.get_logger().info("NAV cancel requested")
        return CancelResponse.ACCEPT

    def execute_cb(self, goal_handle):
        target_x = float(goal_handle.request.target_x)
        target_y = float(goal_handle.request.target_y)
        total_steps = max(1, int(self.duration_sec / 0.1))

        for step in range(total_steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Navigate.Result(success=False, message="mock navigation canceled")

            progress = float(step + 1) / float(total_steps)
            feedback = Navigate.Feedback()
            feedback.current_x = target_x * progress
            feedback.current_y = target_y * progress
            feedback.distance_remaining = math.hypot(
                target_x - feedback.current_x,
                target_y - feedback.current_y,
            )
            goal_handle.publish_feedback(feedback)
            time.sleep(0.1)

        goal_handle.succeed()
        self.get_logger().info("NAV goal succeeded")
        return Navigate.Result(success=True, message="mock navigation success")


def main():
    rclpy.init()
    node = MockKachakaNavServer()
    try:
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
