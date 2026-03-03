import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from decision_maker_interfaces.action import Grasp, Place


class MockGraspPlaceServer(Node):
    def __init__(self):
        super().__init__('mock_grasp_place_server')
        # Grasp action server
        self.grasp_server = ActionServer(
            self,
            Grasp,
            'grasp',
            execute_callback=self.execute_grasp,
            goal_callback=self.goal_grasp_cb,
            cancel_callback=self.cancel_grasp_cb,
        )
        # Place action server
        self.place_server = ActionServer(
            self,
            Place,
            'place',
            execute_callback=self.execute_place,
            goal_callback=self.goal_place_cb,
            cancel_callback=self.cancel_place_cb,
        )
        self.get_logger().info('Mock Grasp/Place servers ready at /grasp and /place')

    # ---------------- Grasp callbacks ----------------
    def goal_grasp_cb(self, goal_request: Grasp.Goal):
        self.get_logger().info(f'GRASP goal received: obj={goal_request.object_id}')
        return GoalResponse.ACCEPT

    def cancel_grasp_cb(self, _goal_handle):
        self.get_logger().info('GRASP cancel requested')
        return CancelResponse.ACCEPT

    async def execute_grasp(self, goal_handle):
        fb = Grasp.Feedback()
        phases = ['approach', 'align', 'close_gripper', 'lift', 'done']
        for i, ph in enumerate(phases):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('GRASP goal canceled')
                return Grasp.Result(success=False, message='canceled')
            fb.state = ph
            fb.progress = float(i + 1) / len(phases)
            goal_handle.publish_feedback(fb)
            time.sleep(0.5)
        goal_handle.succeed()
        return Grasp.Result(success=True, message='ok')

    # ---------------- Place callbacks ----------------
    def goal_place_cb(self, goal_request: Place.Goal):
        self.get_logger().info(f'PLACE goal received: obj={goal_request.object_id} -> bin={goal_request.target_bin}')
        return GoalResponse.ACCEPT

    def cancel_place_cb(self, _goal_handle):
        self.get_logger().info('PLACE cancel requested')
        return CancelResponse.ACCEPT

    async def execute_place(self, goal_handle):
        fb = Place.Feedback()
        phases = ['move_to_bin', 'lower', 'open_gripper', 'retract', 'done']
        for i, ph in enumerate(phases):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('PLACE goal canceled')
                return Place.Result(success=False, message='canceled')
            fb.state = ph
            fb.progress = float(i + 1) / len(phases)
            goal_handle.publish_feedback(fb)
            time.sleep(0.5)
        goal_handle.succeed()
        return Place.Result(success=True, message='ok')


def main():
    rclpy.init()
    node = MockGraspPlaceServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
