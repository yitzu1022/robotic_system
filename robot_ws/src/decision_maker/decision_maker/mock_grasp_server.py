import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from mm_interface.action import TaskCommand


class MockGraspPlaceServer(Node):
    def __init__(self):
        super().__init__('mock_grasp_place_server')
        # Unified TaskCommand action server handles grasp, place, and handover commands.
        self.task_server = ActionServer(
            self,
            TaskCommand,
            'task_command',
            execute_callback=self.execute_task,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
        )
        self.get_logger().info('Mock TaskCommand server ready at /task_command')

    def goal_cb(self, goal_request: TaskCommand.Goal):
        self.get_logger().info(f'TaskCommand goal received: "{goal_request.command}"')
        return GoalResponse.ACCEPT

    def cancel_cb(self, _goal_handle):
        self.get_logger().info('TaskCommand cancel requested')
        return CancelResponse.ACCEPT

    async def execute_task(self, goal_handle):
        command = goal_handle.request.command.strip().lower()
        fb = TaskCommand.Feedback()

        if command.startswith('grasp'):
            phases = ['approach', 'align', 'close_gripper', 'lift', 'done']
        elif command.startswith('place'):
<<<<<<< HEAD
            phases = ['move_to_bin', 'lower', 'open_gripper', 'retract', 'done']
=======
            phases = ['move_to_target', 'lower', 'open_gripper', 'retract', 'done']
        elif command.startswith('handover'):
            phases = ['move_to_person', 'wait_for_take', 'open_gripper', 'retract', 'done']
>>>>>>> f3d94b527b089f0735202ad3d974137243cbb97e
        else:
            self.get_logger().warn(f'Unknown command: "{command}"')
            goal_handle.succeed()
            return TaskCommand.Result(success=False, message=f'unknown command: {command}')

        for ph in phases:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('TaskCommand goal canceled')
                return TaskCommand.Result(success=False, message='canceled')
            fb.feedback = ph
            goal_handle.publish_feedback(fb)
            time.sleep(0.5)

        goal_handle.succeed()
        return TaskCommand.Result(success=True, message='ok')


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
