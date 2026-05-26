from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="decision_maker",
                executable="agent_decision_maker_node",
                name="agent_decision_maker_node",
                output="screen",
                parameters=[
                    {
                        "enable_map_visualizer": False,
                        "mock_execution": True,
                        "mock_object_query": False,
                        "mock_failure_rate": 0.0,
                        "mock_fail_once_capabilities": "navigation",
                        "agent_max_steps": 25,
                        "agent_max_replans": 2,
                    }
                ],
            ),
        ]
    )
