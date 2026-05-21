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
                        "agent_max_steps": 10,
                    }
                ],
            ),
        ]
    )
