import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node

from object_query_interfaces.srv import ObjectQuery


class MockObjectQueryServer(Node):
    """Small semantic lookup mock for offline ROS interface tests."""

    def __init__(self):
        super().__init__("mock_object_query_server")
        self.declare_parameter("fail_unknown", True)
        self.fail_unknown = (
            self.get_parameter("fail_unknown").get_parameter_value().bool_value
        )
        self.object_db = self._default_object_db()
        self.server = self.create_service(ObjectQuery, "/object_query", self.handle_query)
        self.get_logger().info(
            "Mock ObjectQuery server ready at /object_query with targets: "
            + ", ".join(sorted(self.object_db))
        )

    def handle_query(self, request, response):
        name = request.name.strip().lower()
        pose = self.object_db.get(name)

        if pose is None and not self.fail_unknown:
            pose = self.object_db["home"]

        if pose is None:
            response.found = False
            response.position = Point()
            response.message = f"mock target not found: {name}"
            self.get_logger().warn(response.message)
            return response

        response.found = True
        response.position = Point(x=float(pose[0]), y=float(pose[1]), z=float(pose[2]))
        response.message = f"mock target found: {name}"
        self.get_logger().info(
            f"ObjectQuery({name}) -> ({pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f})"
        )
        return response

    def _default_object_db(self):
        return {
            "home": (0.0, 0.0, 0.0),
            "me": (0.0, 0.0, 0.0),
            "cabinet": (1.0, 0.5, 0.0),
            "table": (2.0, 0.0, 0.0),
            "sofa": (3.0, 1.0, 0.0),
            "chair": (2.5, -1.0, 0.0),
            "trash_bin": (-1.0, 1.0, 0.0),
            "kitchen": (4.0, 0.0, 0.0),
            "fridge": (4.5, 0.5, 0.0),
            "apple": (2.0, 0.0, 0.0),
            "bottle": (1.0, 0.5, 0.0),
            "pringle": (1.0, 0.5, 0.0),
            "pringles": (1.0, 0.5, 0.0),
            "drink": (4.5, 0.5, 0.0),
        }


def main():
    rclpy.init()
    node = MockObjectQueryServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
