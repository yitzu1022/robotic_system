import time
import rclpy
from geometry_msgs.msg import Point
from object_query_interfaces.srv import ObjectQuery


class WorldModel:
    """
    The WorldModel class bridges high-level commands with semantic world data.
    It can resolve objects and places by querying an external service or internal mapping.
    """

    def __init__(self, node=None):
        # Reuse existing node if given, otherwise create a standalone one
        self.node = node or rclpy.create_node('world_model_node')
        self.obj_client = self.node.create_client(ObjectQuery, 'object_query')

    # =========================================================
    # OBJECT RESOLUTION
    # =========================================================
    def resolve_object(self, name: str):
        """Query the ObjectQuery service for the 3D position of a given object name."""
        name = name.strip().lower()

        # Wait for the service to be available
        if not self.obj_client.wait_for_service(timeout_sec=3.0):
            self.node.get_logger().warn("❌ ObjectQuery service unavailable.")
            return None

        req = ObjectQuery.Request()
        req.name = name
        future = self.obj_client.call_async(req)

        start = time.time()
        timeout = 60.0                  # timeout from 10.0 to 60.0s
        while rclpy.ok() and not future.done():
            # rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.05)
            if time.time() - start > timeout:
                self.node.get_logger().warn(f"⏰ Timeout: object query for '{name}' took > {timeout}s")
                return None

        if not future.done() or future.result() is None:
            self.node.get_logger().error(f"⚠️ ObjectQuery returned no result for '{name}'")
            return None

        res = future.result()
        if not res.found:
            self.node.get_logger().warn(f"⚠️ Object '{name}' not found in semantic map.")
            return None

        pos = res.position
        self.node.get_logger().info(
            f"✅ Object '{name}' located at ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})"
        )
        return (pos.x, pos.y, pos.z)

    # =========================================================
    # PLACE RESOLUTION (FIXED)
    # =========================================================
    def resolve_place(self, name: str):
        name = name.strip().lower()
        
        # 1. First, check your hardcoded list (Fastest)
        static_places = {
            "me": (0.0, 0.0, 0.0),
            "home": (1.5, 0.0, 0.0),
            # "table": (1.0, 1.0, 0.0),
            "kitchen": (3.0, -1.0, 0.0),
        }
        
        if name in static_places:
            return static_places[name]

        # 2. IF NOT FOUND above, send command to Object Query Node (The Missing Link)
        self.node.get_logger().info(f"UNKNOWN PLACE '{name}': Forwarding to Object Query Node...")
        
        # This calls the function that actually talks to the service
        dynamic_coords = self.resolve_object(name)
        
        if dynamic_coords is not None:
            return dynamic_coords

        # 3. If BOTH fail, then use default
        self.node.get_logger().warn(f"⚠️ Unknown destination '{name}', using (0,0,0).")
        return (0.0, 0.0, 0.0)

# =========================================================
# GOTO FORMATTER
# =========================================================
def fmt_goto(pose_tuple):
    """
    Convert a (x, y, yaw) or (x, y) tuple to a 'goto:x,y,th' string.
    """
    if isinstance(pose_tuple, (list, tuple)):
        if len(pose_tuple) == 2:
            x, y = pose_tuple
            th = 0.0
        elif len(pose_tuple) == 3:
            x, y, th = pose_tuple
        else:
            raise ValueError("Pose tuple must have 2 or 3 elements.")
        return f"goto:{x:.2f},{y:.2f},{th:.2f}"
    raise ValueError("fmt_goto expects a tuple or list of coordinates.")
