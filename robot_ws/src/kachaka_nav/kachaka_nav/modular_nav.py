import rclpy
import math
import time
import yaml
import os
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from kachaka_interfaces.action import Navigate
from kachaka_nav.robot_driver import KachakaRealDriver, RosSimDriver
from tf2_ros import TransformBroadcaster
from transforms3d.euler import euler2quat

class ModularNavNode(Node):
    def __init__(self):
        super().__init__('modular_nav_node')
        
        # === CALLBACK GROUPS ===
        # use separate callback groups for timers and actions to allow them to run concurrently without blocking each other
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        self.action_callback_group = ReentrantCallbackGroup()
        
        # --- PARAMETERS ---
        self.declare_parameter('use_sim', False)
        self.declare_parameter('kachaka_ip', '192.168.0.157:26400')
        self.declare_parameter('user_map_yaml', '')
        # distance tolerance (meters) to consider goal reached
        self.declare_parameter('goal_xy_tolerance', 0.6)

        self.use_sim = self.get_parameter('use_sim').value
        robot_ip = self.get_parameter('kachaka_ip').value
        yaml_path = self.get_parameter('user_map_yaml').value
        self.goal_xy_tolerance = float(self.get_parameter('goal_xy_tolerance').value)

        # --- MAP ALIGNMENT STATE ---
        self.map_offset_x = 0.0
        self.map_offset_y = 0.0
        self.map_yaw = 0.0
        
        if yaml_path and os.path.exists(yaml_path):
            self.load_map_alignment(yaml_path)

        # --- DRIVER SETUP ---
        if self.use_sim:
            self.driver = RosSimDriver(self)
        else:
            self.get_logger().info(f"Connecting to REAL KACHAKA at {robot_ip}...")
            self.driver = KachakaRealDriver(robot_ip)

        # --- PUBLISHERS ---
        # 1. Pose in YOUR Map Frame (for your custom map view)
        self.user_pose_pub = self.create_publisher(PoseStamped, '/user_pose', 10)
        
        # 2. Pose in KACHAKA Map Frame
        self.kachaka_pose_pub = self.create_publisher(PoseStamped, '/kachaka_pose', 10)
        
        # 3. Path (trajectory) Publisher
        self.path_pub = self.create_publisher(Path, '/robot_path', 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"
        
        # 4. TF Broadcaster for robot position
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer to publish both poses (10Hz) - use separate callback group
        self.create_timer(0.1, self.publish_pose_callback, callback_group=self.timer_callback_group)

        # --- ACTION SERVER ---
        self._action_server = ActionServer(
            self, Navigate, '/Navigate_to_pose', 
            self.execute_callback, 
            callback_group=self.action_callback_group
        )
        self.get_logger().info("✅ Modular Nav Ready (Dual-Frame Publishing).")

    def load_map_alignment(self, yaml_path):
        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
            origin = data.get('origin', [0.0, 0.0, 0.0])
            self.map_offset_x = float(origin[0])
            self.map_offset_y = float(origin[1])
            self.map_yaw = float(origin[2])
        except Exception as e:
            self.get_logger().error(f"❌ YAML Error: {e}")

    # --- TRANSFORM HELPERS ---
    def transform_user_to_kachaka(self, user_x, user_y):
        cos_t = math.cos(self.map_yaw)
        sin_t = math.sin(self.map_yaw)
        rot_x = (user_x * cos_t) - (user_y * sin_t)
        rot_y = (user_x * sin_t) + (user_y * cos_t)
        return rot_x + self.map_offset_x, rot_y + self.map_offset_y

    def transform_kachaka_to_user(self, k_x, k_y):
        dx = k_x - self.map_offset_x
        dy = k_y - self.map_offset_y
        cos_t = math.cos(-self.map_yaw)
        sin_t = math.sin(-self.map_yaw)
        u_x = (dx * cos_t) - (dy * sin_t)
        u_y = (dx * sin_t) + (dy * cos_t)
        return u_x, u_y

    # --- POSE LOOP ---
    def publish_pose_callback(self):
        try:
            # A. Get Raw Pose
            k_x, k_y, k_yaw = self.driver.get_pose()
            current_time = self.get_clock().now().to_msg()
            
            # Debug: Log position every 50 calls (every 5 seconds at 10Hz)
            if not hasattr(self, '_pose_call_count'):
                self._pose_call_count = 0
                self.get_logger().info(f"📍 Starting pose publishing...")
            
            self._pose_call_count += 1
            # if self._pose_call_count % 50 == 0:
            #    self.get_logger().info(f"📍 Robot at: ({k_x:.2f}, {k_y:.2f}, {math.degrees(k_yaw):.1f}°)")
            
            # B. Publish Raw Pose (Kachaka Frame)
            msg_k = PoseStamped()
            msg_k.header.stamp = current_time
            msg_k.header.frame_id = "map" # Kachaka's map frame
            msg_k.pose.position.x = k_x
            msg_k.pose.position.y = k_y
            msg_k.pose.position.z = 0.0
            
            # Convert yaw to quaternion
            quat = euler2quat(0, 0, k_yaw)  # Returns (w, x, y, z)
            msg_k.pose.orientation.w = quat[0]
            msg_k.pose.orientation.x = quat[1]
            msg_k.pose.orientation.y = quat[2]
            msg_k.pose.orientation.z = quat[3]
            
            self.kachaka_pose_pub.publish(msg_k)
            
            # B2. Broadcast TF transform (map -> base_link)
            t = TransformStamped()
            t.header.stamp = current_time
            t.header.frame_id = "map"
            t.child_frame_id = "base_link"
            t.transform.translation.x = k_x
            t.transform.translation.y = k_y
            t.transform.translation.z = 0.0
            t.transform.rotation.w = quat[0]
            t.transform.rotation.x = quat[1]
            t.transform.rotation.y = quat[2]
            t.transform.rotation.z = quat[3]
            self.tf_broadcaster.sendTransform(t)
            
            # B3. Add to path trajectory
            self.path_msg.header.stamp = current_time
            self.path_msg.poses.append(msg_k)
            
            # Keep only last 8000 points to avoid memory issues
            if len(self.path_msg.poses) > 8000:
                self.path_msg.poses.pop(0)
            
            self.path_pub.publish(self.path_msg)

            # C. Transform & Publish User Pose (User Frame)
            u_x, u_y = self.transform_kachaka_to_user(k_x, k_y)
            msg_u = PoseStamped()
            msg_u.header.stamp = current_time
            msg_u.header.frame_id = "user_map"
            msg_u.pose.position.x = u_x
            msg_u.pose.position.y = u_y
            msg_u.pose.position.z = 0.0
            msg_u.pose.orientation.w = quat[0]
            msg_u.pose.orientation.x = quat[1]
            msg_u.pose.orientation.y = quat[2]
            msg_u.pose.orientation.z = quat[3]
            self.user_pose_pub.publish(msg_u)
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to publish pose: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

    def execute_callback(self, goal_handle):
        # Receive in User Frame -> Convert to Kachaka -> Move
        u_x = goal_handle.request.target_x
        u_y = goal_handle.request.target_y
        k_x, k_y = self.transform_user_to_kachaka(u_x, u_y)

        self.get_logger().info(f"Navigating -> User({u_x:.2f}, {u_y:.2f}) | Kachaka({k_x:.2f}, {k_y:.2f})")
        # Start non-blocking move so we can cancel when within threshold
        try:
            self.driver.move_native(k_x, k_y, yaw=0.0, wait_for_completion=False)
        except TypeError:
            # older driver may not accept wait_for_completion; fallback to blocking
            self.driver.move_native(k_x, k_y, yaw=0.0)

        start_time = time.time()
        success = False
        feedback = Navigate.Feedback()

        while rclpy.ok():
            if time.time() - start_time > 60.0:
                # timeout: attempt to cancel and stop
                try:
                    self.driver.cancel_current_command()
                except Exception:
                    pass
                try:
                    self.driver.stop()
                except Exception:
                    pass
                break

            curr_k_x, curr_k_y, _ = self.driver.get_pose()
            dist = math.hypot(k_x - curr_k_x, k_y - curr_k_y)
            
            feedback.distance_remaining = dist
            goal_handle.publish_feedback(feedback)

            if dist < self.goal_xy_tolerance:
                # we're within desired tolerance: cancel robot's internal nav and stop
                try:
                    self.driver.cancel_current_command()
                except Exception:
                    pass
                try:
                    self.driver.stop()
                except Exception:
                    pass
                success = True
                break
            time.sleep(0.5)

        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return Navigate.Result(success=success)

def main(args=None):
    rclpy.init(args=args)
    
    # 使用multi-threaded executor來允許多個callback同時運行
    node = ModularNavNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()