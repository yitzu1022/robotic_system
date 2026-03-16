import sys
import os
import yaml
import kachaka_api
from geometry_msgs.msg import Twist

class KachakaRealDriver:import os
import yaml
import kachaka_api

class KachakaRealDriver:
    def __init__(self, robot_ip):
        # Connect to the robot
        self.client = kachaka_api.KachakaApiClient(robot_ip)

    def export_native_map(self):
        # 1. Define the correct absolute path
        # Assuming your workspace is at /home/acm118/robot_ws
        # We join paths to get: /home/acm118/robot_ws/data/tmp
        home_dir = os.path.expanduser('~') 
        workspace_dir = os.path.join(home_dir, 'robot_ws')
        target_folder = os.path.join(workspace_dir, 'data', 'tmp')
        
        image_path = os.path.join(target_folder, 'kachaka_native.png')
        yaml_path = os.path.join(target_folder, 'kachaka_native.yaml')

        # 2. IMPORTANT: Create the folder if it doesn't exist
        # This fixes the "No such file or directory" error
        os.makedirs(target_folder, exist_ok=True)

        print(f"Saving map to: {target_folder}")

        try:
            # 3. Download and Save
            map_data = self.client.get_png_map()
            
            with open(image_path, "wb") as f:
                f.write(map_data.data)
            
            map_metadata = {
                "image": image_path,
                "resolution": map_data.resolution,
                "origin": [map_data.origin.x, map_data.origin.y, 0.0],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.196
            }

            with open(yaml_path, "w") as f:
                yaml.dump(map_metadata, f)
            
            return image_path, yaml_path

        except Exception as e:
            print(f"Failed to download map: {e}")
            return None, None

    def get_pose(self):
        """
        Returns the current robot pose (x, y, theta) from the real Kachaka.
        """
        # Call the API to get the pose
        pose = self.client.get_robot_pose()
        
        # Return x, y, and theta (yaw)
        # The Kachaka API provides these directly
        return pose.x, pose.y, pose.theta

    def set_velocity(self, linear, angular):
        """
        Sends a velocity command to the real robot.
        linear: forward speed (m/s)
        angular: turning speed (rad/s)
        """
        # The API method is set_robot_velocity(linear, angular)
        self.client.set_robot_velocity(linear, angular)

    def stop(self):
        self.set_velocity(0.0, 0.0)
    def move_native(self, x, y, yaw=0.0, wait_for_completion: bool = True):
        """
        Commands the real Kachaka to go to (x, y) using its internal navigation.
        If `wait_for_completion` is False the command will be started and
        this method will return immediately (the robot will continue moving).
        """
        return self.client.move_to_pose(x, y, yaw, wait_for_completion=wait_for_completion)

    def cancel_current_command(self):
        """
        Cancel the currently running command on the robot (if any).
        Returns the API response (result, command) or None on error.
        """
        try:
            return self.client.cancel_command()
        except Exception as e:
            print(f"cancel_current_command failed: {e}")
            return None
    
    def is_command_running(self):
        """
        Checks if the robot is currently executing a command.
        """
        # The API provides 'get_command_state'
        # If tuple, usually (state, command_id). Check API docs for exact return.
        # For simplicity, we can check if we are moving:
        state = self.client.get_robot_command_state()
        
        # State 0 usually means idle/success. 
        # State 1 (RUNNING) is what we look for.
        # (Note: exact ID might vary, but we can also check velocity)
        
        # Alternative: Just check if we are close to the goal in the Node loop.
        return state


class RosSimDriver:
    """Mock driver for Simulation (No Connection needed)"""
    def __init__(self, node):
        self.node = node
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.pub = node.create_publisher(Twist, '/cmd_vel', 10)

    def get_pose(self):
        # In a real sim driver, you would subscribe to /odom here
        return self.current_x, self.current_y, self.current_theta

    def set_velocity(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.pub.publish(msg)

    def stop(self):
        self.set_velocity(0.0, 0.0)