import threading
import queue
import time
import math
from typing import List, Optional
import yaml
import numpy as np
import cv2
import os

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient

from std_msgs.msg import String
from kachaka_interfaces.action import Navigate
from mm_interface.action import TaskCommand
from object_query_interfaces.srv import ObjectQuery

# Assuming these exist in your package
from .command_types import Command
from .scenario_library import SCENARIO_REGISTRY
from .nl_planner import WorldModel


class MapVisualizer:
    """Thread-safe OpenCV visualizer with a dedicated GUI loop thread.
    Keeps an internal display buffer which other threads update via methods.
    The GUI thread performs cv2.imshow/cv2.waitKey to ensure proper display.
    """
    def __init__(self, map_yaml_path: str, logger):
        import threading as _th
        self.logger = logger
        if not os.path.exists(map_yaml_path):
            raise FileNotFoundError(map_yaml_path)

        with open(map_yaml_path, 'r') as f:
            cfg = yaml.safe_load(f)

        self.resolution = float(cfg.get('resolution', 0.05))
        self.origin = cfg.get('origin', [0.0, 0.0, 0.0])

        map_dir = os.path.dirname(map_yaml_path)
        image_file = cfg.get('image')
        if image_file is None:
            raise ValueError('map yaml missing image field')

        image_path = os.path.join(map_dir, image_file)
        if not os.path.exists(image_path):
            raise FileNotFoundError(image_path)

        img = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
        if img is None:
            raise ValueError(f'failed to load map image {image_path}')

        # Normalize image to 3-channel BGR if necessary
        if img.ndim == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        elif img.shape[2] == 4:
            # drop alpha by converting to BGR
            img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

        self.map_image = img
        self.h, self.w = self.map_image.shape[:2]
        self.window_name = 'map_visualizer'

        # thread-safe buffer and controls
        self._lock = _th.Lock()
        self._buffer = self.map_image.copy()
        self._stop = _th.Event()

        # start GUI thread
        self._thread = _th.Thread(target=self._gui_loop, daemon=True)
        self._thread.start()
        self.logger.info(f'📍 MapVisualizer initialized: {self.w}x{self.h}, res={self.resolution:.3f}m/px')

    def world_to_pixel(self, x: float, y: float) -> tuple:
        px = int((x - self.origin[0]) / self.resolution)
        py = int((y - self.origin[1]) / self.resolution)
        py = self.h - py
        return px, py

    def reset(self):
        with self._lock:
            self._buffer = self.map_image.copy()

    def draw_marker(self, x: float, y: float, label: str = '', color=(0, 255, 255), radius: int = 10):
        px, py = self.world_to_pixel(x, y)
        with self._lock:
            if 0 <= px < self.w and 0 <= py < self.h:
                cv2.circle(self._buffer, (px, py), radius, color, -1)
                cv2.circle(self._buffer, (px, py), radius + 2, (255, 255, 255), 2)
                if label:
                    cv2.putText(self._buffer, label, (px + 12, py - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    cv2.putText(self._buffer, label, (px + 12, py - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)
                self.logger.info(f"Drew '{label}' at world({x:.2f},{y:.2f}) -> pixel({px},{py})")
                return True
            else:
                self.logger.warn(f"Marker out of bounds: world({x:.2f},{y:.2f}) -> pixel({px},{py})")
                return False

    def _gui_loop(self):
        # create window in this thread
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        while not self._stop.is_set():
            with self._lock:
                frame = self._buffer.copy()
            cv2.imshow(self.window_name, frame)
            # small wait to process GUI events
            key = cv2.waitKey(50)
            if key == 27:  # ESC to close
                self._stop.set()
                break
        try:
            cv2.destroyWindow(self.window_name)
        except Exception:
            pass

    def stop(self):
        try:
            self._stop.set()
            if hasattr(self, '_thread'):
                self._thread.join(timeout=1.0)
        except Exception:
            pass


class DecisionMakingNode(Node):
    def __init__(self):
        super().__init__('decision_making_node')

        # ====== Core setup ======
        self.cmd_queue: "queue.Queue[dict]" = queue.Queue(maxsize=50)
        self._shutdown = threading.Event()
        self.world = WorldModel(node=self)

        # ====== 🛠️ MAP CALIBRATION CONFIG (3D->2D YAML) 🛠️ ======
        # Load 3D->2D transform parameters from YAML
        self.declare_parameter('map3d_to_map2d_yaml', 'data/Util/alignment.yaml')
        yaml_path = self.get_parameter('map3d_to_map2d_yaml').get_parameter_value().string_value
        
        self.map2d_params = None
        if yaml_path:
            # Support both absolute and relative paths
            if not os.path.isabs(yaml_path):
                # Try relative to workspace or current directory
                yaml_path = os.path.abspath(yaml_path)
            
            if os.path.exists(yaml_path):
                try:
                    self.map2d_params = load_map3d_to_map2d(yaml_path)
                    self.get_logger().info(f'✅ Loaded 3D->2D transform from: {yaml_path}')
                except Exception as e:
                    self.get_logger().error(f'❌ Failed loading 3D->2D yaml: {e}')
                    import traceback
                    traceback.print_exc()
            else:
                self.get_logger().error(f'❌ YAML file not found: {yaml_path}')
        else:
            self.get_logger().warn('⚠️ No map3d_to_map2d_yaml specified, 3D->2D transform disabled')
        # ==========================================================

        # ====== Map visualizer (direct OpenCV display) ======
        # Optional parameter to point to map yaml (PNG must be next to it)
        try:
            self.declare_parameter('map_yaml', 'data/lab/kachaka_native.yaml')
            map_yaml = self.get_parameter('map_yaml').get_parameter_value().string_value
            if map_yaml and not os.path.isabs(map_yaml):
                map_yaml = os.path.abspath(map_yaml)
            if map_yaml and os.path.exists(map_yaml):
                try:
                    self.visualizer = MapVisualizer(map_yaml, self.get_logger())
                except Exception as e:
                    self.get_logger().error(f"❌ Failed to init MapVisualizer: {e}")
                    self.visualizer = None
            else:
                self.visualizer = None
        except Exception:
            self.visualizer = None
        
        # ====== Grasp approach threshold ======
        self.declare_parameter('grasp_approach_dist', 0.5)   # set the arm to reach the object within 0.5 meters
        self._grasp_threshold = self.get_parameter('grasp_approach_dist').get_parameter_value().double_value
        self._nav_distance_remaining = float('inf')          # update by nav feedback to know how far we are from the target, used for grasp approach logic

        # ====== ROS entities ======
        self.sub_manual = self.create_subscription(String, '/manual_command', self.on_text_event, 10)
        self.cancel_sub = self.create_subscription(String, '/cancel_command', self.on_cancel_event, 10)
        self.status_pub = self.create_publisher(String, '/task_status', 10)

        # ====== Action clients ======
        self.nav_client = ActionClient(self, Navigate, '/Navigate_to_pose')         #change to similar name but not the same name with nav2's topic
        self.task_client = ActionClient(self, TaskCommand, '/task_command')

        # ====== Service client (object query) ======
        self.obj_client = self.create_client(ObjectQuery, '/object_query')

        # ====== Threads ======
        self.sensor_worker = threading.Thread(target=self.sensor_processing_loop, daemon=True)
        self.exec_worker = threading.Thread(target=self.command_executor_loop, daemon=True)
        self.sensor_worker.start()
        self.exec_worker.start()

        self.get_logger().info("🧭 DecisionMakingNode ready with 3D->2D Map Calibration.")

    # =============================================================
    # SENSOR LOOP (Removed spin_once to avoid conflict with MultiThreadedExecutor)
    # =============================================================
    def sensor_processing_loop(self):
        """Background thread for sensor data processing.
        Note: Actual spinning is handled by MultiThreadedExecutor in main().
        """
        rate = self.create_rate(10)
        while not self._shutdown.is_set():
            # Process sensor data here if needed
            # DO NOT call rclpy.spin_once() - causes race condition with executor
            rate.sleep()

    # =============================================================
    # MANUAL COMMAND HANDLER
    # =============================================================
    def on_text_event(self, msg: String):
        text = msg.data.strip().lower()
        self.get_logger().info(f"🗣 Received command: '{text}'")

        try:
            scenario_fn = None
            matched_key = None

            for key, fn in SCENARIO_REGISTRY.items():
                if text.startswith(key):
                    scenario_fn = fn
                    matched_key = key
                    break

            if not scenario_fn:
                raise ValueError(f"No matching scenario for '{text}'")

            argument_str = text[len(matched_key):].strip()

            # Run scenario planning in a background thread so that any
            # blocking service calls (ObjectQuery) do not block the
            # rclpy executor thread and prevent service responses.
            def _plan_and_enqueue():
                try:
                    if argument_str:
                        primitives = scenario_fn(self.world, argument_str)
                    else:
                        try:
                            primitives = scenario_fn(self.world)
                        except TypeError:
                            raise ValueError(f"Command '{matched_key}' requires a target.")
                    self.enqueue_command(text, primitives)
                except Exception as e:
                    self.get_logger().error(f"❌ Failed to plan command '{text}': {e}")

            t = threading.Thread(target=_plan_and_enqueue, daemon=True)
            t.start()

        except Exception as e:
            self.get_logger().error(f"❌ Failed to interpret command: {e}")

    def enqueue_command(self, name: str, primitives: List[str]):
        try:
            batch = {"name": name, "actions": primitives, "timestamp": time.time()}
            self.cmd_queue.put(batch, timeout=0.2)
            self.get_logger().info(f"📦 Enqueued '{name}' → {primitives}")
        except queue.Full:
            self.get_logger().warn("⚠️ Command queue full. Dropping command.")

    # =============================================================
    # COMMAND EXECUTION LOOP
    # =============================================================
    def command_executor_loop(self):
        while not self._shutdown.is_set():
            try:
                batch = self.cmd_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            name, actions = batch["name"], batch["actions"]
            self.get_logger().info(f"🚀 Executing batch: {name}")

            success = self._execute_batch(name, actions)
            if success:
                self.get_logger().info(f"✅ Finished batch: {name}")
            else:
                self.get_logger().warn(f"❌ Batch '{name}' failed or timed out.")

            self.cmd_queue.task_done()

    def _execute_batch(self, name: str, actions: List[str], timeout_sec: float = 300.0) -> bool:
        start_time = time.time()
        for i, act in enumerate(actions):
            if time.time() - start_time > timeout_sec:
                self.get_logger().warn(f"⏰ Timeout: '{name}' exceeded {timeout_sec}s, cancelling.")
                self._send_cancel()
                return False

            act = act.strip().lower()
            if act.startswith('goto:'):
                self.get_logger().info(f"📍 Executing {act}")
                next_act = actions[i + 1].strip().lower() if i + 1 < len(actions) else ''
                for_grasp = next_act.startswith('grasp:')
                if not self._execute_nav(act, for_grasp=for_grasp):
                    return False
            elif act.startswith('grasp:'):
                self.get_logger().info(f"✋ Executing {act}")
                if not self._execute_grasp(act):
                    return False
            elif act.startswith('place:'):
                self.get_logger().info(f"📦 Executing {act}")
                if not self._execute_place(act):
                    return False
            elif act.startswith('handover:'):
                self.get_logger().info(f"🤝 Executing {act}")
                if not self._execute_handover(act):
                    return False
            else:
                self.get_logger().warn(f"⚠️ Unknown action: {act}")
        return True

    # =============================================================
    # 📐 MAP TRANSFORM HELPER (3D->2D with plane_fit + sim2)
    # =============================================================
    def _apply_transform(self, x_in: float, y_in: float, z_in: float = 0.0) -> tuple:
        """Converts 3D Map Coordinates -> 2D Navigation Coordinates using YAML params"""
        if self.map2d_params is None:
            self.get_logger().warn('⚠️ No 3D->2D params loaded, returning raw (x,y)')
            return (x_in, y_in)
        
        mu, e1, e2, s, R, t = self.map2d_params
        xy = map3d_point_to_map2d_xy((x_in, y_in, z_in), mu, e1, e2, s, R, t)
        return (float(xy[0]), float(xy[1]))

    # =============================================================
    # OBJECT QUERY WRAPPER
    # =============================================================
    def _query_object_position(self, object_name: str, timeout_sec: float = 30.0) -> Optional[tuple]:
        """Request 3D position AND transform it to 2D Nav Frame."""
        if not self.obj_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("❌ ObjectQuery service not available.")
            return None

        req = ObjectQuery.Request()
        req.name = object_name
        future = self.obj_client.call_async(req)

        # Wait for result (MultiThreadedExecutor handles spinning)
        start = time.time()
        while not future.done():
            if time.time() - start > timeout_sec:
                self.get_logger().warn(f"⏰ Object query timeout for '{object_name}'.")
                return None
            time.sleep(0.05)  # Small sleep to avoid busy waiting

        result = future.result()
        if result is None or not result.found:
            self.get_logger().warn(f"⚠️ Object '{object_name}' not found.")
            return None

        # --- TRANSFORM LOGIC ---
        # Get raw coordinates (x, y, z) from the 3D map
        raw_x = result.position.x
        raw_y = result.position.y
        raw_z = result.position.z
        
        # Apply the 3D->2D calibration (plane_fit + sim2)
        nav_x, nav_y = self._apply_transform(raw_x, raw_y, raw_z)
        
        self.get_logger().info(f"✅ Found '{object_name}'")
        self.get_logger().info(f"   Raw 3D: ({raw_x:.2f}, {raw_y:.2f})")
        self.get_logger().info(f"   Nav 2D: ({nav_x:.2f}, {nav_y:.2f})")
        
        # Visualize on desktop map window (direct OpenCV display)
        try:
            if hasattr(self, 'visualizer') and self.visualizer:
                # reset to base map and draw marker
                self.visualizer.reset()
                self.visualizer.draw_marker(nav_x, nav_y, label=object_name, color=(0, 255, 255), radius=12)
                # show briefly (non-blocking)
                # self.visualizer.show(wait_ms=1)
        except Exception as e:
            self.get_logger().warn(f"⚠️ Visualization error: {e}")

        return (nav_x, nav_y, 0.0)

    # =============================================================
    # NAV / GRASP / PLACE
    # =============================================================
    def _execute_nav(self, cmd: str, for_grasp: bool = False) -> bool:
        """
        Handles 'goto:x,y,th'.
        CRITICAL CHANGE: Now applies calibration transform to these coordinates too.
        for_grasp=True enables the early-exit shortcut when the robot is within
        grasp_approach_dist of the target (so the arm can reach the object).
        for_grasp=False (default) waits for the nav to fully succeed.
        """
        # Reset stale distance so a previous nav's final reading cannot
        # trigger the threshold check at the start of this nav.
        self._nav_distance_remaining = float('inf')
        try:
            _, payload = cmd.split(':', 1)
            payload = payload.strip()
            
            x, y, th = 0.0, 0.0, 0.0

            # CASE A: Planner sent coordinates (e.g., "goto:-0.27, 1.76, -0.97")
            if ',' in payload:
                parts = [float(v) for v in payload.split(',')]
                raw_x, raw_y, raw_z = parts[0], parts[1], parts[2]
                if len(parts) > 2: 
                    # We typically don't transform theta unless the map is rotated 90/180 deg
                    # For now, we pass theta through, or you can add self.yaw to it if needed.
                    th = parts[2] 

                # === APPLY TRANSFORM HERE ===
                # The planner sends RAW 3D coordinates. We must convert to NAV coordinates.
                x, y = self._apply_transform(raw_x, raw_y, raw_z)
                
                self.get_logger().info(f"🔄 Transformed: ({raw_x:.2f}, {raw_y:.2f}, {raw_z:.2f}) -> ({x:.2f}, {y:.2f})")
                # show marker on map for this nav target
                try:
                    if hasattr(self, 'visualizer') and self.visualizer:
                        self.visualizer.reset()
                        self.visualizer.draw_marker(x, y, label='NAV_TARGET', color=(0, 0, 255), radius=14)
                        if th is not None:
                            # draw a simple orientation arrow (approx)
                            end_px, end_py = self.visualizer.world_to_pixel(x + 0.5 * math.cos(th), y + 0.5 * math.sin(th))
                            start_px, start_py = self.visualizer.world_to_pixel(x, y)
                            with self.visualizer._lock:
                                cv2.arrowedLine(
                                    self.visualizer._buffer,
                                    (start_px, start_py), (end_px, end_py),
                                    (0, 0, 255), 3, tipLength=0.3)
                except Exception:
                    pass
            
            # CASE B: Object Name "goto:chair" (Fallback if planner didn't resolve it)
            else:
                self.get_logger().info(f"🔍 Looking up coordinates for '{payload}'...")
                pos = self._query_object_position(payload) # This method already transforms
                if not pos:
                    return False
                x, y, _ = pos
                th = 0.0 

            # --- SEND TO NAV2 ---
            if not self.nav_client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error("❌ Nav2 server not available.")
                return False

            goal = Navigate.Goal()
            goal.target_x = float(x)
            goal.target_y = float(y)
            # goal.target_theta = float(th) 

            self.get_logger().info(f"🚀 Sending Nav Goal: ({x:.2f}, {y:.2f})")
            
            fut = self.nav_client.send_goal_async(goal, feedback_callback=self._on_nav_feedback)
            start = time.time()
            while not fut.done():
                if time.time() - start > 10.0:
                    self.get_logger().error("⏰ NAV goal send timeout")
                    return False
                time.sleep(0.05)

            gh = fut.result()
            if not gh.accepted:
                self.get_logger().warn("NAV goal rejected.")
                return False

            res_future = gh.get_result_async()
            start = time.time()
            threshold_triggered = False
            while not res_future.done():
                if time.time() - start > 150.0:
                    self.get_logger().warn("⏰ NAV timeout")
                    self._send_cancel()
                    return False
                
                # Early-exit shortcut: only when navigating toward an object
                # that will be grasped next (not when heading to a place location).
                if for_grasp and self._nav_distance_remaining < self._grasp_threshold:
                    self.get_logger().info(f"Within grasp threshold ({self._nav_distance_remaining:.2f}m), proceeding to grasp.")
                    gh.cancel_goal_async()
                    threshold_triggered = True
                    break
                time.sleep(0.1)

            if not threshold_triggered:             # check whether the robot is close enough to the object
                nav_result = res_future.result().result
                if not nav_result.success:
                    self.get_logger().warn(f"❌ NAV failed: {nav_result.message}")
                    return False
            # reset flag for next invocation
            threshold_triggered = False
            self.get_logger().info("✅ NAV success.")
            return True

        except Exception as e:
            self.get_logger().error(f"❌ NAV error: {e}")
            return False

    def _send_task_command(self, command: str, label: str, timeout_sec: float = 300.0) -> bool:
        """Send a TaskCommand goal and wait for the result. Used by both grasp and place."""
        try:
            if not self.task_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error(f"❌ TaskCommand server not available for {label}.")
                return False

            goal = TaskCommand.Goal()
            goal.command = command

            fut = self.task_client.send_goal_async(goal, feedback_callback=self._on_task_feedback)
            start = time.time()
            while not fut.done():
                if time.time() - start > 10.0:
                    self.get_logger().error(f"⏰ {label} goal send timeout")
                    return False
                time.sleep(0.05)

            handle = fut.result()
            if not handle.accepted:
                self.get_logger().warn(f"{label} goal rejected.")
                return False

            res_future = handle.get_result_async()
            start = time.time()
            while not res_future.done():
                if time.time() - start > timeout_sec:
                    self.get_logger().warn(f"⏰ {label} timeout")
                    self._send_cancel()
                    return False
                time.sleep(0.1)

            result = res_future.result().result
            if not result.success:
                self.get_logger().warn(f"❌ {label} failed: {result.message}")
                return False

            self.get_logger().info(f"✅ {label} success: {result.message}")
            return True
        except Exception as e:
            self.get_logger().error(f"❌ {label} error: {e}")
            return False

    def _execute_grasp(self, cmd: str) -> bool:
        obj = cmd.split(':', 1)[1].strip()

        pos = self._query_object_position(obj)
        if not pos:
            self.get_logger().warn(f"⚠️ Skipping grasp — position unavailable for '{obj}'.")
            return False

        return self._send_task_command(f'grasp the {obj}', 'GRASP', timeout_sec=300.0)

    def _execute_place(self, cmd: str) -> bool:
        payload = cmd.split(':', 1)[1].strip()
        parts = [part.strip() for part in payload.split(':', 1)]
        if len(parts) == 2 and parts[0] and parts[1]:
            obj, dest = parts
            return self._send_task_command(f'place {obj} to {dest}', 'PLACE', timeout_sec=150.0)

        # Legacy primitive form: "place:<destination>".
        return self._send_task_command(f'place to {payload}', 'PLACE', timeout_sec=150.0)

    def _execute_handover(self, cmd: str) -> bool:
        obj = cmd.split(':', 1)[1].strip()
        return self._send_task_command(f'handover {obj}', 'HANDOVER', timeout_sec=300.0)

    # =============================================================
    # FEEDBACK / CANCEL / UTILITIES
    # =============================================================
    def _on_nav_feedback(self, fb): 
        dist = fb.feedback.distance_remaining
        self._nav_distance_remaining = dist
        self.get_logger().debug(f"NAV feedback: distance_remaining={dist:.2f} m")
    
    def _on_task_feedback(self, feedback_msg):
        state = feedback_msg.feedback.feedback
        self.get_logger().info(f"🤖 Task feedback: {state}")
    
    def _send_cancel(self):
        msg = String()
        msg.data = "cancel"
        self.status_pub.publish(msg)
        self.get_logger().warn("🛑 Cancel broadcast sent.")

    def on_cancel_event(self, msg: String):
        if msg.data.strip().lower() == "cancel":
            self._send_cancel()

    def create_rate(self, hz: float):
        period = 1.0 / hz
        class _Rate:
            def __init__(self, p): self.p = p
            def sleep(self): time.sleep(self.p)
        return _Rate(period)

    def destroy_node(self):
        self._shutdown.set()
        try:
            if hasattr(self, 'visualizer') and self.visualizer:
                self.visualizer.stop()
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = DecisionMakingNode()
    try:
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


# =============================================================
# 📐 3D -> 2D Map Transform Helpers
# =============================================================

def load_map3d_to_map2d(yaml_path: str):
    """Load 3D->2D transform parameters from YAML (plane_fit + sim2)."""
    d = yaml.safe_load(open(yaml_path, "r"))

    mu = np.array([d["plane_fit"]["mu"]["x"],
                   d["plane_fit"]["mu"]["y"],
                   d["plane_fit"]["mu"]["z"]], dtype=float)
    e1 = np.array(d["plane_fit"]["basis_e1"], dtype=float)
    e2 = np.array(d["plane_fit"]["basis_e2"], dtype=float)

    s = float(d["sim2"]["s"])
    R = np.array(d["sim2"]["R"], dtype=float)  # 2x2
    t = np.array([d["sim2"]["t"]["x"], d["sim2"]["t"]["y"]], dtype=float)

    return mu, e1, e2, s, R, t


def map3d_point_to_map2d_xy(p_xyz, mu, e1, e2, s, R, t):
    """Project 3D point onto 2D plane, then apply similarity transform."""
    p = np.array(p_xyz, dtype=float)
    d = p - mu
    uv = np.array([d @ e1, d @ e2], dtype=float)
    xy = s * (R @ uv) + t
    return xy  # (x, y)
