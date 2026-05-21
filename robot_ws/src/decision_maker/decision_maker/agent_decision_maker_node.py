import json
import threading
import time

import rclpy
from kachaka_interfaces.action import Navigate
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

from .decision_agent import DecisionAgent, PlanValidationError
from .decision_maker_node import DecisionMakingNode


class AgentDecisionMakingNode(DecisionMakingNode):
    """Decision maker variant that uses closed-loop agent decisions."""

    def __init__(self):
        super().__init__(node_name="agent_decision_making_node")
        self.agent = DecisionAgent()
        self.declare_parameter("mock_execution", False)
        self.mock_execution = (
            self.get_parameter("mock_execution").get_parameter_value().bool_value
        )
        self.declare_parameter("agent_max_steps", 10)
        self.agent_max_steps = (
            self.get_parameter("agent_max_steps").get_parameter_value().integer_value
        )
        mode = "mock execution" if self.mock_execution else "live execution"
        self.get_logger().info(
            f"🧠 AgentDecisionMakingNode ready with closed-loop planner ({mode})."
        )

    def on_text_event(self, msg: String):
        text = msg.data.strip()
        if not text:
            self.get_logger().warn("⚠️ Received empty manual command. Ignoring.")
            return

        self.get_logger().info(f"📥 Received agent command: '{text}'")
        thread = threading.Thread(
            target=self.run_iterative_task_loop,
            args=(text,),
            daemon=True,
        )
        thread.start()

    def run_iterative_task_loop(self, task_instruction: str) -> bool:
        current_state = {
            "task": task_instruction,
            "known_poses": {},
            "held_object": None,
            "step_index": 0,
        }
        history = []
        last_result = None

        for step_index in range(1, self.agent_max_steps + 1):
            current_state["step_index"] = step_index
            try:
                capability_call = self.agent.decide_next_capability(
                    task_instruction,
                    current_state,
                    history,
                    last_result,
                )
            except PlanValidationError as exc:
                self.get_logger().error(
                    f"❌ Invalid agent capability decision at step {step_index}: {exc}"
                )
                return False
            except Exception as exc:
                self.get_logger().error(
                    f"❌ Failed to get agent capability decision at step {step_index}: {exc}"
                )
                return False

            self.get_logger().info(
                "🧠 Agent capability call: "
                + json.dumps(capability_call, ensure_ascii=False)
            )

            if capability_call["capability"] == "finish":
                self.get_logger().info(
                    f"✅ Agent finished task '{task_instruction}': "
                    f"{capability_call.get('reason', '')}"
                )
                return True

            result = self.execute_capability_call(capability_call, current_state)
            history.append({"step": step_index, "call": capability_call, "result": result})
            last_result = result

            self.get_logger().info(
                "👁️ Capability observation: " + json.dumps(result, ensure_ascii=False)
            )

            if not result.get("success", False):
                self.get_logger().error(
                    f"🛑 Stopping task '{task_instruction}' after failed capability: "
                    f"{result.get('message', 'unknown error')}"
                )
                return False

        self.get_logger().error(
            f"⏰ Agent task '{task_instruction}' exceeded max_steps={self.agent_max_steps}."
        )
        return False

    def execute_capability_call(self, capability_call: dict, current_state: dict) -> dict:
        capability = capability_call["capability"]
        if capability == "object_query":
            return self.execute_object_query(capability_call["target"], current_state)
        if capability == "navigation":
            return self.execute_navigation(
                target=capability_call.get("target"),
                pose=capability_call.get("pose"),
                current_state=current_state,
            )
        if capability == "grasp_place":
            return self.execute_grasp_place(
                action=capability_call["action"],
                target=capability_call["target"],
                destination=capability_call.get("destination"),
                current_state=current_state,
            )
        return {
            "last_action": capability,
            "success": False,
            "message": f"Unsupported capability: {capability}",
        }

    def execute_object_query(self, target: str, current_state: dict) -> dict:
        if self.mock_execution:
            pose = self._mock_pose_for_target(target)
            current_state.setdefault("known_poses", {})[target] = pose
            self.get_logger().info(f"🧪🔎 [MOCK] object_query({target}) -> {pose}")
            return {
                "last_action": "object_query",
                "target": target,
                "success": True,
                "result": {"pose": pose},
                "message": "mock object query completed",
            }

        pose_tuple = self._query_object_position(target)
        if not pose_tuple:
            return {
                "last_action": "object_query",
                "target": target,
                "success": False,
                "message": f"Object or place '{target}' was not found.",
            }

        pose = {
            "x": float(pose_tuple[0]),
            "y": float(pose_tuple[1]),
            "theta": float(pose_tuple[2]) if len(pose_tuple) > 2 else 0.0,
        }
        current_state.setdefault("known_poses", {})[target] = pose
        return {
            "last_action": "object_query",
            "target": target,
            "success": True,
            "result": {"pose": pose},
        }

    def execute_navigation(
        self,
        target: str | None = None,
        pose: dict | None = None,
        current_state: dict | None = None,
    ) -> dict:
        if pose is None and target and current_state is not None:
            pose = current_state.get("known_poses", {}).get(target)
            if pose:
                self.get_logger().info(
                    f"📌 Using cached pose for navigation target '{target}'."
                )

        if self.mock_execution:
            result = {
                "last_action": "navigation",
                "success": True,
                "message": "mock navigation completed",
            }
            if target:
                result["target"] = target
            if pose:
                result["pose"] = pose
            self.get_logger().info(
                "🧪🧭 [MOCK] navigation -> " + json.dumps(result, ensure_ascii=False)
            )
            return result

        if pose:
            success = self._execute_nav_pose(pose)
            result = {
                "last_action": "navigation",
                "pose": pose,
                "success": bool(success),
                "message": "navigation completed" if success else "navigation failed",
            }
            if target:
                result["target"] = target
            return result

        if target:
            success = self._execute_nav(f"goto:{target}")
            return {
                "last_action": "navigation",
                "target": target,
                "success": bool(success),
                "message": "navigation completed" if success else "navigation failed",
            }

        return {
            "last_action": "navigation",
            "success": False,
            "message": "navigation requires target or pose",
        }

    def execute_grasp_place(
        self,
        action: str,
        target: str,
        destination: str | None = None,
        current_state: dict | None = None,
    ) -> dict:
        if self.mock_execution:
            if action == "grasp" and current_state is not None:
                current_state["held_object"] = target
            elif action == "place" and current_state is not None:
                current_state["held_object"] = None
            result = {
                "last_action": "grasp_place",
                "action": action,
                "target": target,
                "success": action in {"grasp", "place"},
                "message": (
                    f"mock {action} completed"
                    if action in {"grasp", "place"}
                    else f"unsupported mock action: {action}"
                ),
            }
            if destination:
                result["destination"] = destination
            self.get_logger().info(
                "🧪🦾 [MOCK] grasp_place -> " + json.dumps(result, ensure_ascii=False)
            )
            return result

        if action == "grasp":
            success = self._execute_grasp(f"grasp:{target}")
            if success and current_state is not None:
                current_state["held_object"] = target
            return {
                "last_action": "grasp_place",
                "action": "grasp",
                "target": target,
                "success": bool(success),
                "message": "grasp completed" if success else "grasp failed",
            }

        if action == "place":
            cmd = f"place:{target}:{destination}"
            success = self._execute_place(cmd)
            if success and current_state is not None:
                current_state["held_object"] = None
            return {
                "last_action": "grasp_place",
                "action": "place",
                "target": target,
                "destination": destination,
                "success": bool(success),
                "message": "place completed" if success else "place failed",
            }

        return {
            "last_action": "grasp_place",
            "action": action,
            "target": target,
            "success": False,
            "message": f"Unsupported grasp_place action: {action}",
        }

    def _mock_pose_for_target(self, target: str) -> dict:
        mock_poses = {
            "home": {"x": 0.0, "y": 0.0, "theta": 0.0},
            "me": {"x": 0.0, "y": 0.0, "theta": 0.0},
            "cabinet": {"x": 1.0, "y": 0.5, "theta": 0.0},
            "table": {"x": 2.0, "y": 0.0, "theta": 0.0},
            "sofa": {"x": 3.0, "y": 1.0, "theta": 0.0},
            "chair": {"x": 2.5, "y": -1.0, "theta": 0.0},
            "kitchen": {"x": 4.0, "y": 0.0, "theta": 0.0},
            "fridge": {"x": 4.5, "y": 0.5, "theta": 0.0},
        }
        return dict(mock_poses.get(target, {"x": 1.0, "y": 1.0, "theta": 0.0}))

    def _execute_nav_pose(self, pose: dict, timeout_sec: float = 150.0) -> bool:
        try:
            if not self.nav_client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error("❌ Nav server not available.")
                return False

            goal = Navigate.Goal()
            goal.target_x = float(pose["x"])
            goal.target_y = float(pose["y"])

            self.get_logger().info(
                f"🧭 Sending Nav Goal from pose: ({goal.target_x:.2f}, {goal.target_y:.2f})"
            )
            fut = self.nav_client.send_goal_async(
                goal,
                feedback_callback=self._on_nav_feedback,
            )
            start = time.time()
            while not fut.done():
                if time.time() - start > 10.0:
                    self.get_logger().error("⏰ NAV goal send timeout")
                    return False
                time.sleep(0.05)

            gh = fut.result()
            if not gh.accepted:
                self.get_logger().warn("⚠️ NAV goal rejected.")
                return False

            res_future = gh.get_result_async()
            start = time.time()
            while not res_future.done():
                if time.time() - start > timeout_sec:
                    self.get_logger().warn("⏰ NAV timeout")
                    self._send_cancel()
                    return False
                time.sleep(0.1)

            nav_result = res_future.result().result
            if not nav_result.success:
                self.get_logger().warn(f"❌ NAV failed: {nav_result.message}")
                return False

            self.get_logger().info("✅ NAV success.")
            return True
        except Exception as exc:
            self.get_logger().error(f"❌ NAV pose error: {exc}")
            return False

    def _build_agent_context(self) -> dict:
        return {
            "supported_capabilities": [
                "object_query",
                "navigation",
                "grasp_place",
                "finish",
            ],
            "execution_note": (
                "Choose only the next capability call. The system executes it and "
                "returns an observation before the next decision."
            ),
        }


def main():
    rclpy.init()
    node = AgentDecisionMakingNode()
    try:
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
