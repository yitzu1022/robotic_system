import json
import threading
import time
from typing import List, Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .decision_maker_node import DecisionMakingNode
from .roboagent_agent import (
    RoboAgentDecisionAgent,
    RoboAgentValidationError,
    VisualObservation,
    verify_primitive_actions,
)


class RoboAgentDecisionMakingNode(DecisionMakingNode):
    """Decision node that wraps a RoboAgent-style scheduler around the executor."""

    def __init__(self):
        super().__init__(node_name="roboagent_decision_making_node")
        self._execution_agent: Optional[RoboAgentDecisionAgent] = None
        self._image_lock = threading.Lock()
        self._latest_image: Optional[Image] = None
        self._latest_image_time = 0.0

        self.declare_parameter("camera_topic", "/camera/color/image_raw")
        self.declare_parameter("max_replans", 1)
        self.camera_topic = self.get_parameter("camera_topic").get_parameter_value().string_value
        self.max_replans = self.get_parameter("max_replans").get_parameter_value().integer_value

        self.image_sub = self.create_subscription(Image, self.camera_topic, self.on_camera_image, 10)
        self.get_logger().info(
            "RoboAgentDecisionMakingNode ready. "
            f"camera_topic={self.camera_topic}, max_replans={self.max_replans}"
        )

    def on_camera_image(self, msg: Image):
        with self._image_lock:
            self._latest_image = msg
            self._latest_image_time = time.time()

    def on_text_event(self, msg: String):
        text = msg.data.strip()
        if not text:
            self.get_logger().warn("Received empty manual command. Ignoring.")
            return

        self.get_logger().info(f"Received RoboAgent command: '{text}'")

        def _plan_and_enqueue():
            try:
                observation = self._build_visual_observation()
                if not observation.available:
                    self.get_logger().warn(
                        "No camera image received yet; RoboAgent scheduler is using mock/text-only mode."
                    )

                agent = RoboAgentDecisionAgent()
                agent.process_task(text)
                agent.process_observation(observation)

                plan = agent.get_scheduler_result(text, observation)
                primitives = verify_primitive_actions(plan["primitive_actions"])

                self.get_logger().info(
                    "RoboAgent scheduler JSON: " + json.dumps(plan, ensure_ascii=False)
                )
                if plan["stop"]:
                    self.get_logger().warn(
                        f"RoboAgent scheduler stopped before execution: {plan['stop_reason']}"
                    )
                    return

                self.enqueue_roboagent_command(text, primitives, agent)
            except RoboAgentValidationError as exc:
                self.get_logger().error(f"Invalid RoboAgent scheduler output for '{text}': {exc}")
            except Exception as exc:
                self.get_logger().error(f"Failed to plan RoboAgent command '{text}': {exc}")

        thread = threading.Thread(target=_plan_and_enqueue, daemon=True)
        thread.start()


    def enqueue_roboagent_command(
        self,
        name: str,
        primitives: List[str],
        agent: RoboAgentDecisionAgent,
    ):
        try:
            batch = {
                "name": name,
                "actions": primitives,
                "timestamp": time.time(),
                "roboagent": agent,
            }
            self.cmd_queue.put(batch, timeout=0.2)
            self.get_logger().info(f"RoboAgent enqueued '{name}' -> {primitives}")
        except Exception as exc:
            self.get_logger().warn(f"Failed to enqueue RoboAgent command '{name}': {exc}")

    def command_executor_loop(self):
        while not self._shutdown.is_set():
            try:
                batch = self.cmd_queue.get(timeout=0.2)
            except Exception:
                continue

            name, actions = batch["name"], batch["actions"]
            self._execution_agent = batch.get("roboagent")
            self.get_logger().info(f"Executing RoboAgent batch: {name}")
            self._reset_preview_path(reason=f"batch_start:{name}")
            self._clear_preview_goal(reason=f"batch_start:{name}")

            success = self._execute_batch(name, actions)
            if success:
                self.get_logger().info(f"Finished RoboAgent batch: {name}")
                self._clear_preview_goal(reason="batch_complete")
            else:
                self.get_logger().warn(f"RoboAgent batch '{name}' failed or timed out.")
                self._clear_preview_goal(reason="batch_failed")

            self._execution_agent = None
            self.cmd_queue.task_done()

    def _build_visual_observation(self) -> VisualObservation:
        with self._image_lock:
            image = self._latest_image
            received_time = self._latest_image_time

        if image is None:
            return VisualObservation(available=False)

        stamp_sec = float(image.header.stamp.sec) + float(image.header.stamp.nanosec) * 1e-9
        if stamp_sec <= 0.0:
            stamp_sec = received_time

        return VisualObservation(
            available=True,
            width=int(image.width),
            height=int(image.height),
            encoding=image.encoding,
            stamp_sec=stamp_sec,
            frame_id=image.header.frame_id,
        )

    def _execute_batch(self, name: str, actions: List[str], timeout_sec: float = 300.0) -> bool:
        try:
            actions = verify_primitive_actions(actions)
        except RoboAgentValidationError as exc:
            self.get_logger().error(f"Refusing to execute invalid RoboAgent primitives: {exc}")
            return False

        start_time = time.time()
        current_actions = list(actions)
        replan_count = 0

        while current_actions:
            for index, action in enumerate(current_actions):
                if time.time() - start_time > timeout_sec:
                    self.get_logger().warn(f"Timeout: '{name}' exceeded {timeout_sec}s, cancelling.")
                    self._send_cancel()
                    return False

                success = self._execute_verified_primitive(action, current_actions, index)
                agent = self._execution_agent
                if agent is not None:
                    agent.record_action(action, success)
                    agent.process_feedback(success, action)

                if success:
                    continue

                if replan_count >= self.max_replans:
                    self.get_logger().warn(
                        f"RoboAgent max replans reached after failed primitive '{action}'."
                    )
                    return False

                replan_count += 1
                replanned = self._request_replan(name, action, replan_count)
                if not replanned:
                    return False
                current_actions = replanned
                break
            else:
                return True

        return True

    def _execute_verified_primitive(self, action: str, actions: List[str], index: int) -> bool:
        action = action.strip().lower()
        if action.startswith("goto:"):
            next_action = actions[index + 1].strip().lower() if index + 1 < len(actions) else ""
            self.get_logger().info(f"RoboAgent executing {action}")
            return self._execute_nav(action, for_grasp=next_action.startswith("grasp:"))
        if action.startswith("grasp:"):
            self.get_logger().info(f"RoboAgent executing {action}")
            return self._execute_grasp(action)
        if action.startswith("place:"):
            self.get_logger().info(f"RoboAgent executing {action}")
            return self._execute_place(action)
        if action.startswith("handover:"):
            self.get_logger().info(f"RoboAgent executing {action}")
            return self._execute_handover(action)

        self.get_logger().error(f"Verifier missed invalid primitive: {action}")
        return False

    def _request_replan(self, task_name: str, failed_action: str, replan_count: int) -> Optional[List[str]]:
        observation = self._build_visual_observation()
        if not observation.available:
            self.get_logger().warn("No camera image available during replanning; using text-only mode.")

        try:
            agent = self._execution_agent
            if agent is None:
                self.get_logger().error("No RoboAgent history is attached to the active batch.")
                return None

            plan = agent.get_scheduler_result(
                task_name,
                observation,
                replan_context={
                    "failed_action": failed_action,
                    "replan_count": replan_count,
                },
            )
            self.get_logger().info(
                "RoboAgent replan JSON: " + json.dumps(plan, ensure_ascii=False)
            )
            if plan["stop"]:
                self.get_logger().warn(f"RoboAgent replanner stopped: {plan['stop_reason']}")
                return None
            return verify_primitive_actions(plan["primitive_actions"])
        except RoboAgentValidationError as exc:
            self.get_logger().error(f"Invalid RoboAgent replan after '{failed_action}': {exc}")
            return None
        except Exception as exc:
            self.get_logger().error(f"RoboAgent replan failed after '{failed_action}': {exc}")
            return None


def main():
    rclpy.init()
    node = RoboAgentDecisionMakingNode()
    try:
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
