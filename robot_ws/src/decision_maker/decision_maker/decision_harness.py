"""Lightweight feedforward/feedback harness for agent planning.

The harness is intentionally ROS-free. It does not execute robot capabilities;
it only packages planning rules, verifies high-level calls, and records execution
observations so the closed-loop agent can replan with better context.
"""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any, Dict, List, Tuple


class LightweightPlanningHarness:
    """Small harness layer around closed-loop capability planning."""

    RULE_DOCUMENT_NAMES = (
        "DECOMPOSITION.md",
        "DECOMPOSITION_FORMAT.md",
        "PLANNING.md",
        "SKILLS.md",
        "OUTPUT_FORMAT.md",
        "VERIFICATION.md",
        "REPLANNING.md",
        "MEMORY.md",
    )

    def __init__(
        self,
        max_replans_per_task: int = 1,
        memory_limit: int = 20,
        rules_dir: str | None = None,
    ):
        self.max_replans_per_task = max(0, int(max_replans_per_task))
        self.memory_limit = max(1, int(memory_limit))
        self.rules_dir = Path(rules_dir) if rules_dir else Path(__file__).with_name("harness_rules")
        self.rule_documents = self._load_rule_documents()

    def _load_rule_documents(self) -> Dict[str, str]:
        documents = {}
        for name in self.RULE_DOCUMENT_NAMES:
            path = self.rules_dir / name
            try:
                documents[name] = path.read_text(encoding="utf-8")
            except OSError:
                documents[name] = ""
        return documents

    def build_initial_state(
        self,
        task_instruction: str,
        supported_capabilities: List[str],
        mock_execution: bool,
        extra_context: Dict[str, Any] | None = None,
    ) -> Dict[str, Any]:
        state = {
            "task": task_instruction,
            "known_poses": {},
            "held_object": None,
            "robot_pose": None,
            "step_index": 0,
            "harness": {
                "mode": "mock" if mock_execution else "live",
                "feedforward": self.build_feedforward_context(
                    supported_capabilities,
                    extra_context or {},
                ),
                "feedback": {
                    "execution_memory": [],
                    "last_failure": None,
                    "replans_used": 0,
                    "max_replans": self.max_replans_per_task,
                },
            },
        }
        return state

    def build_feedforward_context(
        self,
        supported_capabilities: List[str],
        extra_context: Dict[str, Any],
    ) -> Dict[str, Any]:
        return {
            "planning_rules": [
                "Choose exactly one next capability call per decision.",
                "Do not output a full multi-step plan in closed-loop mode.",
                "Use observations from previous capability calls before deciding the next call.",
                "For bring/place tasks with an explicit source, locate and navigate to the source before grasping the object.",
                "For move tasks with 'from <source>', first query the object; if that fails, query and navigate to the source.",
                "After grasping, locate and navigate to the destination before placing.",
                "Use finish only after the task goal has been achieved.",
            ],
            "skill_constraints": {
                "object_query": "Requires target. Returns a symbolic target pose if found.",
                "navigation": "Requires target or pose. Prefer a pose returned by object_query when available.",
                "grasp_place": "Requires action grasp/place. Grasp requires target. Place requires target and destination.",
                "robot_pose": "Requires no target. Returns the robot current pose from TF or mock state.",
                "finish": "Requires task_done=true and should only be used when the task is complete.",
            },
            "output_format": {
                "allowed_capabilities": list(supported_capabilities),
                "one_json_object_only": True,
                "examples": [
                    {"capability": "object_query", "target": "table", "reason": "Need source pose."},
                    {"capability": "navigation", "target": "table", "pose": {"x": 1.0, "y": 2.0, "theta": 0.0}, "reason": "Navigate to source."},
                    {"capability": "grasp_place", "action": "grasp", "target": "pringles", "reason": "Grasp target object."},
                    {"capability": "robot_pose", "reason": "Check current robot pose."},
                    {"capability": "finish", "task_done": True, "reason": "Task completed."},
                ],
            },
            "failure_policy": [
                "If a capability fails, inspect last_execution_result before choosing the next call.",
                "For move tasks, an object_query failure on the object can be corrected by querying the declared source.",
                "Do not immediately finish after a failed capability.",
                "Avoid repeating an identical failed call unless new information is available.",
            ],
            "available_rule_documents": list(self.RULE_DOCUMENT_NAMES),
            "task_start_rule_documents": ["PLANNING.md", "SKILLS.md", "OUTPUT_FORMAT.md"],
            "decision_rule_documents": ["MEMORY.md", "OUTPUT_FORMAT.md"],
            "verification_rule_documents": ["VERIFICATION.md"],
            "replanning_rule_documents": ["REPLANNING.md"],
            "extra_context": extra_context,
        }

    def build_decomposition_context(
        self,
        task_instruction: str,
        supported_capabilities: List[str],
        mock_execution: bool,
        extra_context: Dict[str, Any] | None = None,
    ) -> Dict[str, Any]:
        return {
            "stage": "task_decomposition",
            "original_task": task_instruction,
            "mode": "mock" if mock_execution else "live",
            "rule_documents": self._select_rule_documents([
                "DECOMPOSITION.md",
                "DECOMPOSITION_FORMAT.md",
                "SKILLS.md",
            ]),
            "decomposition_goal": (
                "Split the original command into ordered atomic subtasks before "
                "each subtask enters the closed-loop agent planner. Do not emit "
                "capability calls at this stage."
            ),
            "decomposition_constraints": [
                "Each subtask should be executable by the existing agent flow.",
                "Multiple objects should become multiple ordered subtasks.",
                "Sequential clauses such as then/and then/after that should preserve order.",
                "Move tasks should preserve source and destination.",
                "Simple conditional tasks should keep the condition in the subtask text.",
            ],
            "supported_capabilities_after_decomposition": list(supported_capabilities),
            "extra_context": extra_context or {},
        }

    def verify_decomposition(
        self,
        original_task: str,
        decomposition: Dict[str, Any],
    ) -> Tuple[bool, str]:
        subtasks = decomposition.get("subtasks") if isinstance(decomposition, dict) else None
        if not isinstance(subtasks, list) or not subtasks:
            return False, "decomposition must contain at least one subtask."

        for expected_id, subtask in enumerate(subtasks, start=1):
            if not isinstance(subtask, dict):
                return False, f"subtask {expected_id} is not a dictionary."
            if subtask.get("subtask_id") != expected_id:
                return False, "subtask_id values must be sequential starting from 1."
            text = subtask.get("text")
            if not isinstance(text, str) or not text.strip():
                return False, f"subtask {expected_id} must contain non-empty text."

        if _looks_like_multi_object_task(original_task) and len(subtasks) == 1:
            subtask = subtasks[0]
            text = str(subtask.get("text", "")).lower()
            obj = str(subtask.get("object", "")).lower()
            if " and " in text or " and " in obj:
                return False, "multi-object command was not decomposed into separate subtasks."

        return True, "decomposition verified"

    def prepare_decision_state(
        self,
        current_state: Dict[str, Any],
        history: List[dict],
        last_execution_result: dict | None,
    ) -> Dict[str, Any]:
        decision_state = copy.deepcopy(current_state)
        harness = decision_state.setdefault("harness", {})
        feedback = harness.setdefault("feedback", {})
        recent_history = history[-5:]
        feedback["recent_history"] = recent_history
        feedback["last_execution_result"] = last_execution_result

        stage = "decision"
        document_names = ["MEMORY.md", "OUTPUT_FORMAT.md"]
        if not history and last_execution_result is None:
            stage = "task_start"
            document_names = ["PLANNING.md", "SKILLS.md", "OUTPUT_FORMAT.md", "MEMORY.md"]
        elif last_execution_result and last_execution_result.get("success") is False:
            if last_execution_result.get("last_action") == "harness_verification":
                stage = "verification_replan"
                document_names = ["MEMORY.md", "OUTPUT_FORMAT.md", "VERIFICATION.md", "REPLANNING.md"]
            else:
                stage = "execution_replan"
                document_names = ["MEMORY.md", "OUTPUT_FORMAT.md", "REPLANNING.md"]

        harness["active_context"] = {
            "stage": stage,
            "rule_documents": self._select_rule_documents(document_names),
            "recent_memory": recent_history,
            "last_execution_result": last_execution_result,
        }
        return decision_state

    def build_verification_context(
        self,
        capability_call: Dict[str, Any],
        current_state: Dict[str, Any],
        history: List[dict],
    ) -> Dict[str, Any]:
        return {
            "stage": "verification",
            "rule_documents": self._select_rule_documents(["VERIFICATION.md"]),
            "candidate_call": capability_call,
            "current_state_summary": {
                "held_object": current_state.get("held_object"),
                "known_pose_targets": sorted(current_state.get("known_poses", {}).keys()),
                "robot_pose_available": bool(current_state.get("robot_pose")),
            },
            "recent_history": history[-5:],
        }

    def _select_rule_documents(self, document_names: List[str]) -> Dict[str, str]:
        return {name: self.rule_documents.get(name, "") for name in document_names}

    def verify_capability_call(
        self,
        capability_call: Dict[str, Any],
        current_state: Dict[str, Any],
        history: List[dict],
    ) -> Tuple[bool, str]:
        capability = capability_call.get("capability")
        if capability == "finish":
            if history and history[-1].get("result", {}).get("success") is False:
                return False, "finish is not allowed immediately after a failed capability."
            return True, "finish verified"

        if history:
            previous = history[-1]
            if (
                previous.get("result", {}).get("success") is False
                and previous.get("call") == capability_call
            ):
                return False, "identical failed capability call was repeated without new information."

        if capability == "navigation" and "pose" not in capability_call:
            target = capability_call.get("target")
            if target and target not in current_state.get("known_poses", {}):
                return True, "navigation target has no cached pose; executor may perform live lookup."

        if capability == "grasp_place" and capability_call.get("action") == "place":
            held_object = current_state.get("held_object")
            target = capability_call.get("target")
            if held_object and target and held_object != target:
                return False, f"cannot place {target}; current held_object is {held_object}."

        return True, "capability call verified"

    def record_observation(
        self,
        current_state: Dict[str, Any],
        capability_call: Dict[str, Any],
        result: Dict[str, Any],
    ) -> None:
        harness = current_state.setdefault("harness", {})
        feedback = harness.setdefault("feedback", {})
        memory = feedback.setdefault("execution_memory", [])
        memory.append({"call": capability_call, "result": result})
        if len(memory) > self.memory_limit:
            del memory[:-self.memory_limit]

        if result.get("success"):
            feedback["last_failure"] = None
        else:
            feedback["last_failure"] = {"call": capability_call, "result": result}

    def can_replan_after_failure(self, current_state: Dict[str, Any]) -> bool:
        feedback = current_state.get("harness", {}).get("feedback", {})
        return int(feedback.get("replans_used", 0)) < self.max_replans_per_task

    def mark_replan_used(self, current_state: Dict[str, Any]) -> int:
        feedback = current_state.setdefault("harness", {}).setdefault("feedback", {})
        replans_used = int(feedback.get("replans_used", 0)) + 1
        feedback["replans_used"] = replans_used
        return replans_used


def _looks_like_multi_object_task(text: str) -> bool:
    normalized = f" {text.strip().lower()} "
    return " and " in normalized or "," in normalized
