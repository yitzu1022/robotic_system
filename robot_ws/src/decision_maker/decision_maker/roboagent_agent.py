"""ROS-free RoboAgent-style scheduler and capability adapter.

The first implementation is intentionally deterministic and lightweight, but
keeps the reference RoboAgent shape: scheduler output, capability calls,
capability/action/feedback history, and feedback-aware replanning.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional

from . import roboagent_prompts as prompts


CAPABILITY_NAMES = {
    "exploration_guidance",
    "exploration_planner",
    "object_grounding",
    "scene_description",
    "manipulation_planner",
    "experience_summarization",
    "question_answering",
}

PRIMITIVE_PATTERNS = {
    "goto": re.compile(r"^goto:([^:]+)$"),
    "grasp": re.compile(r"^grasp:([^:]+)$"),
    "place": re.compile(r"^place:([^:]+):([^:]+)$"),
    "handover": re.compile(r"^handover:([^:]+)$"),
}


class RoboAgentValidationError(ValueError):
    """Raised when scheduler output cannot be safely executed."""


@dataclass
class VisualObservation:
    """Small, model-agnostic description of the latest visual observation."""

    available: bool = False
    width: int = 0
    height: int = 0
    encoding: str = ""
    stamp_sec: float = 0.0
    frame_id: str = ""

    def describe(self) -> str:
        if not self.available:
            return "no camera image is available; use text-only mock mode"
        return (
            f"image {self.width}x{self.height}, encoding={self.encoding}, "
            f"frame_id={self.frame_id}, stamp_sec={self.stamp_sec:.3f}"
        )


class RoboAgentModelClient:
    """Interface for future Qwen-VL or RoboAgent model backends."""

    def generate_scheduler_json(
        self,
        task_instruction: str,
        observation: VisualObservation,
        capability_history: List[str],
        action_history: List[str],
        feedback_history: List[str],
        replan_context: Optional[dict] = None,
    ) -> Dict[str, Any]:
        raise NotImplementedError


class MockRoboAgentModelClient(RoboAgentModelClient):
    """Deterministic scheduler used until a real VLM backend is connected."""

    def generate_scheduler_json(
        self,
        task_instruction: str,
        observation: VisualObservation,
        capability_history: List[str],
        action_history: List[str],
        feedback_history: List[str],
        replan_context: Optional[dict] = None,
    ) -> Dict[str, Any]:
        if replan_context and replan_context.get("failed_action"):
            failed_action = replan_context["failed_action"]
            return {
                "think": (
                    "The previous primitive failed. I should summarize the "
                    "experience and stop rather than send an unsafe duplicate "
                    "mock action."
                ),
                "capability_calls": [
                    {
                        "name": "experience_summarization",
                        "args": f"failure after {failed_action}",
                    }
                ],
                "primitive_actions": [],
                "stop": True,
                "stop_reason": f"mock replanner stopped after failed action: {failed_action}",
            }

        text = task_instruction.strip().lower()
        actions = _heuristic_primitives(text)
        target = _primary_target_from_primitives(actions) or _clean_phrase(text)

        calls = [
            {"name": "object_grounding", "args": target},
            {"name": "scene_description", "args": target},
            {"name": "manipulation_planner", "args": task_instruction.strip()},
        ]
        if actions and actions[0].startswith("goto:"):
            calls.insert(0, {"name": "exploration_guidance", "args": target})
            calls.insert(1, {"name": "exploration_planner", "args": target})

        return {
            "think": (
                "Use the current egocentric observation and symbolic map-backed "
                "executor primitives to attempt the user task."
            ),
            "capability_calls": calls,
            "primitive_actions": actions,
            "stop": not bool(actions),
            "stop_reason": "" if actions else "mock scheduler could not infer safe primitives",
        }


class RoboAgentDecisionAgent:
    """RoboAgent-style high-level decision agent for robotic_system."""

    def __init__(self, model_client: Optional[RoboAgentModelClient] = None):
        self.model_client = model_client or MockRoboAgentModelClient()
        self.reset()

    def reset(self, observed_objects: Optional[Iterable[str]] = None) -> None:
        self.task_instruction = ""
        self.observed_objects = sorted(observed_objects or [])
        self.capability_history: List[str] = []
        self.action_history: List[str] = []
        self.feedback_history: List[str] = []
        self.core_history = ""
        self.last_observation = VisualObservation()
        self.last_scheduler_output: Dict[str, Any] = {}

    def process_observation(self, observation: VisualObservation) -> None:
        self.last_observation = observation

    def process_task(self, task_instruction: str) -> None:
        self.task_instruction = task_instruction.strip()

    def process_feedback(self, success: bool, last_action: str, message: str = "") -> None:
        status = "success" if success else "failure"
        entry = f"[feedback {len(self.feedback_history) + 1}] {last_action}: {status}"
        if message:
            entry += f" ({message})"
        self.feedback_history.append(entry)

    def get_scheduler_result(
        self,
        task_instruction: Optional[str] = None,
        observation: Optional[VisualObservation] = None,
        replan_context: Optional[dict] = None,
    ) -> Dict[str, Any]:
        if task_instruction is not None:
            self.process_task(task_instruction)
        if observation is not None:
            self.process_observation(observation)

        raw = self.model_client.generate_scheduler_json(
            self.task_instruction,
            self.last_observation,
            self.capability_history,
            self.action_history,
            self.feedback_history,
            replan_context=replan_context,
        )
        result = validate_scheduler_output(raw)

        capability_results = []
        for call in result["capability_calls"]:
            capability_results.append(self.get_capability_result(call["name"], call["args"]))

        if capability_results:
            result["capability_results"] = capability_results

        self.last_scheduler_output = result
        return result

    def get_capability_result(self, capability_name: str, args: str) -> Dict[str, Any]:
        if capability_name not in CAPABILITY_NAMES:
            raise RoboAgentValidationError(f"Unknown capability '{capability_name}'.")

        handler = getattr(self, capability_name)
        result = handler(args)
        entry = f"{capability_name}({args}) -> {result}"
        self.capability_history.append(entry)
        self.core_history += entry + "\n"
        return {"name": capability_name, "args": args, "result": result}

    def record_action(self, action: str, success: bool) -> None:
        status = "success" if success else "failure"
        self.action_history.append(f"[action {len(self.action_history) + 1}] {action}: {status}")

    def build_scheduler_prompt(self) -> str:
        return prompts.SCHEDULER_PROMPT.format(
            self.task_instruction,
            self.last_observation.describe(),
            "\n".join(self.capability_history),
            "\n".join(self.action_history),
            "\n".join(self.feedback_history),
        )

    def exploration_guidance(self, args: str) -> str:
        target = _clean_phrase(args)
        return f"target {target}" if target else "target unknown"

    def exploration_planner(self, args: str) -> List[str]:
        target = _clean_phrase(args)
        return [f"goto:{target}"] if target else []

    def object_grounding(self, args: str) -> Dict[str, Any]:
        return {
            "found": self.last_observation.available,
            "label": _clean_phrase(args),
            "mode": "mock_visual_grounding",
            "bbox": None,
        }

    def scene_description(self, args: str) -> str:
        if not self.last_observation.available:
            return "No egocentric image is available; using text-only mock scene."
        return f"Mock scene description around {_clean_phrase(args)} from {self.last_observation.describe()}."

    def manipulation_planner(self, args: str) -> List[str]:
        return _heuristic_primitives(args)

    def experience_summarization(self, args: str) -> str:
        feedback = "; ".join(self.feedback_history[-3:]) or "no execution feedback yet"
        return f"Task context '{args}'. Recent feedback: {feedback}."

    def question_answering(self, args: str) -> str:
        if not self.last_observation.available:
            return "I cannot answer visually because no camera image is available."
        return f"Mock visual answer for question: {args}"


def validate_scheduler_output(output: Dict[str, Any]) -> Dict[str, Any]:
    if not isinstance(output, dict):
        raise RoboAgentValidationError("Scheduler output must be a dictionary.")

    think = output.get("think", "")
    if not isinstance(think, str):
        raise RoboAgentValidationError("Scheduler field 'think' must be a string.")

    capability_calls = output.get("capability_calls", [])
    if not isinstance(capability_calls, list):
        raise RoboAgentValidationError("Scheduler field 'capability_calls' must be a list.")

    normalized_calls = []
    for index, call in enumerate(capability_calls):
        if not isinstance(call, dict):
            raise RoboAgentValidationError(f"Capability call {index} must be a dictionary.")
        name = call.get("name")
        args = call.get("args", "")
        if not isinstance(name, str) or name not in CAPABILITY_NAMES:
            raise RoboAgentValidationError(f"Capability call {index} has invalid name '{name}'.")
        if not isinstance(args, str):
            raise RoboAgentValidationError(f"Capability call {index} args must be a string.")
        normalized_calls.append({"name": name, "args": args.strip()})

    primitive_actions = output.get("primitive_actions", [])
    if not isinstance(primitive_actions, list):
        raise RoboAgentValidationError("Scheduler field 'primitive_actions' must be a list.")
    normalized_primitives = verify_primitive_actions(primitive_actions)

    stop = output.get("stop", False)
    if not isinstance(stop, bool):
        raise RoboAgentValidationError("Scheduler field 'stop' must be a boolean.")

    stop_reason = output.get("stop_reason", "")
    if not isinstance(stop_reason, str):
        raise RoboAgentValidationError("Scheduler field 'stop_reason' must be a string.")

    if stop and normalized_primitives:
        raise RoboAgentValidationError("Scheduler cannot set stop=true while returning primitives.")
    if not stop and not normalized_primitives:
        raise RoboAgentValidationError("Scheduler must either return primitives or set stop=true.")

    return {
        "think": think.strip(),
        "capability_calls": normalized_calls,
        "primitive_actions": normalized_primitives,
        "stop": stop,
        "stop_reason": stop_reason.strip(),
    }


def verify_primitive_actions(actions: Iterable[Any]) -> List[str]:
    normalized = []
    for index, action in enumerate(actions):
        if not isinstance(action, str):
            raise RoboAgentValidationError(f"Primitive {index} must be a string.")
        primitive = _normalize_primitive(action)
        if not is_valid_primitive(primitive):
            raise RoboAgentValidationError(f"Invalid primitive {index}: '{action}'.")
        normalized.append(primitive)
    return normalized


def is_valid_primitive(action: str) -> bool:
    primitive = _normalize_primitive(action)
    return any(pattern.match(primitive) for pattern in PRIMITIVE_PATTERNS.values())


def _heuristic_primitives(task_instruction: str) -> List[str]:
    text = task_instruction.strip().lower()
    if not text:
        return []

    if text.startswith("go to "):
        return [f"goto:{_clean_phrase(text[6:])}"]
    if text in {"go home", "park"}:
        return ["goto:home"]
    if text.startswith("goto:"):
        try:
            return verify_primitive_actions([text])
        except RoboAgentValidationError:
            return []

    for prefix in ("give me ", "bring me "):
        if text.startswith(prefix):
            item = _clean_phrase(text[len(prefix) :])
            return [f"goto:{item}", f"grasp:{item}", "goto:me", f"handover:{item}"]

    if text.startswith("handover "):
        item = _clean_phrase(text[len("handover ") :])
        return [f"handover:{item}"]

    for prefix in ("bring ", "place "):
        if text.startswith(prefix):
            parsed = _parse_transfer(text[len(prefix) :])
            if parsed is None:
                return []
            obj, nav_target, dest = parsed
            return [f"goto:{nav_target}", f"grasp:{obj}", f"goto:{dest}", f"place:{obj}:{dest}"]

    return []


def _parse_transfer(arg: str) -> Optional[tuple[str, str, str]]:
    text = arg.strip().lower()
    text = re.sub(r"^\s*from\s+", "", text)
    if " to " in text:
        src, dest = [part.strip() for part in text.split(" to ", 1)]
    elif " into " in text:
        src, dest = [part.strip() for part in text.split(" into ", 1)]
    else:
        return None

    src = _clean_phrase(src)
    dest = _clean_phrase(dest)
    prep_match = re.search(
        r"\s+(?:on|in|at|near|by|inside|from|above|below|under|beside|next\s+to)\s+",
        src,
    )
    if prep_match:
        obj = _clean_phrase(src[: prep_match.start()])
        nav_target = _clean_phrase(src[prep_match.end() :])
    else:
        obj = src
        nav_target = src

    if not obj or not nav_target or not dest:
        return None
    return obj, nav_target, dest


def _primary_target_from_primitives(actions: List[str]) -> str:
    for action in actions:
        if ":" in action:
            parts = action.split(":")
            if len(parts) > 1 and parts[1]:
                return parts[1]
    return ""


def _normalize_primitive(action: str) -> str:
    return ":".join(part.strip().lower() for part in action.strip().split(":"))


def _clean_phrase(text: str) -> str:
    cleaned = re.sub(r"^(the|a|an)\s+", "", text.strip().lower())
    return " ".join(cleaned.split())
