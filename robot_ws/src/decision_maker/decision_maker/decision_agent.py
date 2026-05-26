"""
Agent-side planning helpers for the decision_maker package.

This module is intentionally ROS-free so plan validation and conversion can be
tested without starting rclpy. Runtime execution stays in decision_maker_node.py.
"""

from __future__ import annotations

import json
import os
import re
from typing import Any, Dict, Iterable, List


SUPPORTED_ACTIONS = {"goto", "grasp", "place", "handover"}
SUPPORTED_CAPABILITIES = {"object_query", "navigation", "grasp_place", "robot_pose", "finish"}
SUPPORTED_GRASP_PLACE_ACTIONS = {"grasp", "place"}


class PlanValidationError(ValueError):
    """Raised when an agent plan or capability call is invalid."""


class QwenClient:
    """Modular Qwen client with placeholder and local Transformers backends."""

    DEFAULT_MODEL_PATH = "/home/acm/robotic_agent/models/Qwen3-VL-4B-Instruct-FP8"

    def __init__(self):
        self.backend = os.getenv("LLM_BACKEND", "placeholder").strip().lower()
        self.model_path = os.getenv("QWEN_MODEL_PATH", self.DEFAULT_MODEL_PATH)
        self.max_new_tokens = int(os.getenv("QWEN_MAX_NEW_TOKENS", "512"))
        self._model = None
        self._processor = None

    # ------------------------------------------------------------------
    # Baseline single-shot plan API, kept for comparison.
    # ------------------------------------------------------------------
    def generate_plan(self, task_instruction: str, context: dict) -> dict:
        text = task_instruction.strip().lower()
        if not text:
            raise PlanValidationError("Task instruction is empty.")

        if self.backend == "local_transformers":
            return self._generate_plan_local_transformers(task_instruction, context)

        steps = self._heuristic_steps(text)
        if not steps:
            raise PlanValidationError(
                "QwenClient placeholder could not generate a plan for this task."
            )

        return {"task": task_instruction.strip(), "steps": steps}

    # ------------------------------------------------------------------
    # Closed-loop capability decision API.
    # ------------------------------------------------------------------
    def decide_next_capability(
        self,
        original_task: str,
        current_state: dict,
        history: List[dict],
        last_execution_result: dict | None,
    ) -> dict:
        text = original_task.strip().lower()
        if not text:
            raise PlanValidationError("Original task is empty.")

        if self.backend == "local_transformers":
            return self._decide_next_capability_local_transformers(
                original_task,
                current_state,
                history,
                last_execution_result,
            )

        return self._heuristic_next_capability(
            text,
            current_state,
            history,
            last_execution_result,
        )

    def _heuristic_steps(self, text: str) -> List[dict]:
        if text.startswith("go to "):
            return [{"action": "goto", "target": _clean_phrase(text[6:])}]

        if text in {"go home", "park"}:
            return [{"action": "goto", "target": "home"}]

        if text.startswith("give me "):
            item = _clean_phrase(text[len("give me ") :])
            return [
                {"action": "goto", "target": item},
                {"action": "grasp", "target": item},
                {"action": "goto", "target": "me"},
                {"action": "handover", "target": item},
            ]

        if text.startswith("bring me "):
            item = _clean_phrase(text[len("bring me ") :])
            return [
                {"action": "goto", "target": item},
                {"action": "grasp", "target": item},
                {"action": "goto", "target": "me"},
                {"action": "handover", "target": item},
            ]

        if text.startswith("move "):
            transfers = _try_parse_transfer_sequence(text) or []
            steps = []
            for obj, nav_target, dest, item_final_action, _fallback_source in transfers:
                final_step = {"action": item_final_action, "target": obj}
                if item_final_action == "place":
                    final_step["destination"] = dest
                steps.extend([
                    {"action": "goto", "target": nav_target},
                    {"action": "grasp", "target": obj},
                    {"action": "goto", "target": dest},
                    final_step,
                ])
            return steps

        for prefix, final_action in (
            ("bring ", "place"),
            ("place ", "place"),
            ("handover ", "handover"),
        ):
            if text.startswith(prefix):
                transfers = _parse_transfer_sequence(text[len(prefix) :], final_action)
                steps = []
                for obj, nav_target, dest, item_final_action, _fallback_source in transfers:
                    final_step = {"action": item_final_action, "target": obj}
                    if item_final_action == "place":
                        final_step["destination"] = dest
                    steps.extend([
                        {"action": "goto", "target": nav_target},
                        {"action": "grasp", "target": obj},
                        {"action": "goto", "target": dest},
                        final_step,
                    ])
                return steps

        return []

    def _heuristic_next_capability(
        self,
        text: str,
        current_state: dict,
        history: List[dict],
        last_execution_result: dict | None,
    ) -> dict:
        transfer_sequence = _try_parse_transfer_sequence(text)
        if transfer_sequence is not None:
            call = _expected_transfer_sequence_capability(
                transfer_sequence,
                len(history),
                current_state,
                last_execution_result,
            )
            if call is not None:
                return call

        if text.startswith("go to "):
            target = _clean_phrase(text[len("go to ") :])
            script = [
                {
                    "capability": "object_query",
                    "target": target,
                    "reason": "Need to locate the navigation target.",
                },
                {
                    "capability": "navigation",
                    "target": target,
                    "reason": "Navigate to the requested target.",
                },
                {
                    "capability": "finish",
                    "task_done": True,
                    "reason": "The navigation task is complete.",
                },
            ]
            return self._attach_known_pose_if_available(
                script[min(len(history), len(script) - 1)],
                current_state,
                last_execution_result,
            )

        if text in {"go home", "park"}:
            script = [
                {
                    "capability": "navigation",
                    "target": "home",
                    "reason": "Navigate to the home location.",
                },
                {
                    "capability": "finish",
                    "task_done": True,
                    "reason": "The robot is parked.",
                },
            ]
            return self._attach_known_pose_if_available(
                script[min(len(history), len(script) - 1)],
                current_state,
                last_execution_result,
            )

        if _is_robot_pose_query(text):
            script = [
                {
                    "capability": "robot_pose",
                    "reason": "Need to observe the robot's current pose.",
                },
                {
                    "capability": "finish",
                    "task_done": True,
                    "reason": "The robot pose has been reported.",
                },
            ]
            return script[min(len(history), len(script) - 1)]

        raise PlanValidationError(
            "QwenClient placeholder could not decide the next capability for this task."
        )

    def _attach_known_pose_if_available(
        self,
        call: dict,
        current_state: dict,
        last_execution_result: dict | None,
    ) -> dict:
        if call.get("capability") != "navigation" or "pose" in call:
            return call

        target = call.get("target")
        if not target:
            return call

        known_pose = current_state.get("known_poses", {}).get(target)
        if known_pose:
            enriched = dict(call)
            enriched["pose"] = known_pose
            return enriched

        if (
            last_execution_result
            and last_execution_result.get("last_action") == "object_query"
            and last_execution_result.get("target") == target
            and last_execution_result.get("success")
        ):
            pose = last_execution_result.get("result", {}).get("pose")
            if pose:
                enriched = dict(call)
                enriched["pose"] = pose
                return enriched

        return call

    def _decide_next_capability_local_transformers(
        self,
        original_task: str,
        current_state: dict,
        history: List[dict],
        last_execution_result: dict | None,
    ) -> dict:
        self._ensure_local_model_loaded()
        prompt = self._build_capability_prompt(
            original_task,
            current_state,
            history,
            last_execution_result,
        )
        messages = [
            {
                "role": "system",
                "content": (
                    "You are a closed-loop robot decision agent. "
                    "Return only one valid JSON object for the next capability call. "
                    "Do not include markdown or explanations."
                ),
            },
            {"role": "user", "content": prompt},
        ]
        content = self._generate_chat_content(messages)
        return self._parse_json_response(content)

    def _generate_plan_local_transformers(self, task_instruction: str, context: dict) -> dict:
        self._ensure_local_model_loaded()
        prompt = self._build_planning_prompt(task_instruction, context)
        messages = [
            {
                "role": "system",
                "content": (
                    "You are a robot task planner. Return only valid JSON. "
                    "Do not include markdown or explanations."
                ),
            },
            {"role": "user", "content": prompt},
        ]
        content = self._generate_chat_content(messages)
        return self._parse_json_response(content)

    def _generate_chat_content(self, messages: List[dict]) -> str:
        try:
            import torch
        except ImportError as exc:
            raise PlanValidationError(
                "Local Qwen backend requires torch. Install torch in the ROS container."
            ) from exc

        try:
            chat_text = self._processor.apply_chat_template(
                messages,
                tokenize=False,
                add_generation_prompt=True,
            )
            inputs = self._processor(text=[chat_text], return_tensors="pt")
        except TypeError:
            chat_text = self._processor.apply_chat_template(
                messages,
                tokenize=False,
                add_generation_prompt=True,
            )
            inputs = self._processor(chat_text, return_tensors="pt")

        if torch.cuda.is_available():
            inputs = inputs.to("cuda")

        with torch.inference_mode():
            generated_ids = self._model.generate(
                **inputs,
                max_new_tokens=self.max_new_tokens,
                do_sample=False,
            )

        input_len = inputs["input_ids"].shape[1]
        generated_ids = generated_ids[:, input_len:]
        return self._processor.batch_decode(
            generated_ids,
            skip_special_tokens=True,
            clean_up_tokenization_spaces=False,
        )[0]

    def _ensure_local_model_loaded(self) -> None:
        if self._model is not None and self._processor is not None:
            return

        if not os.path.isdir(self.model_path):
            raise PlanValidationError(f"Local Qwen model path does not exist: {self.model_path}")

        try:
            import torch
            from transformers import AutoProcessor
        except ImportError as exc:
            raise PlanValidationError(
                "Local Qwen backend requires transformers and torch. "
                "Install them in the ROS container before using LLM_BACKEND=local_transformers."
            ) from exc

        self._disable_deepgemm_hub_probe()

        try:
            from transformers import AutoModelForImageTextToText as ModelClass
        except ImportError:
            try:
                from transformers import AutoModelForVision2Seq as ModelClass
            except ImportError as exc:
                raise PlanValidationError(
                    "Installed transformers is too old for Qwen3-VL. "
                    "Upgrade transformers before loading the local model."
                ) from exc

        dtype = torch.bfloat16 if torch.cuda.is_available() else torch.float32
        self._processor = AutoProcessor.from_pretrained(
            self.model_path,
            trust_remote_code=True,
        )
        self._model = ModelClass.from_pretrained(
            self.model_path,
            dtype=dtype,
            device_map="auto" if torch.cuda.is_available() else None,
            trust_remote_code=True,
        )
        self._model.eval()

    def _disable_deepgemm_hub_probe(self) -> None:
        """Skip DeepGEMM Hub probing on non-Hopper devices.

        Jetson-class GPUs cannot use DeepGEMM, and the probe may hit Hugging
        Face rate limits. Raising ImportError here makes Transformers use the
        Triton finegrained-FP8 fallback immediately.
        """
        try:
            import transformers.integrations.finegrained_fp8 as finegrained_fp8
        except Exception:
            return

        def _unavailable_deepgemm():
            raise ImportError(
                "DeepGEMM disabled for this robot runtime; use Triton finegrained-fp8 fallback."
            )

        finegrained_fp8._load_deepgemm_kernel = _unavailable_deepgemm

    def _build_planning_prompt(self, task_instruction: str, context: dict) -> str:
        schema = {
            "task": "bring apple to table",
            "steps": [
                {"action": "goto", "target": "apple"},
                {"action": "grasp", "target": "apple"},
                {"action": "goto", "target": "table"},
                {"action": "place", "target": "apple", "destination": "table"},
            ],
        }
        payload = {
            "task_instruction": task_instruction,
            "context": context,
            "allowed_actions": sorted(SUPPORTED_ACTIONS),
            "required_json_schema_example": schema,
            "rules": [
                "Use only goto, grasp, place, and handover actions.",
                "Every step must include action and target.",
                "place steps must include destination.",
                "Use symbolic names such as apple or table, not coordinates.",
                "For '<object> on/in/at/from <source> to <destination>', goto the source, grasp only the object, then goto the destination.",
                "For multiple objects joined by and/commas, decompose into one transfer per object; never use the combined list as a target.",
                "Never use combined phrases such as 'pringle on cabinet' as a target.",
                "Return exactly one JSON object and no extra text.",
            ],
        }
        return json.dumps(payload, ensure_ascii=False)

    def _build_capability_prompt(
        self,
        original_task: str,
        current_state: dict,
        history: List[dict],
        last_execution_result: dict | None,
    ) -> str:
        payload = {
            "original_task": original_task,
            "current_state": current_state,
            "history": history,
            "last_execution_result": last_execution_result,
            "allowed_capabilities": sorted(SUPPORTED_CAPABILITIES),
            "rules": [
                "Choose exactly one next capability call.",
                "Do not output a full multi-step plan.",
                "object_query requires target.",
                "navigation requires target or pose.",
                "grasp_place action must be grasp or place.",
                "grasp requires target.",
                "place requires target and destination.",
                "robot_pose returns the robot current pose and requires no target.",
                "finish requires task_done true and should only be used after the task is complete.",
                "For 'move <object> from <source> to <destination>', first try object_query on the object. If the object lookup fails, query the source and navigate there.",
                "For '<object> on/in/at/from <source> to <destination>', first try object_query on the object. If the object lookup fails, query the source and navigate there.",
                "If a capability failed, use last_execution_result to select a corrected next call instead of stopping.",
                "For multiple objects joined by and/commas, move one object at a time through source, grasp, destination, and place.",
                "Return exactly one JSON object and no extra text.",
            ],
            "examples": [
                {
                    "state": "need source location",
                    "output": {
                        "capability": "object_query",
                        "target": "table",
                        "reason": "Need to locate the source before navigation.",
                    },
                },
                {
                    "state": "arrived at source",
                    "output": {
                        "capability": "grasp_place",
                        "action": "grasp",
                        "target": "pringles",
                        "reason": "After reaching the source, grasp the target object.",
                    },
                },
            ],
        }
        return json.dumps(payload, ensure_ascii=False)

    def _parse_json_response(self, content: str) -> dict:
        text = self._strip_json_fence(content)
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            start = text.find("{")
            end = text.rfind("}")
            if start >= 0 and end > start:
                try:
                    return json.loads(text[start : end + 1])
                except json.JSONDecodeError:
                    pass
            raise PlanValidationError(f"Local Qwen returned non-JSON content: {content!r}")

    def _strip_json_fence(self, text: str) -> str:
        stripped = text.strip()
        if stripped.startswith("```json"):
            stripped = stripped[len("```json") :].strip()
        elif stripped.startswith("```"):
            stripped = stripped[len("```") :].strip()
        if stripped.endswith("```"):
            stripped = stripped[:-3].strip()
        return stripped


class DecisionAgent:
    """Wrapper around an LLM client for single-shot and iterative decisions."""

    def __init__(self, llm_client: QwenClient | None = None):
        self.llm_client = llm_client or QwenClient()

    def plan(self, task_instruction: str, context: dict | None = None) -> dict:
        raw_plan = self.llm_client.generate_plan(task_instruction, context or {})
        validated = validate_plan(raw_plan)
        normalized = normalize_plan(task_instruction, validated)
        return validate_plan(normalized)

    def decide_next_capability(
        self,
        original_task: str,
        current_state: dict,
        history: List[dict],
        last_execution_result: dict | None,
    ) -> dict:
        raw_call = self.llm_client.decide_next_capability(
            original_task,
            current_state,
            history,
            last_execution_result,
        )
        try:
            validated = validate_capability_call(raw_call)
        except PlanValidationError as first_error:
            # Local LLMs sometimes choose the right high-level capability but omit
            # a required field, most commonly task_done=true for finish. For
            # structured transfer tasks, recover from the task/history-derived
            # expected next capability before failing the command.
            try:
                normalized = normalize_capability_call(
                    original_task,
                    current_state,
                    history,
                    last_execution_result,
                    raw_call if isinstance(raw_call, dict) else {},
                )
                return validate_capability_call(normalized)
            except PlanValidationError:
                raise first_error

        normalized = normalize_capability_call(
            original_task,
            current_state,
            history,
            last_execution_result,
            validated,
        )
        return validate_capability_call(normalized)


def validate_capability_call(call: Dict[str, Any]) -> Dict[str, Any]:
    """Validate and normalize a single closed-loop capability call."""
    if not isinstance(call, dict):
        raise PlanValidationError("Capability call must be a dictionary.")

    capability = _required_top_level_string(call, "capability")
    if capability not in SUPPORTED_CAPABILITIES:
        raise PlanValidationError(f"Unsupported capability '{capability}'.")

    normalized = {"capability": capability}
    reason = call.get("reason")
    if isinstance(reason, str) and reason.strip():
        normalized["reason"] = reason.strip()

    if capability == "object_query":
        normalized["target"] = _required_top_level_string(call, "target")
    elif capability == "navigation":
        target = call.get("target")
        pose = call.get("pose")
        if isinstance(target, str) and target.strip():
            normalized["target"] = target.strip().lower()
        if isinstance(pose, dict):
            normalized["pose"] = _normalize_pose(pose)
        if "target" not in normalized and "pose" not in normalized:
            raise PlanValidationError("navigation requires target or pose.")
    elif capability == "grasp_place":
        action = _required_top_level_string(call, "action")
        if action not in SUPPORTED_GRASP_PLACE_ACTIONS:
            raise PlanValidationError("grasp_place action must be 'grasp' or 'place'.")
        normalized["action"] = action
        normalized["target"] = _required_top_level_string(call, "target")
        if action == "place":
            normalized["destination"] = _required_top_level_string(call, "destination")
    elif capability == "robot_pose":
        frame = call.get("frame")
        if isinstance(frame, str) and frame.strip():
            normalized["frame"] = frame.strip().lower()
    elif capability == "finish":
        if call.get("task_done") is not True:
            raise PlanValidationError("finish requires task_done=true.")
        normalized["task_done"] = True

    return normalized


def normalize_capability_call(
    original_task: str,
    current_state: dict,
    history: List[dict],
    last_execution_result: dict | None,
    call: Dict[str, Any],
) -> Dict[str, Any]:
    """Ground next capability calls for common transfer commands."""
    transfer_sequence = _try_parse_transfer_sequence(original_task)
    if transfer_sequence is None:
        return call

    # Multi-object transfer decomposition is intentionally left to the LLM.
    # The harness feeds PLANNING.md/SKILLS.md/OUTPUT_FORMAT.md at task start so
    # the model can reason through each object. We keep deterministic grounding
    # only for single-object transfer safety and for the placeholder backend.
    if _is_multi_object_transfer_sequence(transfer_sequence):
        return call

    expected = _expected_transfer_sequence_capability(
        transfer_sequence,
        len(history),
        current_state,
        last_execution_result,
    )
    if expected is None:
        return call

    if call.get("capability") == expected.get("capability") and call.get("reason"):
        expected = dict(expected)
        expected["reason"] = call["reason"]
    return expected


def _expected_transfer_capability(
    obj: str,
    nav_target: str,
    dest: str,
    final_action: str,
    step_index: int,
    current_state: dict,
    last_execution_result: dict | None,
) -> dict | None:
    return _expected_transfer_sequence_capability(
        [(obj, nav_target, dest, final_action, None)],
        step_index,
        current_state,
        last_execution_result,
    )


def _expected_transfer_sequence_capability(
    transfers: List[tuple[str, str, str, str, str | None]],
    step_index: int,
    current_state: dict,
    last_execution_result: dict | None,
) -> dict | None:
    script = []
    for obj, primary_target, dest, final_action, fallback_source in transfers:
        source_target = _resolve_source_target(
            obj,
            primary_target,
            fallback_source,
            current_state,
            last_execution_result,
        )
        script.extend([
            {"capability": "object_query", "target": source_target, "reason": f"Need to locate the source for {obj}."},
            {"capability": "navigation", "target": source_target, "reason": f"Navigate to the source for {obj}."},
            {"capability": "grasp_place", "action": "grasp", "target": obj, "reason": f"Grasp {obj} at the source."},
            {"capability": "object_query", "target": dest, "reason": f"Need to locate the destination before placing {obj}."},
            {"capability": "navigation", "target": dest, "reason": f"Navigate to the destination for {obj}."},
        ])
        if final_action == "place":
            script.append(
                {"capability": "grasp_place", "action": "place", "target": obj, "destination": dest, "reason": f"Place {obj} at the destination."}
            )
        elif final_action == "handover":
            script.append(
                {"capability": "grasp_place", "action": "place", "target": obj, "destination": dest, "reason": f"Hand over {obj} at the destination."}
            )
    script.append({"capability": "finish", "task_done": True, "reason": "The transfer task is complete."})

    call = dict(script[min(step_index, len(script) - 1)])
    if call.get("capability") == "navigation":
        pose = current_state.get("known_poses", {}).get(call.get("target"))
        if not pose and last_execution_result:
            if (
                last_execution_result.get("last_action") == "object_query"
                and last_execution_result.get("target") == call.get("target")
                and last_execution_result.get("success")
            ):
                pose = last_execution_result.get("result", {}).get("pose")
        if pose:
            call["pose"] = pose
    return call


def _resolve_source_target(
    obj: str,
    primary_target: str,
    fallback_source: str | None,
    current_state: dict,
    last_execution_result: dict | None,
) -> str:
    known_poses = current_state.get("known_poses", {})
    if primary_target in known_poses:
        return primary_target

    if fallback_source:
        if fallback_source in known_poses:
            return fallback_source
        if last_execution_result and last_execution_result.get("last_action") == "object_query":
            last_target = last_execution_result.get("target")
            if last_target == primary_target and last_execution_result.get("success") is False:
                return fallback_source
            if last_target == fallback_source and last_execution_result.get("success"):
                return fallback_source

    return primary_target


def normalize_plan(task_instruction: str, plan: Dict[str, Any]) -> Dict[str, Any]:
    """Apply deterministic safety normalization to an LLM-produced plan."""
    transfer_sequence = _try_parse_transfer_sequence(task_instruction)
    if transfer_sequence is None:
        return plan

    # Let the LLM own multi-object decomposition in single-shot comparisons.
    # Placeholder plans are already decomposed before this normalization step.
    if _is_multi_object_transfer_sequence(transfer_sequence):
        return plan

    steps = []
    for obj, nav_target, dest, final_action, _fallback_source in transfer_sequence:
        final_step = {"action": final_action, "target": obj}
        if final_action == "place":
            final_step["destination"] = dest
        steps.extend([
            {"action": "goto", "target": nav_target},
            {"action": "grasp", "target": obj},
            {"action": "goto", "target": dest},
            final_step,
        ])

    return {
        "task": plan.get("task") or task_instruction.strip(),
        "steps": steps,
    }


def validate_plan(plan: Dict[str, Any]) -> Dict[str, Any]:
    """Validate and normalize a structured JSON-like task plan."""
    if not isinstance(plan, dict):
        raise PlanValidationError("Plan must be a dictionary.")

    task = plan.get("task")
    if not isinstance(task, str) or not task.strip():
        raise PlanValidationError("Plan must include a non-empty string 'task'.")

    steps = plan.get("steps")
    if not isinstance(steps, list) or not steps:
        raise PlanValidationError("Plan must include a non-empty list 'steps'.")

    normalized_steps = []
    for index, step in enumerate(steps):
        if not isinstance(step, dict):
            raise PlanValidationError(f"Step {index} must be a dictionary.")

        action = _required_string(step, "action", index).lower()
        if action not in SUPPORTED_ACTIONS:
            raise PlanValidationError(f"Step {index} has unsupported action '{action}'.")

        target = _required_string(step, "target", index)
        normalized = {"action": action, "target": target}

        if action == "place":
            normalized["destination"] = _required_string(step, "destination", index)
        elif "destination" in step and isinstance(step["destination"], str):
            destination = step["destination"].strip().lower()
            if destination:
                normalized["destination"] = destination

        normalized_steps.append(normalized)

    return {"task": task.strip(), "steps": normalized_steps}


def plan_to_primitives(plan: Dict[str, Any]) -> List[str]:
    """Convert a validated plan to legacy decision_maker primitive strings."""
    validated = validate_plan(plan)
    primitives = []

    for step in validated["steps"]:
        action = step["action"]
        target = step["target"]
        if action == "goto":
            primitives.append(f"goto:{target}")
        elif action == "grasp":
            primitives.append(f"grasp:{target}")
        elif action == "place":
            primitives.append(f"place:{target}:{step['destination']}")
        elif action == "handover":
            primitives.append(f"handover:{target}")
        else:
            raise PlanValidationError(f"Unsupported action '{action}'.")

    return primitives


def _required_top_level_string(data: Dict[str, Any], key: str) -> str:
    value = data.get(key)
    if not isinstance(value, str) or not value.strip():
        raise PlanValidationError(f"Capability call must include a non-empty '{key}'.")
    return value.strip().lower()


def _normalize_pose(pose: Dict[str, Any]) -> Dict[str, float]:
    normalized = {}
    for key in ("x", "y", "z", "theta"):
        if key in pose:
            try:
                normalized[key] = float(pose[key])
            except (TypeError, ValueError) as exc:
                raise PlanValidationError(f"pose.{key} must be numeric.") from exc
    if "x" not in normalized or "y" not in normalized:
        raise PlanValidationError("pose requires at least numeric x and y.")
    return normalized


def _required_string(step: Dict[str, Any], key: str, index: int) -> str:
    value = step.get(key)
    if not isinstance(value, str) or not value.strip():
        raise PlanValidationError(f"Step {index} must include a non-empty '{key}'.")
    return value.strip().lower()


_PREPOSITIONS = r"(?:on|in|at|near|by|inside|from|above|below|under|beside|next\s+to)"


def _is_robot_pose_query(text: str) -> bool:
    normalized = text.strip().lower()
    english_patterns = (
        "where are you",
        "where is the robot",
        "where is robot",
        "current robot pose",
        "current robot position",
        "robot current pose",
        "robot current position",
        "get robot pose",
        "get current pose",
    )
    if any(pattern in normalized for pattern in english_patterns):
        return True

    chinese_patterns = (
        "現在位置",
        "目前位置",
        "機器人位置",
        "機器人在哪",
        "你在哪",
    )
    return any(pattern in normalized for pattern in chinese_patterns)


def _is_multi_object_transfer_sequence(transfers: List[tuple[str, str, str, str, str | None]]) -> bool:
    return len(transfers) > 1


def _try_parse_transfer_command(task_instruction: str) -> tuple[str, str, str, str, str | None] | None:
    sequence = _try_parse_transfer_sequence(task_instruction)
    if not sequence:
        return None
    return sequence[0]


def _try_parse_transfer_sequence(task_instruction: str) -> List[tuple[str, str, str, str, str | None]] | None:
    text = task_instruction.strip().lower()
    if text.startswith("move "):
        match = re.match(r"^move\s+(.+?)\s+from\s+(.+?)\s+to\s+(.+)$", text)
        if not match:
            return None
        object_phrase, source, destination = match.groups()
        destination = _clean_phrase(re.split(r"\s+" + _PREPOSITIONS + r"\s+", destination)[0])
        source = _clean_phrase(source)
        objects = _split_object_list(object_phrase)
        if not objects or not source or not destination:
            return None
        return [(obj, obj, destination, "place", source) for obj in objects]

    for prefix, final_action in (
        ("bring ", "place"),
        ("place ", "place"),
        ("handover ", "handover"),
    ):
        if text.startswith(prefix):
            try:
                return _parse_transfer_sequence(text[len(prefix) :], final_action)
            except PlanValidationError:
                return None
    return None


def _parse_transfer(arg: str) -> tuple[str, str, str]:
    first = _parse_transfer_sequence(arg, "place")[0]
    return first[:3]


def _parse_transfer_sequence(arg: str, final_action: str) -> List[tuple[str, str, str, str, str | None]]:
    text = arg.strip().lower()
    text = re.sub(r"^\s*from\s+", "", text)

    if " to " in text:
        src, dest = [part.strip() for part in text.split(" to ", 1)]
    elif " into " in text:
        src, dest = [part.strip() for part in text.split(" into ", 1)]
    else:
        parts = text.split()
        if len(parts) < 2:
            raise PlanValidationError(f"Cannot parse transfer task: '{arg}'.")
        src, dest = parts[0], parts[1]

    src = _clean_phrase(src)
    dest = _clean_phrase(re.split(r"\s+" + _PREPOSITIONS + r"\s+", dest)[0])

    prep_match = re.search(r"\s+" + _PREPOSITIONS + r"\s+", src)
    if prep_match:
        obj_phrase = _clean_phrase(src[: prep_match.start()])
        shared_nav_target = _clean_phrase(src[prep_match.end() :])
    else:
        obj_phrase = src
        shared_nav_target = None

    objects = _split_object_list(obj_phrase)
    if not objects or not dest:
        raise PlanValidationError(f"Cannot parse transfer task: '{arg}'.")

    transfers = []
    for obj in objects:
        primary_target = obj
        fallback_source = shared_nav_target
        if not obj or not primary_target:
            raise PlanValidationError(f"Cannot parse transfer task: '{arg}'.")
        transfers.append((obj, primary_target, dest, final_action, fallback_source))
    return transfers


def _split_object_list(text: str) -> List[str]:
    normalized = re.sub(r"\s*,\s*and\s+", ", ", text.strip().lower())
    parts = re.split(r"\s*(?:,|\band\b)\s*", normalized)
    return [_clean_phrase(part) for part in parts if _clean_phrase(part)]


def _clean_phrase(text: str) -> str:
    cleaned = re.sub(r"^(the|a|an)\s+", "", text.strip().lower())
    return " ".join(cleaned.split())


def primitives_from_steps(steps: Iterable[Dict[str, Any]]) -> List[str]:
    """Convenience helper for tests and future callers that only have steps."""
    return plan_to_primitives({"task": "ad hoc", "steps": list(steps)})
