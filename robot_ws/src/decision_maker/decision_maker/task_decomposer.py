"""Task decomposition helpers for agent-based decision making.

This module is ROS-free. It decomposes a complex user command into simpler
atomic subtasks before each subtask is handed to the existing closed-loop agent
planner/executor.
"""

from __future__ import annotations

import json
import re
from typing import Any, Dict, List

from .decision_agent import PlanValidationError, QwenClient


SUPPORTED_SUBTASK_TYPES = {"bring", "move", "conditional", "query", "other"}


class TaskDecompositionError(ValueError):
    """Raised when task decomposition output is invalid."""


class DecomposerClient:
    """Modular decomposer backend with local-Qwen and deterministic fallback."""

    def __init__(self, llm_client: QwenClient | None = None):
        self.llm_client = llm_client or QwenClient()

    def decompose_task(self, task_instruction: str, context: dict | None = None) -> dict:
        text = task_instruction.strip()
        if not text:
            raise TaskDecompositionError("Task instruction is empty.")

        if self.llm_client.backend == "local_transformers":
            return self._decompose_task_local_transformers(text, context or {})

        return self._decompose_task_placeholder(text)

    def _decompose_task_local_transformers(self, task_instruction: str, context: dict) -> dict:
        # Reuse the existing QwenClient loading/generation path to avoid adding a
        # second model lifecycle. These are private helpers, but keeping them here
        # avoids touching the runtime model setup.
        self.llm_client._ensure_local_model_loaded()
        prompt = self._build_decomposition_prompt(task_instruction, context)
        messages = [
            {
                "role": "system",
                "content": (
                    "You are a robot task decomposer. Return only one valid JSON "
                    "object. Do not include markdown or explanations."
                ),
            },
            {"role": "user", "content": prompt},
        ]
        content = self.llm_client._generate_chat_content(messages)
        return self.llm_client._parse_json_response(content)

    def _build_decomposition_prompt(self, task_instruction: str, context: dict) -> str:
        payload = {
            "task_instruction": task_instruction,
            "context": context,
            "goal": "Decompose the command into atomic robot subtasks.",
            "active_rule_documents": context.get("rule_documents", {}),
            "output_schema": {
                "original_task": task_instruction,
                "subtasks": [
                    {
                        "subtask_id": 1,
                        "text": "bring bottle on sofa to table",
                        "type": "bring",
                        "object": "bottle",
                        "source": "sofa",
                        "destination": "table",
                    }
                ],
            },
            "rules": [
                "Return exactly one JSON object and no extra text.",
                "Each subtask must be atomic and executable by the existing agent planner.",
                "Do not output object_query/navigation/grasp_place/finish capability calls in this stage.",
                "For multiple objects joined by and/commas, create one subtask per object.",
                "For then/and then/after that, preserve the requested order.",
                "For move <object> from <source> to <destination>, keep source and destination.",
                "If the user says <object> on/in/at <source> to <destination>, preserve the source in both the source field and the subtask text.",
                "For simple conditions, keep the condition in the subtask text and fill the object/source/destination fields when possible.",
                "Resolve simple pronouns like it to the object mentioned in the condition or previous clause.",
            ],
            "examples": [
                {
                    "input": "bring bottle and apple to table",
                    "output": {
                        "original_task": "bring bottle and apple to table",
                        "subtasks": [
                            {"subtask_id": 1, "text": "bring bottle to table", "type": "bring", "object": "bottle", "source": None, "destination": "table"},
                            {"subtask_id": 2, "text": "bring apple to table", "type": "bring", "object": "apple", "source": None, "destination": "table"},
                        ],
                    },
                },
                {
                    "input": "if the bottle is on sofa, bring it to table",
                    "output": {
                        "original_task": "if the bottle is on sofa, bring it to table",
                        "subtasks": [
                            {"subtask_id": 1, "text": "if bottle is on sofa, bring bottle to table", "type": "conditional", "object": "bottle", "source": "sofa", "destination": "table"}
                        ],
                    },
                },
            ],
        }
        return json.dumps(payload, ensure_ascii=False)

    def _decompose_task_placeholder(self, task_instruction: str) -> dict:
        clauses = _split_sequential_clauses(task_instruction)
        subtasks = []
        last_object = None

        for clause in clauses:
            clause = _clean_text(clause)
            if not clause:
                continue

            conditional = _parse_conditional_clause(clause, last_object)
            if conditional:
                subtasks.append(conditional)
                last_object = conditional.get("object") or last_object
                continue

            parsed = _parse_transfer_clause(clause, last_object)
            if parsed:
                for subtask in parsed:
                    subtasks.append(subtask)
                    last_object = subtask.get("object") or last_object
                continue

            subtasks.append({
                "text": clause,
                "type": "other",
                "object": None,
                "source": None,
                "destination": None,
            })

        if not subtasks:
            subtasks = [{
                "text": _clean_text(task_instruction),
                "type": "other",
                "object": None,
                "source": None,
                "destination": None,
            }]

        return _with_ids({"original_task": task_instruction.strip(), "subtasks": subtasks})


class TaskDecomposer:
    """Validating wrapper around the decomposer backend."""

    def __init__(self, client: DecomposerClient | None = None):
        self.client = client or DecomposerClient()

    def decompose(self, task_instruction: str, context: dict | None = None) -> dict:
        raw = self.client.decompose_task(task_instruction, context or {})
        return validate_decomposition(task_instruction, raw)

    def decompose_texts(self, task_instruction: str, context: dict | None = None) -> List[str]:
        plan = self.decompose(task_instruction, context)
        return [subtask["text"] for subtask in plan["subtasks"]]


def validate_decomposition(original_task: str, decomposition: Dict[str, Any]) -> Dict[str, Any]:
    if not isinstance(decomposition, dict):
        raise TaskDecompositionError("Decomposition must be a dictionary.")

    task = decomposition.get("original_task")
    if not isinstance(task, str) or not task.strip():
        task = original_task.strip()

    subtasks = decomposition.get("subtasks")
    if not isinstance(subtasks, list) or not subtasks:
        raise TaskDecompositionError("Decomposition must include a non-empty subtasks list.")

    normalized = []
    for index, subtask in enumerate(subtasks, start=1):
        if not isinstance(subtask, dict):
            raise TaskDecompositionError(f"Subtask {index} must be a dictionary.")
        text = subtask.get("text")
        if not isinstance(text, str) or not text.strip():
            raise TaskDecompositionError(f"Subtask {index} must include non-empty text.")

        subtask_type = subtask.get("type")
        if not isinstance(subtask_type, str) or not subtask_type.strip():
            subtask_type = "other"
        subtask_type = subtask_type.strip().lower()
        if subtask_type not in SUPPORTED_SUBTASK_TYPES:
            subtask_type = "other"

        normalized.append({
            "subtask_id": int(subtask.get("subtask_id") or index),
            "text": _clean_text(text),
            "type": subtask_type,
            "object": _optional_clean(subtask.get("object")),
            "source": _optional_clean(subtask.get("source")),
            "destination": _optional_clean(subtask.get("destination")),
        })

    for index, subtask in enumerate(normalized, start=1):
        subtask["subtask_id"] = index

    return {"original_task": task.strip(), "subtasks": normalized}


def _with_ids(decomposition: Dict[str, Any]) -> Dict[str, Any]:
    for index, subtask in enumerate(decomposition["subtasks"], start=1):
        subtask["subtask_id"] = index
    return decomposition


def _split_sequential_clauses(text: str) -> List[str]:
    normalized = text.strip()
    normalized = re.sub(r"\s+and\s+then\s+", " then ", normalized, flags=re.IGNORECASE)
    normalized = re.sub(r"\s+after\s+that\s+", " then ", normalized, flags=re.IGNORECASE)
    normalized = re.sub(r"\s*,\s*then\s+", " then ", normalized, flags=re.IGNORECASE)
    return [part.strip(" ,") for part in re.split(r"\s+then\s+", normalized, flags=re.IGNORECASE) if part.strip(" ,")]


def _parse_conditional_clause(clause: str, last_object: str | None) -> dict | None:
    text = _clean_text(clause)
    match = re.match(
        r"^if\s+(?:the\s+)?(.+?)\s+is\s+(?:on|in|at|inside)\s+(?:the\s+)?(.+?),\s*(.+)$",
        text,
        flags=re.IGNORECASE,
    )
    if not match:
        return None

    obj = _clean_text(match.group(1))
    source = _clean_text(match.group(2))
    action = _clean_text(match.group(3))
    action = _replace_pronoun_object(action, obj or last_object)

    transfer = _parse_transfer_clause(action, obj)
    destination = transfer[0].get("destination") if transfer else None
    normalized_action = transfer[0]["text"] if transfer else action
    return {
        "text": f"if {obj} is on {source}, {normalized_action}",
        "type": "conditional",
        "object": obj,
        "source": source,
        "destination": destination,
    }


def _parse_transfer_clause(clause: str, last_object: str | None) -> List[dict] | None:
    text = _replace_pronoun_object(_clean_text(clause), last_object)
    for pattern, task_type in (
        (r"^(?:bring|take|carry)\s+(.+?)\s+to\s+(.+)$", "bring"),
        (r"^move\s+(.+?)\s+from\s+(.+?)\s+to\s+(.+)$", "move"),
    ):
        match = re.match(pattern, text, flags=re.IGNORECASE)
        if not match:
            continue

        if task_type == "move":
            object_phrase, source, destination = match.groups()
        else:
            object_phrase, destination = match.groups()
            source = None

        destination = _strip_destination_modifiers(destination)
        objects = _split_object_list(object_phrase)
        if not objects:
            return None

        subtasks = []
        for obj in objects:
            if task_type == "move":
                subtask_text = f"move {obj} from {_clean_text(source)} to {destination}"
            else:
                subtask_text = f"bring {obj} to {destination}"
            subtasks.append({
                "text": subtask_text,
                "type": task_type,
                "object": obj,
                "source": _optional_clean(source),
                "destination": destination,
            })
        return subtasks
    return None


def _split_object_list(text: str) -> List[str]:
    normalized = _clean_text(text)
    normalized = re.sub(r"\s*,\s*and\s+", ", ", normalized, flags=re.IGNORECASE)
    parts = re.split(r"\s*(?:,|\band\b)\s*", normalized, flags=re.IGNORECASE)
    return [_clean_text(part) for part in parts if _clean_text(part)]


def _replace_pronoun_object(text: str, obj: str | None) -> str:
    if not obj:
        return text
    return re.sub(r"\b(it|them)\b", obj, text, flags=re.IGNORECASE)


def _strip_destination_modifiers(text: str) -> str:
    cleaned = _clean_text(text)
    return re.split(r"\s+(?:and|then|after that)\s+", cleaned, maxsplit=1, flags=re.IGNORECASE)[0]


def _optional_clean(value: Any) -> str | None:
    if not isinstance(value, str):
        return None
    cleaned = _clean_text(value)
    return cleaned or None


def _clean_text(text: str) -> str:
    cleaned = re.sub(r"^(the|a|an)\s+", "", str(text).strip().lower())
    return " ".join(cleaned.split())
