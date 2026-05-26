import pytest

from decision_maker.decision_agent import (
    DecisionAgent,
    PlanValidationError,
    QwenClient,
    normalize_plan,
    plan_to_primitives,
    validate_capability_call,
    validate_plan,
)


def test_valid_bring_plan_converts_to_primitives():
    plan = {
        "task": "bring apple to table",
        "steps": [
            {"action": "goto", "target": "apple"},
            {"action": "grasp", "target": "apple"},
            {"action": "goto", "target": "table"},
            {"action": "place", "target": "apple", "destination": "table"},
        ],
    }

    assert plan_to_primitives(validate_plan(plan)) == [
        "goto:apple",
        "grasp:apple",
        "goto:table",
        "place:apple:table",
    ]


def test_handover_plan_converts_to_primitives():
    plan = {
        "task": "bring me apple",
        "steps": [
            {"action": "goto", "target": "apple"},
            {"action": "grasp", "target": "apple"},
            {"action": "goto", "target": "me"},
            {"action": "handover", "target": "apple"},
        ],
    }

    assert plan_to_primitives(validate_plan(plan))[-1] == "handover:apple"


@pytest.mark.parametrize(
    "plan",
    [
        {"task": "bad"},
        {"task": "bad", "steps": [{"action": "dance", "target": "apple"}]},
        {"task": "bad", "steps": [{"action": "goto"}]},
        {"task": "bad", "steps": [{"action": "place", "target": "apple"}]},
    ],
)
def test_invalid_plans_are_rejected(plan):
    with pytest.raises(PlanValidationError):
        validate_plan(plan)


def test_parse_json_response_accepts_fenced_json():
    fence = chr(96) * 3
    content = fence + 'json\n{"task":"go to apple","steps":[{"action":"goto","target":"apple"}]}\n' + fence

    parsed = QwenClient()._parse_json_response(content)

    assert parsed["steps"][0]["target"] == "apple"


def test_normalize_plan_splits_object_from_source_phrase():
    plan = {
        "task": "bring pringle on cabinet to table",
        "steps": [
            {"action": "goto", "target": "pringle on cabinet"},
            {"action": "grasp", "target": "pringle on cabinet"},
            {"action": "goto", "target": "table"},
            {"action": "place", "target": "pringle on cabinet", "destination": "table"},
        ],
    }

    normalized = normalize_plan("bring pringle on cabinet to table", validate_plan(plan))

    assert plan_to_primitives(normalized) == [
        "goto:cabinet",
        "grasp:pringle",
        "goto:table",
        "place:pringle:table",
    ]


class BadTransferClient:
    def generate_plan(self, task_instruction, context):
        return {
            "task": task_instruction,
            "steps": [
                {"action": "goto", "target": "pringle on cabinet"},
                {"action": "grasp", "target": "pringle on cabinet"},
                {"action": "goto", "target": "table"},
                {"action": "place", "target": "pringle on cabinet", "destination": "table"},
            ],
        }

    def decide_next_capability(self, original_task, current_state, history, last_execution_result):
        return {
            "capability": "object_query",
            "target": "pringles",
            "reason": "bad first guess",
        }


def test_decision_agent_normalizes_single_shot_plan():
    agent = DecisionAgent(BadTransferClient())

    plan = agent.plan("bring pringle on cabinet to table", {})

    assert plan_to_primitives(plan) == [
        "goto:cabinet",
        "grasp:pringle",
        "goto:table",
        "place:pringle:table",
    ]


def test_validate_capability_call_rules():
    assert validate_capability_call({"capability": "object_query", "target": "table"}) == {
        "capability": "object_query",
        "target": "table",
    }

    with pytest.raises(PlanValidationError):
        validate_capability_call({"capability": "dance", "target": "table"})

    with pytest.raises(PlanValidationError):
        validate_capability_call({
            "capability": "grasp_place",
            "action": "place",
            "target": "pringles",
        })


def test_placeholder_iterative_sequence_for_transfer_task():
    agent = DecisionAgent(QwenClient())
    task = "bring pringles on table to sofa"
    history = []
    state = {"known_poses": {}}
    last = None
    calls = []

    for _ in range(7):
        call = agent.decide_next_capability(task, state, history, last)
        calls.append(call)
        if call["capability"] == "object_query":
            state["known_poses"][call["target"]] = {"x": 1.0, "y": 2.0, "theta": 0.0}
            last = {
                "last_action": "object_query",
                "target": call["target"],
                "success": True,
                "result": {"pose": state["known_poses"][call["target"]]},
            }
        else:
            last = {"last_action": call["capability"], "success": True}
        history.append({"call": call, "result": last})

    assert [call["capability"] for call in calls] == [
        "object_query",
        "navigation",
        "grasp_place",
        "object_query",
        "navigation",
        "grasp_place",
        "finish",
    ]
    assert calls[0]["target"] == "pringles"
    assert calls[1]["target"] == "pringles"
    assert calls[1]["pose"] == {"x": 1.0, "y": 2.0, "theta": 0.0}
    assert calls[2]["target"] == "pringles"
    assert calls[5]["destination"] == "sofa"


def test_iterative_transfer_overrides_bad_first_qwen_guess():
    agent = DecisionAgent(BadTransferClient())

    call = agent.decide_next_capability(
        "bring pringles on table to sofa",
        {"known_poses": {}},
        [],
        None,
    )

    assert call["capability"] == "object_query"
    assert call["target"] == "pringles"


def test_robot_pose_capability_validation():
    assert validate_capability_call({"capability": "robot_pose"}) == {
        "capability": "robot_pose",
    }
    assert validate_capability_call({"capability": "robot_pose", "frame": "Map"}) == {
        "capability": "robot_pose",
        "frame": "map",
    }


def test_placeholder_robot_pose_query_sequence():
    agent = DecisionAgent(QwenClient())
    history = []
    state = {"known_poses": {}}
    last = None

    first = agent.decide_next_capability("where is the robot", state, history, last)
    assert first == {
        "capability": "robot_pose",
        "reason": "Need to observe the robot's current pose.",
    }

    last = {
        "last_action": "robot_pose",
        "success": True,
        "result": {"pose": {"x": 0.0, "y": 0.0, "theta": 0.0}},
    }
    history.append({"call": first, "result": last})
    second = agent.decide_next_capability("where is the robot", state, history, last)
    assert second["capability"] == "finish"
    assert second["task_done"] is True


def test_multi_object_single_shot_plan_is_decomposed_per_object():
    agent = DecisionAgent(QwenClient())

    plan = agent.plan("bring bottle and apple to table", {})

    assert plan_to_primitives(plan) == [
        "goto:bottle",
        "grasp:bottle",
        "goto:table",
        "place:bottle:table",
        "goto:apple",
        "grasp:apple",
        "goto:table",
        "place:apple:table",
    ]


def test_multi_object_iterative_sequence_uses_one_object_at_a_time():
    agent = DecisionAgent(QwenClient())
    task = "bring bottle and apple to table"
    state = {"known_poses": {}}
    history = []
    last = None
    calls = []

    for _ in range(13):
        call = agent.decide_next_capability(task, state, history, last)
        calls.append(call)
        if call["capability"] == "object_query":
            state["known_poses"][call["target"]] = {"x": 1.0, "y": 2.0, "theta": 0.0}
            last = {
                "last_action": "object_query",
                "target": call["target"],
                "success": True,
                "result": {"pose": state["known_poses"][call["target"]]},
            }
        else:
            last = {"last_action": call["capability"], "success": True}
        history.append({"call": call, "result": last})

    assert [call["capability"] for call in calls] == [
        "object_query",
        "navigation",
        "grasp_place",
        "object_query",
        "navigation",
        "grasp_place",
        "object_query",
        "navigation",
        "grasp_place",
        "object_query",
        "navigation",
        "grasp_place",
        "finish",
    ]
    assert calls[0]["target"] == "bottle"
    assert calls[2]["target"] == "bottle"
    assert calls[5]["target"] == "bottle"
    assert calls[5]["destination"] == "table"
    assert calls[6]["target"] == "apple"
    assert calls[8]["target"] == "apple"
    assert calls[11]["target"] == "apple"
    assert calls[11]["destination"] == "table"


def test_multi_object_shared_source_is_decomposed_per_object():
    agent = DecisionAgent(QwenClient())

    plan = agent.plan("bring bottle and apple on cabinet to table", {})

    assert plan_to_primitives(plan) == [
        "goto:cabinet",
        "grasp:bottle",
        "goto:table",
        "place:bottle:table",
        "goto:cabinet",
        "grasp:apple",
        "goto:table",
        "place:apple:table",
    ]


class BadMultiObjectClient:
    def generate_plan(self, task_instruction, context):
        return {
            "task": task_instruction,
            "steps": [
                {"action": "goto", "target": "bottle and apple"},
                {"action": "grasp", "target": "bottle and apple"},
                {"action": "goto", "target": "table"},
                {"action": "place", "target": "bottle and apple", "destination": "table"},
            ],
        }

    def decide_next_capability(self, original_task, current_state, history, last_execution_result):
        return {
            "capability": "object_query",
            "target": "bottle and apple",
            "reason": "bad multi-object guess",
        }


def test_multi_object_llm_output_is_not_deterministically_overridden():
    agent = DecisionAgent(BadMultiObjectClient())

    call = agent.decide_next_capability(
        "bring bottle and apple to table",
        {"known_poses": {}},
        [],
        None,
    )

    assert call == {
        "capability": "object_query",
        "target": "bottle and apple",
        "reason": "bad multi-object guess",
    }


def test_multi_object_single_shot_llm_plan_is_not_deterministically_overridden():
    agent = DecisionAgent(BadMultiObjectClient())

    plan = agent.plan("bring bottle and apple to table", {})

    assert plan_to_primitives(plan) == [
        "goto:bottle and apple",
        "grasp:bottle and apple",
        "goto:table",
        "place:bottle and apple:table",
    ]


def test_move_from_source_queries_object_before_source():
    agent = DecisionAgent(QwenClient())

    call = agent.decide_next_capability(
        "move bottle from sofa to table",
        {"known_poses": {}},
        [],
        None,
    )

    assert call["capability"] == "object_query"
    assert call["target"] == "bottle"


def test_move_from_source_falls_back_to_source_after_object_query_failure():
    agent = DecisionAgent(QwenClient())
    last = {
        "last_action": "object_query",
        "target": "bottle",
        "success": False,
        "message": "not found",
    }

    call = agent.decide_next_capability(
        "move bottle from sofa to table",
        {"known_poses": {}},
        [],
        last,
    )

    assert call["capability"] == "object_query"
    assert call["target"] == "sofa"


def test_source_aware_bring_queries_object_before_source():
    agent = DecisionAgent(QwenClient())

    call = agent.decide_next_capability(
        "bring pringles on table to sofa",
        {"known_poses": {}},
        [],
        None,
    )

    assert call["capability"] == "object_query"
    assert call["target"] == "pringles"


def test_source_aware_bring_falls_back_to_source_after_object_query_failure():
    agent = DecisionAgent(QwenClient())
    last = {
        "last_action": "object_query",
        "target": "pringles",
        "success": False,
        "message": "not found",
    }

    call = agent.decide_next_capability(
        "bring pringles on table to sofa",
        {"known_poses": {}},
        [],
        last,
    )

    assert call["capability"] == "object_query"
    assert call["target"] == "table"
