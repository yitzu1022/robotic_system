from decision_maker.decision_harness import LightweightPlanningHarness


def test_harness_builds_feedforward_state():
    harness = LightweightPlanningHarness(max_replans_per_task=2)

    state = harness.build_initial_state(
        task_instruction="bring pringles on table to sofa",
        supported_capabilities=["object_query", "navigation", "grasp_place", "robot_pose", "finish"],
        mock_execution=True,
    )

    assert state["task"] == "bring pringles on table to sofa"
    assert state["known_poses"] == {}
    assert state["harness"]["mode"] == "mock"
    assert "planning_rules" in state["harness"]["feedforward"]
    assert "robot_pose" in state["harness"]["feedforward"]["skill_constraints"]
    assert state["harness"]["feedback"]["max_replans"] == 2


def test_harness_records_observation_and_limits_memory():
    harness = LightweightPlanningHarness(memory_limit=2)
    state = harness.build_initial_state("task", ["finish"], mock_execution=True)

    for index in range(3):
        harness.record_observation(
            state,
            {"capability": "robot_pose", "index": index},
            {"success": True, "index": index},
        )

    memory = state["harness"]["feedback"]["execution_memory"]
    assert len(memory) == 2
    assert memory[0]["call"]["index"] == 1
    assert state["harness"]["feedback"]["last_failure"] is None


def test_harness_replan_budget():
    harness = LightweightPlanningHarness(max_replans_per_task=1)
    state = harness.build_initial_state("task", ["finish"], mock_execution=True)

    assert harness.can_replan_after_failure(state) is True
    assert harness.mark_replan_used(state) == 1
    assert harness.can_replan_after_failure(state) is False


def test_harness_rejects_finish_after_failed_history():
    harness = LightweightPlanningHarness()
    ok, message = harness.verify_capability_call(
        {"capability": "finish", "task_done": True},
        {},
        [{"call": {"capability": "navigation"}, "result": {"success": False}}],
    )

    assert ok is False
    assert "finish" in message


def test_harness_loads_rule_documents():
    harness = LightweightPlanningHarness()

    assert "PLANNING.md" in harness.rule_documents
    assert "SKILLS.md" in harness.rule_documents
    assert "object_query" in harness.rule_documents["SKILLS.md"]


def test_harness_task_start_uses_startup_documents_only():
    harness = LightweightPlanningHarness()
    state = harness.build_initial_state("where is the robot", ["robot_pose", "finish"], True)

    decision_state = harness.prepare_decision_state(state, [], None)
    active = decision_state["harness"]["active_context"]
    assert active["stage"] == "task_start"
    assert set(active["rule_documents"]) == {
        "PLANNING.md",
        "SKILLS.md",
        "OUTPUT_FORMAT.md",
        "MEMORY.md",
    }
    assert "REPLANNING.md" not in active["rule_documents"]
    assert "VERIFICATION.md" not in active["rule_documents"]


def test_harness_normal_decision_uses_memory_document_only():
    harness = LightweightPlanningHarness()
    state = harness.build_initial_state("task", ["robot_pose", "finish"], True)
    history = [{"call": {"capability": "robot_pose"}, "result": {"success": True}}]

    decision_state = harness.prepare_decision_state(state, history, history[-1]["result"])
    active = decision_state["harness"]["active_context"]
    assert active["stage"] == "decision"
    assert set(active["rule_documents"]) == {"MEMORY.md", "OUTPUT_FORMAT.md"}


def test_harness_execution_failure_uses_replanning_documents():
    harness = LightweightPlanningHarness()
    state = harness.build_initial_state("task", ["navigation", "finish"], True)
    last_result = {"last_action": "navigation", "success": False, "message": "nav failed"}

    decision_state = harness.prepare_decision_state(state, [], last_result)
    active = decision_state["harness"]["active_context"]
    assert active["stage"] == "execution_replan"
    assert set(active["rule_documents"]) == {"MEMORY.md", "OUTPUT_FORMAT.md", "REPLANNING.md"}


def test_harness_verification_failure_uses_verification_and_replanning_documents():
    harness = LightweightPlanningHarness()
    state = harness.build_initial_state("task", ["finish"], True)
    last_result = {
        "last_action": "harness_verification",
        "success": False,
        "message": "finish rejected",
    }

    decision_state = harness.prepare_decision_state(state, [], last_result)
    active = decision_state["harness"]["active_context"]
    assert active["stage"] == "verification_replan"
    assert set(active["rule_documents"]) == {
        "MEMORY.md",
        "OUTPUT_FORMAT.md",
        "VERIFICATION.md",
        "REPLANNING.md",
    }


def test_harness_builds_decomposition_context():
    harness = LightweightPlanningHarness()

    context = harness.build_decomposition_context(
        "bring bottle and apple to table",
        ["object_query", "navigation", "grasp_place", "finish"],
        mock_execution=True,
    )

    assert context["stage"] == "task_decomposition"
    assert set(context["rule_documents"]) == {
        "DECOMPOSITION.md",
        "DECOMPOSITION_FORMAT.md",
        "SKILLS.md",
    }
    assert "Multiple objects" in " ".join(context["decomposition_constraints"])
    assert "OUTPUT_FORMAT.md" not in context["rule_documents"]
    assert "subtasks" in context["rule_documents"]["DECOMPOSITION_FORMAT.md"]


def test_harness_verifies_valid_decomposition():
    harness = LightweightPlanningHarness()
    ok, message = harness.verify_decomposition(
        "bring bottle and apple to table",
        {
            "original_task": "bring bottle and apple to table",
            "subtasks": [
                {"subtask_id": 1, "text": "bring bottle to table", "object": "bottle"},
                {"subtask_id": 2, "text": "bring apple to table", "object": "apple"},
            ],
        },
    )

    assert ok is True
    assert message == "decomposition verified"


def test_harness_rejects_undecomposed_multi_object_subtask():
    harness = LightweightPlanningHarness()
    ok, message = harness.verify_decomposition(
        "bring bottle and apple to table",
        {
            "original_task": "bring bottle and apple to table",
            "subtasks": [
                {"subtask_id": 1, "text": "bring bottle and apple to table", "object": "bottle and apple"},
            ],
        },
    )

    assert ok is False
    assert "multi-object" in message


def test_harness_loads_decomposition_rule_documents():
    harness = LightweightPlanningHarness()

    assert "DECOMPOSITION.md" in harness.rule_documents
    assert "DECOMPOSITION_FORMAT.md" in harness.rule_documents
    assert "atomic subtasks" in harness.rule_documents["DECOMPOSITION.md"]
    assert "subtasks" in harness.rule_documents["DECOMPOSITION_FORMAT.md"]
