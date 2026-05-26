from decision_maker.task_decomposer import TaskDecomposer, validate_decomposition


def _texts(task):
    return TaskDecomposer().decompose_texts(task)


def test_decompose_multi_object_bring():
    assert _texts("bring bottle and apple to table") == [
        "bring bottle to table",
        "bring apple to table",
    ]


def test_decompose_move_then_bring():
    assert _texts("move bottle from sofa to table, then bring apple to cabinet") == [
        "move bottle from sofa to table",
        "bring apple to cabinet",
    ]


def test_decompose_bring_then_move():
    assert _texts("bring apple to table, then move bottle from chair to cabinet") == [
        "bring apple to table",
        "move bottle from chair to cabinet",
    ]


def test_decompose_simple_conditional_with_pronoun_resolution():
    plan = TaskDecomposer().decompose("if the bottle is on sofa, bring it to table")

    assert [subtask["text"] for subtask in plan["subtasks"]] == [
        "if bottle is on sofa, bring bottle to table"
    ]
    assert plan["subtasks"][0] == {
        "subtask_id": 1,
        "text": "if bottle is on sofa, bring bottle to table",
        "type": "conditional",
        "object": "bottle",
        "source": "sofa",
        "destination": "table",
    }


def test_validate_decomposition_normalizes_ids_and_fields():
    plan = validate_decomposition(
        "bring apple to table",
        {
            "original_task": "bring apple to table",
            "subtasks": [
                {
                    "subtask_id": 99,
                    "text": " Bring Apple to Table ",
                    "type": "BRING",
                    "object": "Apple",
                    "source": None,
                    "destination": "Table",
                }
            ],
        },
    )

    assert plan == {
        "original_task": "bring apple to table",
        "subtasks": [
            {
                "subtask_id": 1,
                "text": "bring apple to table",
                "type": "bring",
                "object": "apple",
                "source": None,
                "destination": "table",
            }
        ],
    }
