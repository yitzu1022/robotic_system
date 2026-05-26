# Task Decomposition Output Format

The task decomposer must return exactly one JSON object. Do not return markdown, prose, or capability calls.

## Schema

```json
{
  "original_task": "bring bottle and apple to table",
  "subtasks": [
    {
      "subtask_id": 1,
      "text": "bring bottle to table",
      "type": "bring",
      "object": "bottle",
      "source": null,
      "destination": "table"
    }
  ]
}
```

## Required Fields

- `original_task`: original user command string.
- `subtasks`: ordered non-empty list.
- `subtask_id`: integer starting at 1 and increasing by 1.
- `text`: atomic natural-language subtask to pass to the existing agent planner.
- `type`: one of `bring`, `move`, `conditional`, `query`, or `other`.
- `object`: object name if known, otherwise null.
- `source`: source location if known, otherwise null.
- `destination`: destination location if known, otherwise null.

## Examples

Multiple objects:

```json
{
  "original_task": "bring bottle and apple to table",
  "subtasks": [
    {"subtask_id": 1, "text": "bring bottle to table", "type": "bring", "object": "bottle", "source": null, "destination": "table"},
    {"subtask_id": 2, "text": "bring apple to table", "type": "bring", "object": "apple", "source": null, "destination": "table"}
  ]
}
```

Sequential move and bring:

```json
{
  "original_task": "move bottle from sofa to table, then bring apple to cabinet",
  "subtasks": [
    {"subtask_id": 1, "text": "move bottle from sofa to table", "type": "move", "object": "bottle", "source": "sofa", "destination": "table"},
    {"subtask_id": 2, "text": "bring apple to cabinet", "type": "bring", "object": "apple", "source": null, "destination": "cabinet"}
  ]
}
```

Conditional:

```json
{
  "original_task": "if the bottle is on sofa, bring it to table",
  "subtasks": [
    {"subtask_id": 1, "text": "if bottle is on sofa, bring bottle to table", "type": "conditional", "object": "bottle", "source": "sofa", "destination": "table"}
  ]
}
```
